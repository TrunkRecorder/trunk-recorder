#include "smartnet_parser.h"
#include <boost/log/trivial.hpp>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <json.hpp>

using json = nlohmann::json;

// Constants
#define EXPIRY_TIMER 1.0
#define TGID_EXPIRY_TIME 3.0 
#define PATCH_EXPIRY_TIME 5.0
#define ADJ_SITE_EXPIRY_TIME 60.0
#define ALT_CC_EXPIRY_TIME 60.0
#define TGID_DEFAULT_PRIO 3

SmartnetParser::SmartnetParser(System *system) : system(system) {
    this->debug_level = 0;
    this->msgq_id = -1;
    this->sysnum = system->get_sys_num();
    this->osw_count = 0;
    this->last_osw = 0.0;
    this->rx_cc_freq = 0.0;
    this->rx_sys_id = 0;
    this->rx_site_id = 0;
    this->last_expiry_check = 0.0;
}

SmartnetParser::~SmartnetParser() {
}

TrunkMessage SmartnetParser::create_trunk_message(MessageType type, double freq, long talkgroup, int source, bool encrypted, bool emergency) {
    TrunkMessage msg;
    msg.message_type = type;
    msg.freq = freq;
    msg.talkgroup = talkgroup >> 4; // Shift out status bits for the main talkgroup ID? Usually TrunkRecorder expects base TGID
    // But wait, if we pass talkgroup with flags, we should strip them for the ID, but keep them for analysis.
    // In P25 decoder, talkgroup is usually the ID.
    // In Smartnet, bits 0-3 are status.
    // I'll assume the caller passes the full OSW value and I strip it here or caller strips it.
    // For now, I will strip it here:
    msg.talkgroup = talkgroup >> 4;
    
    msg.source = source;
    msg.encrypted = encrypted;
    msg.emergency = emergency;
    msg.sys_num = this->sysnum;
    msg.sys_id = this->rx_sys_id;
    msg.sys_site_id = this->rx_site_id;
    
    // Defaults
    msg.phase2_tdma = false;
    msg.tdma_slot = 0;
    msg.mode = false; // analog default?
    msg.duplex = false;
    msg.priority = TGID_DEFAULT_PRIO;
    
    // Extract flags from tgid if not passed explicitly? 
    // The caller passes 'encrypted' and 'emergency' calculated from tgid status.
    
    return msg;
}

void SmartnetParser::enqueue(int addr, int grp, int cmd, double ts) {
    OSW osw;
    osw.addr = addr;
    osw.grp = (grp != 0);
    osw.cmd = cmd;
    osw.ts = ts;
    
    osw.ch_rx = is_chan(cmd, false);
    osw.ch_tx = is_chan(cmd, true);
    
    osw.f_rx = 0.0;
    osw.f_tx = 0.0;
    if (osw.ch_rx) osw.f_rx = get_freq(cmd, false);
    if (osw.ch_tx) osw.f_tx = get_freq(cmd, true);
    
    if (osw_q.size() >= OSW_QUEUE_SIZE) {
        osw_q.pop_front();
    }
    osw_q.push_back(osw);
}

std::vector<TrunkMessage> SmartnetParser::parse_message(gr::message::sptr msg, System *system) {
    int sysnum = system->get_sys_num();
    time_t curr_time = time(NULL);
    std::vector<TrunkMessage> messages;
    
    if (!msg) return messages;

    long m_proto = (msg->type() >> 16);
    if (m_proto != 2) return messages;

    long m_type = (msg->type() & 0xffff);
    double m_ts = msg->arg2(); 

    if (m_type == M_SMARTNET_TIMEOUT) {
        if (debug_level > 10) {
            BOOST_LOG_TRIVIAL(debug) << "[" << msgq_id << "] control channel timeout";
        }
    } else if (m_type == M_SMARTNET_BAD_OSW) {
        osw_q.clear();
        enqueue(0xffff, 0x1, OSW_QUEUE_RESET_CMD, m_ts);
    } else if (m_type == M_SMARTNET_OSW) {
        std::string s = msg->to_string();
        if (s.length() >= 5) {
            int osw_addr = ((unsigned char)s[0] << 8) | (unsigned char)s[1];
            int osw_grp = (unsigned char)s[2];
            int osw_cmd = ((unsigned char)s[3] << 8) | (unsigned char)s[4];
            enqueue(osw_addr, osw_grp, osw_cmd, m_ts);
            osw_count++;
            last_osw = m_ts;
        }
    }

    std::vector<TrunkMessage> new_msgs = process_osws();
    messages.insert(messages.end(), new_msgs.begin(), new_msgs.end());

    if (curr_time >= last_expiry_check + EXPIRY_TIMER) {
        expire_talkgroups(curr_time);
        expire_patches(curr_time);
        expire_adjacent_sites(curr_time);
        expire_alternate_cc_freqs(curr_time);
        last_expiry_check = curr_time;
    }

    return messages;
}

std::vector<TrunkMessage> SmartnetParser::process_osws() {
    std::vector<TrunkMessage> messages;
    if (osw_q.empty()) return messages;
    
    if (osw_q.size() < OSW_QUEUE_SIZE) return messages;
    
    OSW osw2 = osw_q.front();
    osw_q.pop_front();
    
    bool is_unknown_osw = false;
    bool is_queue_reset = false;
    
    while (osw2.cmd == OSW_QUEUE_RESET_CMD) {
        is_queue_reset = true;
        
        OSW queue_reset = osw2; 
        
        if (osw_q.empty()) break;
        osw2 = osw_q.front();
        osw_q.pop_front();
        
        if (is_queue_reset) {
            if (osw_q.size() == OSW_QUEUE_SIZE - 2) {
                 if (debug_level >= 11) BOOST_LOG_TRIVIAL(debug) << "[" << msgq_id << "] QUEUE RESET";
            } else {
                osw_q.push_front(queue_reset);
                return messages;
            }
        }
    }
    
    if (is_obt_system() && osw2.ch_tx) {
        if (osw_q.empty()) return messages;
        OSW osw1 = osw_q.front(); osw_q.pop_front();
        
        if (osw1.cmd == 0x320 && osw2.grp && osw1.grp) {
             if (osw_q.empty()) { osw_q.push_front(osw1); osw_q.push_front(osw2); return messages; }
             OSW osw0 = osw_q.front(); osw_q.pop_front();
             
             if (osw0.cmd == 0x30b && (osw0.addr & 0xfc00) == 0x6000) {
                 int system = osw2.addr;
                 int site = ((osw1.addr & 0xfc00) >> 10) + 1;
                 int cc_rx_chan = osw0.addr & 0x3ff;
                 double cc_rx_freq = get_freq(cc_rx_chan);
                 double cc_tx_freq = osw2.f_tx;
                 
                 this->rx_sys_id = system;
                 if (osw0.grp) {
                     add_adjacent_site(osw1.ts, site, cc_rx_freq, cc_tx_freq);
                 } else {
                     this->rx_site_id = site;
                     add_alternate_cc_freq(osw1.ts, cc_rx_freq, cc_tx_freq);
                 }
             } else {
                 is_unknown_osw = true;
                 osw_q.push_front(osw0);
             }
        }
        else if (osw1.cmd == 0x2f8 && osw2.ch_tx) {
            // Idle
        }
        else if (osw2.ch_tx && osw1.ch_rx && osw1.grp && osw1.addr != 0 && osw2.addr != 0) {
             // Group Grant
             int mode = osw2.grp ? 0 : 1; // 0=analog, 1=digital? Check python. 
             // Python: mode = 0 if osw2_grp else 1 (ANALOG if osw2_grp else DIGITAL)
             long src_rid = osw2.addr;
             long dst_tgid = osw1.addr;
             double vc_rx_freq = osw1.f_rx;
             
             bool encrypted = (dst_tgid & 0x8) >> 3;
             int options = (dst_tgid & 0x7);
             bool emergency = (options == 2 || options == 4 || options == 5);

             messages.push_back(create_trunk_message(GRANT, vc_rx_freq * 1000000.0, dst_tgid, src_rid, encrypted, emergency));
             update_voice_frequency(osw1.ts, vc_rx_freq, dst_tgid, src_rid, mode);
        }
        else if (osw2.ch_tx && osw1.ch_rx && !osw1.grp && osw1.addr != 0 && osw2.addr != 0) {
             // Private Call
        }
        else {
            is_unknown_osw = true;
            osw_q.push_front(osw1);
        }
    }
    // One-OSW voice update
    else if (osw2.ch_rx && osw2.grp) {
        long dst_tgid = osw2.addr;
        double vc_freq = osw2.f_rx;
        
        bool encrypted = (dst_tgid & 0x8) >> 3;
        int options = (dst_tgid & 0x7);
        bool emergency = (options == 2 || options == 4 || options == 5);
        
        messages.push_back(create_trunk_message(UPDATE, vc_freq * 1000000.0, dst_tgid, 0, encrypted, emergency));
        update_voice_frequency(osw2.ts, vc_freq, dst_tgid);
    }
    // Control Channel Broadcast
    else if (osw2.ch_rx && !osw2.grp && ((osw2.addr & 0xff00) == 0x1f00)) {
        this->rx_cc_freq = osw2.f_rx * 1000000.0;
    }
    // Group Busy Queued
    else if (osw2.cmd == 0x300 && osw2.grp) {
        // Busy
    }
    // Two or Three OSW message
    else if (osw2.cmd == 0x308) {
        if (osw_q.empty()) { osw_q.push_front(osw2); return messages; }
        OSW osw1 = osw_q.front(); osw_q.pop_front();
        
        // Control Channel 2
        if (osw1.ch_rx && !osw1.grp && ((osw1.addr & 0xff00) == 0x1f00)) {
            this->rx_sys_id = osw2.addr;
            this->rx_cc_freq = osw1.f_rx * 1000000.0;
        }
        // Analog Group Voice Grant
        else if (osw1.ch_rx && osw1.grp && osw1.addr != 0 && osw2.addr != 0) {
            long src_rid = osw2.addr;
            long dst_tgid = osw1.addr;
            double vc_freq = osw1.f_rx;
            
            bool encrypted = (dst_tgid & 0x8) >> 3;
            int options = (dst_tgid & 0x7);
            bool emergency = (options == 2 || options == 4 || options == 5);

            messages.push_back(create_trunk_message(GRANT, vc_freq * 1000000.0, dst_tgid, src_rid, encrypted, emergency));
            update_voice_frequency(osw1.ts, vc_freq, dst_tgid, src_rid, 0);
        }
        // Analog Private Call
        else if (osw1.ch_rx && !osw1.grp && osw1.addr != 0 && osw2.addr != 0) {
             // Private call logic
        }
        // System Idle
        else if (osw1.cmd == 0x2f8) {
            if (osw_q.empty()) { osw_q.push_front(osw1); osw_q.push_front(osw2); return messages; }
            OSW osw0 = osw_q.front(); osw_q.pop_front();
            
            // Assume consumed for simplicity or minimal logic
             osw_q.push_front(osw0); 
        }
        else {
             // Check other cases
             if (osw1.cmd == 0x30a) {
                 // Dynamic Regroup
             } else if (osw1.cmd == 0x30b) {
                 // Three OSW...
                 if (osw_q.empty()) { osw_q.push_front(osw1); osw_q.push_front(osw2); return messages; }
                 OSW osw0 = osw_q.front(); osw_q.pop_front();
                 
                 // System ID + CC
                 if (osw1.grp && !osw0.grp && osw0.ch_rx && 
                    (osw0.addr & 0xff00) == 0x1f00 && 
                    (osw1.addr & 0xfc00) == 0x2800 &&
                    (osw1.addr & 0x3ff) == osw0.cmd) {
                        this->rx_sys_id = osw2.addr;
                        this->rx_cc_freq = osw0.f_rx * 1000000.0;
                } else {
                     osw_q.push_front(osw0);
                }
             } else {
                 is_unknown_osw = true;
                 osw_q.push_front(osw1);
             }
        }
    }
    // Digital Group Grant (321)
    else if (osw2.cmd == 0x321) {
         if (osw_q.empty()) { osw_q.push_front(osw2); return messages; }
         OSW osw1 = osw_q.front(); osw_q.pop_front();
         
         if (osw1.ch_rx && osw2.grp && osw1.grp && osw1.addr != 0) {
             long src_rid = osw2.addr;
             long dst_tgid = osw1.addr;
             double vc_freq = osw1.f_rx;
             
             bool encrypted = (dst_tgid & 0x8) >> 3;
             int options = (dst_tgid & 0x7);
             bool emergency = (options == 2 || options == 4 || options == 5);

             messages.push_back(create_trunk_message(GRANT, vc_freq * 1000000.0, dst_tgid, src_rid, encrypted, emergency));
             update_voice_frequency(osw1.ts, vc_freq, dst_tgid, src_rid, 1);
         } else {
             is_unknown_osw = true;
             osw_q.push_front(osw1);
         }
    }
    else if (osw2.cmd == 0x340 && osw2.grp) {
        // Patch
        if (osw_q.empty()) { osw_q.push_front(osw2); return messages; }
        OSW osw1 = osw_q.front(); osw_q.pop_front();
        if (osw1.grp) {
            long tgid = (osw1.addr & 0xfff) << 4; // Reconstruct? Python: (osw1_addr & 0xfff) << 4
            long sub_tgid = osw2.addr & 0xfff0;
            int mode = osw2.addr & 0xf;
            add_patch(osw1.ts, tgid, sub_tgid, mode);
        } else {
            osw_q.push_front(osw1);
        }
    }
    
    if (is_unknown_osw) {
        if (debug_level >= 11) BOOST_LOG_TRIVIAL(debug) << "[" << msgq_id << "] Unknown OSW cmd=" << std::hex << osw2.cmd;
    }
    
    return messages;
}

std::vector<TrunkMessage> SmartnetParser::update_voice_frequency(double ts, double freq, long tgid, int srcaddr, int mode) {
    std::vector<TrunkMessage> msgs;
    if (freq == 0.0) return msgs;
    
    int frequency = (int)(freq * 1000000.0);
    update_talkgroups(ts, frequency, tgid, srcaddr, mode);
    
    int base_tgid = tgid & 0xfff0;
    int flags = tgid & 0x000f;
    
    if (voice_frequencies.find(frequency) == voice_frequencies.end()) {
        VoiceFrequency vf;
        vf.frequency = frequency;
        vf.counter = 0;
        voice_frequencies[frequency] = vf;
    }
    
    if (mode != -1) {
        voice_frequencies[frequency].mode = mode;
    }
    
    voice_frequencies[frequency].tgid = base_tgid;
    voice_frequencies[frequency].flags = flags;
    voice_frequencies[frequency].counter++;
    voice_frequencies[frequency].time = ts;
    
    return msgs;
}

std::vector<TrunkMessage> SmartnetParser::update_talkgroups(double ts, int frequency, long tgid, int srcaddr, int mode) {
    std::vector<TrunkMessage> msgs;
    update_talkgroup(ts, frequency, tgid, srcaddr, mode);
    
    std::lock_guard<std::mutex> lock(patches_mutex);
    if (patches.find(tgid) != patches.end()) {
        for (auto const& [sub_tgid, val] : patches[tgid]) {
             update_talkgroup(ts, frequency, sub_tgid, srcaddr, mode);
        }
    }
    return msgs;
}

bool SmartnetParser::update_talkgroup(double ts, int frequency, long tgid, int srcaddr, int mode) {
    long base_tgid = tgid & 0xfff0;
    int tgid_stat = tgid & 0x000f;
    
    std::lock_guard<std::mutex> lock(talkgroups_mutex);
    if (talkgroups.find(base_tgid) == talkgroups.end()) {
        add_default_tgid(base_tgid);
    } else if (ts < talkgroups[base_tgid].release_time) {
        return false;
    }
    
    talkgroups[base_tgid].time = ts; 
    talkgroups[base_tgid].release_time = 0;
    talkgroups[base_tgid].frequency = frequency;
    talkgroups[base_tgid].status = tgid_stat;
    if (srcaddr >= 0) talkgroups[base_tgid].srcaddr = srcaddr;
    if (mode >= 0) talkgroups[base_tgid].mode = mode;
    
    return true;
}

void SmartnetParser::add_default_tgid(long tgid) {
    TalkgroupInfo ti;
    ti.tgid = tgid;
    ti.priority = TGID_DEFAULT_PRIO;
    ti.srcaddr = 0;
    ti.time = 0;
    ti.release_time = 0;
    ti.mode = -1;
    ti.status = 0;
    ti.frequency = 0;
    talkgroups[tgid] = ti;
}

void SmartnetParser::add_patch(double ts, long tgid, long sub_tgid, int mode) {
    std::lock_guard<std::mutex> lock(patches_mutex);
    if (patches.find(tgid) == patches.end()) {
        patches[tgid] = std::map<long, std::pair<double, int>>();
    }
    patches[tgid][sub_tgid] = std::make_pair(ts, mode);
}

void SmartnetParser::delete_patches(long tgid) {
    std::lock_guard<std::mutex> lock(patches_mutex);
    patches.erase(tgid);
}

bool SmartnetParser::expire_talkgroups(double curr_time) {
    std::lock_guard<std::mutex> lock(talkgroups_mutex);
    // Expiry logic can be implemented here if we want to clean up map
    return true;
}

bool SmartnetParser::expire_patches(double curr_time) {
    std::lock_guard<std::mutex> lock(patches_mutex);
    for (auto it = patches.begin(); it != patches.end(); ) {
        for (auto sub_it = it->second.begin(); sub_it != it->second.end(); ) {
             if (curr_time > sub_it->second.first + PATCH_EXPIRY_TIME) {
                 sub_it = it->second.erase(sub_it);
             } else {
                 ++sub_it;
             }
        }
        if (it->second.empty()) {
            it = patches.erase(it);
        } else {
            ++it;
        }
    }
    return true;
}

bool SmartnetParser::expire_adjacent_sites(double curr_time) {
    for (auto it = adjacent_sites.begin(); it != adjacent_sites.end(); ) {
        if (curr_time > it->second.time + ADJ_SITE_EXPIRY_TIME) {
            it = adjacent_sites.erase(it);
        } else {
            ++it;
        }
    }
    return true;
}

bool SmartnetParser::expire_alternate_cc_freqs(double curr_time) {
    for (auto it = alternate_cc_freqs.begin(); it != alternate_cc_freqs.end(); ) {
        if (curr_time > it->second.time + ALT_CC_EXPIRY_TIME) {
            it = alternate_cc_freqs.erase(it);
        } else {
            ++it;
        }
    }
    return true;
}

void SmartnetParser::add_adjacent_site(double ts, int site, double cc_rx_freq, double cc_tx_freq) {
    AdjacentSite as;
    as.time = ts;
    as.cc_rx_freq = cc_rx_freq;
    as.cc_tx_freq = cc_tx_freq;
    adjacent_sites[site] = as;
}

void SmartnetParser::add_alternate_cc_freq(double ts, double cc_rx_freq, double cc_tx_freq) {
    AlternateCCFreq ac;
    ac.time = ts;
    ac.cc_rx_freq = cc_rx_freq;
    ac.cc_tx_freq = cc_tx_freq;
    int key = (int)(cc_rx_freq * 1000000.0);
    alternate_cc_freqs[key] = ac;
}

std::tuple<std::string, bool, bool, bool, bool> SmartnetParser::get_bandplan_details() {
    std::string bandplan = system->get_bandplan();
    
    if (bandplan == "400") bandplan = "OBT";
    if (bandplan == "800_reband") bandplan = "800_rebanded";
    if (bandplan == "800_standard") bandplan = "800_domestic";
    if (bandplan == "800_splinter") bandplan = "800_domestic_splinter";
    
    bandplan += "_";
    
    std::string band = bandplan.substr(0, 3);
    bool is_rebanded = (bandplan == "800_rebanded_");
    bool is_international = (bandplan.find("_international_") != std::string::npos);
    bool is_splinter = (bandplan.find("_splinter_") != std::string::npos);
    bool is_shuffled = (bandplan.find("_shuffled_") != std::string::npos);
    
    return std::make_tuple(band, is_rebanded, is_international, is_splinter, is_shuffled);
}

bool SmartnetParser::is_obt_system() {
    return std::get<0>(get_bandplan_details()) == "OBT";
}

double SmartnetParser::get_freq(int chan, bool is_tx) {
    auto [band, is_rebanded, is_international, is_splinter, is_shuffled] = get_bandplan_details();
    double freq = 0.0;
    
    if (band == "800") {
        if (!is_international && !is_shuffled) {
            if (is_rebanded) {
                if (chan <= 0x1b7) freq = 851.0125 + (0.025 * chan);
                else if (chan >= 0x1b8 && chan <= 0x22f) freq = 851.0250 + (0.025 * (chan - 0x1b8));
            } else if (is_splinter) {
                 if (chan <= 0x257) freq = 851.0000 + (0.025 * chan);
                 else if (chan >= 0x258 && chan <= 0x2cf) freq = 866.0125 + (0.025 * (chan - 0x258));
            } else {
                if (chan <= 0x2cf) freq = 851.0125 + (0.025 * chan);
            }
            if (chan >= 0x2d0 && chan <= 0x2f7) freq = 866.0000 + (0.025 * (chan - 0x2d0));
            else if (chan >= 0x32f && chan <= 0x33f) freq = 867.0000 + (0.025 * (chan - 0x32f));
            else if (chan >= 0x3c1 && chan <= 0x3fe) freq = 867.4250 + (0.025 * (chan - 0x3c1));
            else if (chan == 0x3be) freq = 868.9750;
        }
        if (is_tx && freq != 0.0) freq -= 45.0;
    } else if (band == "900") {
        freq = 935.0125 + (0.0125 * chan);
        if (is_tx && freq != 0.0) freq -= 39.0;
    } else if (band == "OBT") {
         double bp_base = system->get_bandplan_base();
         double bp_spacing = system->get_bandplan_spacing();
         int bp_base_offset = system->get_bandplan_offset();
         
         if (!is_tx) {
             if (chan >= bp_base_offset) {
                 freq = bp_base + (bp_spacing * (chan - bp_base_offset));
             }
         } else {
             freq = 0.0; 
         }
    }
    
    return std::round(freq * 100000.0) / 100000.0;
}

bool SmartnetParser::is_chan(int chan, bool is_tx) {
    auto [band, is_rebanded, is_international, is_splinter, is_shuffled] = get_bandplan_details();
    if (chan < 0) return false;
    
    if (band == "800") {
        if (!is_international && !is_shuffled) {
             if ((chan >= 0x2d0 && chan <= 0x2f7) ||
                 (chan >= 0x32f && chan <= 0x33f) ||
                 (chan >= 0x3c1 && chan <= 0x3fe) ||
                 (chan == 0x3be)) return true;
             if (is_rebanded && chan <= 0x22f) return true;
             else if (chan <= 0x2cf) return true;
        }
    } else if (band == "900") {
        if (chan <= 0x1de) return true;
    } else if (band == "OBT") {
        int bp_base_offset = system->get_bandplan_offset();
        int bp_tx_base_offset = bp_base_offset - 380; // Default assumption
        
        if (is_tx && chan >= bp_tx_base_offset && chan < 380) return true;
        else if (!is_tx && chan >= bp_base_offset && chan < 760) return true;
    }
    return false;
}

std::string SmartnetParser::get_group_str(bool is_group) {
    return is_group ? "G" : "I";
}

// Other stubs to complete the class
std::string SmartnetParser::get_band_str(int band) { return std::to_string(band); }
double SmartnetParser::get_connect_tone(int index) { return 0.0; }
std::string SmartnetParser::get_features_str(int feat) { return ""; }
std::string SmartnetParser::get_call_options_str(int tgid, bool include_clear) { return ""; }
std::string SmartnetParser::get_call_options_flags_str(int tgid, int mode) { return ""; }
std::string SmartnetParser::get_call_options_flags_web_str(int tgid, int mode) { return ""; }
bool SmartnetParser::is_patch_group(int tgid) { return (tgid & 0x7) == 3 || (tgid & 0x7) == 4; }
bool SmartnetParser::is_multiselect_group(int tgid) { return (tgid & 0x7) == 5 || (tgid & 0x7) == 7; }

double SmartnetParser::get_expected_obt_tx_freq(double rx_freq) {
    if (rx_freq >= 136.0 && rx_freq < 174.0) return rx_freq;
    if (rx_freq >= 380.0 && rx_freq < 406.0) return rx_freq + 10.0;
    if (rx_freq >= 406.0 && rx_freq < 420.0) return rx_freq + 9.0;
    if (rx_freq >= 450.0 && rx_freq < 470.0) return rx_freq + 5.0;
    if (rx_freq >= 470.0 && rx_freq < 512.0) return rx_freq + 3.0;
    return 0.0;
}

std::string SmartnetParser::to_json() {
    json j;
    j["type"] = "smartnet";
    j["system"] = sysnum;
    
    std::string top_line = "Smartnet System ID " + std::to_string(rx_sys_id);
    if (rx_site_id != 0) top_line += " Site " + std::to_string(rx_site_id);
    top_line += " OSW count " + std::to_string(osw_count);
    
    j["top_line"] = top_line;
    
    json freqs = json::object();
    for (const auto& [freq, vf] : voice_frequencies) {
        json f_data;
        f_data["tgid"] = vf.tgid;
        f_data["mode"] = vf.mode;
        f_data["count"] = vf.counter;
        f_data["time"] = vf.time;
        freqs[std::to_string(freq)] = f_data;
    }
    j["frequencies"] = freqs;
    
    return j.dump();
}
