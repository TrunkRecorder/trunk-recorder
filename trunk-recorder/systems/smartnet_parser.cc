#include "smartnet_parser.h"
#include <algorithm>
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
    this->debug_level = 1;
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

void SmartnetParser::log_bandplan() {
     auto [band, is_rebanded, is_international, is_splinter, is_shuffled] = get_bandplan_details();
     BOOST_LOG_TRIVIAL(info) << "[SmartnetParser] Bandplan: " << system->get_bandplan() 
                             << " -> Band: " << band 
                             << " Rebanded: " << is_rebanded;
}

std::vector<TrunkMessage> SmartnetParser::parse_message(gr::message::sptr msg, System *system) {
    int sysnum = system->get_sys_num();
    time_t curr_time = time(NULL);
    std::vector<TrunkMessage> messages;


    
    if (!msg) {
        return messages;
    }

    long m_proto = (msg->type() >> 16);
    if (m_proto != 2) {
        return messages;
    }

    long m_type = (msg->type() & 0xffff);
    double m_ts = msg->arg2(); 

    if (m_type == M_SMARTNET_TIMEOUT) {
        if (this->debug_level > 10) {
            BOOST_LOG_TRIVIAL(debug) << "[" << msgq_id << "] control channel timeout";
        }
    } else if (m_type == M_SMARTNET_BAD_OSW) {
        osw_q.clear();
        enqueue(0xffff, 0x1, OSW_QUEUE_RESET_CMD, m_ts);
    } else if (m_type == M_SMARTNET_OSW) {
        if (osw_count == 0) log_bandplan(); // Log bandplan on first OSW
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

    std::vector<TrunkMessage> new_msgs = process_osws(curr_time);
    messages.insert(messages.end(), new_msgs.begin(), new_msgs.end());

    if (curr_time >= last_expiry_check + EXPIRY_TIMER) {
        expire_talkgroups(curr_time);
        expire_patches(curr_time);
        expire_adjacent_sites(curr_time);
        expire_alternate_cc_freqs(curr_time);
        last_expiry_check = curr_time;
    }

    if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET PARSE MESSAGE messages.size(" << messages.size() << ")";
    return messages;
}

std::vector<TrunkMessage> SmartnetParser::process_osws(time_t curr_time) {
    std::vector<TrunkMessage> messages;
    if (osw_q.empty()) {
        return messages;
    }
    
    if (osw_q.size() < OSW_QUEUE_SIZE) {
        return messages;
    }
    
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
                 if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(debug) << "[" << msgq_id << "] QUEUE RESET";
            } else {
                osw_q.push_front(queue_reset);
                if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET PARSE MESSAGE QUEUE RESET PUSHED BACK";
                return messages;
            }
        }
    }
    
    if (is_obt_system() && osw2.ch_tx) {
        if (osw_q.empty()) {
            return messages;
        }
        OSW osw1 = osw_q.front(); 
        osw_q.pop_front();
        
        if (osw1.cmd == 0x320 && osw2.grp && osw1.grp) {
             if (osw_q.empty()) { 
                osw_q.push_front(osw1); 
                osw_q.push_front(osw2); 
                if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET PARSE MESSAGE OSW QUEUE PUSHED FRONT";
                return messages; 
             }
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
                     if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET OBT ADJACENT SITE sys(" << std::hex << system << ") site(" << std::dec << site << ") freq(" << cc_rx_freq << ")";
                 } else {
                     this->rx_site_id = site;
                     add_alternate_cc_freq(osw1.ts, cc_rx_freq, cc_tx_freq);
                     if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET OBT ALT CC sys(" << std::hex << system << ") site(" << std::dec << site << ") freq(" << cc_rx_freq << ")";
                 }
             } else {
                 is_unknown_osw = true;
                 osw_q.push_front(osw0);
             }
        }
        else if (osw1.cmd == 0x2f8 && osw2.ch_tx) {
            // Idle
            if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET OBT IDLE";
        }
        else if (osw2.ch_tx && osw1.ch_rx && osw1.grp && osw1.addr != 0 && osw2.addr != 0) {
             // Group Grant
             int mode = osw2.grp ? 0 : 1; 
             long src_rid = osw2.addr;
             long dst_tgid = osw1.addr;
             double vc_rx_freq = osw1.f_rx;
             
             bool encrypted = (dst_tgid & 0x8) >> 3;
             int options = (dst_tgid & 0x7);
             bool emergency = (options == 2 || options == 4 || options == 5);

             messages.push_back(create_trunk_message(GRANT, vc_rx_freq * 1000000.0, dst_tgid, src_rid, encrypted, emergency));
             update_voice_frequency(osw1.ts, vc_rx_freq, dst_tgid, src_rid, mode);
             
             if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET OBT GROUP GRANT src(" << std::dec << src_rid << ") tgid(" << dst_tgid << ") freq(" << vc_rx_freq << ")";
        }
        else if (osw2.ch_tx && osw1.ch_rx && !osw1.grp && osw1.addr != 0 && osw2.addr != 0) {
             // Private Call
             if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET OBT PRIVATE CALL";
        } 
        else if (osw2.ch_tx && osw1.ch_rx && !osw2.grp && !osw1.grp &&(osw1.addr != 0) && (osw2.addr != 0)) {
             // Interconnect Call
             if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET OBT INTERCONNECT CALL";
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
        
        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET VOICE UPDATE tgid(" << std::dec << dst_tgid << ") freq(" << vc_freq << ")";
    }
    // Control Channel Broadcast
    else if (osw2.ch_rx && !osw2.grp && ((osw2.addr & 0xff00) == 0x1f00)) {
        this->rx_cc_freq = osw2.f_rx * 1000000.0;
        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET CC UPDATE freq(" << osw2.f_rx << ")";
    }
    // Group Busy Queued
    else if (osw2.cmd == 0x300 && osw2.grp) {
        // Busy
        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET GROUP BUSY tgid(" << std::dec << osw2.addr << ")";
    }
    // Emergency Busy Queued
    else if (osw2.cmd == 0x2F8 && !osw2.grp) {
        // Idle
        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET IDLE";
    }
    else if (osw2.cmd == 0x300 && osw2.grp) {
        // Group Busy Queued
        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET GROUP BUSY tgid(" << std::dec << osw2.addr << ")";
    }
    else if (osw2.cmd == 0x303 && osw2.grp) {
        // Emergency Busy Queued
        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET EMERGENCY BUSY tgid(" << std::dec << osw2.addr << ")";
    }
    // Two or Three OSW message
    else if (osw2.cmd == 0x308) {
        if (osw_q.empty()) { osw_q.push_front(osw2); return messages; }
        OSW osw1 = osw_q.front(); osw_q.pop_front();
        
        // Control Channel 2
        if (osw1.ch_rx && !osw1.grp && ((osw1.addr & 0xff00) == 0x1f00)) {
            this->rx_sys_id = osw2.addr;
            this->rx_cc_freq = osw1.f_rx * 1000000.0;
            if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET CC2 sys(" << std::hex << this->rx_sys_id << ") freq(" << osw1.f_rx << ")";
        }
        // Patch/Multiselect (Group) Grant - OBT?
        // This is often 308 followed by channel
        // But for now let's check generic 308 handlers
        else if (osw1.ch_rx && osw1.grp && osw1.addr != 0 && osw2.addr != 0) {
            long src_rid = osw2.addr;
            long dst_tgid = osw1.addr;
            double vc_freq = osw1.f_rx;
            
            bool encrypted = (dst_tgid & 0x8) >> 3;
            int options = (dst_tgid & 0x7);
            bool emergency = (options == 2 || options == 4 || options == 5);

            messages.push_back(create_trunk_message(GRANT, vc_freq * 1000000.0, dst_tgid, src_rid, encrypted, emergency));
            update_voice_frequency(osw1.ts, vc_freq, dst_tgid, src_rid, 0);
            
            if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET ANALOG GRANT src(" << std::dec << src_rid << ") tgid(" << dst_tgid << ") freq(" << vc_freq << ")";
        }
        // Analog Private Call
        else if (osw1.ch_rx && !osw1.grp && osw1.addr != 0 && osw2.addr != 0) {
             // Private call logic
             if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET ANALOG PRIVATE CALL src(" << std::dec << osw1.addr << ") dst(" << osw2.addr << ")";
        }
        else if (osw1.ch_rx && !osw1.grp && !osw2.grp && (osw1.addr != 0) && (osw2.addr != 0)) {
             // Interconnect Call
             if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET INTERCONNECT CALL src(" << std::dec << osw1.addr << ") dst(" << osw2.addr << ")";
        } else if (osw1.cmd == 0x2f8) {
            OSW osw0 = osw_q.front(); osw_q.pop_front();
            
            static const std::vector<int> valid_cmds = {0x30a, 0x30b, 0x30d, 0x310, 0x311, 0x317, 0x318, 0x319, 0x31a, 0x320, 0x322, 0x32e, 0x340};
            bool found = std::find(valid_cmds.begin(), valid_cmds.end(), osw0.cmd) != valid_cmds.end();

            if (!found) {
                osw_q.push_front(osw0);
            } else {
                osw_q.push_front(osw0);
                osw_q.push_front(osw2);
                if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET IDLE (Potential interleaved)";
            }
        } 
        else if (osw1.cmd == 0x300 && osw1.grp) {
            // Group Busy Queued
            if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET GROUP BUSY tgid(" << std::dec << osw1.addr << ")";
        }
        else if (osw1.cmd == 0x302 && !osw1.grp) {
            // Private Call Busy Queued
            if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET PRIVATE CALL BUSY tgid(" << std::dec << osw1.addr << ")";
        }
        else if (osw1.cmd == 0x303 && osw1.grp) {
            // Emergency Busy Queued
            if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET EMERGENCY BUSY tgid(" << std::dec << osw1.addr << ")";
        } else if (osw1.cmd == 0x308) {
            // Two-OSW system idle that got separated and interleaved with a different two- or three-OSW message.
            //
            // Example:
            //   [OSW A-1] [OSW A-2] [IDLE-1] [OSW B-1] [IDLE-2] [OSW B-2] [OSW C-1] [OSW C-2]
            //
            // Reorder it (process it after OSW A-2 and before OSW B-1) and put back the message that it was
            // interleaved with to try processing the message again in the next pass.
            OSW osw0 = osw_q.front(); osw_q.pop_front();

            if (osw0.cmd == 0x2f8) {
                osw_q.push_front(osw0);
                if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET IDLE (Potential interleaved)";
            } else {
                is_unknown_osw = true;
                osw_q.push_front(osw0);
                osw_q.push_front(osw1);
                if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET UNKNOWN OSW (Potential interleaved)";
            }
        }
        else if ((osw1.cmd == 0x30a) && (!osw1.grp && !osw2.grp)) {
            // two-OSW Dynamic Regroup
            if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET DYNAMIC REGROUP";
        }
        else if (osw1.cmd == 0x30b) {  //One of many possible two- or three-OSW meanings...
            // get next OSW in the queue
            
            OSW osw0 = osw_q.front(); osw_q.pop_front();
            if (osw0.cmd == 0x2f8 && !osw0.grp) {
                osw0 = osw_q.front(); osw_q.pop_front();
                if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET IDLE DELAYED 2-1 data(" << osw0.addr << ")";
            }
            
            // Three-OSW system ID + control channel broadcast
            if (osw1.grp && !osw0.grp && osw0.ch_rx && (osw0.addr & 0xff00) == 0x1f00 && (osw1.addr & 0xfc00) == 0x2800 && (osw1.addr & 0x3ff) == osw0.cmd) {
                this->rx_sys_id = osw2.addr;
                this->rx_cc_freq = osw0.f_rx * 1000000.0;
                if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET CC2 sys(" << std::hex << this->rx_sys_id << ") freq(" << osw0.f_rx << ")";
            } else {
                osw_q.push_front(osw0);

                if ((osw1.addr & 0xFC00) == 0x2800 && osw1.grp) {
                    this->rx_sys_id = osw2.addr;
                    this->rx_cc_freq = osw0.f_rx * 1000000.0;
                    if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET CC2 sys(" << std::hex << this->rx_sys_id << ") freq(" << osw0.f_rx << ")";
                } else if ((osw1.addr & 0xFC00) == 0x6000) {
                    int site = ((osw1.addr & 0xFC00) >> 10) + 1;
                    int cc_rx_chan = osw1.addr & 0x3ff;
                    double cc_rx_freq = get_freq(cc_rx_chan);
                    double cc_tx_freq = osw2.f_tx;
                    this->rx_sys_id = osw2.addr;
                    if (!osw1.grp) {
                        add_alternate_cc_freq(curr_time, cc_rx_freq, cc_tx_freq);
                    }
                    if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET ADJACENT/ALTERNATE CC sys(" << std::hex << this->rx_sys_id << ") freq(" << cc_rx_freq << ")";
                } else if (osw1.grp) {  //extended functions on groups
                    // Patch/multiselect cancel
                    if (osw1.addr == 0x2021 && (this->is_patch_group(osw2.addr) || this->is_multiselect_group(osw2.addr))) {
                        //std::string type_str = this->get_call_options_str(osw2.addr, false);
                        long tgid = osw2.addr & 0xfff0;
                        //rc |= this->delete_patches(tgid);
                        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET PATCH/MULTISELECT CANCEL tgid(" << std::dec << tgid << ")";
                    } else {
                        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET GROUP EXTENDED FUNCTION tgid(" << std::dec << osw2.addr << ")";
                    }

                } else {
                    // Extended functions on individuals
                    // Radio check
                    if (osw1.addr == 0x261b) {
                        long tgt_rid = osw2.addr;
                        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET RADIO CHECK tgt(" << std::dec << tgt_rid << ")";
                    } else if (osw1.addr == 0x261c) {  // Deaffiliation
                        long src_rid = osw2.addr;
                        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET DEAFFILIATION src(" << std::dec << src_rid << ")";
                    } else if (osw1.addr >= 0x26e0 && osw1.addr <= 0x26e7) {  // Status acknowledgement
                        long src_rid = osw2.addr;
                        long status = (osw1.addr & 0x7) + 1;
                        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET STATUS ACK src(" << std::dec << src_rid << ") status(" << std::dec << status << ")";
                    } else if (osw1.addr == 0x26e8) {  // Emergency acknowledgement
                        long src_rid = osw2.addr;
                        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET EMERGENCY ALARM ACK src(" << std::dec << src_rid << ")";
                    } else if (osw1.addr >= 0x26f0 && osw1.addr <= 0x26ff) {  // Message acknowledgement
                        long src_rid = osw2.addr;
                        long message = (osw1.addr & 0xf) + 1;
                        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET MESSAGE ACK src(" << std::dec << src_rid << ") msg(" << std::dec << message << ")";
                    } else if (osw1.addr == 0x2c04) {  // Invalid talkgroup (e.g. TGID 0xfff)
                        long src_rid = osw2.addr;
                        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET DENIED INVALID TALKGROUP src(" << std::dec << src_rid << ")";
                    } else if (osw1.addr == 0x2c11) {  // Announcement listen only
                        long src_rid = osw2.addr;
                        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET DENIED ANNOUNCEMENT LISTEN ONLY src(" << std::dec << src_rid << ")";
                    } else if (osw1.addr == 0x2c12) {  // Clear TX only
                        long src_rid = osw2.addr;
                        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET DENIED CLEAR TX ONLY src(" << std::dec << src_rid << ")";
                    } else if (osw1.addr == 0x2c13) {  // Listen only
                        long src_rid = osw2.addr;
                        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET DENIED CLEAR RX ONLY src(" << std::dec << src_rid << ")";
                    } else if (osw1.addr == 0x2c14) {  // No private call
                        long src_rid = osw2.addr;
                        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET DENIED NO PRIVATE CALL src(" << std::dec << src_rid << ")";
                    } else if (osw1.addr == 0x2c15) {  // Private call invalid ID
                        long src_rid = osw2.addr;
                        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET DENIED PRIVATE CALL INVALID ID src(" << std::dec << src_rid << ")";
                    } else if (osw1.addr == 0x2c16) {  // No interconnect
                        long src_rid = osw2.addr;
                        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET DENIED NO INTERCONNECT src(" << std::dec << src_rid << ")";
                    } else if (osw1.addr == 0x2c20) {  // Unsupported mode (CVSD, digital)
                        long src_rid = osw2.addr;
                        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET DENIED UNSUPPORTED MODE src(" << std::dec << src_rid << ")";
                    } else if (osw1.addr == 0x2c41) {  // Private call target offline
                        long src_rid = osw2.addr;
                        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET DENIED PRIVATE CALL TARGET OFFLINE src(" << std::dec << src_rid << ")";
                    } else if (osw1.addr == 0x2c47) {  // Group busy (call in progress)
                        long src_rid = osw2.addr;
                        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET DENIED GROUP BUSY CALL IN PROGRESS src(" << std::dec << src_rid << ")";
                    } else if (osw1.addr == 0x2c48) {  // Private call ring target offline
                        long src_rid = osw2.addr;
                        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET DENIED PRIVATE CALL RING TARGET OFFLINE src(" << std::dec << src_rid << ")";
                    } else if (osw1.addr == 0x2c4a) {  // Radio ID and/or talkgroup forbidden on site
                        long src_rid = osw2.addr;
                        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET DENIED FORBIDDEN ON SITE src(" << std::dec << src_rid << ")";
                    } else if (osw1.addr == 0x2c4e) {  // Call alert invalid ID
                        long src_rid = osw2.addr;
                        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET DENIED CALL ALERT INVALID ID src(" << std::dec << src_rid << ")";
                    } else if (osw1.addr == 0x2c4f) {  // Call alert target offline
                        long src_rid = osw2.addr;
                        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET DENIED CALL ALERT TARGET OFFLINE src(" << std::dec << src_rid << ")";
                    } else if (osw1.addr == 0x2c56) {  // Denied radio wrong modulation (e.g. radio digital, talkgroup analog)
                        long src_rid = osw2.addr;
                        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET DENIED RADIO WRONG MODULATION src(" << std::dec << src_rid << ")";
                    } else if (osw1.addr == 0x2c60) {  // OmniLink trespass rejected
                        long src_rid = osw2.addr;
                        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET DENIED OMNILINK TRESPASS src(" << std::dec << src_rid << ")";
                    } else if (osw1.addr == 0x2c65) {  // Denied radio ID
                        long src_rid = osw2.addr;
                        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET DENIED RADIO ID src(" << std::dec << src_rid << ")";
                    } else if (osw1.addr == 0x2c6a) {  // Denied talkgroup ID
                        long src_rid = osw2.addr;
                        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET DENIED TALKGROUP ID src(" << std::dec << src_rid << ")";
                    } else if (osw1.addr == 0x2c66) {  // Group busy (call is just starting)
                        long src_rid = osw2.addr;
                        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET DENIED GROUP BUSY CALL STARTING src(" << std::dec << src_rid << ")";
                    } else if (osw1.addr == 0x2c90) {  // Private call target busy
                        long src_rid = osw2.addr;
                        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET DENIED PRIVATE CALL TARGET BUSY src(" << std::dec << src_rid << ")";
                    } else if (osw1.addr == 0x8301) {  // Failsoft assign
                        long tgt_rid = osw2.addr;
                        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET FAILSOFT ASSIGN tgt(" << std::dec << tgt_rid << ")";
                    } else if (osw1.addr == 0x8302) {  // Selector unlocked
                        long tgt_rid = osw2.addr;
                        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET SELECTOR UNLOCKED tgt(" << std::dec << tgt_rid << ")";
                    } else if (osw1.addr == 0x8303) {  // Selector locked
                        long tgt_rid = osw2.addr;
                        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET SELECTOR LOCKED tgt(" << std::dec << tgt_rid << ")";
                    } else if (osw1.addr == 0x8305) {  // Failsoft canceled
                        long src_rid = osw2.addr;
                        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET FAILSOFT CANCELED src(" << std::dec << src_rid << ")";
                    } else if (osw1.addr == 0x8306) {  // Radio inhibited
                        long src_rid = osw2.addr;
                        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET RADIO INHIBITED src(" << std::dec << src_rid << ")";
                    } else if (osw1.addr == 0x8307) {  // Radio inhibited
                        long src_rid = osw2.addr;
                        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET RADIO UNINHIBITED src(" << std::dec << src_rid << ")";
                    } else if (osw1.addr == 0x8312) {  // Selector unlock
                        long tgt_rid = osw2.addr;
                        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET SELECTOR UNLOCK tgt(" << std::dec << tgt_rid << ")";
                    } else if (osw1.addr == 0x8313) {  // Selector lock
                        long tgt_rid = osw2.addr;
                        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET SELECTOR LOCK tgt(" << std::dec << tgt_rid << ")";
                    } else if (osw1.addr == 0x8315) {  // Failsoft cancel
                        long tgt_rid = osw2.addr;
                        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET FAILSOFT CANCEL tgt(" << std::dec << tgt_rid << ")";
                    } else if (osw1.addr == 0x8316) {  // Radio inhibited
                        long tgt_rid = osw2.addr;
                        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET RADIO INHIBITED tgt(" << std::dec << tgt_rid << ")";
                    } else if (osw1.addr == 0x8317) {  // Radio uninhibited
                        long tgt_rid = osw2.addr;
                        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET RADIO UNINHIBITED tgt(" << std::dec << tgt_rid << ")";
                    } else if ((osw1.addr & 0xfc00) == 0x2c00) { // Denial
                        long src_rid = osw2.addr;
                        long reason = osw1.addr & 0x3ff;
                        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET DENIED src(" << std::dec << src_rid << ") code(0x" << std::hex << reason << ")";
                    } else {
                        long src_rid = osw2.addr;
                        long opcode = osw1.addr;
                        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET INDIVIDUAL EXTENDED FUNCTION src(" << std::dec << src_rid << ") opcode(0x" << std::hex << opcode << ")";
                    }
                }   
            }
        }
        else if (osw1.cmd == 0x30d && !osw1.grp && !osw2.grp) {
            // Two-OSW status / emergency / dynamic regroup acknowledgement
            long src_rid = osw2.addr;
            long dst_tgid = osw1.addr & 0xfff0;
            long opcode = osw1.addr & 0xf;
            if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET STATUS/EMERGENCY/DYNAMIC REGROUP ACK src(" << std::dec << src_rid << ") tgid(" << std::dec << dst_tgid << ") opcode(0x" << std::hex << opcode << ")";
        }
        else if (osw1.cmd == 0x310 && !osw1.grp && !osw2.grp) {
            // Two-OSW affiliation
            long src_rid = osw2.addr;
            long dst_tgid = osw1.addr & 0xfff0;
            if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET AFFILIATION src(" << std::dec << src_rid << ") tgid(" << std::dec << dst_tgid << ")";
        }
        else if (osw1.cmd == 0x311 && !osw1.grp && !osw2.grp) {
            // Two-OSW message
            long src_rid = osw2.addr;
            long dst_tgid = osw1.addr & 0xfff0;
            long message = (osw1.addr & 0xf) + 1;
            if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET MESSAGE src(" << std::dec << src_rid << ") tgid(" << std::dec << dst_tgid << ") msg(" << std::dec << message << ")";
        } else if (osw1.cmd == 0x315 && !osw1.grp && !osw2.grp) {
            // Two-OSW encrypted private call ring
            long dst_rid = osw2.addr;
            long src_rid = osw1.addr;
            if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET ANALOG ENCRYPTED PRIVATE CALL RING src(" << std::dec << src_rid << ") dst(" << std::dec << dst_rid << ")";
        } else if (osw1.cmd == 0x317 && !osw1.grp && !osw2.grp) {
            // Two-OSW clear private call ring
            long dst_rid = osw2.addr;
            long src_rid = osw1.addr;
            if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET ANALOG CLEAR PRIVATE CALL RING src(" << std::dec << src_rid << ") dst(" << std::dec << dst_rid << ")";
        } else if (osw1.cmd == 0x318 && !osw1.grp && !osw2.grp) {
            // Two-OSW private call ring acknowledgement
            long dst_rid = osw2.addr;
            long src_rid = osw1.addr;
            if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET PRIVATE CALL RING ACK src(" << std::dec << src_rid << ") dst(" << std::dec << dst_rid << ")";
        } else if (osw1.cmd == 0x319 && !osw1.grp && !osw2.grp) {
            // Two-OSW call alert
            long dst_rid = osw2.addr;
            long src_rid = osw1.addr;
            if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET CALL ALERT src(" << std::dec << src_rid << ") dst(" << std::dec << dst_rid << ")";
        } else if (osw1.cmd == 0x31a && !osw1.grp && !osw2.grp) {
            // Two-OSW call alert acknowledgement
            long dst_rid = osw2.addr;
            long src_rid = osw1.addr;
            if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET CALL ALERT ACK src(" << std::dec << src_rid << ") dst(" << std::dec << dst_rid << ")";
        } else if (osw1.cmd == 0x31b && !osw1.grp && !osw2.grp) {
            // Two-OSW OmniLink trespass permitted
            long src_rid = osw2.addr;
            long system = osw1.addr;
            if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET OMNILINK TRESPASS PERMITTED sys(0x" << std::hex << system << ") src(" << std::dec << src_rid << ")";
        } else if (osw1.cmd == 0x320) {
            // Three-OSW system information
            // Get OSW0
            OSW osw0 = osw_q.front(); osw_q.pop_front();

            if (osw0.cmd == 0x2f8 && !osw0.grp) {
                // One-OSW system idle that was delayed by two OSWs and is now stuck between the last two OSWs of a
                // of a different three-OSW message.
                //
                // Example:
                //   [OSW A-1] [OSW A-2] [OSW B-1] [OSW B-2] [IDLE] [OSW B-3] [OSW C-1] [OSW C-2]
                //
                // Reorder it (process it after OSW A-2 and before OSW B-1) and continue processing using the following
                // OSW.

                osw0 = osw_q.front(); osw_q.pop_front();
                if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET IDLE DELAYED 2-2 data(" << osw0.addr << ")";
            }
            else if (osw0.cmd == 0x30b && (osw0.addr & 0xfc00) == 0x6000) {
                // adjacent site
                if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET ADJACENT SITE";
            } else {
                is_unknown_osw = true;
                osw_q.push_front(osw0);
                if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] UKNOWN OSW AFTER BAD OSW";
            }
        } else if (osw1.cmd == 0x322 and osw2.grp and osw1.grp) {
            int year = ((osw2.addr & 0xFE00) >> 9) + 2000;
            int month = (osw2.addr & 0x1E0) >> 5;
            int day = (osw2.addr & 0x1F);
            int dayofweek = (osw1.addr & 0xE000) >> 13;
            std::string dayofweek_str;
            if (dayofweek == 0) {
                dayofweek_str = "Sunday";
            } else if (dayofweek == 1) {
                dayofweek_str = "Monday";
            }
            else if (dayofweek == 2) {
                dayofweek_str = "Tuesday";
            }
            else if (dayofweek == 3) {
                dayofweek_str = "Wednesday";
            }
            else if (dayofweek == 4) {
                dayofweek_str = "Thursday";
            }
            else if (dayofweek == 5) {
                dayofweek_str = "Friday";
            }
            else if (dayofweek == 6) {
                dayofweek_str = "Saturday";
            }
            else {
                dayofweek_str = "unknown day of week";
            }
            int hour = (osw1.addr & 0x1F00) >> 8;
            int minute = osw1.addr & 0xFF;
            if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET DATE/TIME " << year << "-" << month << "-" << day << " " << hour << ":" << minute << " (" << dayofweek_str << ")";
        }
        else if (osw1.cmd == 0x32e && osw2.grp && osw1.grp) {
            long src_rid = osw2.addr;
            long dst_tgid = osw1.addr & 0xfff0;
            if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET EMEREGENCY PTT src(" << std::dec << src_rid << ") tgid(" << std::dec << dst_tgid << ")";
        }
        else if (osw1.cmd == 0x340 && osw2.grp && osw1.grp and (this->is_patch_group(osw2.addr) || this->is_multiselect_group(osw2.addr))) {
            int tgid = (osw1.addr & 0xfff) << 4;
            int sub_tgid = osw2.addr & 0xfff0;
            int mode = osw2.addr & 0xf;
            if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET PATCH/MULTISELECT tgid(" << std::dec << tgid << ") sub_tgid(" << std::dec << sub_tgid << ") mode(0x" << std::hex << mode << ")";
        } else {
            is_unknown_osw = true;
            osw_q.push_front(osw1);
            if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] UKNOWN OSW AFTER BAD OSW"; 
        }
    } else if (osw2.cmd == 0x321) {
        // Get next OSW in the queue
        OSW osw1 = osw_q.front(); osw_q.pop_front();
        if (osw1.ch_rx && osw2.grp && osw1.grp && (osw1.addr != 0)) {
            long src_rid = osw2.addr;
            long dst_tgid = osw1.addr;
            double vc_freq = osw1.f_rx;
            bool encrypted = (dst_tgid & 0x8) >> 3;
            int options = (dst_tgid & 0x7);
            bool emergency = (options == 2 || options == 4 || options == 5);
            messages.push_back(create_trunk_message(GRANT, vc_freq * 1000000.0, dst_tgid, src_rid, encrypted, emergency));
            if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET DIGITAL GROUP GRANT src(" << std::dec << src_rid << ") tgid(" << std::dec << dst_tgid << ") vc_freq(" << vc_freq << ")";
        } else if (osw1.ch_rx && !osw1.grp && (osw1.addr != 0) && (osw2.addr != 0)) {
            if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET DIGITAL PRIVATE CALL src(" << std::dec << osw1.addr << ") dst(" << osw2.addr << ")";
        } else if (osw1.cmd == 0x2f8) {
            OSW osw0 = osw_q.front(); osw_q.pop_front();
            if (osw0.cmd != 0x317 && osw0.cmd != 0x318) {
                osw_q.push_front(osw0);
                if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET IDLE DIGITAL";
            } else {
                osw_q.push_front(osw0);
                osw_q.push_front(osw2);
                if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET IDLE DELAYED 1-2 data(" << osw1.addr << ")";
            }
        } else if (osw1.cmd == 0x315 && !osw1.grp && !osw2.grp) {
            long dst_rid = osw2.addr;
            long src_rid = osw1.addr;
            if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET DIGITAL ENCRYPTED PRIVATE CALL RING src(" << std::dec << src_rid << ") dst(" << std::dec << dst_rid << ")";
        } else {
            is_unknown_osw = true;
            osw_q.push_front(osw1);
            if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] UKNOWN OSW AFTER BAD OSW";
        }
    } else if (osw2.cmd == 0x324 && osw2.grp) {
        // One-OSW interconnect reject
        long src_rid = osw2.addr;
        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET INTERCONNECT REJECT src(" << std::dec << src_rid << ")";
    } else if (osw2.cmd == 0x32a && osw2.grp) {
        // One-OSW send affiliation request
        long tgt_rid = osw2.addr;
        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET SEND AFFILIATION REQUEST tgt(" << std::dec << tgt_rid << ")";
    } else if (osw2.cmd == 0x32b && !osw2.grp) {
        // One-OSW system ID / scan marker
        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET SYSTEM ID / SCAN MARKER sys(" << std::hex << osw2.addr << ")";
    } else if (osw2.cmd == 0x32c && !osw2.grp) {
        // One-OSW roaming
        long src_rid = osw2.addr;
        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET ROAMING src(" << std::dec << src_rid << ")";
    } else if (osw2.cmd >= 0x360 && osw2.cmd <= 0x39f) {
        // One-OSW AMSS (Automatic Multiple Site Select) message
        int site = osw2.cmd - 0x360 + 1;
        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET AMSS site(" << std::dec << site << ")";
    } else if (osw2.cmd == 0x3a0 && osw2.grp) {
        // One-OSW BSI / diagnostic
        long opcode = (osw2.addr & 0xf000) >> 12;
        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET BSI / DIAGNOSTIC opcode(0x" << std::hex << opcode << ")";
    } else if (osw2.cmd == 0x3bf || osw2.cmd == 0x3c0) {
        // One-OSW system status update
        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] SMARTNET SYS STATUS";
    } else {
        is_unknown_osw = true;
        osw_q.push_front(osw2);
        if (this->debug_level >= 11) BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] UKNOWN OSW AFTER BAD OSW";
    }


    if (is_unknown_osw) {
        if (this->debug_level >= 1) {
             // We might have pushed back osw1/osw0 so osw2 is the "current" unknown one if we didn't process it?
             // Actually, the logic often pushes back items.
             // If we are here, we processed osw2 (popped it) but couldn't match it or the sequence following it.
             // The original Python code logs "UNKNOWN OSW (0x...)" for the specific OSW that wasn't handled.
             BOOST_LOG_TRIVIAL(info) << "[" << msgq_id << "] Unknown OSW cmd=" << std::hex << osw2.cmd << " addr=" << osw2.addr << " grp=" << osw2.grp
                                     << " ch_rx=" << osw2.ch_rx << " ch_tx=" << osw2.ch_tx;
        }
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
    
    if (bandplan == "400" || bandplan == "400_custom") bandplan = "OBT";
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
