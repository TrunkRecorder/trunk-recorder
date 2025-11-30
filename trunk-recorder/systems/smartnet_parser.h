#ifndef SMARTNET_PARSER_H
#define SMARTNET_PARSER_H

#include "system.h"
#include "parser.h"
#include <gnuradio/message.h>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <vector>
#include <tuple>

// Constants matching Python implementation and protocol definitions
#define OSW_QUEUE_SIZE 5 + 1 // Some messages can be 3 OSWs long, plus up to two IDLEs can be inserted in between
                                // useful messages. Additionally, keep one slot for a QUEUE RESET message.
#define OSW_QUEUE_RESET_CMD 0xFFE
#define M_SMARTNET_TIMEOUT -1
#define M_SMARTNET_OSW 0
#define M_SMARTNET_BAD_OSW -2 // Assumed value for Bad OSW
#define M_SMARTNET_END_PTT 15

struct OSW {
    int addr;
    bool grp;
    int cmd;
    bool ch_rx;
    bool ch_tx;
    double f_rx;
    double f_tx;
    double ts;
};

struct VoiceFrequency {
    int frequency;
    long tgid;
    int flags;
    int mode;
    int counter;
    double time;
};

struct TalkgroupInfo {
    long tgid;
    int priority;
    std::string tag;
    int srcaddr;
    double time;
    double release_time;
    int mode;
    int status;
    int frequency;
};

struct AlternateCCFreq {
    double time;
    double cc_rx_freq;
    double cc_tx_freq;
};

struct AdjacentSite {
    double time;
    double cc_rx_freq;
    double cc_tx_freq;
};

class SmartnetParser {
public:
    SmartnetParser(System *system);
    ~SmartnetParser();

    std::vector<TrunkMessage> parse_message(gr::message::sptr msg, System *system);
    std::vector<TrunkMessage> process_osws(time_t curr_time);
    
    std::string to_json();
    void set_debug(int level) { debug_level = level; }
    void set_msgq_id(int id) { msgq_id = id; }

private:
    System *system;
    int debug_level;
    int sysnum;
    int msgq_id;
    
    std::deque<OSW> osw_q;
    
    std::map<int, VoiceFrequency> voice_frequencies;
    std::map<long, TalkgroupInfo> talkgroups;
    std::mutex talkgroups_mutex;
    
    // tgid -> sub_tgid -> pair<time, mode>
    std::map<long, std::map<long, std::pair<double, int>>> patches;
    std::mutex patches_mutex;

    std::map<int, AlternateCCFreq> alternate_cc_freqs;
    std::map<int, AdjacentSite> adjacent_sites;
    
    // Stats
    long osw_count;
    double last_osw;
    double last_expiry_check;
    double rx_cc_freq;
    long rx_sys_id;
    int rx_site_id;
    
    // Helpers
    void enqueue(int addr, int grp, int cmd, double ts);
    void log_bandplan();
    
    // State updates
    std::vector<TrunkMessage> update_voice_frequency(double ts, double freq, long tgid = -1, int srcaddr = -1, int mode = -1);
    std::vector<TrunkMessage> update_talkgroups(double ts, int frequency, long tgid, int srcaddr, int mode);
    bool update_talkgroup(double ts, int frequency, long tgid, int srcaddr, int mode);
    
    bool expire_talkgroups(double curr_time);
    bool expire_patches(double curr_time);
    bool expire_adjacent_sites(double curr_time);
    bool expire_alternate_cc_freqs(double curr_time);

    void add_adjacent_site(double ts, int site, double cc_rx_freq, double cc_tx_freq);
    void add_alternate_cc_freq(double ts, double cc_rx_freq, double cc_tx_freq);
    void add_patch(double ts, long tgid, long sub_tgid, int mode);
    void delete_patches(long tgid);
    void add_default_tgid(long tgid);

    // Bandplan
    std::tuple<std::string, bool, bool, bool, bool> get_bandplan_details();
    bool is_obt_system();
    double get_expected_obt_tx_freq(double rx_freq);
    bool is_chan(int chan, bool is_tx = false);
    double get_freq(int chan, bool is_tx = false);
    
    // Formatting / Decoding Helpers
    std::string get_group_str(bool is_group);
    std::string get_band_str(int band);
    double get_connect_tone(int index);
    std::string get_features_str(int feat);
    std::string get_call_options_str(int tgid, bool include_clear = true);
    std::string get_call_options_flags_str(int tgid, int mode = -1);
    std::string get_call_options_flags_web_str(int tgid, int mode);
    
    bool is_patch_group(int tgid);
    bool is_multiselect_group(int tgid);
    
    TrunkMessage create_trunk_message(MessageType type, double freq, long talkgroup, int source = 0, bool encrypted = false, bool emergency = false);
};

#endif

