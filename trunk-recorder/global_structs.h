#ifndef GLOBAL_STRUCTS_H
#define GLOBAL_STRUCTS_H
#include <ctime>
#include <string>
#include <vector>
#include <json.hpp>

const int DB_UNSET = 999;

struct Transmission {
  long source;
  long talkgroup;
  unsigned int slot;
  unsigned int color_code;
  long start_time;
  long stop_time;
  long sample_count;
  long spike_count;
  long error_count;
  double freq;
  double length;
  char filename[255];
};

struct Config {
  std::string upload_script;
  std::string upload_server;
  std::string bcfy_calls_server;
  std::string status_server;
  std::string instance_key;
  std::string instance_id;
  std::string capture_dir;
  std::string temp_dir;
  std::string debug_recorder_address;
  std::string log_dir;
  std::string default_mode;
  bool new_call_from_update;
  bool debug_recorder;
  int debug_recorder_port;
  double call_timeout;
  bool console_log;
  bool log_file;
  std::string log_color;
  int control_message_warn_rate;
  int control_retune_limit;
  bool broadcast_signals;
  bool enable_audio_streaming;
  bool soft_vocoder;
  bool record_uu_v_calls;
  bool archive_files_on_failure;
  int frequency_format;
};

struct Call_Source {
  long source;
  long time;
  double position;
  bool emergency;
  std::string signal_system;
  std::string tag;
};

struct Call_Freq {
  double freq;
  long time;
  double position;
  double total_len;
  double error_count;
  double spike_count;
};

struct Call_Error {
  long time;
  double position;
  double total_len;
  double error_count;
  double spike_count;
};

enum Call_Data_Status { INITIAL,
                        SUCCESS,
                        RETRY,
                        FAILED };
                  
enum Recorder_Type { DEBUG,
                      SIGMF,
                      SIGMFC,
                      ANALOG,
                      ANALOGC,
                      P25,
                      P25C,
                      DMR,
                      SMARTNET };

struct Call_Data_t {
  long talkgroup;
  long color_code;
  std::vector<unsigned long> patched_talkgroups;
  std::string talkgroup_tag;
  std::string talkgroup_alpha_tag;
  std::string talkgroup_description;
  std::string talkgroup_display;
  std::string talkgroup_group;
  long call_num;
  double freq;
  int freq_error;
  int source_num;
  int recorder_num;
  double signal;
  double noise;
  long start_time;
  long stop_time;
  long error_count;
  long spike_count;
  bool encrypted;
  bool emergency;
  int priority;
  bool mode;
  bool duplex;
  bool audio_archive;
  bool transmission_archive;
  bool archive_files_on_failure;
  bool call_log;
  bool compress_wav;
  char filename[300];
  char status_filename[300];
  char converted[300];
  int min_transmissions_removed;

  int sys_num;
  std::string short_name;
  std::string upload_script;
  std::string audio_type;

  int tdma_slot;
  double length;
  bool phase2_tdma;

  std::vector<Call_Source> transmission_source_list;
  std::vector<Call_Error> transmission_error_list;
  std::vector<Transmission> transmission_list;

  Call_Data_Status status;
  time_t process_call_time;
  int retry_attempt;

  std::vector<int> plugin_retry_list;
  nlohmann::ordered_json call_json;
};

#endif
