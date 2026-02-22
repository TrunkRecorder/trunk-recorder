#ifndef CALL_IMPL_H
#define CALL_IMPL_H

#include "./global_structs.h"
#include "gr_blocks/decoder_wrapper.h"
#include <boost/log/trivial.hpp>
#include <string>
#include <sys/time.h>
#include <vector>

class Recorder;
class System;

#include "call.h"
#include "state.h"
#include "systems/parser.h"
#include "systems/system.h"
#include "systems/system_impl.h"
#include <op25_repeater/include/op25_repeater/rx_status.h>
// enum  CallState { MONITORING=0, recording=1, stopping=2};

class Call_impl : public Call {
public:
  Call_impl(long t, double f, System *s, Config c);
  Call_impl(TrunkMessage message, System *s, Config c);

  long get_call_num();
  virtual void restart_call();
  void stop_call();
  void conclude_call();
  void set_sigmf_recorder(Recorder *r);
  Recorder *get_sigmf_recorder();
  void set_debug_recorder(Recorder *r);
  Recorder *get_debug_recorder();
  virtual void set_recorder(Recorder *r);
  Recorder *get_recorder();
  double get_freq();
  int get_sys_num();
  std::string get_short_name();
  std::string get_capture_dir();
  std::string get_temp_dir();
  void set_freq(double f);
  long get_talkgroup();

  bool update(TrunkMessage message);
  int get_idle_count();
  void increase_idle_count();
  void reset_idle_count();
  double since_last_voice_update();
  int since_last_update();
  long elapsed();

  double get_current_length();
  long get_stop_time();
  void set_debug_recording(bool m);
  bool get_debug_recording();
  void set_sigmf_recording(bool m);
  bool get_sigmf_recording();
  void set_state(State s);
  State get_state();
  void set_monitoring_state(MonitoringState s);
  MonitoringState get_monitoring_state();
  void set_phase2_tdma(bool m);
  bool get_phase2_tdma();
  void set_tdma_slot(int s);
  int get_tdma_slot();
  bool get_is_analog();
  void set_is_analog(bool a);
  const char *get_xor_mask();
  virtual time_t get_start_time() { return start_time; }
  bool is_conventional() { return false; }
  void set_encrypted(bool m);
  bool get_encrypted();
  void set_emergency(bool m);
  bool get_emergency();
  int get_priority();
  bool get_mode();
  bool get_duplex();
  double get_signal();
  double get_noise();
  int get_freq_error();
  void set_signal(double s);
  void set_noise(double n);
  std::string get_talkgroup_display();
  void set_talkgroup_tag(std::string tag);
  void clear_transmission_list();
  boost::property_tree::ptree get_stats();

  std::string get_talkgroup_tag();
  std::string get_system_type();
  double get_final_length();
  long get_current_source_id();
  bool get_conversation_mode();
  System *get_system();
  std::vector<Transmission> get_transmissions();

protected:
  State state;
  MonitoringState monitoringState;
  static long call_counter;
  long call_num;
  long talkgroup;
  double curr_freq;
  double noise;
  double signal;
  int freq_error;
  std::vector<Transmission> transmission_list;
  System *sys;
  std::string short_name;
  long curr_src_id;
  long error_list_count;
  long freq_count;
  time_t last_update;
  int idle_count;
  time_t stop_time;
  time_t start_time;
  bool debug_recording;
  bool sigmf_recording;
  bool was_update;
  bool encrypted;
  bool emergency;
  bool mode;
  bool duplex;
  bool is_analog;
  int priority;
  std::string filename;
  std::string transmission_filename;
  std::string converted_filename;
  std::string status_filename;
  std::string debug_filename;
  std::string sigmf_filename;
  std::string path;
  bool phase2_tdma;
  int tdma_slot;
  double final_length;

  Config config;
  Recorder *recorder;
  Recorder *debug_recorder;
  Recorder *sigmf_recorder;
  bool add_source(long src);
  std::string talkgroup_display;
  std::string talkgroup_tag;
  void update_talkgroup_display();
};

int plugman_signal(long unitId, const char *signaling_type, gr::blocks::SignalType sig_type, Call *call, System *system, Recorder *recorder);

#endif
