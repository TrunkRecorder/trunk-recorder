#ifndef CALL_H
#define CALL_H

#include "./global_structs.h"
#include <boost/log/trivial.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <string>
#include <sys/time.h>
#include <vector>
#include <cstdint>

class Recorder;
class System;

#include "state.h"
#include "systems/parser.h"
#include "systems/system.h"

class Call {
public:
  // static Call * make(long t, double f, System *s, Config c);
  static Call *make(TrunkMessage message, System *s, Config c);
  virtual ~Call(){};
  virtual long get_call_num() = 0;
  virtual void restart_call() = 0;
  virtual void stop_call() = 0;
  virtual void conclude_call() = 0;
  virtual void set_sigmf_recorder(Recorder *r) = 0;
  virtual Recorder *get_sigmf_recorder() = 0;
  virtual void set_debug_recorder(Recorder *r) = 0;
  virtual Recorder *get_debug_recorder() = 0;
  virtual void set_recorder(Recorder *r) = 0;
  virtual Recorder *get_recorder() = 0;
  virtual double get_freq() = 0;
  virtual int get_sys_num() = 0;
  virtual std::string get_short_name() = 0;
  virtual std::string get_capture_dir() = 0;
  virtual std::string get_temp_dir() = 0;
  virtual void set_freq(double f) = 0;
  virtual long get_talkgroup() = 0;

  virtual bool update(TrunkMessage message) = 0;
  virtual int get_idle_count() = 0;
  virtual void increase_idle_count() = 0;
  virtual void reset_idle_count() = 0;
  virtual int since_last_update() = 0;
  virtual double since_last_voice_update() = 0;
  virtual long elapsed() = 0;

  virtual double get_current_length() = 0;
  virtual long get_stop_time() = 0;
  virtual void set_debug_recording(bool m) = 0;
  virtual bool get_debug_recording() = 0;
  virtual void set_sigmf_recording(bool m) = 0;
  virtual bool get_sigmf_recording() = 0;
  virtual void set_state(State s) = 0;
  virtual State get_state() = 0;
  virtual void set_monitoring_state(MonitoringState s) = 0;
  virtual MonitoringState get_monitoring_state() = 0;
  virtual void set_phase2_tdma(bool m) = 0;
  virtual bool get_phase2_tdma() = 0;
  virtual void set_tdma_slot(int s) = 0;
  virtual int get_tdma_slot() = 0;
  virtual bool get_is_analog() = 0;
  virtual void set_is_analog(bool a) = 0;
  virtual const char *get_xor_mask() = 0;
  virtual time_t get_start_time() = 0;
  virtual std::int64_t get_start_time_ms() = 0;
  virtual bool is_conventional() = 0;
  virtual void set_encrypted(bool m) = 0;
  virtual bool get_encrypted() = 0;
  virtual void set_emergency(bool m) = 0;
  virtual bool get_emergency() = 0;
  virtual int get_priority() = 0;
  virtual bool get_mode() = 0;
  virtual bool get_duplex() = 0;
  virtual double get_signal() = 0;
  virtual double get_noise() = 0;
  virtual int get_freq_error() = 0;
  virtual void set_signal(double s) = 0;
  virtual void set_noise(double n) = 0;
  virtual std::string get_talkgroup_display() = 0;
  virtual void set_talkgroup_tag(std::string tag) = 0;
  virtual void clear_transmission_list() = 0;
  virtual boost::property_tree::ptree get_stats() = 0;

  virtual std::string get_talkgroup_tag() = 0;
  virtual std::string get_system_type() = 0;
  virtual double get_final_length() = 0;
  virtual long get_current_source_id() = 0;
  virtual bool get_conversation_mode() = 0;
  virtual System *get_system() = 0;
  virtual std::vector<Transmission> get_transmissions() = 0;
};

#endif
