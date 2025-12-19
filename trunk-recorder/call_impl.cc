#include "call_impl.h"
#include "call.h"
#include "call_concluder/call_concluder.h"
#include "formatter.h"
#include "recorder_globals.h"
#include "recorders/recorder.h"
#include "source.h"
#include <boost/algorithm/string.hpp>
#include <signal.h>
#include <stdio.h>

std::string Call_impl::get_capture_dir() {
  return this->config.capture_dir;
}

std::string Call_impl::get_temp_dir() {
  return this->config.temp_dir;
}

/*
Call * Call::make(long t, double f, System *s, Config c) {

  return (Call *) new Call_impl(t, f, s, c);
}*/

Call *Call::make(TrunkMessage message, System *s, Config c) {
  return (Call *)new Call_impl(message, s, c);
}

Call_impl::Call_impl(long t, double f, System *s, Config c) {
  config = c;
  call_num = call_counter++;
  noise = DB_UNSET;
  signal = DB_UNSET;
  final_length = 0;
  idle_count = 0;
  curr_freq = 0;
  freq_error = 0;
  curr_src_id = -1;
  talkgroup = t;
  sys = s;
  start_time = time(NULL);
  stop_time = time(NULL);
  last_update = time(NULL);
  state = MONITORING;
  monitoringState = UNSPECIFIED;
  debug_recording = false;
  sigmf_recording = false;
  recorder = NULL;
  phase2_tdma = false;
  tdma_slot = 0;
  encrypted = false;
  emergency = false;
  duplex = false;
  mode = false;
  is_analog = false;
  was_update = false;
  priority = 0;
  set_freq(f);
  this->update_talkgroup_display();
}

Call_impl::Call_impl(TrunkMessage message, System *s, Config c) {
  config = c;
  call_num = call_counter++;
  noise = DB_UNSET;
  signal = DB_UNSET;
  final_length = 0;
  idle_count = 0;
  curr_src_id = -1;
  curr_freq = 0;
  freq_error = 0;
  talkgroup = message.talkgroup;
  sys = s;
  start_time = time(NULL);
  stop_time = time(NULL);
  last_update = time(NULL);
  state = MONITORING;
  monitoringState = UNSPECIFIED;
  debug_recording = false;
  sigmf_recording = false;
  recorder = NULL;
  phase2_tdma = message.phase2_tdma;
  tdma_slot = message.tdma_slot;
  encrypted = message.encrypted;
  emergency = message.emergency;
  duplex = message.duplex;
  mode = message.mode;
  is_analog = false;
  priority = message.priority;
  if (message.message_type == GRANT) {
    was_update = false;
  } else {
    was_update = true;
  }
  set_freq(message.freq);
  add_source(message.source);
  this->update_talkgroup_display();
}
/*
Call_impl::~Call_impl() {

}*/

void Call_impl::restart_call() {
}

void Call_impl::stop_call() {

  if (this->get_recorder() != NULL) {
    // If the call is being recorded, check to see if the recorder is currently in an INACTIVE state. This means that the recorder is not
    // doing anything and can be stopped.
    if ((state == RECORDING) && this->get_recorder()->is_idle()) {
      std::string loghdr = log_header( sys->get_short_name(), this->get_call_num(), this->get_talkgroup_display(), this->get_freq());
      BOOST_LOG_TRIVIAL(info) << loghdr << "Stopping Recorded Call_impl - Last Update: " << this->since_last_update() << "s";
    }
  }
}
long Call_impl::get_call_num() {
  return call_num;
}
void Call_impl::conclude_call() {

  // BOOST_LOG_TRIVIAL(info) << "conclude_call()";
  stop_time = time(NULL);

  if (state == RECORDING || (state == MONITORING && monitoringState == SUPERSEDED)) {
    if (!recorder) {
      BOOST_LOG_TRIVIAL(error) << "Call_impl::end_call() State is recording, but no recorder assigned!";
    } else {
      final_length = recorder->get_current_length();

    }


    if (this->is_conventional()) {
      // Update the signal and noise levels for the call
      // the squelch could be open if the program is being forced to stop
      if (this->get_recorder()->is_idle()) {
        this->set_noise(this->get_recorder()->get_pwr());
      } else {
        this->set_signal(this->get_recorder()->get_pwr());
      }
        std::string loghdr = log_header( sys->get_short_name(), this->get_call_num(), this->get_talkgroup_display(), this->get_freq());
        BOOST_LOG_TRIVIAL(info) << loghdr << "\u001b[33mConcluding Recorded Call\u001b[0m - Last Update: " << this->since_last_update() << "s\tRecorder last write:" << recorder->since_last_write() << "\tCall Elapsed: " << this->elapsed() << "\t Signal: " << floor(this->get_signal()) << "dBm\t Noise: " << floor(this->get_noise()) << "dBm";
    } else {
        std::string loghdr = log_header( sys->get_short_name(), this->get_call_num(), this->get_talkgroup_display(), this->get_freq());
        BOOST_LOG_TRIVIAL(info) << loghdr << "\u001b[33mConcluding Recorded Call\u001b[0m - Last Update: " << this->since_last_update() << "s\tRecorder last write:" << recorder->since_last_write() << "\tCall Elapsed: " << this->elapsed();
    }
    if (was_update) {
      std::string loghdr = log_header( sys->get_short_name(), this->get_call_num(), this->get_talkgroup_display(), this->get_freq());
      BOOST_LOG_TRIVIAL(info) << loghdr << "\u001b[33mCall was UPDATE not GRANT\u001b[0m";
    }
    freq_error = this->get_recorder()->get_freq_error();
    this->get_recorder()->stop();

    if (this->get_sigmf_recording() == true) {
      this->get_sigmf_recorder()->stop();
    }

    if (this->get_debug_recording() == true) {
      this->get_debug_recorder()->stop();
    }

    if (this->sys->get_system_type() == "conventionalDMR") {
      dmr_recorder *recorder = dynamic_cast<dmr_recorder *>(this->get_recorder());
      // Conventional DMR is recorded on two slots, so we need to conclude the call for each slot
      transmission_list = recorder->get_transmission_list(0);
      tdma_slot = 0;
      Call_Concluder::conclude_call(this, sys, config);
      transmission_list = recorder->get_transmission_list(1);
      tdma_slot = 1;
      Call_Concluder::conclude_call(this, sys, config);
    } else {
      // All other system types do not have multiple recorders
      transmission_list = this->get_recorder()->get_transmission_list();
      Call_Concluder::conclude_call(this, sys, config);
   }

  }
}
void Call_impl::set_sigmf_recorder(Recorder *r) {
  sigmf_recorder = r;
}

Recorder *Call_impl::get_sigmf_recorder() {
  return sigmf_recorder;
}

void Call_impl::set_debug_recorder(Recorder *r) {
  debug_recorder = r;
}

Recorder *Call_impl::get_debug_recorder() {
  return debug_recorder;
}

void Call_impl::set_recorder(Recorder *r) {
  recorder = r;
}

Recorder *Call_impl::get_recorder() {
  return recorder;
}

double Call_impl::get_freq() {
  return curr_freq;
}

int Call_impl::get_freq_error() {
  return freq_error;
}

double Call_impl::get_final_length() {
  return final_length;
}

double Call_impl::get_current_length() {
  if ((state == RECORDING) && recorder) {
    if (!recorder) {
      BOOST_LOG_TRIVIAL(error) << "Call_impl::get_current_length() State is recording, but no recorder assigned!";
    }
    return get_recorder()->get_current_length(); // This could SegFault
  } else {
    return 0; // time(NULL) - start_time;
  }
}

System *Call_impl::get_system() {
  return sys;
}

void Call_impl::set_freq(double f) {
  if (f != curr_freq) {
    curr_freq = f;
  }
}

int Call_impl::get_sys_num() {
  return sys->get_sys_num();
}
std::string Call_impl::get_short_name() {
  return sys->get_short_name();
}
long Call_impl::get_talkgroup() {
  return talkgroup;
}

std::vector<Transmission> Call_impl::get_transmissions() {
  return transmission_list;
}

void Call_impl::clear_transmission_list() {
  transmission_list.clear();
  transmission_list.shrink_to_fit();
}

void Call_impl::set_debug_recording(bool m) {
  debug_recording = m;
}

bool Call_impl::get_debug_recording() {
  return debug_recording;
}

void Call_impl::set_sigmf_recording(bool m) {
  sigmf_recording = m;
}

void Call_impl::set_is_analog(bool a) {
  is_analog = a;
}

bool Call_impl::get_is_analog() {
  return is_analog;
}

bool Call_impl::get_sigmf_recording() {
  return sigmf_recording;
}

void Call_impl::set_state(State s) {
  state = s;
}

State Call_impl::get_state() {
  return state;
}

void Call_impl::set_monitoring_state(MonitoringState s) {
  monitoringState = s;
}

MonitoringState Call_impl::get_monitoring_state() {
  return monitoringState;
}

void Call_impl::set_encrypted(bool m) {
  if (encrypted != m) {
    encrypted = m;
    update_talkgroup_display();
  }
}

bool Call_impl::get_encrypted() {
  return encrypted;
}

void Call_impl::set_emergency(bool m) {
  emergency |= m;
}

bool Call_impl::get_emergency() {
  return emergency;
}

int Call_impl::get_priority() {
  return priority;
}

bool Call_impl::get_mode() {
  return mode;
}

bool Call_impl::get_duplex() {
  return duplex;
}

double Call_impl::get_signal() {
  return signal;
}

double Call_impl::get_noise() {
  return noise;
}

void Call_impl::set_signal(double s) {
  signal = s;
}

void Call_impl::set_noise(double n) {
  noise = n;
}

void Call_impl::set_tdma_slot(int m) {
  tdma_slot = m;
  if (!phase2_tdma && tdma_slot) {
    BOOST_LOG_TRIVIAL(error) << "WHAT! SLot is 1 and TDMA is off";
  }
}

int Call_impl::get_tdma_slot() {
  return tdma_slot;
}

void Call_impl::set_phase2_tdma(bool p) {
  phase2_tdma = p;
}

bool Call_impl::get_phase2_tdma() {
  return phase2_tdma;
}

const char *Call_impl::get_xor_mask() {
  return sys->get_xor_mask();
}

long Call_impl::get_current_source_id() {
  return curr_src_id;
}

bool Call_impl::add_source(long src) {
  if (src == -1) {
    return false;
  }

  if (src == curr_src_id) {
    return false;
  }

  curr_src_id = src;

  if (state == RECORDING) {
    Recorder *rec = this->get_recorder();
    if (rec != NULL) {
      rec->set_source(src);
    }
  }

  plugman_signal(src, NULL, gr::blocks::SignalType::Normal, this, this->get_system(), NULL);

  return true;
}

bool Call_impl::update(TrunkMessage message) {
  last_update = time(NULL);
  if ((message.freq != this->curr_freq) || (message.talkgroup != this->talkgroup)) {
    std::string loghdr = log_header( sys->get_short_name(), this->get_call_num(), this->get_talkgroup_display(), this->get_freq());
    BOOST_LOG_TRIVIAL(error) << loghdr << "C\033[0m\tCall_impl Update, message mismatch - \ttMsg Tg: " << message.talkgroup << "\tMsg Freq: " << message.freq;
  } else {
    return add_source(message.source);
  }
  return false;
}

int Call_impl::since_last_update() {
  return time(NULL) - last_update;
}

double Call_impl::since_last_voice_update() {
  if (state == RECORDING) {
    Recorder *rec = this->get_recorder();
    if (rec != NULL) {
      return rec->since_last_write();
    }
  }
  return -1;
}

long Call_impl::elapsed() {
  return time(NULL) - start_time;
}

int Call_impl::get_idle_count() {
  return idle_count;
}

void Call_impl::reset_idle_count() {
  idle_count = 0;
}

void Call_impl::increase_idle_count() {
  idle_count++;
}

long Call_impl::get_stop_time() {
  return stop_time;
}

std::string Call_impl::get_system_type() {
  return sys->get_system_type();
}

void Call_impl::set_talkgroup_tag(std::string tag) {
  talkgroup_tag = tag;
  update_talkgroup_display();
}

std::string Call_impl::get_talkgroup_display() {
  return talkgroup_display;
}

std::string Call_impl::get_talkgroup_tag() {
  return talkgroup_tag;
}

bool Call_impl::get_conversation_mode() {
  if (!sys) {
    BOOST_LOG_TRIVIAL(error) << "\tWEIRD! for some reason, call has no sys - Call_impl TG: " << get_talkgroup() << "\t Call_impl Freq: " << get_freq();
    return false;
  }
  return sys->get_conversation_mode();
}

void Call_impl::update_talkgroup_display() {
  boost::trim(talkgroup_tag);
  if (talkgroup_tag.empty()) {
    talkgroup_tag = "-";
  }

  char formattedTalkgroup[62];
  int color = encrypted ? 31 : 35; // Red for encrypted, magenta for normal
  if (this->sys->get_talkgroup_display_format() == talkGroupDisplayFormat_id_tag) {
    snprintf(formattedTalkgroup, 61, "%10ld (%c[%dm%23s%c[0m)", talkgroup, 0x1B, color, talkgroup_tag.c_str(), 0x1B);
  } else if (this->sys->get_talkgroup_display_format() == talkGroupDisplayFormat_tag_id) {
    snprintf(formattedTalkgroup, 61, "%c[%dm%23s%c[0m (%10ld)", 0x1B, color, talkgroup_tag.c_str(), 0x1B, talkgroup);
  } else {
    snprintf(formattedTalkgroup, 61, "%c[%dm%10ld%c[0m", 0x1B, color, talkgroup, 0x1B);
  }
  talkgroup_display = boost::lexical_cast<std::string>(formattedTalkgroup);
}

boost::property_tree::ptree Call_impl::get_stats() {
  boost::property_tree::ptree call_node;
  boost::property_tree::ptree freq_list_node;
  boost::property_tree::ptree source_list_node;
  call_node.put("id", boost::lexical_cast<std::string>(this->get_sys_num()) + "_" + boost::lexical_cast<std::string>(this->get_talkgroup()) + "_" + boost::lexical_cast<std::string>(this->get_start_time()));
  call_node.put("callNum", this->get_call_num());
  call_node.put("freq", this->get_freq());
  call_node.put("sysNum", this->get_sys_num());
  call_node.put("shortName", this->get_short_name());
  call_node.put("talkgroup", this->get_talkgroup());
  call_node.put("talkgrouptag", this->get_talkgroup_tag());
  call_node.put("elapsed", this->elapsed());
  if (get_state() == RECORDING)
    call_node.put("length", this->get_current_length());
  else
    call_node.put("length", this->get_final_length());
  call_node.put("state", this->get_state());
  call_node.put("monState", this->get_monitoring_state());
  call_node.put("phase2", this->get_phase2_tdma());
  call_node.put("conventional", this->is_conventional());
  call_node.put("encrypted", this->get_encrypted());
  call_node.put("emergency", this->get_emergency());
  call_node.put("priority", this->get_priority());
  call_node.put("mode", this->get_mode());
  call_node.put("duplex", this->get_duplex());
  call_node.put("startTime", this->get_start_time());
  call_node.put("stopTime", this->get_stop_time());
  call_node.put("srcId", this->get_current_source_id());

  Recorder *recorder = this->get_recorder();

  if (recorder) {
    call_node.put("recNum", recorder->get_num());
    call_node.put("srcNum", recorder->get_source()->get_num());
    call_node.put("recState", recorder->get_state());
    call_node.put("analog", recorder->is_analog());
  }

  return call_node;
}

long Call_impl::call_counter = 0;