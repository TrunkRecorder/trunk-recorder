
#include "call_conventional.h"
#include "formatter.h"
#include "recorders/recorder.h"
#include <boost/algorithm/string.hpp>
#include <chrono>

Call_conventional::Call_conventional(long t, double f, System *s, Config c, double squelch_db, bool signal_detection) : Call_impl(t, f, s, c) {
  this->squelch_db = squelch_db;
  this->signal_detection = signal_detection;
  BOOST_LOG_TRIVIAL(info) << "[" << sys->get_short_name() << "]\tFreq: " << format_freq(f) << "\tSquelch: " << squelch_db << " dB\tSignal Detection: " << signal_detection;
}

void Call_conventional::restart_call() {
  call_num = call_counter++;
  idle_count = 0;
  signal = DB_UNSET;
  noise = DB_UNSET;
  curr_src_id = -1;

  auto now = std::chrono::system_clock::now();
  start_time    = std::chrono::system_clock::to_time_t(now);
  start_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
  stop_time     = start_time;
  stop_time_ms  = start_time_ms;

  last_update = time(NULL);
  state = RECORDING;
  debug_recording = false;
  phase2_tdma = false;
  tdma_slot = 0;
  encrypted = false;
  emergency = false;
  this->update_talkgroup_display();
  recorder->start(this);
}

// derive start from stop − final_length:
time_t Call_conventional::get_start_time() {
  // Fixes https://github.com/robotastic/trunk-recorder/issues/103#issuecomment-284825841
  start_time = stop_time - final_length;
  // keep ms in sync (rounded):
  start_time_ms = stop_time_ms - static_cast<std::int64_t>(std::llround(final_length * 1000.0));
  return start_time;
}

void Call_conventional::set_recorder(Recorder *r) {
  recorder = r;
  BOOST_LOG_TRIVIAL(info) << "[" << sys->get_short_name() << "]\tTG: " << this->get_talkgroup_display() << "\tFreq: " << format_freq(this->get_freq());
}

void Call_conventional::recording_started() {
  auto now = std::chrono::system_clock::now();
  start_time    = std::chrono::system_clock::to_time_t(now);
  start_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
}

double Call_conventional::get_squelch_db() {
  return squelch_db;
}

bool Call_conventional::get_signal_detection() {
  return signal_detection;
}