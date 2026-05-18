#ifndef CALL_CONVENTIONAL_H
#define CALL_CONVENTIONAL_H
#include "global_structs.h"
class System;
class Recorder;
class Talkgroup;

#include "call.h"
#include "call_impl.h"
#include <string>
#include <vector>

class Call_conventional : public Call_impl {
public:
  Call_conventional(long t, double f, System *s, Config c, double squelch_db, bool signal_detection);
  Call_conventional(long t, double f, System *s, Config c, double squelch_db, bool signal_detection, const Tone_Config &tone_config);
  time_t get_start_time();
  bool is_conventional() { return true; }
  void restart_call();
  void set_recorder(Recorder *r);
  void recording_started();
  double get_squelch_db();
  bool get_signal_detection();
  const Tone_Config &get_tone_config() const { return tone_config; }

  // Multi-row freq-group support. When the channel CSV has multiple rows at
  // the same frequency, setup_systems creates ONE Call_conventional /
  // analog_recorder for the group (squelch + signal-detect from row 0) and
  // attaches the full row list here. At call-conclude time, the detected
  // tone is matched against each row's tone_config; the matched row's
  // metadata (TG number, alpha tag, description, etc.) overrides the
  // call's defaults. If no row matches and no catch-all (Tone=0 / Tone=S)
  // is present, the call is marked SKIPPED. Empty for single-row channels.
  void set_alternate_channels(const std::vector<Talkgroup *> &tgs) { alternate_channels = tgs; }
  const std::vector<Talkgroup *> &get_alternate_channels() const { return alternate_channels; }
  bool has_alternate_channels() const { return alternate_channels.size() > 1; }

  // Match a detected-tone string (e.g. "118.8", "D023N", "D565N/D703I", or
  // empty) against the alternate_channels list. Returns the matched
  // Talkgroup* or nullptr if no row matches and no catch-all is present.
  // Catch-all priority: an exact-tone row wins over a Tone=0 / Tone=S row.
  Talkgroup *find_matching_channel(const std::string &detected_tone) const;

private:
  double squelch_db;
  bool signal_detection;
  Tone_Config tone_config; // TONE_OFF when channel was created without a Tone column
  std::vector<Talkgroup *> alternate_channels;
};

#endif
