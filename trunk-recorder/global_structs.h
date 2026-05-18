#ifndef GLOBAL_STRUCTS_H
#define GLOBAL_STRUCTS_H
#include <cstdint>
#include <ctime>
#include <cctype>
#include <cstdlib>
#include <string>
#include <vector>
#include <json.hpp>

const int DB_UNSET = 999;

// Conventional-channel sub-audible squelch / identification mode. Drives
// what (if any) CTCSS or DCS block sits in the analog_recorder audio path
// and whether a side-chain detector also runs to identify the actual
// on-air tone for end-of-call reporting.
enum ToneMode {
  TONE_OFF    = 0, // no gating, no detection (legacy "Tone=0" channels)
  TONE_CTCSS  = 1, // gate on configured ctcss_hz; DCS detector runs side-chain
  TONE_DCS    = 2, // gate on configured dcs_code/inverted; CTCSS detector runs side-chain
  TONE_SEARCH = 3  // no gate; both detectors run side-chain to identify on-air tone
};

struct Tone_Config {
  ToneMode mode = TONE_OFF;
  double   ctcss_hz = 0.0;   // valid only when mode == TONE_CTCSS
  int      dcs_code = 0;     // valid only when mode == TONE_DCS (octal as decimal int)
  bool     dcs_inverted = false; // valid only when mode == TONE_DCS
};

// Parse a Tone column value into a Tone_Config. Accepted forms:
//   ""  or  "0"  / "0.0"               -> TONE_OFF
//   "S" / "s"                          -> TONE_SEARCH
//   strict /D\d{3}[NI]/i               -> TONE_DCS (3-digit octal, polarity required)
//   any positive numeric (CTCSS Hz)    -> TONE_CTCSS
// Anything else returns TONE_OFF with no warning here — the caller is
// expected to detect TONE_OFF when it was unintended and log.
inline Tone_Config parse_tone_spec(const std::string &raw) {
  Tone_Config tc; // default OFF
  if (raw.empty()) return tc;

  // Trim whitespace
  size_t a = 0;
  size_t b = raw.size();
  while (a < b && std::isspace(static_cast<unsigned char>(raw[a]))) ++a;
  while (b > a && std::isspace(static_cast<unsigned char>(raw[b - 1]))) --b;
  if (a == b) return tc;
  const std::string s = raw.substr(a, b - a);

  // Search mode
  if (s.size() == 1 && (s[0] == 'S' || s[0] == 's')) {
    tc.mode = TONE_SEARCH;
    return tc;
  }

  // DCS: strict "D" + exactly 3 octal digits + 'N'/'n'/'I'/'i'
  if ((s[0] == 'D' || s[0] == 'd') && s.size() == 5) {
    bool ok = true;
    for (int i = 1; i <= 3; ++i) {
      char c = s[i];
      if (c < '0' || c > '7') { ok = false; break; }
    }
    char pol = s[4];
    if (ok && (pol == 'N' || pol == 'n' || pol == 'I' || pol == 'i')) {
      tc.mode = TONE_DCS;
      tc.dcs_code = ((s[1] - '0') * 100) + ((s[2] - '0') * 10) + (s[3] - '0');
      tc.dcs_inverted = (pol == 'I' || pol == 'i');
      return tc;
    }
    // Falls through to OFF below if malformed.
    return tc;
  }

  // CTCSS: any positive number. strtod tolerates ints and floats.
  char *endp = nullptr;
  const double v = std::strtod(s.c_str(), &endp);
  if (endp != s.c_str() && *endp == '\0' && v > 0.0) {
    tc.mode = TONE_CTCSS;
    tc.ctcss_hz = v;
    return tc;
  }

  // Unrecognized — leave as OFF (caller can log if surprising).
  return tc;
}

struct Transmission {
  long source;
  long talkgroup;
  unsigned int slot;
  unsigned int color_code;
  long start_time;
  long stop_time;
  std::int64_t start_time_ms;
  std::int64_t stop_time_ms;
  long sample_count;
  long spike_count;
  long error_count;
  double freq;
  double length;
  std::string filename;
};

struct Config {
  std::string config_file;
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
  bool syslog_friendly;
  std::string log_color;
  int control_message_warn_rate;
  int control_retune_limit;
  bool broadcast_signals;
  bool enable_audio_streaming;
  bool soft_vocoder;
  bool record_uu_v_calls;
  bool archive_files_on_failure;
  int frequency_format;
  std::string filename_format;
};

struct Audio_Postprocess_Config {
  bool enabled = false;

  int highpass_hz = 0;
  int lowpass_hz = 0;

  int bandreject_hz = 0;
  int bandreject_width_hz = 0;

  bool loudnorm = true;
  bool loudnorm_two_pass = true;
  double loudnorm_i = -16.0;
  double loudnorm_tp = -0.1;
  double loudnorm_lra = 11.0;

  std::string ffmpeg_filter = "";

  bool output_raw_audio = false;
};

struct Call_Source {
  long source;
  long time;
  double position;
  bool emergency;
  std::string signal_system;
  std::string tag;
  std::string tag_ota;
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
  std::int64_t start_time_ms;
  std::int64_t stop_time_ms;
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
  std::string audio_bitrate = "32k";
  std::string filename;
  std::string status_filename;
  std::string converted;
  std::string raw_audio_filename;
  int min_transmissions_removed;

  int sys_num;
  std::string short_name;
  std::string upload_script;
  std::string audio_type;

  Audio_Postprocess_Config audio_postprocess;

  int tdma_slot;
  double length;
  std::int64_t call_length_ms;
  bool phase2_tdma;

  std::vector<Call_Source> transmission_source_list;
  std::vector<Call_Error> transmission_error_list;
  std::vector<Transmission> transmission_list;

  // Sub-audible squelch / identification verdict for analog conventional
  // calls. Populated in Call_Concluder::create_call_data() from the
  // analog_recorder; empty / "off" for digital and trunked calls. See
  // ToneMode enum above for the configured-mode strings.
  std::string tone_mode;       // "off" | "ctcss" | "dcs" | "search"
  std::string tone_detected;   // "" | "173.8" | "D023N" | ...
  float       tone_confidence; // 0.0 (no lock) .. 1.0 (fully confident)

  Call_Data_Status status;
  time_t process_call_time;
  int retry_attempt;

  std::vector<int> plugin_retry_list;
  nlohmann::ordered_json call_json;
};

#endif