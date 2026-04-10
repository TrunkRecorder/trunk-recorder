#include "call_concluder.h"
#include "../plugin_manager/plugin_manager.h"

#include <boost/filesystem.hpp>
#include <filesystem>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <random>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <cerrno>
#include <cctype>

namespace fs = std::filesystem;

// ---------------------------------------------------------------------------
// Call_Concluder static storage
// ---------------------------------------------------------------------------
const int Call_Concluder::MAX_RETRY = 2;
std::list<std::future<Call_Data_t>> Call_Concluder::call_data_workers = {};
std::list<Call_Data_t> Call_Concluder::retry_call_list = {};

// ---------------------------------------------------------------------------
// String utilities
// ---------------------------------------------------------------------------

// Static constant avoids rebuilding the std::string on every call.
static const char WHITESPACE[] = " \t\r\n";

static std::string trim_whitespace(const std::string &value) {
  const std::size_t start = value.find_first_not_of(WHITESPACE);
  if (start == std::string::npos) return "";
  return value.substr(start, value.find_last_not_of(WHITESPACE) - start + 1);
}

static std::string lowercase_copy(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

static std::string shell_escape(const std::string &input) {
  std::string out = "'";
  for (char c : input) {
    if (c == '\'') out += "'\\''";
    else           out += c;
  }
  out += "'";
  return out;
}

// Replace filesystem-unsafe characters in a token value with underscores.
static std::string sanitize_token(const std::string &str) {
  std::string result;
  result.reserve(str.size());
  for (char c : str) {
    switch (c) {
    case '/': case '\\': case ':': case '*':
    case '?': case '"':  case '<': case '>': case '|':
      result += '_'; break;
    default:
      result += c;
    }
  }
  return result;
}

static std::string escape_ffmpeg_concat_path(const std::string &input) {
  std::string out;
  for (char c : input) {
    if (c == '\'' || c == '\\') out += '\\';
    out += c;
  }
  return out;
}

// ---------------------------------------------------------------------------
// BUG FIX: thread-safe, properly-seeded RNG.
// rand() is unseeded (same sequence every run) and not thread-safe.
// ---------------------------------------------------------------------------
static int random_jitter(int max_exclusive) {
  static std::mt19937 rng(std::random_device{}());
  static std::mutex   rng_mutex;
  std::lock_guard<std::mutex> lock(rng_mutex);
  return std::uniform_int_distribution<int>(0, max_exclusive - 1)(rng);
}

// ---------------------------------------------------------------------------
// Process execution helpers
// ---------------------------------------------------------------------------

// Shared argv-builder — eliminates the duplicated pointer-cast loop that
// previously appeared verbatim in every fork/exec call site.
static std::vector<char *> make_argv(const std::vector<std::string> &args) {
  std::vector<char *> argv;
  argv.reserve(args.size() + 1);
  for (const auto &a : args) argv.push_back(const_cast<char *>(a.c_str()));
  argv.push_back(nullptr);
  return argv;
}

static std::string render_command_for_logging(const std::vector<std::string> &args) {
  std::ostringstream out;
  for (std::size_t i = 0; i < args.size(); ++i) {
    if (i > 0) out << ' ';
    out << shell_escape(args[i]);
  }
  return out.str();
}

// BUG FIX: replaced popen()/system() throughout with fork()/execvp().
// system() and popen() pass the command through /bin/sh, exposing filenames
// and filter strings to shell injection. execvp() passes args directly.
static int run_process_wait(const std::vector<std::string> &args,
                            const std::string &loghdr,
                            const std::string &friendly_name) {
  if (args.empty()) {
    BOOST_LOG_TRIVIAL(error) << loghdr << "\033[0;31mCannot execute empty command for "
                              << friendly_name << "\033[0m";
    return -1;
  }

  BOOST_LOG_TRIVIAL(trace) << loghdr << "Running " << friendly_name;
  BOOST_LOG_TRIVIAL(trace) << loghdr << "Command: " << render_command_for_logging(args);

  auto argv = make_argv(args);

  const pid_t pid = fork();
  if (pid < 0) {
    BOOST_LOG_TRIVIAL(error) << loghdr << "\033[0;31mFailed to fork for "
                              << friendly_name << ": " << std::strerror(errno) << "\033[0m";
    return -1;
  }
  if (pid == 0) {
    execvp(argv[0], argv.data());
    std::fprintf(stderr, "execvp failed for %s '%s': %s\n",
                 friendly_name.c_str(), argv[0], std::strerror(errno));
    _exit(127);
  }

  int status = 0;
  if (waitpid(pid, &status, 0) < 0) {
    BOOST_LOG_TRIVIAL(error) << loghdr << "\033[0;31mwaitpid failed for "
                              << friendly_name << ": " << std::strerror(errno) << "\033[0m";
    return -1;
  }
  if (WIFEXITED(status))   return WEXITSTATUS(status);
  if (WIFSIGNALED(status)) {
    BOOST_LOG_TRIVIAL(error) << loghdr << "\033[0;31m" << friendly_name
                              << " terminated by signal " << WTERMSIG(status) << "\033[0m";
    return 128 + WTERMSIG(status);
  }
  BOOST_LOG_TRIVIAL(error) << loghdr << "\033[0;31m" << friendly_name
                            << " ended in an unknown state\033[0m";
  return -1;
}

static bool run_process_capture_combined_output(const std::vector<std::string> &args,
                                                const std::string &loghdr,
                                                const std::string &friendly_name,
                                                std::string &output,
                                                int &exit_code) {
  output.clear();
  exit_code = -1;

  if (args.empty()) {
    BOOST_LOG_TRIVIAL(error) << loghdr << "\033[0;31mCannot execute empty command for "
                              << friendly_name << "\033[0m";
    return false;
  }

  BOOST_LOG_TRIVIAL(trace) << loghdr << "Running " << friendly_name;
  BOOST_LOG_TRIVIAL(trace) << loghdr << "Command: " << render_command_for_logging(args);

  int pipefd[2];
  if (pipe(pipefd) < 0) {
    BOOST_LOG_TRIVIAL(error) << loghdr << "\033[0;31mFailed to create pipe for "
                              << friendly_name << ": " << std::strerror(errno) << "\033[0m";
    return false;
  }

  auto argv = make_argv(args);

  const pid_t pid = fork();
  if (pid < 0) {
    close(pipefd[0]); close(pipefd[1]);
    BOOST_LOG_TRIVIAL(error) << loghdr << "\033[0;31mFailed to fork for "
                              << friendly_name << ": " << std::strerror(errno) << "\033[0m";
    return false;
  }
  if (pid == 0) {
    close(pipefd[0]);
    if (dup2(pipefd[1], STDOUT_FILENO) < 0 || dup2(pipefd[1], STDERR_FILENO) < 0) {
      std::fprintf(stderr, "dup2 failed for %s: %s\n", friendly_name.c_str(), std::strerror(errno));
      _exit(127);
    }
    close(pipefd[1]);
    execvp(argv[0], argv.data());
    std::fprintf(stderr, "execvp failed for %s '%s': %s\n",
                 friendly_name.c_str(), argv[0], std::strerror(errno));
    _exit(127);
  }

  close(pipefd[1]);
  char buf[4096];
  ssize_t nread;
  while ((nread = read(pipefd[0], buf, sizeof(buf))) > 0)
    output.append(buf, static_cast<std::size_t>(nread));
  close(pipefd[0]);

  int status = 0;
  if (waitpid(pid, &status, 0) < 0) {
    BOOST_LOG_TRIVIAL(error) << loghdr << "\033[0;31mwaitpid failed for "
                              << friendly_name << ": " << std::strerror(errno) << "\033[0m";
    return false;
  }
  if (WIFEXITED(status)) { exit_code = WEXITSTATUS(status); return true; }
  if (WIFSIGNALED(status)) {
    exit_code = 128 + WTERMSIG(status);
    BOOST_LOG_TRIVIAL(error) << loghdr << "\033[0;31m" << friendly_name
                              << " terminated by signal " << WTERMSIG(status) << "\033[0m";
    return true;
  }
  BOOST_LOG_TRIVIAL(error) << loghdr << "\033[0;31m" << friendly_name
                            << " ended in an unknown state\033[0m";
  return false;
}

// ---------------------------------------------------------------------------
// Filename format expansion helpers
// ---------------------------------------------------------------------------

// Format a time string using strftime, plus a custom %f specifier for ms.
static std::string format_time_custom(const std::string &fmt, const struct tm *tm_val, int ms = 0) {
  if (!tm_val || fmt.empty()) return "";

  std::string processed;
  processed.reserve(fmt.size() + 8);
  for (size_t i = 0; i < fmt.size(); ++i) {
    if (fmt[i] == '%' && i + 1 < fmt.size() && fmt[i + 1] == 'f') {
      char ms_buf[4];
      snprintf(ms_buf, sizeof(ms_buf), "%03d", std::clamp(ms, 0, 999));
      processed += ms_buf;
      ++i;
    } else {
      processed += fmt[i];
    }
  }

  for (size_t buf_size = std::max<size_t>(64, processed.size() * 2);
       buf_size <= 65536; buf_size *= 2) {
    std::string out(buf_size, '\0');
    const size_t written = strftime(out.data(), out.size(), processed.c_str(), tm_val);
    if (written > 0) { out.resize(written); return out; }
  }

  BOOST_LOG_TRIVIAL(warning) << "\033[0;33mFilename time format output exceeded 64KiB.\033[0m";
  return "";
}

// BUG FIX: use localtime_r / gmtime_r (POSIX re-entrant) instead of
// localtime / gmtime, which return a pointer to a shared static buffer and
// are not thread-safe. upload_call_worker runs concurrently.
static std::string expand_filename_format(const std::string &format,
                                          const Call_Data_t &call_info,
                                          time_t start_time) {
  std::string result;
  result.reserve(format.size() * 2);

  for (size_t i = 0; i < format.size(); ) {
    if (format[i] != '{') { result += format[i++]; continue; }

    const size_t end = format.find('}', i);
    if (end == std::string::npos) { result += format[i++]; continue; }

    // string_view avoids a heap allocation for every {token} dispatched.
    const std::string_view token(format.data() + i + 1, end - i - 1);
    i = end + 1;

    if      (token == "talkgroup")             result += std::to_string(call_info.talkgroup);
    else if (token == "talkgroup_tag")         result += sanitize_token(call_info.talkgroup_tag);
    else if (token == "talkgroup_alpha_tag")   result += sanitize_token(call_info.talkgroup_alpha_tag);
    else if (token == "talkgroup_description") result += sanitize_token(call_info.talkgroup_description);
    else if (token == "talkgroup_group")       result += sanitize_token(call_info.talkgroup_group);
    else if (token == "talkgroup_display")     result += sanitize_token(call_info.talkgroup_display);
    else if (token == "short_name")            result += sanitize_token(call_info.short_name);
    else if (token == "freq") {
      char buf[32]; snprintf(buf, sizeof(buf), "%.0f", call_info.freq); result += buf;
    } else if (token == "freq_mhz") {
      char buf[32]; snprintf(buf, sizeof(buf), "%.4f", call_info.freq / 1e6); result += buf;
    }
    else if (token == "call_num")     result += std::to_string(call_info.call_num);
    else if (token == "tdma_slot") {
      if (call_info.tdma_slot != -1) result += std::to_string(call_info.tdma_slot);
    }
    else if (token == "sys_num")      result += std::to_string(call_info.sys_num);
    else if (token == "epoch")        result += std::to_string(static_cast<long>(start_time));
    else if (token == "source_num")   result += std::to_string(call_info.source_num);
    else if (token == "recorder_num") result += std::to_string(call_info.recorder_num);
    else if (token == "audio_type")   result += sanitize_token(call_info.audio_type);
    else if (token == "emergency")    result += (call_info.emergency  ? "1" : "0");
    else if (token == "encrypted")    result += (call_info.encrypted  ? "1" : "0");
    else if (token == "priority")     result += std::to_string(call_info.priority);
    else if (token == "signal")       result += std::to_string(static_cast<int>(call_info.signal));
    else if (token == "noise")        result += std::to_string(static_cast<int>(call_info.noise));
    else if (token == "color_code")   result += std::to_string(call_info.color_code);
    else if (token.size() > 5 && token.substr(0, 5) == "time:") {
      const std::string fmt(token.substr(5));
      struct tm ltm {}; localtime_r(&start_time, &ltm);
      if      (fmt == "iso")    result += format_time_custom("%Y-%m-%dT%H:%M:%S",    &ltm);
      else if (fmt == "iso_ms") result += format_time_custom("%Y-%m-%dT%H:%M:%S.%f", &ltm);
      else                      result += format_time_custom(fmt,                     &ltm);
    } else if (token.size() > 6 && token.substr(0, 6) == "ztime:") {
      const std::string fmt(token.substr(6));
      struct tm gtm {}; gmtime_r(&start_time, &gtm);
      if      (fmt == "iso")    result += format_time_custom("%Y-%m-%dT%H:%M:%SZ",    &gtm);
      else if (fmt == "iso_ms") result += format_time_custom("%Y-%m-%dT%H:%M:%S.%fZ", &gtm);
      else                      result += format_time_custom(fmt,                      &gtm);
    } else {
      result += '{'; result += token; result += '}';
      BOOST_LOG_TRIVIAL(warning) << "\033[0;33mUnknown filename format token: {"
                                  << token << "}\033[0m";
    }
  }

  return result;
}

// ---------------------------------------------------------------------------
// Audio post-processing helpers
// ---------------------------------------------------------------------------

static std::string build_cleanup_filter(const Audio_Postprocess_Config &cfg) {
  if (!cfg.enabled) return "";

  const std::string override = trim_whitespace(cfg.ffmpeg_filter);
  if (!override.empty()) return override;

  std::ostringstream oss;
  bool first = true;
  auto emit = [&](auto... parts) {
    if (!first) oss << ',';
    first = false;
    (oss << ... << parts);
  };

  if (cfg.highpass_hz > 0)
    emit("highpass=f=", cfg.highpass_hz);
  if (cfg.bandreject_hz > 0 && cfg.bandreject_width_hz > 0)
    emit("bandreject=f=", cfg.bandreject_hz, ":w=", cfg.bandreject_width_hz);
  if (cfg.lowpass_hz > 0)
    emit("lowpass=f=", cfg.lowpass_hz);

  return oss.str();
}

static bool is_invalid_loudnorm_value(const std::string &v) {
  const char *p = v.data(), *e = p + v.size();
  while (p < e && std::isspace(static_cast<unsigned char>(*p)))    ++p;
  while (e > p && std::isspace(static_cast<unsigned char>(e[-1]))) --e;
  if (p == e) return true;

  const std::ptrdiff_t len = e - p;
  if (len > 4) return false;   // longest invalid token is 4 chars

  char buf[5];
  for (std::ptrdiff_t j = 0; j < len; ++j)
    buf[j] = static_cast<char>(std::tolower(static_cast<unsigned char>(p[j])));
  buf[len] = '\0';

  const std::string_view sv(buf, static_cast<std::size_t>(len));
  return sv == "-inf" || sv == "inf"  || sv == "+inf" ||
         sv == "nan"  || sv == "+nan" || sv == "-nan";
}

static bool override_filter_contains_loudnorm(const Audio_Postprocess_Config &cfg) {
  const std::string f = trim_whitespace(cfg.ffmpeg_filter);
  return !f.empty() && lowercase_copy(f).find("loudnorm") != std::string::npos;
}

static bool should_apply_structured_loudnorm(const Audio_Postprocess_Config &cfg) {
  if (!cfg.loudnorm) return false;

  if (override_filter_contains_loudnorm(cfg)) {
    BOOST_LOG_TRIVIAL(warning)
    << "\033[0;33maudio_postprocess.ffmpeg_filter already contains loudnorm; "
    << "built-in loudnorm settings will be skipped to avoid duplicate normalization.\033[0m";
    return false;
  }

  return true;
}

static void append_ffmpeg_output_args(std::vector<std::string> &args,
                                      bool compressed,
                                      const std::string &audio_bitrate) {
  if (compressed) {
    const std::string bitrate =
        trim_whitespace(audio_bitrate).empty() ? "32k" : trim_whitespace(audio_bitrate);

    args.insert(args.end(), {"-c:a", "aac", "-ar", "16000", "-ac", "1",
                             "-b:a", bitrate, "-movflags", "+faststart"});
  } else {
    args.insert(args.end(), {"-c:a", "pcm_s16le", "-ar", "16000", "-ac", "1"});
  }
}

struct LoudnormMeasured {
  std::string input_i, input_tp, input_lra, input_thresh, target_offset;
  bool valid = false;
};

static std::string build_loudnorm_analysis_filter(const Audio_Postprocess_Config &cfg) {
  std::ostringstream f;
  f << std::fixed << std::setprecision(1)
    << "loudnorm=I=" << cfg.loudnorm_i
    << ":TP="        << cfg.loudnorm_tp
    << ":LRA="       << cfg.loudnorm_lra
    << ":print_format=json";
  return f.str();
}

static bool analyze_loudnorm_from_concat(const Call_Data_t &call_info,
                                         const std::string &list_filename,
                                         const std::string &cleanup_filter,
                                         LoudnormMeasured &measured) {
  const std::string loghdr =
      log_header(call_info.short_name, call_info.call_num, call_info.talkgroup_display, call_info.freq);

  const std::string analysis_filter = build_loudnorm_analysis_filter(call_info.audio_postprocess);
  const std::string full_filter =
      cleanup_filter.empty() ? analysis_filter : cleanup_filter + "," + analysis_filter;

  const std::vector<std::string> args = {
      "ffmpeg", "-y", "-hide_banner", "-nostats",
      "-loglevel", "info",
      "-f", "concat", "-safe", "0", "-i", list_filename,
      "-af", full_filter, "-vn", "-f", "null", "-"
  };

  std::string output;
  int exit_code = -1;
  if (!run_process_capture_combined_output(args, loghdr, "ffmpeg loudnorm analysis", output, exit_code)) {
    BOOST_LOG_TRIVIAL(error) << loghdr << "\033[0;31mFailed to start ffmpeg loudnorm analysis pass\033[0m";
    return false;
  }
  if (exit_code != 0)
    BOOST_LOG_TRIVIAL(warning) << loghdr
        << "\033[0;33mffmpeg loudnorm first pass returned non-zero exit status: " << exit_code << "\033[0m";

  // Extract the last complete JSON object from ffmpeg's combined output.
  const std::size_t json_end   = output.rfind('}');
  const std::size_t json_start = (json_end != std::string::npos)
                                     ? output.rfind('{', json_end) : std::string::npos;

  if (json_end == std::string::npos || json_start == std::string::npos || json_start >= json_end) {
    BOOST_LOG_TRIVIAL(error) << loghdr
    << "\033[0;31mFailed to parse loudnorm first-pass JSON: no valid JSON object found; "
    << "two-pass loudnorm cannot be used for this call\033[0m";
    return false;
  }

  try {
    // Parse directly from existing buffer — no substr copy.
    const nlohmann::json stats =
        nlohmann::json::parse(output.data() + json_start, output.data() + json_end + 1);

    auto json_str = [](const nlohmann::json &v) -> std::string {
      if (v.is_string())          return v.get<std::string>();
      if (v.is_number_float())    { std::ostringstream o; o << v.get<double>(); return o.str(); }
      if (v.is_number_integer())  return std::to_string(v.get<long long>());
      if (v.is_number_unsigned()) return std::to_string(v.get<unsigned long long>());
      return v.dump();
    };

    measured.input_i       = json_str(stats.at("input_i"));
    measured.input_tp      = json_str(stats.at("input_tp"));
    measured.input_lra     = json_str(stats.at("input_lra"));
    measured.input_thresh  = json_str(stats.at("input_thresh"));
    measured.target_offset = json_str(stats.at("target_offset"));

    if (is_invalid_loudnorm_value(measured.input_i)      ||
        is_invalid_loudnorm_value(measured.input_tp)     ||
        is_invalid_loudnorm_value(measured.input_lra)    ||
        is_invalid_loudnorm_value(measured.input_thresh) ||
        is_invalid_loudnorm_value(measured.target_offset)) {
      BOOST_LOG_TRIVIAL(warning) << loghdr
      << "\033[0;33mLoudnorm first-pass returned unusable values "
      << "(input_i=" << measured.input_i << ", input_tp=" << measured.input_tp
      << ", input_lra=" << measured.input_lra << ", input_thresh=" << measured.input_thresh
      << ", target_offset=" << measured.target_offset
      << "); two-pass loudnorm is unavailable for this call and rendering will fall back to single-pass loudnorm\033[0m";

      return false;
    }

    measured.valid = true;
    return true;
  } catch (const std::exception &e) {
    BOOST_LOG_TRIVIAL(error) << loghdr
    << "\033[0;31mFailed to decode loudnorm first-pass JSON: " << e.what()
    << "; two-pass loudnorm cannot be used for this call\033[0m";
    return false;
  }
}

static std::string build_loudnorm_render_filter(const Audio_Postprocess_Config &cfg,
                                                const LoudnormMeasured &m) {
  std::ostringstream f;
  f << std::fixed << std::setprecision(1)
    << "loudnorm=I="       << cfg.loudnorm_i
    << ":TP="              << cfg.loudnorm_tp
    << ":LRA="             << cfg.loudnorm_lra
    << ":measured_I="      << m.input_i
    << ":measured_TP="     << m.input_tp
    << ":measured_LRA="    << m.input_lra
    << ":measured_thresh=" << m.input_thresh
    << ":offset="          << m.target_offset
    << ":linear=true:dual_mono=true";
  return f.str();
}

static std::string build_loudnorm_single_pass_filter(const Audio_Postprocess_Config &cfg) {
  std::ostringstream f;
  f << std::fixed << std::setprecision(1)
    << "loudnorm=I=" << cfg.loudnorm_i
    << ":TP="        << cfg.loudnorm_tp
    << ":LRA="       << cfg.loudnorm_lra;
  return f.str();
}

static bool write_concat_list(const std::vector<std::string> &input_files,
                              const std::string &list_filename) {
  std::ofstream list_file(list_filename);
  if (!list_file.is_open()) {
    BOOST_LOG_TRIVIAL(error) << "\033[0;31mCall uploader: Unable to create ffmpeg concat list: "
                              << list_filename << "\033[0m";
    return false;
  }
  for (const auto &f : input_files)
    list_file << "file '" << escape_ffmpeg_concat_path(fs::path(f).filename().string()) << "'\n";

  list_file.flush();
  if (!list_file.good()) {
    BOOST_LOG_TRIVIAL(error) << "\033[0;31mCall uploader: Failed to write ffmpeg concat list: "
                              << list_filename << " (disk full?)\033[0m";
    return false;
  }
  return true;
}

static void append_common_metadata_args(std::vector<std::string> &args,
                                        const std::string &date,
                                        const std::string &short_name,
                                        const std::string &talkgroup) {
  args.reserve(args.size() + 6);
  args.insert(args.end(), {
      "-metadata", "date="   + date,
      "-metadata", "artist=" + short_name,
      "-metadata", "title="  + talkgroup
  });
}

static int render_call_audio_artifacts(const Call_Data_t &call_info,
                                       const std::vector<std::string> &input_files,
                                       const std::string &date,
                                       const std::string &short_name,
                                       const std::string &talkgroup) {
  if (input_files.empty()) {
    BOOST_LOG_TRIVIAL(error) << "\033[0;31mCall uploader: No input files for render_call_audio_artifacts\033[0m";
    return -1;
  }

  const fs::path transmission_dir = fs::path(input_files[0]).parent_path();
  const std::string list_filename = (transmission_dir /
      (fs::path(call_info.filename).filename().string() + ".concat.txt")).string();
  if (!write_concat_list(input_files, list_filename)) return -1;

  const std::string loghdr =
      log_header(call_info.short_name, call_info.call_num, call_info.talkgroup_display, call_info.freq);

  const std::string cleanup_filter = build_cleanup_filter(call_info.audio_postprocess);
  const bool do_compress           = call_info.compress_wav;

  const bool loudnorm_requested = should_apply_structured_loudnorm(call_info.audio_postprocess);
  bool apply_loudnorm_two_pass = false;
  bool apply_loudnorm_single_pass = false;

  const bool too_short_for_two_pass = (call_info.length > 0.0 && call_info.length < 1.5);

  LoudnormMeasured measured;
  if (loudnorm_requested) {
    if (call_info.audio_postprocess.loudnorm_two_pass) {
      if (too_short_for_two_pass) {
        BOOST_LOG_TRIVIAL(debug) << loghdr
            << "Call too short for reliable loudnorm first pass (" << call_info.length
            << "s); falling back to single-pass loudnorm";
        apply_loudnorm_single_pass = true;
      } else if (analyze_loudnorm_from_concat(call_info, list_filename, cleanup_filter, measured) && measured.valid) {
        apply_loudnorm_two_pass = true;
        BOOST_LOG_TRIVIAL(debug) << loghdr
        << "Two-pass loudnorm analysis succeeded; using two-pass loudnorm rendering";
      } else {
        BOOST_LOG_TRIVIAL(warning) << loghdr
        << "\033[0;33mTwo-pass loudnorm analysis was not usable for this call; "
        << "falling back to single-pass loudnorm rendering\033[0m";
        apply_loudnorm_single_pass = true;
      }

    } else {
      BOOST_LOG_TRIVIAL(debug) << loghdr
      << "audio_postprocess.loudnorm_two_pass is disabled; using single-pass loudnorm rendering";
      apply_loudnorm_single_pass = true;
    }
  }

  std::string final_filter = cleanup_filter;
  if (apply_loudnorm_two_pass) {
    if (!final_filter.empty()) final_filter += ',';
    final_filter += build_loudnorm_render_filter(call_info.audio_postprocess, measured);
  } else if (apply_loudnorm_single_pass) {
    if (!final_filter.empty()) final_filter += ',';
    final_filter += build_loudnorm_single_pass_filter(call_info.audio_postprocess);
  }

  // Pre-reserve: compressed path ~36 args, uncompressed ~22.
  auto run_render = [&](const std::string &filter) -> int {
    std::vector<std::string> args;
    args.reserve(do_compress ? 36 : 22);
    args.insert(args.end(), {
        "ffmpeg", "-y", "-hide_banner", "-loglevel", "error",
        "-f", "concat", "-safe", "0", "-i", list_filename, "-vn"
    });

    if (do_compress) {
      const std::string split = filter.empty()
          ? "[0:a]asplit=2[awav][aaac]"
          : "[0:a]" + filter + ",asplit=2[awav][aaac]";
      args.insert(args.end(), {"-filter_complex", split, "-map", "[awav]"});
      append_common_metadata_args(args, date, short_name, talkgroup);
      append_ffmpeg_output_args(args, false, call_info.audio_bitrate);
      args.push_back(call_info.filename);

      args.insert(args.end(), {"-map", "[aaac]"});
      append_common_metadata_args(args, date, short_name, talkgroup);
      append_ffmpeg_output_args(args, true, call_info.audio_bitrate);
      args.push_back(call_info.converted);
    } else {
      if (!filter.empty()) args.insert(args.end(), {"-af", filter});
      append_common_metadata_args(args, date, short_name, talkgroup);
      append_ffmpeg_output_args(args, false, call_info.audio_bitrate);
      args.push_back(call_info.filename);
    }

    return run_process_wait(args, loghdr, "ffmpeg render");
  };

  int rc = run_render(final_filter);

  if (rc != 0 && final_filter != cleanup_filter) {
    BOOST_LOG_TRIVIAL(warning) << loghdr
        << "\033[0;33mLoudnorm render failed; retrying with cleanup-only filtering\033[0m";
    rc = run_render(cleanup_filter);
  }

  if (rc != 0 && !cleanup_filter.empty()) {
    BOOST_LOG_TRIVIAL(warning) << loghdr
        << "\033[0;33mCleanup-filter render failed; falling back to unfiltered rendering\033[0m";
    rc = run_render("");
  }

  std::remove(list_filename.c_str());

  if (rc != 0) {
    BOOST_LOG_TRIVIAL(error) << loghdr
        << "\033[0;31mFailed to render call audio artifacts. Make sure ffmpeg is installed.\033[0m";
    return -1;
  }
  return 0;
}

// ---------------------------------------------------------------------------
// Upload script helpers
// ---------------------------------------------------------------------------

static bool parse_command_arguments(const std::string &command,
                                    std::vector<std::string> &args,
                                    std::string &error) {
  args.clear(); error.clear();

  std::string current;
  bool in_single = false, in_double = false, escaping = false;

  for (char c : command) {
    if (escaping) { current.push_back(c); escaping = false; continue; }
    if (c == '\\' && !in_single) { escaping = true;         continue; }
    if (c == '\'' && !in_double) { in_single = !in_single;  continue; }
    if (c == '"'  && !in_single) { in_double = !in_double;  continue; }
    if (std::isspace(static_cast<unsigned char>(c)) && !in_single && !in_double) {
      if (!current.empty()) { args.push_back(std::move(current)); current.clear(); }
      continue;
    }
    current.push_back(c);
  }

  if (escaping)            { error = "uploadScript ends with a trailing backslash"; return false; }
  if (in_single||in_double){ error = "uploadScript contains unmatched quotes";      return false; }
  if (!current.empty())    args.push_back(std::move(current));
  if (args.empty())        { error = "uploadScript is empty after parsing";         return false; }
  return true;
}

static int run_upload_script_argv(const Call_Data_t &call_info) {
  const std::string script_spec = trim_whitespace(call_info.upload_script);
  if (script_spec.empty()) return 0;

  const std::string loghdr =
      log_header(call_info.short_name, call_info.call_num, call_info.talkgroup_display, call_info.freq);

  std::vector<std::string> args;
  std::string parse_error;
  if (!parse_command_arguments(script_spec, args, parse_error)) {
    BOOST_LOG_TRIVIAL(error) << loghdr << "\033[0;31mInvalid uploadScript: "
                              << parse_error << "\033[0m";
    return -1;
  }

  args.push_back(call_info.filename);
  args.push_back(call_info.status_filename);
  args.push_back(call_info.converted);

  BOOST_LOG_TRIVIAL(info) << loghdr << "\033[0m\tRunning upload script";

  const int rc = run_process_wait(args, loghdr, "upload script");
  if (rc != 0)
    BOOST_LOG_TRIVIAL(error) << loghdr << "\033[0;31mUpload script failed with status "
                              << rc << "\033[0m";
  return rc;
}

// ---------------------------------------------------------------------------
// Call JSON / file lifecycle helpers
// ---------------------------------------------------------------------------

int create_call_json(Call_Data_t &call_info) {
  nlohmann::ordered_json json_data = {
      {"freq",                  int(call_info.freq)},
      {"freq_error",            int(call_info.freq_error)},
      {"signal",                int(call_info.signal)},
      {"noise",                 int(call_info.noise)},
      {"source_num",            int(call_info.source_num)},
      {"recorder_num",          int(call_info.recorder_num)},
      {"tdma_slot",             int(call_info.tdma_slot)},
      {"phase2_tdma",           int(call_info.phase2_tdma)},
      {"start_time",            call_info.start_time},
      {"stop_time",             call_info.stop_time},
      {"start_time_ms",         call_info.start_time_ms},
      {"stop_time_ms",          call_info.stop_time_ms},
      {"emergency",             int(call_info.emergency)},
      {"priority",              call_info.priority},
      {"mode",                  int(call_info.mode)},
      {"duplex",                int(call_info.duplex)},
      {"encrypted",             int(call_info.encrypted)},
      {"call_length",           int(std::round(call_info.length))},
      {"call_length_ms",        call_info.call_length_ms},
      {"talkgroup",             call_info.talkgroup},
      {"talkgroup_tag",         call_info.talkgroup_alpha_tag},
      {"talkgroup_description", call_info.talkgroup_description},
      {"talkgroup_group_tag",   call_info.talkgroup_tag},
      {"talkgroup_group",       call_info.talkgroup_group},
      {"color_code",            call_info.color_code},
      {"audio_type",            call_info.audio_type},
      {"short_name",            call_info.short_name}
  };

  if (call_info.patched_talkgroups.size() > 1) {
    for (auto tgid : call_info.patched_talkgroups)
      json_data["patched_talkgroups"] += int(tgid);
  }

  for (const auto &err : call_info.transmission_error_list) {
    json_data["freqList"] += {
        {"freq",        int(call_info.freq)},
        {"time",        err.time},
        {"pos",         std::round(err.position * 100.0) / 100.0},
        {"len",         err.total_len},
        {"error_count", int(err.error_count)},
        {"spike_count", int(err.spike_count)}
    };
  }

  // BUG FIX: the original srcList loop used transmission_error_list[i] to get
  // position while iterating transmission_source_list — UB if sizes ever diverge
  // (they're built in parallel and should match, but guard defensively).
  const std::size_t src_count = std::min(call_info.transmission_source_list.size(),
                                         call_info.transmission_error_list.size());
  for (std::size_t i = 0; i < src_count; ++i) {
    const auto &src = call_info.transmission_source_list[i];
    const auto &err = call_info.transmission_error_list[i];
    json_data["srcList"] += {
        {"src",           int(src.source)},
        {"time",          src.time},
        {"pos",           std::round(err.position * 100.0) / 100.0},
        {"emergency",     int(src.emergency)},
        {"signal_system", src.signal_system},
        {"tag",           src.tag},
        {"tag_ota",       src.tag_ota}
    };
  }

  // BUG FIX: move into call_json to avoid holding two full JSON copies in memory.
  call_info.call_json = std::move(json_data);

  std::ofstream json_file(call_info.status_filename);
  if (!json_file.is_open()) {
    BOOST_LOG_TRIVIAL(error)
        << log_header(call_info.short_name, call_info.call_num,
                      call_info.talkgroup_display, call_info.freq)
        << "\033[0;31mUnable to create JSON file: " << call_info.status_filename << "\033[0m";
    return 1;
  }
  json_file << call_info.call_json.dump(2);
  return 0;
}

bool checkIfFile(const std::string &filePath) {
  try {
    const boost::filesystem::path p(filePath);
    return boost::filesystem::exists(p) && boost::filesystem::is_regular_file(p);
  } catch (const boost::filesystem::filesystem_error &e) {
    BOOST_LOG_TRIVIAL(error) << "\033[0;31m" << e.what() << "\033[0m";
    return false;
  }
}

// BUG FIX: const reference — Call_Data_t contains vectors and a JSON object;
// the original by-value signature made a full deep copy on every call.
void remove_call_files(const Call_Data_t &call_info, bool plugin_failure) {
  const std::string loghdr =
      log_header(call_info.short_name, call_info.call_num, call_info.talkgroup_display, call_info.freq);

  if (plugin_failure) {
    if (call_info.archive_files_on_failure)
      BOOST_LOG_TRIVIAL(error) << loghdr << "Upload failed after " << call_info.retry_attempt
                                << " attempts - " << Color::GRN << "Archiving files" << Color::RST;
    else
      BOOST_LOG_TRIVIAL(error) << loghdr << "Upload failed after " << call_info.retry_attempt
                                << " attempts - " << Color::RED << "Removing files" << Color::RST;
  }

  const bool should_archive = call_info.audio_archive ||
                               (plugin_failure && call_info.archive_files_on_failure);
  if (should_archive) {
    if (call_info.transmission_archive) {
      for (const auto &t : call_info.transmission_list) {
        if (!checkIfFile(t.filename)) continue;
        const boost::filesystem::path target =
            boost::filesystem::path(fs::path(call_info.filename)
                                        .replace_filename(fs::path(t.filename).filename()));
        try {
          boost::filesystem::copy_file(t.filename, target);
        } catch (const boost::filesystem::filesystem_error &e) {
          BOOST_LOG_TRIVIAL(error) << loghdr << "\033[0;31mFailed to copy transmission file: "
                                   << e.what() << "\033[0m";
        }
      }
    }
    for (const auto &t : call_info.transmission_list)
      if (checkIfFile(t.filename)) std::remove(t.filename.c_str());
  } else {
    for (const std::string &f : {call_info.filename, call_info.converted})
      if (checkIfFile(f)) std::remove(f.c_str());
    for (const auto &t : call_info.transmission_list)
      if (checkIfFile(t.filename)) std::remove(t.filename.c_str());
  }

  const bool keep_json = call_info.call_log || (plugin_failure && call_info.archive_files_on_failure);
  if (!keep_json && checkIfFile(call_info.status_filename))
    std::remove(call_info.status_filename.c_str());
}

// ---------------------------------------------------------------------------
// Worker
// ---------------------------------------------------------------------------

Call_Data_t upload_call_worker(Call_Data_t call_info) {
  if (call_info.status == INITIAL) {
    std::vector<std::string> input_files;
    input_files.reserve(call_info.transmission_list.size());

    struct stat statbuf;
    for (const auto &t : call_info.transmission_list) {
      if (stat(t.filename.c_str(), &statbuf) == 0)
        input_files.push_back(t.filename);
      else
        BOOST_LOG_TRIVIAL(error) << "\033[0;31mSomehow, " << t.filename
                                  << " doesn't exist; skipping for ffmpeg\033[0m";
    }

    if (input_files.empty()) {
      // BUG FIX: clean up transmission files before returning FAILED so they
      // are not left on disk indefinitely (manage_call_data_workers only acted
      // on RETRY, never on FAILED).
      remove_call_files(call_info);
      call_info.status = FAILED;
      return call_info;
    }

    if (create_call_json(call_info) != 0) {
      remove_call_files(call_info);
      call_info.status = FAILED;
      return call_info;
    }

    const std::string talkgroup_title = call_info.talkgroup_alpha_tag.empty()
                                            ? std::to_string(call_info.talkgroup)
                                            : call_info.talkgroup_alpha_tag;

    // BUG FIX: std::ctime() appends '\n' and is not thread-safe (shared static
    // buffer). Multiple concurrent workers would corrupt each other's date strings.
    // Use localtime_r + strftime into a stack buffer instead.
    const time_t start_time = static_cast<time_t>(call_info.start_time);
    struct tm start_tm {};
    localtime_r(&start_time, &start_tm);
    char date_buf[64] = {};
    strftime(date_buf, sizeof(date_buf), "%c", &start_tm);

    if (render_call_audio_artifacts(call_info, input_files, date_buf,
                                    call_info.short_name, talkgroup_title) < 0) {
      remove_call_files(call_info);
      call_info.status = FAILED;
      return call_info;
    }

    if (!trim_whitespace(call_info.upload_script).empty()) {
      if (run_upload_script_argv(call_info) != 0) {
        remove_call_files(call_info);
        call_info.status = FAILED;
        return call_info;
      }
    }
  }

  int error = 0;

  error = plugman_call_end(call_info);

  if (!error) {
    remove_call_files(call_info);
    call_info.status = SUCCESS;
  } else {
    call_info.status = RETRY;
  }

  return call_info;
}

// ---------------------------------------------------------------------------
// Call_Concluder methods
// ---------------------------------------------------------------------------

// BUG FIX: Config taken by const reference throughout — the original passed by
// value, causing up to 3 deep copies of the full config per call conclusion
// (conclude_call → create_call_data → create_base_filename).
Call_Data_t Call_Concluder::create_base_filename(Call *call,
                                                  Call_Data_t call_info,
                                                  System *sys,
                                                  const Config &config) {
  const std::int64_t start_ms        = call->get_start_time_ms();
  const time_t       work_start_time = static_cast<time_t>(start_ms / 1000);
  const std::string  capture_dir     = call->get_capture_dir();

  const std::string filename_format = !sys->get_filename_format().empty()
                                          ? sys->get_filename_format()
                                          : config.filename_format;

  std::string base_filename;

  if (filename_format.empty()) {
    // BUG FIX: localtime_r instead of localtime.
    struct tm ltm {};
    localtime_r(&work_start_time, &ltm);

    const boost::filesystem::path base_path =
        boost::filesystem::path(capture_dir) /
        call->get_short_name() /
        std::to_string(1900 + ltm.tm_year) /
        std::to_string(1 + ltm.tm_mon) /
        std::to_string(ltm.tm_mday);

    boost::filesystem::create_directories(base_path);

    const long long sec   = start_ms / 1000;
    const int       milli = static_cast<int>(start_ms % 1000);

    std::ostringstream ts;
    ts << sec << '.' << std::setw(3) << std::setfill('0') << milli;

    base_filename = base_path.string() + "/" +
                    std::to_string(call->get_talkgroup()) + "-" +
                    ts.str() + "_" +
                    std::to_string(static_cast<long>(std::llround(call->get_freq())));

    if (call->get_tdma_slot() != -1)
      base_filename += "." + std::to_string(call->get_tdma_slot());
  } else {
    const std::string expanded = expand_filename_format(filename_format, call_info, work_start_time);
    base_filename = capture_dir + "/" + expanded;
    boost::filesystem::create_directories(boost::filesystem::path(base_filename).parent_path());
  }

  const std::string stem = base_filename + "-call_" + std::to_string(call->get_call_num());
  call_info.filename        = stem + ".wav";
  call_info.status_filename = stem + ".json";
  call_info.converted       = stem + ".m4a";

  return call_info;
}

Call_Data_t Call_Concluder::create_call_data(Call *call, System *sys, const Config &config) {
  Call_Data_t call_info;

  call_info.status               = INITIAL;
  call_info.process_call_time    = time(nullptr);
  call_info.retry_attempt        = 0;
  call_info.error_count          = 0;
  call_info.spike_count          = 0;
  call_info.freq                 = call->get_freq();
  call_info.freq_error           = call->get_freq_error();
  call_info.signal               = call->get_signal();
  call_info.noise                = call->get_noise();
  call_info.recorder_num         = call->get_recorder()->get_num();
  call_info.source_num           = call->get_recorder()->get_source()->get_num();
  call_info.encrypted            = call->get_encrypted();
  call_info.emergency            = call->get_emergency();
  call_info.priority             = call->get_priority();
  call_info.mode                 = call->get_mode();
  call_info.duplex               = call->get_duplex();
  call_info.tdma_slot            = call->get_tdma_slot();
  call_info.phase2_tdma          = call->get_phase2_tdma();
  call_info.transmission_list    = call->get_transmissions();
  call_info.sys_num              = sys->get_sys_num();
  call_info.short_name           = sys->get_short_name();
  call_info.upload_script        = sys->get_upload_script();
  call_info.audio_archive        = sys->get_audio_archive();
  call_info.transmission_archive = sys->get_transmission_archive();
  call_info.call_log             = sys->get_call_log();
  call_info.call_num             = call->get_call_num();
  call_info.compress_wav         = sys->get_compress_wav();
  call_info.audio_bitrate        = sys->get_audio_bitrate();

  call_info.audio_postprocess.enabled             = sys->get_audio_postprocess_enabled();
  call_info.audio_postprocess.highpass_hz         = sys->get_audio_highpass_hz();
  call_info.audio_postprocess.lowpass_hz          = sys->get_audio_lowpass_hz();
  call_info.audio_postprocess.bandreject_hz       = sys->get_audio_bandreject_hz();
  call_info.audio_postprocess.bandreject_width_hz = sys->get_audio_bandreject_width_hz();
  call_info.audio_postprocess.loudnorm            = sys->get_audio_loudnorm();
  call_info.audio_postprocess.loudnorm_i          = sys->get_audio_loudnorm_i();
  call_info.audio_postprocess.loudnorm_tp         = sys->get_audio_loudnorm_tp();
  call_info.audio_postprocess.loudnorm_lra        = sys->get_audio_loudnorm_lra();
  call_info.audio_postprocess.ffmpeg_filter       = sys->get_audio_ffmpeg_filter();

  call_info.talkgroup                 = call->get_talkgroup();
  call_info.talkgroup_display         = call->get_talkgroup_display();
  call_info.patched_talkgroups        = sys->get_talkgroup_patch(call_info.talkgroup);
  call_info.min_transmissions_removed = 0;
  call_info.color_code                = -1;

  const std::string loghdr =
      log_header(call_info.short_name, call_info.call_num, call_info.talkgroup_display, call_info.freq);

  if (const Talkgroup *tg = sys->find_talkgroup(call->get_talkgroup())) {
    call_info.talkgroup_tag         = tg->tag;
    call_info.talkgroup_alpha_tag   = tg->alpha_tag;
    call_info.talkgroup_description = tg->description;
    call_info.talkgroup_group       = tg->group;
  }
  // else: string members are value-initialized to "".

  if (call->get_is_analog())        call_info.audio_type = "analog";
  else if (call->get_phase2_tdma()) call_info.audio_type = "digital tdma";
  else                              call_info.audio_type = "digital";

  const double min_tx_s = sys->get_min_tx_duration();

  call_info.transmission_source_list.reserve(call_info.transmission_list.size());
  call_info.transmission_error_list.reserve(call_info.transmission_list.size());

  double       playable_pos_s = 0.0;
  std::int64_t audio_sum_ms   = 0;
  bool         have_any       = false;
  std::int64_t min_start_ms   = 0;
  std::int64_t max_stop_ms    = 0;

  for (auto it = call_info.transmission_list.begin(); it != call_info.transmission_list.end(); ) {
    const Transmission &t = *it;
    const std::int64_t seg_ms    = std::max<std::int64_t>(0, t.stop_time_ms - t.start_time_ms);
    const double       seg_len_s = seg_ms / 1000.0;

    if (seg_len_s < min_tx_s) {
      // BUG FIX: original always logged "Removing" and deleted the file even
      // when transmission_archive was true. Both are now gated correctly.
      ++call_info.min_transmissions_removed;
      if (!call_info.transmission_archive) {
        BOOST_LOG_TRIVIAL(info) << loghdr << "Removing transmission shorter than "
                                 << min_tx_s << "s (actual: " << seg_len_s << "s).";
        if (checkIfFile(t.filename)) std::remove(t.filename.c_str());
      }
      it = call_info.transmission_list.erase(it);
      continue;
    }

    if (!have_any) {
      have_any     = true;
      min_start_ms = t.start_time_ms;
      max_stop_ms  = t.stop_time_ms;
    } else {
      min_start_ms = std::min(min_start_ms, t.start_time_ms);
      max_stop_ms  = std::max(max_stop_ms,  t.stop_time_ms);
    }

    const std::string tag = sys->find_unit_tag(t.source);
    const std::string tag_ota = sys->find_unit_tag_ota(t.source);

    {
      std::ostringstream tx;
      tx << loghdr << "- Transmission src: " << t.source;
      if (!tag.empty()) tx << " (\033[0;34m" << tag << "\033[0m)";
      tx << " pos: " << format_time(playable_pos_s) << " length: " << format_time(seg_len_s);
      if (t.error_count < 1)
        BOOST_LOG_TRIVIAL(info) << tx.str();
      else
        BOOST_LOG_TRIVIAL(info) << tx.str() << "\033[0;31m errors: " << t.error_count
                                << " spikes: " << t.spike_count << "\033[0m";
    }

    // BUG FIX: the original assigned call_info.color_code = t.color_code and
    // then immediately checked if they differed — always false. Fixed: set on
    // first valid transmission, warn on any subsequent mismatch.
    if (t.color_code != -1) {
      if (call_info.color_code == -1)
        call_info.color_code = t.color_code;
      else if (call_info.color_code != t.color_code)
        BOOST_LOG_TRIVIAL(warning) << loghdr
            << "Call has multiple Color Codes - previous: " << call_info.color_code
            << " current: " << t.color_code;
    }

    if (call_info.talkgroup != t.talkgroup) {
      BOOST_LOG_TRIVIAL(warning) << loghdr
          << "Transmission has a different Talkgroup than Call - Call: "
          << call_info.talkgroup << " Transmission: " << t.talkgroup;
      call_info.talkgroup = t.talkgroup;
    }

    call_info.transmission_source_list.push_back({t.source, t.start_time, playable_pos_s, false, "", tag, tag_ota});
    call_info.transmission_error_list.push_back( {t.start_time, playable_pos_s, seg_len_s, t.error_count, t.spike_count});

    call_info.error_count += t.error_count;
    call_info.spike_count += t.spike_count;
    playable_pos_s += seg_len_s;
    audio_sum_ms   += seg_ms;
    ++it;
  }

  if (have_any) {
    call_info.start_time_ms  = min_start_ms;
    call_info.stop_time_ms   = max_stop_ms;
    call_info.start_time     = static_cast<time_t>(min_start_ms / 1000);
    call_info.stop_time      = static_cast<time_t>(max_stop_ms  / 1000);
    call_info.call_length_ms = audio_sum_ms;
    call_info.length         = audio_sum_ms / 1000.0;
  } else {
    call_info.length = 0.0;
    call_info.start_time_ms = call_info.stop_time_ms = 0;
    call_info.start_time    = call_info.stop_time    = 0;
    call_info.call_length_ms = 0;
  }

  call_info = create_base_filename(call, call_info, sys, config);
  call_info.archive_files_on_failure = config.archive_files_on_failure;
  return call_info;
}

void Call_Concluder::conclude_call(Call *call, System *sys, const Config &config) {
  Call_Data_t call_info = create_call_data(call, sys, config);
  const std::string loghdr =
      log_header(call_info.short_name, call_info.call_num, call_info.talkgroup_display, call_info.freq);

  if (call->get_state() == MONITORING && call->get_monitoring_state() == SUPERSEDED) {
    BOOST_LOG_TRIVIAL(info) << loghdr << "Call has been superseded. Removing files.";
    remove_call_files(call_info);
    return;
  }

  if (call_info.encrypted) {
    if (!call_info.transmission_list.empty() || call_info.min_transmissions_removed > 0) {
      if (create_call_json(call_info) < 0)
        BOOST_LOG_TRIVIAL(error) << loghdr
            << "\033[0;31mFailed to create metadata JSON for encrypted call\033[0m";
    }
    remove_call_files(call_info);
    return;
  }

  if (call_info.transmission_list.empty()) {
    if (call_info.min_transmissions_removed == 0)
      BOOST_LOG_TRIVIAL(error) << loghdr << "\033[0;31mNo Transmissions were recorded!\033[0m";
    else
      BOOST_LOG_TRIVIAL(info) << loghdr
          << "No Transmissions were recorded! "
          << call_info.min_transmissions_removed << " transmissions less than "
          << sys->get_min_tx_duration() << " seconds were removed.";
    return;
  }

  if (call_info.length <= sys->get_min_duration()) {
    BOOST_LOG_TRIVIAL(info) << loghdr << "Call length: " << call_info.length
                             << " is less than min duration: " << sys->get_min_duration();
    remove_call_files(call_info);
    return;
  }

  call_data_workers.push_back(std::async(std::launch::async, upload_call_worker, call_info));
}

void Call_Concluder::manage_call_data_workers() {
  for (auto it = call_data_workers.begin(); it != call_data_workers.end(); ) {
    if (it->wait_for(std::chrono::seconds(0)) != std::future_status::ready) { ++it; continue; }

    Call_Data_t call_info = it->get();
    it = call_data_workers.erase(it);

    if (call_info.status != RETRY) continue;

    ++call_info.retry_attempt;
    const time_t      start_time = call_info.start_time;
    const std::string loghdr =
        log_header(call_info.short_name, call_info.call_num, call_info.talkgroup_display, call_info.freq);

    if (call_info.retry_attempt > Call_Concluder::MAX_RETRY) {
      remove_call_files(call_info, true);
      BOOST_LOG_TRIVIAL(error) << loghdr << "Failed to conclude call - "
                                << std::put_time(std::localtime(&start_time), "%c %Z");
    } else {
      const long backoff = (1L << call_info.retry_attempt) * 60 + random_jitter(10);
      call_info.process_call_time = time(nullptr) + backoff;
      retry_call_list.push_back(call_info);
      BOOST_LOG_TRIVIAL(error) << loghdr
          << std::put_time(std::localtime(&start_time), "%c %Z")
          << " retry attempt " << call_info.retry_attempt
          << " in " << backoff << "s\t retry queue: " << retry_call_list.size() << " calls";
    }
  }

  for (auto it = retry_call_list.begin(); it != retry_call_list.end(); ) {
    if (it->process_call_time <= time(nullptr)) {
      call_data_workers.push_back(std::async(std::launch::async, upload_call_worker, *it));
      it = retry_call_list.erase(it);
    } else {
      ++it;
    }
  }
}

bool Call_Concluder::shutdown_call_data_workers(std::chrono::seconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;

  while (std::chrono::steady_clock::now() < deadline) {
    for (auto it = call_data_workers.begin(); it != call_data_workers.end(); ) {
      if (it->wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) { ++it; continue; }

      Call_Data_t call_info = it->get();
      it = call_data_workers.erase(it);

      if (call_info.status == RETRY) {
        if (++call_info.retry_attempt > Call_Concluder::MAX_RETRY)
          remove_call_files(call_info, true);
        else
          call_data_workers.push_back(std::async(std::launch::async, upload_call_worker, call_info));
      }
    }

    // During shutdown fire queued retries immediately rather than waiting for backoff.
    for (auto &pending : retry_call_list)
      call_data_workers.push_back(std::async(std::launch::async, upload_call_worker, pending));
    retry_call_list.clear();

    if (call_data_workers.empty()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  for (auto &pending : retry_call_list) remove_call_files(pending, true);
  retry_call_list.clear();

  if (!call_data_workers.empty()) {
    BOOST_LOG_TRIVIAL(error) << "\033[0;31mCall concluder shutdown timed out after "
                              << timeout.count() << "s; force exiting with "
                              << call_data_workers.size() << " worker(s) still running.\033[0m";
    // Intentional leak: splice futures aside so destructors don't block exit.
    auto *abandoned = new std::list<std::future<Call_Data_t>>();
    abandoned->splice(abandoned->end(), call_data_workers);
  }

  return false;
}