#include <curl/curl.h>
#include <boost/dll/alias.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>

#include "../../trunk-recorder/plugin_manager/plugin_api.h"
#include "../../trunk-recorder/call_concluder/call_concluder.h"

#include <chrono>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

struct Whisper_Transcribe_System {
  std::string short_name;
  bool enabled = true;
  std::string language;
  std::string prompt;
  bool include_segments = true;

  // Talkgroup filters
  std::vector<boost::regex> tg_allow;
  std::vector<boost::regex> tg_deny;
  std::vector<std::string> tg_allow_raw;
  std::vector<std::string> tg_deny_raw;
};

struct Whisper_Transcribe_Data {
  std::string server;     // ex: http://127.0.0.1:8000/v1/audio/transcriptions
  std::string api_key;    // optional for some local services
  std::string model;      // ex: whisper-1 or any compatible model name
  int timeout_seconds = 300;

  std::vector<Whisper_Transcribe_System> systems;
};

static size_t write_callback(void *contents, size_t size, size_t nmemb, void *userp) {
  ((std::string *)userp)->append((char *)contents, size * nmemb);
  return size * nmemb;
}

class Whisper_Transcribe : public Plugin_Api {
  Whisper_Transcribe_Data data;
  std::string plugin_name;

private:
  static std::string glob_to_regex_str(const std::string& glob) {
    std::string rx;
    rx.reserve(glob.size() * 2);
    rx += "^";

    for (char c : glob) {
      switch (c) {
        case '*': rx += ".*"; break;
        case '?': rx += "."; break;

        case '.': case '+': case '(': case ')': case '^': case '$':
        case '|': case '{': case '}': case '[': case ']': case '\\':
          rx += "\\";
          rx += c;
          break;

        default:
          rx += c;
          break;
      }
    }

    rx += "$";
    return rx;
  }

  static bool match_any(const std::string& value, const std::vector<boost::regex>& patterns) {
    for (const auto& r : patterns) {
      if (boost::regex_match(value, r)) {
        return true;
      }
    }
    return false;
  }

  static bool passes_talkgroup_filter(const Whisper_Transcribe_System* sys, uint32_t talkgroup) {
    if (!sys) return true;

    const std::string tg = std::to_string(talkgroup);

    if (!sys->tg_allow.empty() && !match_any(tg, sys->tg_allow)) {
      return false;
    }

    if (!sys->tg_deny.empty() && match_any(tg, sys->tg_deny)) {
      return false;
    }

    return true;
  }

  static void compile_patterns_from_json(
      const json& parent,
      const char* key,
      std::vector<boost::regex>& out_compiled,
      std::vector<std::string>& out_raw,
      const std::string& log_prefix,
      const std::string& sys_short_name) {
    out_compiled.clear();
    out_raw.clear();

    if (!parent.contains(key)) return;
    const auto& j = parent.at(key);

    if (!j.is_array()) {
      BOOST_LOG_TRIVIAL(error) << log_prefix << sys_short_name
                               << " " << key << " must be an array";
      return;
    }

    for (const auto& v : j) {
      std::string pat;

      if (v.is_string()) {
        pat = v.get<std::string>();
      } else if (v.is_number_unsigned() || v.is_number_integer()) {
        pat = std::to_string(v.get<long long>());
      } else {
        continue;
      }

      boost::algorithm::trim(pat);
      if (pat.empty()) continue;

      try {
        out_raw.push_back(pat);
        out_compiled.emplace_back(glob_to_regex_str(pat));
      } catch (const boost::regex_error& e) {
        BOOST_LOG_TRIVIAL(error) << log_prefix << sys_short_name
                                 << " invalid pattern in " << key
                                 << " value='" << pat << "' : " << e.what();
      }
    }
  }

public:
  Whisper_Transcribe_System *get_system(const std::string &short_name) {
    for (auto &sys : data.systems) {
      if (sys.short_name == short_name) {
        return &sys;
      }
    }
    return nullptr;
  }

  int parse_config(json config_data) override {
    plugin_name = config_data.value("name", "whisper_transcribe");
    std::string log_prefix = "\t[Whisper Transcribe]\t";

    data.server = config_data.value("server", "");
    data.api_key = config_data.value("apiKey", "");
    data.model = config_data.value("model", "whisper-1");
    data.timeout_seconds = config_data.value("timeoutSeconds", 300);

    boost::algorithm::trim(data.server);
    boost::algorithm::trim(data.api_key);
    boost::algorithm::trim(data.model);

    if (data.server.empty()) {
      BOOST_LOG_TRIVIAL(error) << log_prefix << "server is required";
      return 1;
    }

    if (!config_data.contains("systems") || !config_data["systems"].is_array()) {
      BOOST_LOG_TRIVIAL(error) << log_prefix << "systems array is required";
      return 1;
    }

    for (json element : config_data["systems"]) {
      Whisper_Transcribe_System sys;
      sys.short_name = element.value("shortName", "");
      sys.enabled = element.value("enabled", true);
      sys.language = element.value("language", "");
      sys.prompt = element.value("prompt", "");
      sys.include_segments = element.value("includeSegments", true);

      compile_patterns_from_json(
        element, "talkgroupAllow",
        sys.tg_allow, sys.tg_allow_raw,
        log_prefix, sys.short_name
        );

      compile_patterns_from_json(
        element, "talkgroupDeny",
        sys.tg_deny, sys.tg_deny_raw,
        log_prefix, sys.short_name
        );

      if (sys.short_name.empty()) {
        BOOST_LOG_TRIVIAL(error) << log_prefix << "systems entry missing shortName";
        return 1;
      }

      if (sys.enabled) {
        BOOST_LOG_TRIVIAL(info) << log_prefix
                                << "Configured system: " << sys.short_name
                                << "\t includeSegments=" << sys.include_segments;

        if (!sys.tg_allow_raw.empty() || !sys.tg_deny_raw.empty()) {
          BOOST_LOG_TRIVIAL(info) << log_prefix
                                  << "Talkgroup filters for " << sys.short_name
                                  << " allow=" << sys.tg_allow_raw.size()
                                  << " deny=" << sys.tg_deny_raw.size();
        }

        data.systems.push_back(sys);
      }
    }

    if (data.systems.empty()) {
      BOOST_LOG_TRIVIAL(error) << log_prefix << "No enabled systems configured";
      return 1;
    }

    BOOST_LOG_TRIVIAL(info) << log_prefix
                            << "Server: " << data.server
                            << "\t Model: " << data.model;

    return 0;
  }

  int init(Config *config, std::vector<Source *> sources, std::vector<System *> systems) override {
    frequency_format = config->frequency_format;
    std::string log_prefix = "\t[Whisper Transcribe]\t";

    for (auto &cfg_sys : data.systems) {
      bool found = false;

      for (auto *sys : systems) {
        if (sys && sys->get_short_name() == cfg_sys.short_name) {
          found = true;
          break;
        }
      }

      if (!found) {
        BOOST_LOG_TRIVIAL(error) << log_prefix
                                 << "Configured shortName not found in loaded systems: "
                                 << cfg_sys.short_name;
        return 1;
      }

      BOOST_LOG_TRIVIAL(info) << log_prefix
                              << "Validated system: " << cfg_sys.short_name;
    }

    return 0;
  }

  int request_transcript(const Call_Data_t &call_info,
                         const Whisper_Transcribe_System &sys_cfg,
                         nlohmann::ordered_json &plugin_ctx) {
    CURL *curl = curl_easy_init();
    if (!curl) {
      return 1;
    }

    std::string response_buffer;
    char curl_errbuf[CURL_ERROR_SIZE];
    curl_errbuf[0] = '\0';

    const std::string audio_path =
        call_info.compress_wav && !call_info.converted.empty()
            ? call_info.converted
            : call_info.filename;

    curl_mime *mime = curl_mime_init(curl);
    curl_mimepart *part = nullptr;

    part = curl_mime_addpart(mime);
    curl_mime_name(part, "file");
    curl_mime_filedata(part, audio_path.c_str());

    part = curl_mime_addpart(mime);
    curl_mime_name(part, "model");
    curl_mime_data(part, data.model.c_str(), CURL_ZERO_TERMINATED);

    // Ask for detailed JSON if the service supports it
    part = curl_mime_addpart(mime);
    curl_mime_name(part, "response_format");
    curl_mime_data(part, "verbose_json", CURL_ZERO_TERMINATED);

    if (!sys_cfg.language.empty()) {
      part = curl_mime_addpart(mime);
      curl_mime_name(part, "language");
      curl_mime_data(part, sys_cfg.language.c_str(), CURL_ZERO_TERMINATED);
    }

    if (!sys_cfg.prompt.empty()) {
      part = curl_mime_addpart(mime);
      curl_mime_name(part, "prompt");
      curl_mime_data(part, sys_cfg.prompt.c_str(), CURL_ZERO_TERMINATED);
    }

    struct curl_slist *headers = nullptr;
    if (!data.api_key.empty()) {
      std::string auth_header = "Authorization: Bearer " + data.api_key;
      headers = curl_slist_append(headers, auth_header.c_str());
    }

    curl_easy_setopt(curl, CURLOPT_URL, data.server.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_MIMEPOST, mime);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_buffer);
    curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, curl_errbuf);
    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS, 15000L);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, static_cast<long>(data.timeout_seconds) * 1000L);

    CURLcode res = curl_easy_perform(curl);
    long response_code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response_code);

    curl_slist_free_all(headers);
    curl_mime_free(mime);
    curl_easy_cleanup(curl);

    if (res != CURLE_OK || response_code < 200 || response_code >= 300) {
      std::string loghdr = log_header(call_info.short_name, call_info.call_num,
                                      call_info.talkgroup_display, call_info.freq);
      BOOST_LOG_TRIVIAL(error) << loghdr
                               << this->plugin_name
                               << " transcript request failed (HTTP " << response_code << ") "
                               << response_buffer
                               << (curl_errbuf[0] ? std::string(" curl_err=") + curl_errbuf : "");
      return 1;
    }

    nlohmann::json parsed = nlohmann::json::parse(response_buffer, nullptr, false);
    if (parsed.is_discarded()) {
      std::string loghdr = log_header(call_info.short_name, call_info.call_num,
                                      call_info.talkgroup_display, call_info.freq);
      BOOST_LOG_TRIVIAL(error) << loghdr
                               << this->plugin_name
                               << " transcript response was not valid JSON";
      return 1;
    }

    plugin_ctx["transcript"] = parsed.value("text", "");
    plugin_ctx["segments"] = nlohmann::ordered_json::array();
    plugin_ctx["addresses"] = "";

    if (sys_cfg.include_segments && parsed.contains("segments") && parsed["segments"].is_array()) {
      for (const auto &seg : parsed["segments"]) {
        nlohmann::ordered_json seg_json;
        seg_json["id"] = seg.value("id", 0);
        seg_json["start"] = seg.value("start", 0.0);
        seg_json["end"] = seg.value("end", 0.0);
        seg_json["text"] = seg.value("text", "");
        plugin_ctx["segments"].push_back(seg_json);
      }
    }

    return 0;
  }

  int call_end(Call_Data_t &call_info, nlohmann::ordered_json &plugin_ctx) override {
    // Stable output shape for all outcomes
    plugin_ctx["transcript"] = "";
    plugin_ctx["segments"] = nlohmann::ordered_json::array();
    plugin_ctx["process_time_seconds"] = 0.0;
    plugin_ctx["skipped"] = false;
    plugin_ctx["skip_reason"] = "";
    plugin_ctx["success"] = false;
    plugin_ctx["error"] = "";

    Whisper_Transcribe_System *sys = get_system(call_info.short_name);
    if (!sys || !sys->enabled) {
      plugin_ctx["skipped"] = true;
      plugin_ctx["skip_reason"] = "system_not_enabled";
      return 0;
    }

    if (call_info.encrypted) {
      plugin_ctx["skipped"] = true;
      plugin_ctx["skip_reason"] = "encrypted";
      return 0;
    }

    if (!passes_talkgroup_filter(sys, call_info.talkgroup)) {
      std::string loghdr = log_header(call_info.short_name, call_info.call_num,
                                      call_info.talkgroup_display, call_info.freq);

      plugin_ctx["skipped"] = true;
      plugin_ctx["skip_reason"] = "talkgroup_filter";

      BOOST_LOG_TRIVIAL(info) << loghdr
                              << this->plugin_name
                              << " skipped transcription due to talkgroup filter (tg="
                              << call_info.talkgroup << ")";
      return 0;
    }

    auto start = std::chrono::steady_clock::now();

    int rc = request_transcript(call_info, *sys, plugin_ctx);

    auto end = std::chrono::steady_clock::now();
    plugin_ctx["process_time_seconds"] =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000.0;

    if (rc == 0) {
      plugin_ctx["success"] = true;
    } else {
      plugin_ctx["error"] = "transcription_request_failed";
    }

    return rc;
  }

};

BOOST_DLL_ALIAS(
    Whisper_Transcribe::create,
    create_plugin
)