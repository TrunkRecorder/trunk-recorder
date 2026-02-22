#include <curl/curl.h>
#include <time.h>
#include <vector>

#include "../../trunk-recorder/call_concluder/call_concluder.h"
#include "../../trunk-recorder/plugin_manager/plugin_api.h"
#include "../trunk-recorder/gr_blocks/decoder_wrapper.h"
#include <boost/dll/alias.hpp> // for BOOST_DLL_ALIAS
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>
#include <sys/stat.h>

struct Broadcastify_System_Key {
  std::string api_key;
  int system_id;
  std::string short_name;

  // Talkgroup filters (compiled patterns)
  // allow: if set, talkgroup MUST match at least one pattern
  // deny:  if set, talkgroup MUST NOT match any pattern
  std::vector<boost::regex> tg_allow;
  std::vector<boost::regex> tg_deny;
  std::vector<std::string> tg_allow_raw; // for logging/debug
  std::vector<std::string> tg_deny_raw;  // for logging/debug
};

struct Broadcastify_Uploader_Data {
  std::vector<Broadcastify_System_Key> keys;
  std::string bcfy_calls_server;
  bool ssl_verify_disable;
};

boost::mutex curl_share_mutex;

class Broadcastify_Uploader : public Plugin_Api {
  // float aggr_;
  // my_plugin_aggregator() : aggr_(0) {}
  Broadcastify_Uploader_Data data;
  CURLSH *curl_share;
  long curl_dns_ttl;
  std::string plugin_name;

private:
  static std::string glob_to_regex_str(const std::string& glob) {
    // Convert glob (* and ?) to a fully-anchored regex: ^...$
    std::string rx;
    rx.reserve(glob.size() * 2);
    rx += "^";

    for (char c : glob) {
      switch (c) {
        case '*': rx += ".*"; break;
        case '?': rx += ".";  break;

        // escape regex metacharacters
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
      if (boost::regex_match(value, r)) return true;
    }
    return false;
  }

  static bool passes_talkgroup_filter(const Broadcastify_System_Key* sys, uint32_t talkgroup) {
    if (!sys) return true; // no system config => don't filter here
    const std::string tg = std::to_string(talkgroup);

    // If allow list is set, MUST match it
    if (!sys->tg_allow.empty() && !match_any(tg, sys->tg_allow)) {
      return false;
    }

    // If deny list is set, MUST NOT match it
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
    const std::string& sys_short_name
  ) {
    out_compiled.clear();
    out_raw.clear();

    if (!parent.contains(key)) return;
    const auto& j = parent.at(key);

    if (!j.is_array()) {
      BOOST_LOG_TRIVIAL(error) << log_prefix << sys_short_name
                               << " " << key << " must be an array (e.g. [\"507*\", \"12345\"])";
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

  static bool compile_patterns_try_keys(
    const json& parent,
    const std::vector<const char*>& keys,
    std::vector<boost::regex>& out_compiled,
    std::vector<std::string>& out_raw,
    const std::string& log_prefix,
    const std::string& sys_short_name
  ) {
    for (const char* k : keys) {
      if (parent.contains(k)) {
        compile_patterns_from_json(parent, k, out_compiled, out_raw, log_prefix, sys_short_name);
        return true; // stop on first key present (even if empty array)
      }
    }
    // No key present -> leave as empty
    out_compiled.clear();
    out_raw.clear();
    return false;
  }

public:
  Broadcastify_Uploader() : curl_share(NULL), curl_dns_ttl(300) {}

  Broadcastify_System_Key *get_system(std::string short_name) {
    for (std::vector<Broadcastify_System_Key>::iterator it = data.keys.begin(); it != data.keys.end(); ++it) {
      if (it->short_name == short_name) {
        return &(*it);
      }
    }
    return NULL;
  }

  std::string get_api_key(std::string short_name) {
    for (std::vector<Broadcastify_System_Key>::iterator it = data.keys.begin(); it != data.keys.end(); ++it) {
      Broadcastify_System_Key key = *it;
      if (key.short_name == short_name) {
        return key.api_key;
      }
    }
    return "";
  }

  int get_system_id(std::string short_name) {
    for (std::vector<Broadcastify_System_Key>::iterator it = data.keys.begin(); it != data.keys.end(); ++it) {
      Broadcastify_System_Key key = *it;
      if (key.short_name == short_name) {
        return key.system_id; // Convert int to string for return
      }
    }
    return 0;
  }

  static size_t write_callback(void *contents, size_t size, size_t nmemb, void *userp) {
    ((std::string *)userp)->append((char *)contents, size * nmemb);
    return size * nmemb;
  }

  CURLcode upload_audio_file(std::string converted, std::string url) {
    struct stat file_info;

    /* get the file size of the local file */
    stat(converted.c_str(), &file_info);

    FILE *audio = fopen(converted.c_str(), "rb");

    // Make sure we have something to read.
    if (!audio) {
      BOOST_LOG_TRIVIAL(info) << "Error opening file " << converted;
      return CURLE_READ_ERROR;
    }

    CURL *curl;
    CURLcode res = CURLE_FAILED_INIT;

    struct curl_slist *headers = NULL;

    curl = curl_easy_init();
    if (curl) {
      curl_easy_setopt(curl, CURLOPT_USERAGENT, "TrunkRecorder1.0");

      /* enable uploading */
      curl_easy_setopt(curl, CURLOPT_UPLOAD, 1L);

      /* HTTP PUT please */
      //curl_easy_setopt(curl, CURLOPT_PUT, 1L);

      /* specify target URL, and note that this URL should include a file
       name, not only a directory */
      curl_easy_setopt(curl, CURLOPT_URL, url.c_str());

      /* now specify which file to upload */
      curl_easy_setopt(curl, CURLOPT_READDATA, audio);

      /* provide the size of the upload, we specially typecast the value
       to curl_off_t since we must be sure to use the correct data size */
      curl_easy_setopt(curl, CURLOPT_INFILESIZE_LARGE,
                       (curl_off_t)file_info.st_size);

      headers = curl_slist_append(headers, "Content-Type: audio/aac");
      /* Expect: 100-continue is not wanted */
      headers = curl_slist_append(headers, "Expect:");
      /* Transfer-Encoding: chunked is not wanted */
      headers = curl_slist_append(headers, "Transfer-Encoding:");

      curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

      curl_easy_setopt(curl, CURLOPT_SHARE, curl_share);
      curl_easy_setopt(curl, CURLOPT_DNS_CACHE_TIMEOUT, curl_dns_ttl);

      /* Perform the request, res will get the return code */
      res = curl_easy_perform(curl);

      /* always cleanup */
      curl_slist_free_all(headers);
      curl_easy_cleanup(curl);
    }

    fclose(audio);

    return res;
  }

  int upload(Call_Data_t call_info) {

    CURLMcode res;
    CURLM *multi_handle;
    int still_running = 0;
    std::string response_buffer;

    Broadcastify_System_Key *sys = get_system(call_info.short_name);
    if (!sys) {
      return 0;
    }

    if (call_info.encrypted) {
      return 0;
    }

    // Talkgroup allow/deny filtering
    if (!passes_talkgroup_filter(sys, call_info.talkgroup)) {
      std::string loghdr = log_header(call_info.short_name, call_info.call_num,
                                      call_info.talkgroup_display, call_info.freq);
      BOOST_LOG_TRIVIAL(info) << loghdr << "Broadcastify upload skipped due to talkgroup filter (tg="
                              << call_info.talkgroup << ")";
      return 0;
    }

    std::string api_key = sys->api_key;
    int system_id = sys->system_id;

    if ((api_key.size() == 0) || (system_id == 0)) {
      return 0;
    }

    struct curl_slist *headerlist = NULL;

    CURL *curl = curl_easy_init();
    curl_mime *mime;
    curl_mimepart *part;

    mime = curl_mime_init(curl);

    part = curl_mime_addpart(mime);
    curl_mime_data(part, call_info.call_json.dump().c_str(), CURL_ZERO_TERMINATED);
    curl_mime_filename(part, "call_meta.json");
    curl_mime_type(part, "application/json");
    curl_mime_name(part, "metadata");

    part = curl_mime_addpart(mime);
    curl_mime_data(part, std::to_string(call_info.length).c_str(), CURL_ZERO_TERMINATED);
    curl_mime_name(part, "callDuration");

    part = curl_mime_addpart(mime);
    curl_mime_data(part, std::to_string(system_id).c_str(), CURL_ZERO_TERMINATED);
    curl_mime_name(part, "systemId");

    part = curl_mime_addpart(mime);
    curl_mime_data(part, api_key.c_str(), CURL_ZERO_TERMINATED);
    curl_mime_name(part, "apiKey");

    multi_handle = curl_multi_init();

    /* initialize custom header list (stating that Expect: 100-continue is not wanted */
    headerlist = curl_slist_append(headerlist, "Expect:");
    if (curl && multi_handle) {
      /* what URL that receives this POST */
      curl_easy_setopt(curl, CURLOPT_URL, data.bcfy_calls_server.c_str());

      curl_easy_setopt(curl, CURLOPT_USERAGENT, "TrunkRecorder1.0");
      curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headerlist);
      curl_easy_setopt(curl, CURLOPT_MIMEPOST, mime);

      curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
      curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_buffer);

      curl_easy_setopt(curl, CURLOPT_SHARE, curl_share);
      curl_easy_setopt(curl, CURLOPT_DNS_CACHE_TIMEOUT, curl_dns_ttl);

      // broadcastify seems to make a habit out of letting their ssl certs expire
      if (this->data.ssl_verify_disable) {
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L);
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0L);
      }

      curl_multi_add_handle(multi_handle, curl);

      curl_multi_perform(multi_handle, &still_running);

      while (still_running) {
        struct timeval timeout;
        int rc;       /* select() return code */
        CURLMcode mc; /* curl_multi_fdset() return code */

        fd_set fdread;
        fd_set fdwrite;
        fd_set fdexcep;
        int maxfd = -1;

        long curl_timeo = -1;

        FD_ZERO(&fdread);
        FD_ZERO(&fdwrite);
        FD_ZERO(&fdexcep);

        /* set a suitable timeout to play around with */
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;

        curl_multi_timeout(multi_handle, &curl_timeo);
        if (curl_timeo >= 0) {
          timeout.tv_sec = curl_timeo / 1000;
          if (timeout.tv_sec > 1)
            timeout.tv_sec = 1;
          else
            timeout.tv_usec = (curl_timeo % 1000) * 1000;
        }

        /* get file descriptors from the transfers */
        mc = curl_multi_fdset(multi_handle, &fdread, &fdwrite, &fdexcep, &maxfd);

        if (mc != CURLM_OK) {
          fprintf(stderr, "curl_multi_fdset() failed, code %d.\n", mc);
          break;
        }

        /* On success the value of maxfd is guaranteed to be >= -1. We call
         select(maxfd + 1, ...); specially in case of (maxfd == -1) there are
         no fds ready yet so we call select(0, ...) --or Sleep() on Windows--
         to sleep 100ms, which is the minimum suggested value in the
         curl_multi_fdset() doc. */

        if (maxfd == -1) {
          /* Portable sleep for platforms other than Windows. */
          struct timeval wait = {0, 100 * 1000}; /* 100ms */
          rc = select(0, NULL, NULL, NULL, &wait);
        } else {
          /* Note that on some platforms 'timeout' may be modified by select().
           If you need access to the original value save a copy beforehand. */
          rc = select(maxfd + 1, &fdread, &fdwrite, &fdexcep, &timeout);
        }

        switch (rc) {
        case -1:
          /* select error */
          break;
        case 0:
        default:
          /* timeout or readable/writable sockets */
          curl_multi_perform(multi_handle, &still_running);
          break;
        }
      }

      // Grab HTTP status BEFORE cleaning up easy handle (important!)
      long response_code = 0;
      curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response_code);

      res = curl_multi_cleanup(multi_handle);

      /* always cleanup */
      curl_easy_cleanup(curl);

      /* then cleanup the mime */
      curl_mime_free(mime);

      /* free slist */
      curl_slist_free_all(headerlist);

      std::string loghdr = log_header(call_info.short_name,call_info.call_num,call_info.talkgroup_display,call_info.freq);

      if (res != CURLM_OK || response_code != 200) {
        BOOST_LOG_TRIVIAL(error) << loghdr << this->plugin_name << " Metadata Upload Error: " << response_buffer;
        return 1;
      }

      std::size_t spacepos = response_buffer.find(' ');
      if (spacepos < 1) {
        BOOST_LOG_TRIVIAL(error) << loghdr << response_buffer;
        return 1;
      }

      std::string code = response_buffer.substr(0, spacepos);
      std::string message = response_buffer.substr(spacepos + 1);

      if (code == "1" && (message.rfind("SKIPPED", 0) == 0)) {
        BOOST_LOG_TRIVIAL(info) << loghdr << this->plugin_name << " Upload Skipped: " << message;
        return 0;
      }

      if (code == "1" && (message.rfind("REJECTED", 0) == 0)) {
        BOOST_LOG_TRIVIAL(error) << loghdr << this->plugin_name << " Upload REJECTED: " << message;
        return 0;
      }

      if (code != "0") {
        BOOST_LOG_TRIVIAL(error) << loghdr << this->plugin_name << " Metadata Upload Error: " << message;
        return 1;
      }

      CURLcode audio_error = this->upload_audio_file(call_info.converted, message);

      if (audio_error) {
        BOOST_LOG_TRIVIAL(error) << loghdr << this->plugin_name << " Audio Upload Error: " << curl_easy_strerror(audio_error);
        return 1;
      }

      struct stat file_info;
      stat(call_info.converted.c_str(), &file_info);

      BOOST_LOG_TRIVIAL(info) << loghdr << this->plugin_name << " Upload Success - file size: " << file_info.st_size;
      return 0;
    } else {
      return 1;
    }
  }

  int call_end(Call_Data_t call_info) {
    return upload(call_info);
  }

  int parse_config(json config_data) {
    // Extract plugin name from config, with fallback for default internal plugin name
    std::string config_name = config_data.value("name", "broadcastify_uploader");
    this->plugin_name = (config_name == "broadcastify_uploader") ? "Broadcastify" : config_name;
    std::string log_prefix = "\t[Broadcastify]\t";

    // Tests to see if the uploadServer value exists in the config file
    bool upload_server_exists = config_data.contains("broadcastifyCallsServer");
    if (!upload_server_exists) {
      return 1;
    }

    this->data.bcfy_calls_server = config_data.value("broadcastifyCallsServer", "");
    BOOST_LOG_TRIVIAL(info) << log_prefix << "Broadcastify Server: " << this->data.bcfy_calls_server;

    // from: http://www.zedwood.com/article/cpp-boost-url-regex
    boost::regex api_regex("(.*)(.{2}$)");
    boost::regex ex("(http|https)://([^/ :]+):?([^/ ]*)(/?[^ #?]*)\\x3f?([^ #]*)#?([^ ]*)");
    boost::cmatch what;

    if (!regex_match(this->data.bcfy_calls_server.c_str(), what, ex)) {
      BOOST_LOG_TRIVIAL(info) << log_prefix << "Unable to parse Server URL\n";
      return 1;
    }

    this->data.ssl_verify_disable = config_data.value("broadcastifySslVerifyDisable", false);
    if (this->data.ssl_verify_disable) {
      BOOST_LOG_TRIVIAL(info) << log_prefix << "Broadcastify SSL Verify Disabled";
    }

    for (json element : config_data["systems"]) {
      bool broadcastify_exists = element.contains("broadcastifyApiKey");
      if (broadcastify_exists) {
        Broadcastify_System_Key key;
        key.api_key = element.value("broadcastifyApiKey", "");
        key.system_id = element.value("broadcastifySystemId", 0);
        key.short_name = element.value("shortName", "");

        // Talkgroup filters per system (Optional)
        // New (preferred):
        //   broadcastifyAllow: ["507*", "12?45"]
        //   broadcastifyDeny:  ["99*",  "12345"]
        //
        // Also accepts legacy keys for compatibility:
        //   broadcastifyWhitelist / broadcastifyBlacklist
        //   talkgroupWhitelist / talkgroupBlacklist
        compile_patterns_try_keys(
          element,
          {"broadcastifyAllow", "broadcastifyWhitelist", "talkgroupWhitelist"},
          key.tg_allow, key.tg_allow_raw,
          log_prefix, key.short_name
        );
        compile_patterns_try_keys(
          element,
          {"broadcastifyDeny", "broadcastifyBlacklist", "talkgroupBlacklist"},
          key.tg_deny, key.tg_deny_raw,
          log_prefix, key.short_name
        );

        if (!key.tg_allow_raw.empty() || !key.tg_deny_raw.empty()) {
          BOOST_LOG_TRIVIAL(info) << log_prefix << "Talkgroup filters for " << key.short_name
                                  << " allow=" << key.tg_allow_raw.size()
                                  << " deny=" << key.tg_deny_raw.size();
        }

        regex_match(key.api_key.c_str(), what, api_regex);
        std::string redacted_api(what[2].first, what[2].second);
        BOOST_LOG_TRIVIAL(info) << log_prefix << "Uploading calls for: " << key.short_name
                                << "\t Broadcastify System: " << key.system_id
                                << "\t API Key: ******" << redacted_api;
        this->data.keys.push_back(key);
      }
    }

    if (this->data.keys.size() == 0) {
      BOOST_LOG_TRIVIAL(error) << log_prefix << "Broadcastify Server set, but no Systems are configured\n";
      return 1;
    }

    // Initialize shared curl cache to reduce DNS lookups (5 min TTL)
    curl_share = curl_share_init();
    curl_share_setopt(curl_share, CURLSHOPT_SHARE, CURL_LOCK_DATA_DNS);
    curl_share_setopt(curl_share, CURLSHOPT_LOCKFUNC, curl_lock_cb);
    curl_share_setopt(curl_share, CURLSHOPT_UNLOCKFUNC, curl_unlock_cb);
    curl_dns_ttl = 300;

    return 0;
  }

  // curl callbacks
  static void curl_lock_cb(CURL *handle, curl_lock_data data, curl_lock_access access, void *userptr) {
    curl_share_mutex.lock();
  }

  static void curl_unlock_cb(CURL *handle, curl_lock_data data, curl_lock_access access, void *userptr) {
    curl_share_mutex.unlock();
  }

  /*
 int init(Config *config, std::vector<Source *> sources, std::vector<System *> systems) { return 0; }
   int start() { return 0; }
   int stop() { return 0; }
   int poll_one() { return 0; }
   int signal(long unitId, const char *signaling_type, gr::blocks::SignalType sig_type, Call *call, System *system, Recorder *recorder) { return 0; }
   int audio_stream(Recorder *recorder, float *samples, int sampleCount) { return 0; }
   int call_start(Call *call) { return 0; }
   int calls_active(std::vector<Call *> calls) { return 0; }
   int setup_recorder(Recorder *recorder) { return 0; }
   int setup_system(System *system) { return 0; }
   int setup_systems(std::vector<System *> systems) { return 0; }
   int setup_sources(std::vector<Source *> sources) { return 0; }
   int setup_config(std::vector<Source *> sources, std::vector<System *> systems) { return 0; }
   int system_rates(std::vector<System *> systems, float timeDiff) { return 0; }
*/
  // Factory method
  static boost::shared_ptr<Broadcastify_Uploader> create() {
    return boost::shared_ptr<Broadcastify_Uploader>(
        new Broadcastify_Uploader());
  }
};

BOOST_DLL_ALIAS(
    Broadcastify_Uploader::create, // <-- this function is exported with...
    create_plugin                  // <-- ...this alias name
)
