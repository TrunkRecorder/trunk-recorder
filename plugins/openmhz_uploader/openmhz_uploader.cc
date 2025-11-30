#include <curl/curl.h>
#include <iomanip>
#include <time.h>
#include <vector>

#include "../../trunk-recorder/call_concluder/call_concluder.h"
#include "../../trunk-recorder/plugin_manager/plugin_api.h"
#include "../trunk-recorder/gr_blocks/decoder_wrapper.h"
#include <boost/dll/alias.hpp> // for BOOST_DLL_ALIAS
#include <boost/foreach.hpp>
#include <sys/stat.h>

struct Openmhz_System {
  std::string api_key;
  std::string short_name;
  std::string openmhz_sysid;
};

struct Openmhz_Uploader_Data {
  std::vector<Openmhz_System> systems;
  std::string openmhz_server;
};

boost::mutex curl_share_mutex;

class Openmhz_Uploader : public Plugin_Api {
  // float aggr_;
  // my_plugin_aggregator() : aggr_(0) {}
  Openmhz_Uploader_Data data;
  CURLSH *curl_share;
  long curl_dns_ttl;

public:
  Openmhz_System *get_openmhz_system(std::string short_name) {
    for (std::vector<Openmhz_System>::iterator it = data.systems.begin(); it != data.systems.end(); ++it) {
      Openmhz_System sys = *it;
      if (sys.short_name == short_name) {
        return &(*it);
      }
    }
    return NULL;
  }
  static size_t write_callback(void *contents, size_t size, size_t nmemb, void *userp) {
    ((std::string *)userp)->append((char *)contents, size * nmemb);
    return size * nmemb;
  }
  int upload(Call_Data_t call_info) {
    std::string api_key;
    std::string openmhz_sysid;
    Openmhz_System *sys = get_openmhz_system(call_info.short_name);

    if (sys) {
      api_key = sys->api_key;
      openmhz_sysid = sys->openmhz_sysid;
    }

    if (api_key.size() == 0) {
      // BOOST_LOG_TRIVIAL(error) << "[" << call_info.short_name << "]\tTG: " << talkgroup_display << "\t " << std::put_time(std::localtime(&start_time), "%c %Z") << "\tOpenMHz Upload failed, API Key not found in config for shortName";
      return 0;
    }

    std::ostringstream freq;
    std::string freq_string;
    freq << std::fixed << std::setprecision(0);
    freq << call_info.freq;

    std::ostringstream error_count;
    std::string error_count_string;
    error_count << std::fixed << std::setprecision(0);
    error_count << call_info.error_count;

    std::ostringstream spike_count;
    std::string spike_count_string;
    spike_count << std::fixed << std::setprecision(0);
    spike_count << call_info.spike_count;

    std::ostringstream call_length;
    std::string call_length_string;
    call_length << std::fixed << std::setprecision(0);
    call_length << call_info.length;

    std::ostringstream source_list;
    std::string source_list_string;
    source_list << std::fixed << std::setprecision(2);
    source_list << "[";

    std::ostringstream patch_list;
    std::string patch_list_string;
    patch_list << std::fixed << std::setprecision(2);
    patch_list << "[";

    if (call_info.transmission_source_list.size() != 0) {
      for (unsigned long i = 0; i < call_info.transmission_source_list.size(); i++) {
        source_list << "{ \"pos\": " << std::setprecision(2) << call_info.transmission_source_list[i].position << ", \"src\": " << std::setprecision(0) << call_info.transmission_source_list[i].source << " }";

        if (i < (call_info.transmission_source_list.size() - 1)) {
          source_list << ", ";
        } else {
          source_list << "]";
        }
      }
    } else {
      source_list << "]";
    }

    if (call_info.patched_talkgroups.size() > 1) {
      for (unsigned long i = 0; i < call_info.patched_talkgroups.size(); i++) {
        if (i != 0) {
          patch_list << ",";
        }
        patch_list << (int)call_info.patched_talkgroups[i];
      }
      patch_list << "]";
    } else {
      patch_list << "]";
    }

    char formattedTalkgroup[62];
    snprintf(formattedTalkgroup, 61, "%c[%dm%10ld%c[0m", 0x1B, 35, call_info.talkgroup, 0x1B);
    std::string talkgroup_display = boost::lexical_cast<std::string>(formattedTalkgroup);
    /*CURL *curl;*/
    CURLMcode res;
    CURLM *multi_handle;

    int still_running = 0;
    std::string response_buffer;
    freq_string = freq.str();
    error_count_string = error_count.str();
    spike_count_string = spike_count.str();

    source_list_string = source_list.str();
    call_length_string = call_length.str();
    patch_list_string = patch_list.str();

    struct curl_slist *headerlist = NULL;

    /* Fill in the file upload field. This makes libcurl load data from
     the given file name when curl_easy_perform() is called. */

    CURL *curl = curl_easy_init();
    curl_mime *mime;
    curl_mimepart *part;

    mime = curl_mime_init(curl);
    part = curl_mime_addpart(mime);

    curl_mime_filedata(part, call_info.converted);
    curl_mime_type(part, "application/octet-stream"); /* content-type for this part */
    curl_mime_name(part, "call");

    part = curl_mime_addpart(mime);
    curl_mime_data(part, freq_string.c_str(), CURL_ZERO_TERMINATED);
    curl_mime_name(part, "freq");

    part = curl_mime_addpart(mime);
    curl_mime_data(part, error_count_string.c_str(), CURL_ZERO_TERMINATED);
    curl_mime_name(part, "error_count");

    part = curl_mime_addpart(mime);
    curl_mime_data(part, spike_count_string.c_str(), CURL_ZERO_TERMINATED);
    curl_mime_name(part, "spike_count");

    part = curl_mime_addpart(mime);
    curl_mime_data(part, boost::lexical_cast<std::string>(call_info.start_time).c_str(), CURL_ZERO_TERMINATED);
    curl_mime_name(part, "start_time");

    part = curl_mime_addpart(mime);
    curl_mime_data(part, boost::lexical_cast<std::string>(call_info.stop_time).c_str(), CURL_ZERO_TERMINATED);
    curl_mime_name(part, "stop_time");

    part = curl_mime_addpart(mime);
    curl_mime_data(part, call_length_string.c_str(), CURL_ZERO_TERMINATED);
    curl_mime_name(part, "call_length");

    part = curl_mime_addpart(mime);
    curl_mime_data(part, boost::lexical_cast<std::string>(call_info.talkgroup).c_str(), CURL_ZERO_TERMINATED);
    curl_mime_name(part, "talkgroup_num");

    part = curl_mime_addpart(mime);
    curl_mime_data(part, boost::lexical_cast<std::string>(call_info.emergency).c_str(), CURL_ZERO_TERMINATED);
    curl_mime_name(part, "emergency");

    part = curl_mime_addpart(mime);
    curl_mime_data(part, api_key.c_str(), CURL_ZERO_TERMINATED);
    curl_mime_name(part, "api_key");

    part = curl_mime_addpart(mime);
    curl_mime_data(part, patch_list_string.c_str(), CURL_ZERO_TERMINATED);
    curl_mime_name(part, "patch_list");

    part = curl_mime_addpart(mime);
    curl_mime_data(part, source_list_string.c_str(), CURL_ZERO_TERMINATED);
    curl_mime_name(part, "source_list");

    multi_handle = curl_multi_init();

    /* initialize custom header list (stating that Expect: 100-continue is not wanted */
    headerlist = curl_slist_append(headerlist, "Expect:");
    if (curl && multi_handle) {
      std::string url = data.openmhz_server + "/" + openmhz_sysid + "/upload";

      /* what URL that receives this POST */
      curl_easy_setopt(curl, CURLOPT_URL, url.c_str());

      curl_easy_setopt(curl, CURLOPT_USERAGENT, "TrunkRecorder1.0");
      curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headerlist);
      curl_easy_setopt(curl, CURLOPT_MIMEPOST, mime);

      curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
      curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_buffer);

      curl_easy_setopt(curl, CURLOPT_SHARE, curl_share); 
      curl_easy_setopt(curl, CURLOPT_DNS_CACHE_TIMEOUT, curl_dns_ttl);

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

      long response_code;
      curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response_code);

      res = curl_multi_cleanup(multi_handle);

      /* always cleanup */
      curl_easy_cleanup(curl);

      /* then cleanup the mime */
      curl_mime_free(mime);

      /* free slist */
      curl_slist_free_all(headerlist);

      if (res == CURLM_OK && response_code == 200) {
        struct stat file_info;
        stat(call_info.converted, &file_info);
        std::string loghdr = log_header(call_info.short_name,call_info.call_num,call_info.talkgroup_display,call_info.freq);
        BOOST_LOG_TRIVIAL(info) << loghdr << "OpenMHz Upload Success - file size: " << file_info.st_size;
        ;
        return 0;
      }
    }
    std::string loghdr = log_header(call_info.short_name,call_info.call_num,call_info.talkgroup_display,call_info.freq);

    // Configuration errors - skip the retry queue
    static const struct { const char* match; const char* message; bool is_error; } config_errors[] = {
      {"API Keys do not match",    "Error: Invalid API Key", true},
      {"ShortName does not exist", "Error: Invalid System Name", true},
      {"Error, invalid filename",  "Error: Invalid Filename", true},
      {"Talkgroup does not exist", "Skipped: System Ignoring Unknown Talkgroups", false}
    };
    
    for (const auto& error : config_errors) {
      if (response_buffer.find(error.match) != std::string::npos) {
        if (error.is_error) {
          BOOST_LOG_TRIVIAL(error) << loghdr << "OpenMHz Upload " << error.message;
        } else {
          BOOST_LOG_TRIVIAL(info) << loghdr << "OpenMHz Upload " << error.message;
        }
        return 0;
      }
    }
    
    // Default error - add to the retry queue
    BOOST_LOG_TRIVIAL(error) << loghdr << "OpenMHz Upload Error: " << response_buffer;
    return 1;
  }

  int call_end(Call_Data_t call_info) {
    return upload(call_info);
  }

  int parse_config(json config_data) {
    std::string log_prefix = "\t[OpenMHz]\t";

    // Tests to see if the uploadServer value exists in the config file
    bool upload_server_exists = config_data.contains("uploadServer");
    if (!upload_server_exists) {
      return 1;
    }

    this->data.openmhz_server = config_data.value("uploadServer", "");
    BOOST_LOG_TRIVIAL(info) << log_prefix << "OpenMHz Server: " << this->data.openmhz_server;

    // from: http://www.zedwood.com/article/cpp-boost-url-regex
    boost::regex api_regex("(.*)(.{2}$)");
    boost::regex ex("(http|https)://([^/ :]+):?([^/ ]*)(/?[^ #?]*)\\x3f?([^ #]*)#?([^ ]*)");
    boost::cmatch what;

    if (!regex_match(this->data.openmhz_server.c_str(), what, ex)) {
      BOOST_LOG_TRIVIAL(error) << log_prefix << "Unable to parse Server URL\n";
      return 1;
    }
    // Gets the API key for each system, if defined
    for (json element : config_data["systems"]) {
      bool openmhz_exists = element.contains("apiKey");
      if (openmhz_exists) {
        Openmhz_System sys;
        sys.api_key = element.value("apiKey", "");
        sys.short_name = element.value("shortName", "");
        sys.openmhz_sysid = element.value("openmhzSystemId", sys.short_name);
        regex_match(sys.api_key.c_str(), what, api_regex);
        std::string redacted_api(what[2].first, what[2].second);
        BOOST_LOG_TRIVIAL(info) << log_prefix << "Uploading calls for: " << sys.short_name << "\t OpenMHz System: " << sys.openmhz_sysid << "\t API Key: ******" << redacted_api;
        this->data.systems.push_back(sys);
      }
    }

    if (this->data.systems.size() == 0) {
      BOOST_LOG_TRIVIAL(error) << log_prefix << "OpenMHz Server set, but no Systems are configured\n";
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
  static boost::shared_ptr<Openmhz_Uploader> create() {
    return boost::shared_ptr<Openmhz_Uploader>(
        new Openmhz_Uploader());
  }
};

BOOST_DLL_ALIAS(
    Openmhz_Uploader::create, // <-- this function is exported with...
    create_plugin             // <-- ...this alias name
)
