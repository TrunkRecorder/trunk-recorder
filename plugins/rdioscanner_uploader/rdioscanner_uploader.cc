#include <curl/curl.h>
#include <iomanip>
#include <time.h>
#include <vector>

#include "../../trunk-recorder/call_concluder/call_concluder.h"
#include "../../trunk-recorder/plugin_manager/plugin_api.h"
#include "../trunk-recorder/gr_blocks/decoder_wrapper.h"
#include <boost/algorithm/string.hpp>
#include <boost/dll/alias.hpp> // for BOOST_DLL_ALIAS
#include <boost/foreach.hpp>
#include <sys/stat.h>

struct Rdio_Scanner_System {
  std::string api_key;
  std::string short_name;
  uint32_t system_id;
  std::string talkgroupsFile;
  Talkgroups *talkgroups;
  bool compress_wav;
};

struct Rdio_Scanner_Uploader_Data {
  std::vector<Rdio_Scanner_System> systems;
  std::string server;
};

boost::mutex curl_share_mutex;

class Rdio_Scanner_Uploader : public Plugin_Api {
  Rdio_Scanner_Uploader_Data data;
  CURLSH *curl_share;
  long curl_dns_ttl;

public:
  Rdio_Scanner_System *get_system(std::string short_name) {
    for (std::vector<Rdio_Scanner_System>::iterator it = data.systems.begin(); it != data.systems.end(); ++it) {
      if (it->short_name == short_name) {
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
    uint32_t system_id;
    std::string talkgroup_group = call_info.talkgroup_group;
    std::string talkgroup_tag = call_info.talkgroup_tag;
    std::string talkgroup_alpha_tag = call_info.talkgroup_alpha_tag;
    std::string talkgroup_description = call_info.talkgroup_description;
    bool compress_wav = false;
    Rdio_Scanner_System *sys = get_system(call_info.short_name);

    if (call_info.encrypted) {
      return 0;
    }

    if (sys) {
      api_key = sys->api_key;
      compress_wav = call_info.compress_wav;
      system_id = sys->system_id;
    }

    if (api_key.size() == 0) {
      return 0;
    }

    std::ostringstream freq;
    std::string freq_string;
    freq << std::fixed << std::setprecision(0);
    freq << call_info.freq;

    std::ostringstream call_length;
    std::string call_length_string;
    call_length << std::fixed << std::setprecision(0);
    call_length << call_info.length;

    std::ostringstream source_list;
    std::string source_list_string;
    source_list << std::fixed << std::setprecision(2);
    source_list << "[";

    std::string unit_list_string;
    std::ostringstream unit_list;
    unit_list << std::fixed << std::setprecision(2);
    unit_list << "[";

    std::ostringstream patch_list;
    std::string patch_list_string;
    patch_list << std::fixed << std::setprecision(2);
    patch_list << "[";

    boost::filesystem::path audioPath(compress_wav ? call_info.converted : call_info.filename);
    boost::filesystem::path audioName = audioPath.filename();

    if (call_info.transmission_source_list.size() != 0) {
      for (unsigned long i = 0; i < call_info.transmission_source_list.size(); i++) {
        source_list << "{ \"pos\": " << std::setprecision(2) << call_info.transmission_source_list[i].position << ", \"src\": " << std::setprecision(0) << call_info.transmission_source_list[i].source;
        unit_list << "{ \"offset\": " << std::setprecision(2) << call_info.transmission_source_list[i].position << ", \"id\": " << std::setprecision(0) << call_info.transmission_source_list[i].source;
        // Include alias tag if available
        if (!call_info.transmission_source_list[i].tag.empty()) {
          source_list << ", \"tag\": \"" << call_info.transmission_source_list[i].tag << "\"";
          unit_list << ", \"label\": \"" << call_info.transmission_source_list[i].tag << "\"";
        }

        source_list << " }";
        unit_list << " }";

        if (i < (call_info.transmission_source_list.size() - 1)) {
          source_list << ", ";
          unit_list << ", ";
        } else {
          source_list << "]";
          unit_list << "]";
        }
      }
    } else {
      source_list << "]";
      unit_list << "]";
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
    
    std::ostringstream freq_list;
    std::string freq_list_string;
    freq_list << std::fixed << std::setprecision(2);
    freq_list << "[";

    if (call_info.transmission_error_list.size() != 0) {
      for (std::size_t i = 0; i < call_info.transmission_error_list.size(); i++) {
        freq_list << "{\"freq\": " << std::fixed << std::setprecision(0) << call_info.freq << ", \"time\": " << call_info.transmission_error_list[i].time << ", \"pos\": " << std::fixed << std::setprecision(2) << call_info.transmission_error_list[i].position << ", \"len\": " << call_info.transmission_error_list[i].total_len << ", \"errorCount\": " << std::setprecision(0) << call_info.transmission_error_list[i].error_count << ", \"spikeCount\": " << call_info.transmission_error_list[i].spike_count << "}";

        if (i < (call_info.transmission_error_list.size() - 1)) {
          freq_list << ", ";
        } else {
          freq_list << "]";
        }
      }
    } else {
      freq_list << "]";
    }

    // BOOST_LOG_TRIVIAL(error) << "Got source list: " << source_list.str();

    CURLMcode res;
    CURLM *multi_handle;
    int still_running = 0;
    std::string response_buffer;
    freq_string = freq.str();

    source_list_string = source_list.str();
    freq_list_string = freq_list.str();
    call_length_string = call_length.str();
    patch_list_string = patch_list.str();
    unit_list_string = unit_list.str();

    struct curl_slist *headerlist = NULL;

    /* Fill in the file upload field. This makes libcurl load data from
     the given file name when curl_easy_perform() is called. */

    CURL *curl = curl_easy_init();
    curl_mime *mime;
    curl_mimepart *part;

    mime = curl_mime_init(curl);
    part = curl_mime_addpart(mime);

    curl_mime_filedata(part, compress_wav ? call_info.converted : call_info.filename);
    curl_mime_type(part, "application/octet-stream"); /* content-type for this part */
    curl_mime_name(part, "audio");

    part = curl_mime_addpart(mime);
    curl_mime_data(part, audioName.c_str(), CURL_ZERO_TERMINATED);
    curl_mime_name(part, "audioName");

    part = curl_mime_addpart(mime);
    curl_mime_data(part, compress_wav ? "audio/mp4" : "audio/wav", CURL_ZERO_TERMINATED);
    curl_mime_name(part, "audioType");

    part = curl_mime_addpart(mime);
    curl_mime_data(part, boost::lexical_cast<std::string>(call_info.start_time).c_str(), CURL_ZERO_TERMINATED);
    curl_mime_name(part, "dateTime");

    part = curl_mime_addpart(mime);
    curl_mime_data(part, freq_list_string.c_str(), CURL_ZERO_TERMINATED);
    curl_mime_name(part, "frequencies");

    part = curl_mime_addpart(mime);
    curl_mime_data(part, freq_string.c_str(), CURL_ZERO_TERMINATED);
    curl_mime_name(part, "frequency");

    part = curl_mime_addpart(mime);
    curl_mime_data(part, api_key.c_str(), CURL_ZERO_TERMINATED);
    curl_mime_name(part, "key");

    part = curl_mime_addpart(mime);
    curl_mime_data(part, patch_list_string.c_str(), CURL_ZERO_TERMINATED);
    curl_mime_name(part, "patches");

    part = curl_mime_addpart(mime);
    curl_mime_data(part, boost::lexical_cast<std::string>(call_info.talkgroup).c_str(), CURL_ZERO_TERMINATED);
    curl_mime_name(part, "talkgroup");

    part = curl_mime_addpart(mime);
    curl_mime_data(part, boost::lexical_cast<std::string>(call_info.talkgroup_group).c_str(), CURL_ZERO_TERMINATED);
    curl_mime_name(part, "talkgroupGroup");

    part = curl_mime_addpart(mime);
    curl_mime_data(part, boost::lexical_cast<std::string>(call_info.talkgroup_alpha_tag).c_str(), CURL_ZERO_TERMINATED);
    curl_mime_name(part, "talkgroupLabel");

    part = curl_mime_addpart(mime);
    curl_mime_data(part, boost::lexical_cast<std::string>(call_info.talkgroup_tag).c_str(), CURL_ZERO_TERMINATED);
    curl_mime_name(part, "talkgroupTag");

    part = curl_mime_addpart(mime);
    curl_mime_data(part, boost::lexical_cast<std::string>(call_info.talkgroup_description).c_str(), CURL_ZERO_TERMINATED);
    curl_mime_name(part, "talkgroupName");

    part = curl_mime_addpart(mime);
    curl_mime_data(part, source_list_string.c_str(), CURL_ZERO_TERMINATED);
    curl_mime_name(part, "sources");
    
    // The "units" upload is included for future testing against the unreleased v7 rdio API
    // part = curl_mime_addpart(mime);
    // curl_mime_data(part, unit_list_string.c_str(), CURL_ZERO_TERMINATED);
    // curl_mime_name(part, "units");

    part = curl_mime_addpart(mime);
    curl_mime_data(part, std::to_string(system_id).c_str(), CURL_ZERO_TERMINATED);
    curl_mime_name(part, "system");

    part = curl_mime_addpart(mime);
    curl_mime_data(part, call_info.short_name.c_str(), CURL_ZERO_TERMINATED);
    curl_mime_name(part, "systemLabel");

    multi_handle = curl_multi_init();

    /* initialize custom header list (stating that Expect: 100-continue is not wanted */
    headerlist = curl_slist_append(headerlist, "Expect:");
    if (curl && multi_handle) {
      std::string url = data.server + "/api/call-upload";

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

      res = curl_multi_cleanup(multi_handle);

      /* always cleanup */
      curl_easy_cleanup(curl);

      /* then cleanup the mime */
      curl_mime_free(mime);

      /* free slist */
      curl_slist_free_all(headerlist);

      long response_code;
      curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response_code);

      if (res == CURLM_OK && response_code == 200) {
        struct stat file_info;
        stat(compress_wav ? call_info.converted : call_info.filename, &file_info);
        std::string loghdr = log_header(call_info.short_name,call_info.call_num,call_info.talkgroup_display,call_info.freq);
        BOOST_LOG_TRIVIAL(info) << loghdr << "Rdio Scanner Upload Success - file size: " << file_info.st_size;
        ;
        return 0;
      }
    }
    std::string loghdr = log_header(call_info.short_name,call_info.call_num,call_info.talkgroup_display,call_info.freq);
    BOOST_LOG_TRIVIAL(error) << loghdr << "Rdio Scanner Upload Error: " << response_buffer;
    return 1;
  }

  int call_end(Call_Data_t call_info) {
    return upload(call_info);
  }

  int parse_config(json config_data) {
    std::string log_prefix = "\t[Rdio Scanner]\t";
    /*
          system->set_rdioscanner_api_key(node.second.get<std::string>("rdioscannerApiKey", ""));
      BOOST_LOG_TRIVIAL(info) << "Rdio Scanner API Key: " << system->get_rdioscanner_api_key();
      system->set_rdioscanner_system_id(node.second.get<int>("rdioscannerSystemId", 0));
      BOOST_LOG_TRIVIAL(info) << "Rdio Scanner System ID: " << system->get_rdioscanner_system_id();

    config.rdioscanner_server = pt.get<std::string>("rdioscannerServer", "");
    BOOST_LOG_TRIVIAL(info) << "Rdio Scanner Server: " << config.rdioscanner_server;*/

    // Tests to see if the rdioscannerServer value exists in the config file
    bool upload_server_exists = config_data.contains("server");
    if (!upload_server_exists) {
      return 1;
    }

    this->data.server = config_data.value("server", "");
    BOOST_LOG_TRIVIAL(info) << log_prefix << "Rdio Scanner Server: " << this->data.server;

    // from: http://www.zedwood.com/article/cpp-boost-url-regex
    boost::regex api_regex("(.*)(.{2}$)");
    boost::regex ex("(http|https)://([^/ :]+):?([^/ ]*)(/?[^ #?]*)\\x3f?([^ #]*)#?([^ ]*)");
    boost::cmatch what;

    if (!regex_match(this->data.server.c_str(), what, ex)) {
      BOOST_LOG_TRIVIAL(error) << log_prefix << "Unable to parse Rdio Scanner Server URL\n";
      return 1;
    }

    // Gets the API key for each system, if defined
    for (json element : config_data["systems"]) {
      bool rdioscanner_exists = element.contains("apiKey");
      if (rdioscanner_exists) {
        Rdio_Scanner_System sys;
        sys.api_key = element.value("apiKey", "");
        sys.system_id = element.value("systemId", 0);
        sys.short_name = element.value("shortName", "");
        regex_match(sys.api_key.c_str(), what, api_regex);
        std::string redacted_api(what[2].first, what[2].second);
        BOOST_LOG_TRIVIAL(info) << log_prefix << "Uploading calls for: " << sys.short_name  << "\t Rdio System: " << sys.system_id << "\t API Key: ******" << redacted_api;
        this->data.systems.push_back(sys);
      }
    }

    if (this->data.systems.size() == 0) {
      BOOST_LOG_TRIVIAL(error) << log_prefix << "Rdio Scanner Server set, but no Systems are configured\n";
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
  static boost::shared_ptr<Rdio_Scanner_Uploader> create() {
    return boost::shared_ptr<Rdio_Scanner_Uploader>(
        new Rdio_Scanner_Uploader());
  }
};

BOOST_DLL_ALIAS(
    Rdio_Scanner_Uploader::create, // <-- this function is exported with...
    create_plugin                  // <-- ...this alias name
)
