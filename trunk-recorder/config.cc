/**
 * Method name: load_config()
 * Description: <#description#>
 * Parameters: <#parameters#>
 */
#include "./config.h"

using json = nlohmann::json;

namespace logging = boost::log;
namespace keywords = boost::log::keywords;
namespace src = boost::log::sources;
namespace sinks = boost::log::sinks;

using namespace std;

void set_logging_level(std::string log_level) {
  boost::log::trivial::severity_level sev_level = boost::log::trivial::info;

  if (log_level == "trace")
    sev_level = boost::log::trivial::trace;
  else if (log_level == "debug")
    sev_level = boost::log::trivial::debug;
  else if (log_level == "info")
    sev_level = boost::log::trivial::info;
  else if (log_level == "warning")
    sev_level = boost::log::trivial::warning;
  else if (log_level == "error")
    sev_level = boost::log::trivial::error;
  else if (log_level == "fatal")
    sev_level = boost::log::trivial::fatal;
  else {
    BOOST_LOG_TRIVIAL(error) << "set_logging_level: Unknown logging level: " << log_level;
    return;
  }

  boost::log::core::get()->set_filter(
      boost::log::trivial::severity >= sev_level);
}

struct NoColorLoggingFormatter {
  void operator()(logging::record_view const &rec, logging::formatting_ostream &strm) const {
    auto message = rec.attribute_values()[logging::aux::default_attribute_names::message()];
    auto message_str = message.extract<std::string>().get();

    std::regex escape_seq_regex("\u001B\\[[0-9;]+m");
    strm << std::regex_replace(message_str, escape_seq_regex, "");
  }
};

void setup_console_log(std::string log_color, std::string time_fmt) {
  boost::shared_ptr<sinks::synchronous_sink<sinks::basic_text_ostream_backend<char>>> console_sink = logging::add_console_log(std::clog);

  if ((log_color == "console") || (log_color == "all")) {
    console_sink->set_formatter(logging::expressions::format("[%1%] (%2%)   %3%") %
                                logging::expressions::format_date_time<boost::posix_time::ptime>("TimeStamp", time_fmt) %
                                logging::expressions::attr<logging::trivial::severity_level>("Severity") %
                                logging::expressions::smessage);
  } else {
    console_sink->set_formatter(logging::expressions::format("[%1%] (%2%)   %3%") %
                                logging::expressions::format_date_time<boost::posix_time::ptime>("TimeStamp", time_fmt) %
                                logging::expressions::attr<logging::trivial::severity_level>("Severity") %
                                logging::expressions::wrap_formatter(NoColorLoggingFormatter{}));
  }

  std::locale loc = std::locale("C");
  console_sink->imbue(loc);
}

void setup_file_log(std::string log_dir, std::string log_color, std::string time_fmt) {
  boost::shared_ptr<sinks::synchronous_sink<sinks::text_file_backend>> log_sink = logging::add_file_log(
      keywords::file_name = log_dir + "/%m-%d-%Y_%H%M_%2N.log",
      keywords::rotation_size = 100 * 1024 * 1024,
      keywords::time_based_rotation = sinks::file::rotation_at_time_point(0, 0, 0),
      keywords::auto_flush = true);

  if ((log_color == "logfile") || (log_color == "all")) {
    log_sink->set_formatter(logging::expressions::format("[%1%] (%2%)   %3%") %
                            logging::expressions::format_date_time<boost::posix_time::ptime>("TimeStamp", time_fmt) %
                            logging::expressions::attr<logging::trivial::severity_level>("Severity") %
                            logging::expressions::smessage);
  } else {
    log_sink->set_formatter(logging::expressions::format("[%1%] (%2%)   %3%") %
                            logging::expressions::format_date_time<boost::posix_time::ptime>("TimeStamp", time_fmt) %
                            logging::expressions::attr<logging::trivial::severity_level>("Severity") %
                            logging::expressions::wrap_formatter(NoColorLoggingFormatter{}));
  }
}

bool load_config(string config_file, Config &config, gr::top_block_sptr &tb, std::vector<Source *> &sources, std::vector<System *> &systems) {

  string system_modulation;

  json data;
  int sys_count = 0;
  int source_count = 0;

  try {
    std::ifstream f(config_file);
    data = json::parse(f);
  } catch (const json::parse_error &e) {
    // output exception information
    std::cout << "message: " << e.what() << '\n'
              << "exception id: " << e.id << '\n'
              << "byte position of error: " << e.byte << std::endl;
  }

  try {
    // const std::string json_filename = "config.json";

    boost::property_tree::ptree pt;
    boost::property_tree::read_json(config_file, pt);

    // Set NO_COLOR options for console and log files
    // Default is [color console, no_color logfile] unless NO_COLOR env var is set.  config.json overrides NO_COLOR use.
    char *no_color = getenv("NO_COLOR");
    bool color = true;

    if (no_color != NULL && no_color[0] != '\0')
      color = false;

    config.log_color = data.value("logColor", (color ? "console" : "none"));

    config.console_log = data.value("consoleLog", true);
    if (config.console_log) {
      setup_console_log(config.log_color, "%Y-%m-%d %H:%M:%S.%f");
    }

    BOOST_LOG_TRIVIAL(info) << "\n-------------------------------------\n     Trunk Recorder\n-------------------------------------\n";

    BOOST_LOG_TRIVIAL(info) << "\n\n-------------------------------------\nINSTANCE\n-------------------------------------\n";

    config.log_file = data.value("logFile", false);
    config.log_dir = data.value("logDir", "logs");
    if (config.log_file) {
      setup_file_log(config.log_dir, config.log_color, "%Y-%m-%d %H:%M:%S.%f");
    }

    double config_ver = data.value("ver", 0.0);
    if (config_ver < 2) {
      BOOST_LOG_TRIVIAL(info) << "The formatting for config files has changed.";
      BOOST_LOG_TRIVIAL(info) << "Modulation type, Squelch and audio levels are now set in each System instead of under a Source.";
      BOOST_LOG_TRIVIAL(info) << "See sample config files in the /example folder and look at readme.md for more details.";
      BOOST_LOG_TRIVIAL(info) << "After you have made these updates, make sure you add \"ver\": 2, to the top.\n\n";
      return false;
    }

    BOOST_LOG_TRIVIAL(info) << "Using Config file: " << config_file << "\n";
    BOOST_LOG_TRIVIAL(info) << PROJECT_NAME << ": "
                            << "Version: " << PROJECT_VER << "\n";

    BOOST_LOG_TRIVIAL(info) << "Log to File: " << config.log_file;
    BOOST_LOG_TRIVIAL(info) << "Log Directory: " << config.log_dir;

    std::string defaultTempDir = boost::filesystem::current_path().string();

    if (boost::filesystem::exists("/dev/shm")) {
      defaultTempDir = "/dev/shm";
    }
    config.temp_dir = data.value("tempDir", defaultTempDir);
    size_t pos = config.temp_dir.find_last_of("/");

    if (pos == config.temp_dir.length() - 1) {
      config.temp_dir.erase(config.temp_dir.length() - 1);
    }

    BOOST_LOG_TRIVIAL(info) << "Temporary Transmission Directory: " << config.temp_dir;

    config.archive_files_on_failure = data.value("archiveFilesOnFailure", false);
    BOOST_LOG_TRIVIAL(info) << "Archive Files on Failure: " << config.archive_files_on_failure;

    config.capture_dir = data.value("captureDir", boost::filesystem::current_path().string());
    pos = config.capture_dir.find_last_of("/");

    if (pos == config.capture_dir.length() - 1) {
      config.capture_dir.erase(config.capture_dir.length() - 1);
    }

    BOOST_LOG_TRIVIAL(info) << "Capture Directory: " << config.capture_dir;
    config.upload_server = data.value("uploadServer", "");
    BOOST_LOG_TRIVIAL(info) << "Upload Server: " << config.upload_server;
    config.bcfy_calls_server = data.value("broadcastifyCallsServer", "");
    BOOST_LOG_TRIVIAL(info) << "Broadcastify Calls Server: " << config.bcfy_calls_server;
    config.status_server = data.value("statusServer", "");
    BOOST_LOG_TRIVIAL(info) << "Status Server: " << config.status_server;
    config.instance_key = data.value("instanceKey", "");
    BOOST_LOG_TRIVIAL(info) << "Instance Key: " << config.instance_key;
    config.instance_id = data.value("instanceId", "");
    BOOST_LOG_TRIVIAL(info) << "Instance Id: " << config.instance_id;
    config.broadcast_signals = data.value("broadcastSignals", false);
    BOOST_LOG_TRIVIAL(info) << "Broadcast Signals: " << config.broadcast_signals;
    config.default_mode = data.value("defaultMode", "digital");
    BOOST_LOG_TRIVIAL(info) << "Default Mode: " << config.default_mode;
    config.call_timeout = data.value("callTimeout", 3.0);
    BOOST_LOG_TRIVIAL(info) << "Call Timeout (seconds): " << config.call_timeout;
    config.control_message_warn_rate = data.value("controlWarnRate", 10);
    BOOST_LOG_TRIVIAL(info) << "Control channel warning rate: " << config.control_message_warn_rate;
    config.control_retune_limit = data.value("controlRetuneLimit", 0);
    BOOST_LOG_TRIVIAL(info) << "Control channel retune limit: " << config.control_retune_limit;
    config.soft_vocoder = data.value("softVocoder", false);
    BOOST_LOG_TRIVIAL(info) << "Phase 1 Software Vocoder: " << config.soft_vocoder;
    config.enable_audio_streaming = data.value("audioStreaming", false);
    BOOST_LOG_TRIVIAL(info) << "Enable Audio Streaming: " << config.enable_audio_streaming;
    config.record_uu_v_calls = data.value("recordUUVCalls", true);
    BOOST_LOG_TRIVIAL(info) << "Record Unit to Unit Voice Calls: " << config.record_uu_v_calls;
    config.new_call_from_update = data.value("newCallFromUpdate", true);
    BOOST_LOG_TRIVIAL(info) << "New Call from UPDATE Messages: " << config.new_call_from_update;
    std::string frequency_format_string = data.value("frequencyFormat", "mhz");

    if (boost::iequals(frequency_format_string, "mhz")) {
      frequency_format = 1;
    } else if (boost::iequals(frequency_format_string, "hz")) {
      frequency_format = 2;
    } else {
      frequency_format = 0;
    }
    config.frequency_format = frequency_format;
    BOOST_LOG_TRIVIAL(info) << "Frequency format: " << get_frequency_format();

    statusAsString = data.value("statusAsString", statusAsString);
    BOOST_LOG_TRIVIAL(info) << "Status as String: " << statusAsString;
    std::string log_level = data.value("logLevel", "info");
    BOOST_LOG_TRIVIAL(info) << "Log Level: " << log_level;
    set_logging_level(log_level);
    BOOST_LOG_TRIVIAL(info) << "Color Console/Logfile Output: " << config.log_color;

    config.debug_recorder = data.value("debugRecorder", 0);
    config.debug_recorder_address = data.value("debugRecorderAddress", "127.0.0.1");
    config.debug_recorder_port = data.value("debugRecorderPort", 1234);

    BOOST_LOG_TRIVIAL(info) << "\n-------------------------------------\nSYSTEMS\n-------------------------------------\n";

    for (json element : data["systems"]) {
      bool system_enabled = element.value("enabled", true);
      if (system_enabled) {
        // each system should have a unique index value;
        System *system = System::make(sys_count++);

        std::stringstream default_script;
        unsigned long sys_id;
        unsigned long wacn;
        unsigned long nac;
        default_script << "sys_" << sys_count;

        BOOST_LOG_TRIVIAL(info) << "\n\nSystem Number: " << sys_count << "\n-------------------------------------\n";
        system->set_short_name(element.value("shortName", default_script.str()));
        BOOST_LOG_TRIVIAL(info) << "Short Name: " << system->get_short_name();

        system->set_system_type(element["type"]);
        BOOST_LOG_TRIVIAL(info) << "System Type: " << system->get_system_type();

        // If it is a conventional System
        if ((system->get_system_type() == "conventional") || (system->get_system_type() == "conventionalP25") || (system->get_system_type() == "conventionalDMR") || (system->get_system_type() == "conventionalSIGMF")) {

          bool channel_file_exist = element.contains("channelFile");
          bool channels_exist = element.contains("channels");

          if (channel_file_exist && channels_exist) {
            BOOST_LOG_TRIVIAL(error) << "Both \"channels\" and \"channelFile\" cannot be defined for a system!";
            return false;
          }

          if (channels_exist) {
            BOOST_LOG_TRIVIAL(info) << "Conventional Channels: ";
            std::vector<double> channels = element["channels"];
            for (auto &channel : channels) {
              BOOST_LOG_TRIVIAL(info) << "  " << format_freq(channel);
              system->add_channel(channel);
            }
          } else if (channel_file_exist) {
            std::string channel_file = element["channelFile"];
            BOOST_LOG_TRIVIAL(info) << "Channel File: " << channel_file;
            system->set_channel_file(channel_file);
          } else {
            BOOST_LOG_TRIVIAL(error) << "Either \"channels\" or \"channelFile\" need to be defined for a conventional system!";
            return false;
          }
          // If it is a Trunked System
        } else if ((system->get_system_type() == "smartnet") || (system->get_system_type() == "p25")) {
          BOOST_LOG_TRIVIAL(info) << "Control Channels: ";
          std::vector<double> control_channels = element["control_channels"];
          for (auto &control_channel : control_channels) {
            system->add_control_channel(control_channel);
          }
          for (unsigned int i = 0; i < control_channels.size(); i++) {
            BOOST_LOG_TRIVIAL(info) << "  " << format_freq(control_channels[i]);
          }
          system->set_talkgroups_file(element.value("talkgroupsFile", ""));
          BOOST_LOG_TRIVIAL(info) << "Talkgroups File: " << system->get_talkgroups_file();

          bool custom_freq_table_file_exists = element.contains("customFrequencyTableFile");
          if (custom_freq_table_file_exists)
          {
            std::string custom_freq_table_file = element["customFrequencyTableFile"];
            system->set_custom_freq_table_file(custom_freq_table_file);
            BOOST_LOG_TRIVIAL(info) << "Custom Frequency Table File: " << custom_freq_table_file;
          }


        } else {
          BOOST_LOG_TRIVIAL(error) << "System Type in config.json not recognized";
          return false;
        }

        bool qpsk_mod = true;
        double digital_levels = element.value("digitalLevels", 1.0);
        double analog_levels = element.value("analogLevels", 8.0);
        double squelch_db = element.value("squelch", -160.0);
        float tau = element.value("deemphasisTau", 0.000750);  // Default to 750us if not specified
        int max_dev = element.value("maxDev", 5000);
        double filter_width = element.value("filterWidth", 1.0);
        bool conversation_mode = element.value("conversationMode", true);
        bool mod_exists = element.contains("modulation");

        if (mod_exists) {
          system_modulation = element["modulation"];

          if (boost::iequals(system_modulation, "qpsk")) {
            qpsk_mod = true;
            BOOST_LOG_TRIVIAL(info) << "Modulation: qpsk";
          } else if (boost::iequals(system_modulation, "fsk4")) {
            qpsk_mod = false;
            BOOST_LOG_TRIVIAL(info) << "Modulation: fsk4";
          } else {
            qpsk_mod = true;
            BOOST_LOG_TRIVIAL(error) << "! System Modulation specified but not recognized, it needs to be either \"fsk4\" or \"qpsk\", assuming qpsk";
          }
        } else {
          BOOST_LOG_TRIVIAL(info) << "Modulation: qpsk";
          qpsk_mod = true;
        }
        system->set_tau(tau);
        system->set_squelch_db(squelch_db);
        system->set_analog_levels(analog_levels);
        system->set_digital_levels(digital_levels);
        system->set_qpsk_mod(qpsk_mod);
        system->set_max_dev(max_dev);
        system->set_filter_width(filter_width);
        system->set_conversation_mode(conversation_mode);
        BOOST_LOG_TRIVIAL(info) << "Conversation Mode: " << conversation_mode;
        BOOST_LOG_TRIVIAL(info) << "Analog Recorder Maximum Deviation: " << element.value("maxDev", 5000);
        BOOST_LOG_TRIVIAL(info) << "Filter Width: " << filter_width;
        BOOST_LOG_TRIVIAL(info) << "Squelch: " << element.value("squelch", -160);
        BOOST_LOG_TRIVIAL(info) << "De-emphasis Tau: " << tau;
        system->set_api_key(element.value("apiKey", ""));
        BOOST_LOG_TRIVIAL(info) << "API Key: " << system->get_api_key();
        system->set_bcfy_api_key(element.value("broadcastifyApiKey", ""));
        BOOST_LOG_TRIVIAL(info) << "Broadcastify API Key: " << system->get_bcfy_api_key();
        system->set_bcfy_system_id(element.value("broadcastifySystemId", 0));
        BOOST_LOG_TRIVIAL(info) << "Broadcastify Calls System ID: " << system->get_bcfy_system_id();
        system->set_upload_script(element.value("uploadScript", ""));
        BOOST_LOG_TRIVIAL(info) << "Upload Script: " << system->get_upload_script();
        system->set_compress_wav(element.value("compressWav", true));
        BOOST_LOG_TRIVIAL(info) << "Compress .wav Files: " << system->get_compress_wav();
        system->set_call_log(element.value("callLog", true));
        BOOST_LOG_TRIVIAL(info) << "Call Log: " << system->get_call_log();
        system->set_audio_archive(element.value("audioArchive", true));
        BOOST_LOG_TRIVIAL(info) << "Audio Archive: " << system->get_audio_archive();
        system->set_transmission_archive(element.value("transmissionArchive", false));
        BOOST_LOG_TRIVIAL(info) << "Transmission Archive: " << system->get_transmission_archive();
        system->set_unit_tags_file(element.value("unitTagsFile", ""));
        BOOST_LOG_TRIVIAL(info) << "Unit Tags File: " << system->get_unit_tags_file();
        system->set_unit_tags_ota_file(element.value("unitTagsOTA", ""));
        BOOST_LOG_TRIVIAL(info) << "Unit Tags OTA File: " << system->get_unit_tags_ota_file();
        system->set_unit_tags_mode(element.value("unitTagsMode", "user"));
        BOOST_LOG_TRIVIAL(info) << "Unit Tags Mode: " << system->get_unit_tags_mode();
        system->set_record_unknown(element.value("recordUnknown", true));
        BOOST_LOG_TRIVIAL(info) << "Record Unknown Talkgroups: " << system->get_record_unknown();
        system->set_mdc_enabled(element.value("decodeMDC", false));
        BOOST_LOG_TRIVIAL(info) << "Decode MDC: " << system->get_mdc_enabled();
        system->set_fsync_enabled(element.value("decodeFSync", false));
        BOOST_LOG_TRIVIAL(info) << "Decode FSync: " << system->get_fsync_enabled();
        system->set_star_enabled(element.value("decodeStar", false));
        BOOST_LOG_TRIVIAL(info) << "Decode Star: " << system->get_star_enabled();
        system->set_tps_enabled(element.value("decodeTPS", false));
        BOOST_LOG_TRIVIAL(info) << "Decode TPS: " << system->get_tps_enabled();
        std::string talkgroup_display_format_string = element.value("talkgroupDisplayFormat", "Id");
        if (boost::iequals(talkgroup_display_format_string, "id_tag")) {
          system->set_talkgroup_display_format(talkGroupDisplayFormat_id_tag);
        } else if (boost::iequals(talkgroup_display_format_string, "tag_id")) {
          system->set_talkgroup_display_format(talkGroupDisplayFormat_tag_id);
        } else {
          system->set_talkgroup_display_format(talkGroupDisplayFormat_id);
        }
        BOOST_LOG_TRIVIAL(info) << "Talkgroup Display Format: " << talkgroup_display_format_string;

        sys_id = element.value("sysId", 0);
        nac = element.value("nac", 0);
        wacn = element.value("wacn", 0);
        system->set_xor_mask(sys_id, wacn, nac);
        system->set_bandplan(element.value("bandplan", "800_standard"));
        system->set_bandfreq(800); // Default to 800

        if (boost::starts_with(system->get_bandplan(), "400")) {
          system->set_bandfreq(400);
        }
        system->set_bandplan_base(element.value("bandplanBase", 0.0));
        system->set_bandplan_high(element.value("bandplanHigh", 0.0));
        system->set_bandplan_spacing(element.value("bandplanSpacing", 0.0));
        system->set_bandplan_offset(element.value("bandplanOffset", 0));

        if (system->get_system_type() == "smartnet") {
          BOOST_LOG_TRIVIAL(info) << "Smartnet bandplan: " << system->get_bandplan();
          BOOST_LOG_TRIVIAL(info) << "Smartnet band: " << system->get_bandfreq();

          if (system->get_bandplan_base() || system->get_bandplan_spacing() || system->get_bandplan_offset()) {
            BOOST_LOG_TRIVIAL(info) << "Smartnet bandplan base freq: " << system->get_bandplan_base();
            BOOST_LOG_TRIVIAL(info) << "Smartnet bandplan high freq: " << system->get_bandplan_high();
            BOOST_LOG_TRIVIAL(info) << "Smartnet bandplan spacing: " << system->get_bandplan_spacing();
            BOOST_LOG_TRIVIAL(info) << "Smartnet bandplan offset: " << system->get_bandplan_offset();
          }
        }

        system->set_hideEncrypted(element.value("hideEncrypted", system->get_hideEncrypted()));
        BOOST_LOG_TRIVIAL(info) << "Hide Encrypted Talkgroups: " << system->get_hideEncrypted();
        system->set_monitorEncrypted(element.value("monitorEncrypted", system->get_monitorEncrypted()));
        BOOST_LOG_TRIVIAL(info) << "Monitor Encrypted Calls: " << system->get_monitorEncrypted();
        system->set_hideUnknown(element.value("hideUnknownTalkgroups", system->get_hideUnknown()));
        BOOST_LOG_TRIVIAL(info) << "Hide Unknown Talkgroups: " << system->get_hideUnknown();
        system->set_min_duration(element.value("minDuration", 0.0));
        BOOST_LOG_TRIVIAL(info) << "Minimum Call Duration (in seconds): " << system->get_min_duration();
        system->set_max_duration(element.value("maxDuration", 0.0));
        BOOST_LOG_TRIVIAL(info) << "Maximum Call Duration (in seconds): " << system->get_max_duration();
        system->set_min_tx_duration(element.value("minTransmissionDuration", 0.0));
        BOOST_LOG_TRIVIAL(info) << "Minimum Transmission Duration (in seconds): " << system->get_min_tx_duration();
        system->set_multiSite(element.value("multiSite", false));
        BOOST_LOG_TRIVIAL(info) << "Multiple Site System: " << system->get_multiSite();
        system->set_multiSiteSystemName(element.value("multiSiteSystemName", ""));
        BOOST_LOG_TRIVIAL(info) << "Multiple Site System Name: " << system->get_multiSiteSystemName();
        system->set_multiSiteSystemNumber(element.value("multiSiteSystemNumber", 0));
        BOOST_LOG_TRIVIAL(info) << "Multiple Site System Number: " << system->get_multiSiteSystemNumber();

        if (!system->get_compress_wav()) {
          if ((system->get_api_key().length() > 0) || (system->get_bcfy_api_key().length() > 0)) {
            BOOST_LOG_TRIVIAL(error) << "Compress WAV must be set to true if you are using OpenMHz or Broadcastify";
            return false;
          }
        }

        systems.push_back(system);
        BOOST_LOG_TRIVIAL(info);
      }
    }

    BOOST_LOG_TRIVIAL(info) << "\n\n-------------------------------------\nSOURCES\n-------------------------------------\n";
    for (json element : data["sources"]) {

      bool source_enabled = element.value("enabled", true);
      if (source_enabled) {
        Source *source;
        bool gain_set = false;
        std::string driver = element.value("driver", "");

        if ((driver != "osmosdr") && (driver != "usrp") && (driver != "sigmf") && (driver != "iqfile") && (driver != "iio")) {
          BOOST_LOG_TRIVIAL(error) << "Driver specified in config.json not recognized, needs to be osmosdr, sigmf, iqfile, usrp, or iio";
          return false;
        }

        int digital_recorders = element.value("digitalRecorders", 0);
        int sigmf_recorders = element.value("sigmfRecorders", 0);
        int analog_recorders = element.value("analogRecorders", 0);

        if (driver == "sigmf") {
          string sigmf_data = element.value("sigmfData", "");
          string sigmf_meta = element.value("sigmfMeta", "");
          bool repeat = element.value("repeat", false);
          source = new Source(sigmf_meta, sigmf_data, repeat, &config);
        } else if (driver == "iqfile") {
          string iq_file = element.value("iqFile", "");
          string iq_type = element.value("iqType", "");
          double center = element.value("center", 0.0);
          double rate = element.value("rate", 0.0);
          bool repeat = element.value("repeat", false);
          if ((iq_type != "complex") && (iq_type != "float")) {
            BOOST_LOG_TRIVIAL(error) << "IQ Type specified in config.json not recognized, needs to be complex or float";
            return false;
          }
          source = new Source(iq_file, center, rate, repeat, &config);
        } else {

          std::string device = element.value("device", "");
          BOOST_LOG_TRIVIAL(info) << "Driver: " << element.value("driver", "");
          BOOST_LOG_TRIVIAL(info) << "Device: " << device;

          int silence_frames = element.value("silenceFrames", 0);
          double center = element.value("center", 0.0);
          double rate = element.value("rate", 0.0);
          double error = element.value("error", 0.0);
          double ppm = element.value("ppm", 0.0);
          bool autotune = element.value("autoTune", false);
          bool agc = element.value("agc", false);
          double gain = element.value("gain", 0.0);
          double if_gain = element.value("ifGain", 0.0);
          double bb_gain = element.value("bbGain", 0.0);
          double mix_gain = element.value("mixGain", 0.0);
          double lna_gain = element.value("lnaGain", 0.0);
          double pga_gain = element.value("pgaGain", 0.0);
          double tia_gain = element.value("tiaGain", 0.0);
          double amp_gain = element.value("ampGain", 0.0);
          double vga_gain = element.value("vgaGain", 0.0);
          double vga1_gain = element.value("vga1Gain", 0.0);
          double vga2_gain = element.value("vga2Gain", 0.0);

          std::string antenna = element.value("antenna", "");

          BOOST_LOG_TRIVIAL(info) << "Center: " << format_freq(element.value("center", 0.0));
          BOOST_LOG_TRIVIAL(info) << "Rate: " << FormatSamplingRate(element.value("rate", 0.0));
          BOOST_LOG_TRIVIAL(info) << "Error: " << element.value("error", 0.0);
          BOOST_LOG_TRIVIAL(info) << "PPM Error: " << element.value("ppm", 0.0);
          BOOST_LOG_TRIVIAL(info) << "P25 Autotune: " << element.value("autoTune", false);
          BOOST_LOG_TRIVIAL(info) << "Auto gain control: " << element.value("agc", false);
          BOOST_LOG_TRIVIAL(info) << "Gain: " << element.value("gain", 0.0);
          BOOST_LOG_TRIVIAL(info) << "IF Gain: " << element.value("ifGain", 0.0);
          BOOST_LOG_TRIVIAL(info) << "BB Gain: " << element.value("bbGain", 0.0);
          BOOST_LOG_TRIVIAL(info) << "LNA Gain: " << element.value("lnaGain", 0.0);
          BOOST_LOG_TRIVIAL(info) << "PGA Gain: " << element.value("pgaGain", 0.0);
          BOOST_LOG_TRIVIAL(info) << "TIA Gain: " << element.value("tiaGain", 0.0);
          BOOST_LOG_TRIVIAL(info) << "MIX Gain: " << element.value("mixGain", 0.0);
          BOOST_LOG_TRIVIAL(info) << "AMP Gain: " << element.value("ampGain", 0.0);
          BOOST_LOG_TRIVIAL(info) << "VGA Gain: " << element.value("vgaGain", 0.0);
          BOOST_LOG_TRIVIAL(info) << "VGA1 Gain: " << element.value("vga1Gain", 0.0);
          BOOST_LOG_TRIVIAL(info) << "VGA2 Gain: " << element.value("vga2Gain", 0.0);
          BOOST_LOG_TRIVIAL(info) << "Idle Silence: " << element.value("silenceFrame", 0);

          if ((driver == "osmosdr") && (long(rate) % 24000 != 0)) {
            BOOST_LOG_TRIVIAL(error) << "OsmoSDR must have a sample rate that is a multiple of 24000, current rate: " << rate << " for device: " << device;
            //return false;
          }

          if ((ppm != 0) && (error != 0)) {
            BOOST_LOG_TRIVIAL(info) << "Both PPM and Error should not be set at the same time. Setting Error to 0.";
            error = 0;
          }
          source = new Source(center, rate, error, driver, device, &config);

          // SoapySDRPlay3 quirk: autogain must be disabled before any of the gains can be set
          if (source->get_device().find("sdrplay") != std::string::npos) {
            source->set_gain_mode(agc);
          }

          if (element.contains("signalDetectorThreshold")) {
            source->set_signal_detector_threshold(element["signalDetectorThreshold"]);
          }

          source->set_autotune_source(autotune);

          if (element.contains("gainSettings")) {
            for (auto it = element["gainSettings"].begin(); it != element["gainSettings"].end(); ++it) {

              source->set_gain_by_name(it.key(), it.value());
              gain_set = true;
            }
          }

          if (if_gain != 0) {
            gain_set = true;
            source->set_gain_by_name("IF", if_gain);
          }

          if (bb_gain != 0) {
            gain_set = true;
            source->set_gain_by_name("BB", bb_gain);
          }

          if (mix_gain != 0) {
            gain_set = true;
            source->set_gain_by_name("MIX", mix_gain);
          }

          if (lna_gain != 0) {
            gain_set = true;
            source->set_gain_by_name("LNA", lna_gain);
          }

          if (tia_gain != 0) {
            gain_set = true;
            source->set_gain_by_name("TIA", tia_gain);
          }

          if (pga_gain != 0) {
            gain_set = true;
            source->set_gain_by_name("PGA", pga_gain);
          }

          if (amp_gain != 0) {
            gain_set = true;
            source->set_gain_by_name("AMP", amp_gain);
          }

          if (vga_gain != 0) {
            gain_set = true;
            source->set_gain_by_name("VGA", vga_gain);
          }

          if (vga1_gain != 0) {
            gain_set = true;
            source->set_gain_by_name("VGA1", vga1_gain);
          }

          if (vga2_gain != 0) {
            gain_set = true;
            source->set_gain_by_name("VGA2", vga2_gain);
          }

          if (gain != 0) {
            gain_set = true;
            source->set_gain(gain);
          }

          if (!gain_set) {
            BOOST_LOG_TRIVIAL(error) << "! No Gain was specified! Things will probably not work";
          }

          source->set_gain_mode(agc);
          source->set_antenna(antenna);
          source->set_silence_frames(silence_frames);

          if (ppm != 0) {
            source->set_freq_corr(ppm);
          }
        }
        BOOST_LOG_TRIVIAL(info) << "Max Frequency: " << format_freq(source->get_max_hz());
        BOOST_LOG_TRIVIAL(info) << "Min Frequency: " << format_freq(source->get_min_hz());
        BOOST_LOG_TRIVIAL(info) << "Digital Recorders: " << element.value("digitalRecorders", 0);
        BOOST_LOG_TRIVIAL(info) << "SigMF Recorders: " << element.value("sigmfRecorders", 0);
        BOOST_LOG_TRIVIAL(info) << "Analog Recorders: " << element.value("analogRecorders", 0);
        source->create_digital_recorders(tb, digital_recorders);
        source->create_analog_recorders(tb, analog_recorders);
        source->create_sigmf_recorders(tb, sigmf_recorders);
        if (config.debug_recorder) {
          source->create_debug_recorder(tb, source_count);
        }

        sources.push_back(source);
        source_count++;
        BOOST_LOG_TRIVIAL(info) << "\n-------------------------------------\n\n";
      }
    }

    BOOST_LOG_TRIVIAL(info) << "\n\n-------------------------------------\nPLUGINS\n-------------------------------------\n";
    add_internal_plugin("openmhz_uploader", "libopenmhz_uploader.so", data);
    add_internal_plugin("broadcastify_uploader", "libbroadcastify_uploader.so", data);
    add_internal_plugin("unit_script", "libunit_script.so", data);
    initialize_plugins(data, &config, sources, systems);
  } catch (std::exception const &e) {
    BOOST_LOG_TRIVIAL(error) << "Failed parsing Config: " << e.what();
    return false;
  }
  if (config.debug_recorder) {
    BOOST_LOG_TRIVIAL(info) << "\n\n-------------------------------------\nDEBUG RECORDER\n-------------------------------------\n";
    BOOST_LOG_TRIVIAL(info) << "  Address: " << config.debug_recorder_address;

    for (vector<Source *>::iterator it = sources.begin(); it != sources.end(); it++) {
      Source *source = *it;
      BOOST_LOG_TRIVIAL(info) << "  " << source->get_driver() << " - " << source->get_device() << " [ " << format_freq(source->get_center()) << " ]  Port: " << source->get_debug_recorder_port();
    }
    BOOST_LOG_TRIVIAL(info) << "\n\n-------------------------------------\n";
  }
  BOOST_LOG_TRIVIAL(info) << "\n\n";
  return true;
}
