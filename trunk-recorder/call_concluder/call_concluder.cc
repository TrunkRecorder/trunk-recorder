#include "call_concluder.h"
#include "../plugin_manager/plugin_manager.h"
#include <boost/filesystem.hpp>
#include <filesystem>
namespace fs = std::filesystem;

const int Call_Concluder::MAX_RETRY = 2;
std::list<std::future<Call_Data_t>> Call_Concluder::call_data_workers = {};
std::list<Call_Data_t> Call_Concluder::retry_call_list = {};

int combine_wav(std::string files, char *target_filename) {
  char shell_command[4000];

  int nchars = snprintf(shell_command, 4000, "sox %s %s ", files.c_str(), target_filename);

  if (nchars >= 4000) {
    BOOST_LOG_TRIVIAL(error) << "Call uploader: SOX Combine WAV Command longer than 4000 characters";
    return -1;
  }
  int rc = system(shell_command);

  if (rc > 0) {
    BOOST_LOG_TRIVIAL(info) << "Combining: " << files.c_str() << " into: " << target_filename;
    BOOST_LOG_TRIVIAL(info) << shell_command;
    BOOST_LOG_TRIVIAL(error) << "Failed to combine recordings, see above error. Make sure you have sox and fdkaac installed.";
    return -1;
  }

  return nchars;
}
int convert_media(char *filename, char *converted, char *date, const char *short_name, const char *talkgroup) {
  char shell_command[400];

  int nchars = snprintf(shell_command, 400, "sox %s --norm=-.01 -t wav - | fdkaac --silent  -p 2 --date '%s' --artist '%s' --title '%s' --moov-before-mdat --ignorelength -b 8000 -o %s -", filename, date, short_name, talkgroup, converted);

  if (nchars >= 400) {
    BOOST_LOG_TRIVIAL(error) << "Call uploader: Command longer than 400 characters";
    return -1;
  }
  BOOST_LOG_TRIVIAL(trace) << "Converting: " << converted;
  BOOST_LOG_TRIVIAL(trace) << "Command: " << shell_command;

  int rc = system(shell_command);

  if (rc > 0) {
    BOOST_LOG_TRIVIAL(error) << "Failed to convert call recording, see above error. Make sure you have sox and fdkaac installed.";
    return -1;
  } else {
    BOOST_LOG_TRIVIAL(trace) << "Finished converting call";
  }
  return nchars;
}

int create_call_json(Call_Data_t& call_info) {
  // Create call JSON, write it to disk, and pass back a json object to call_info

  // Using nlohmann::ordered_json to preserve the previous order
  // Bools are stored as 0 or 1 as in previous versions
  // Call length is rounded up to the nearest second as in previous versions
  // Time stored in fractional seconds will omit trailing zeroes per json spec (1.20 -> 1.2)
  nlohmann::ordered_json json_data =
      {
          {"freq", int(call_info.freq)},
          {"freq_error", int(call_info.freq_error)},
          {"signal", int(call_info.signal)},
          {"noise", int(call_info.noise)},
          {"source_num", int(call_info.source_num)},
          {"recorder_num", int(call_info.recorder_num)},
          {"tdma_slot", int(call_info.tdma_slot)},
          {"phase2_tdma", int(call_info.phase2_tdma)},
          {"start_time", call_info.start_time},
          {"stop_time", call_info.stop_time},
          {"emergency", int(call_info.emergency)},
          {"priority", call_info.priority},
          {"mode", int(call_info.mode)},
          {"duplex", int(call_info.duplex)},
          {"encrypted",int(call_info.encrypted)},
          {"call_length", int(std::round(call_info.length))},
          {"talkgroup", call_info.talkgroup},
          {"talkgroup_tag", call_info.talkgroup_alpha_tag},
          {"talkgroup_description", call_info.talkgroup_description},
          {"talkgroup_group_tag", call_info.talkgroup_tag},
          {"talkgroup_group", call_info.talkgroup_group},
          {"color_code", call_info.color_code},
          {"audio_type", call_info.audio_type},
          {"short_name", call_info.short_name}
        };
  // Add any patched talkgroups
  if (call_info.patched_talkgroups.size() > 1) {
    BOOST_FOREACH (auto &TGID, call_info.patched_talkgroups) {
      json_data["patched_talkgroups"] += int(TGID);
    }
  }
  // Add frequencies / IMBE errors
  for (std::size_t i = 0; i < call_info.transmission_error_list.size(); i++) {
    json_data["freqList"] += {
        {"freq", int(call_info.freq)},
        {"time", call_info.transmission_error_list[i].time},
        {"pos", call_info.transmission_error_list[i].position},
        {"len", call_info.transmission_error_list[i].total_len},
        {"error_count", int(call_info.transmission_error_list[i].error_count)},
        {"spike_count", int(call_info.transmission_error_list[i].spike_count)}};
  }
  // Add sources / tags
  for (std::size_t i = 0; i < call_info.transmission_source_list.size(); i++) {
    json_data["srcList"] += {
        {"src", int(call_info.transmission_source_list[i].source)},
        {"time", call_info.transmission_source_list[i].time},
        {"pos", call_info.transmission_source_list[i].position},
        {"emergency", int(call_info.transmission_source_list[i].emergency)},
        {"signal_system", call_info.transmission_source_list[i].signal_system},
        {"tag", call_info.transmission_source_list[i].tag}};
  }
  // Add created JSON to call_info
  call_info.call_json = json_data;

  // Output the JSON status file
  std::ofstream json_file(call_info.status_filename);
  if (json_file.is_open()) {
    // Write the JSON to disk, indented 2 spaces per level
    json_file << json_data.dump(2);
    return 0;
  } else {
    std::string loghdr = log_header( call_info.short_name, call_info.call_num, call_info.talkgroup_display , call_info.freq);
    BOOST_LOG_TRIVIAL(error) << loghdr << "\033[0m\tUnable to create JSON file: " << call_info.status_filename;
    return 1;
  }
}

bool checkIfFile(std::string filePath) {
  try {
    // Create a Path object from given path string
    boost::filesystem::path pathObj(filePath);
    // Check if path exists and is of a regular file
    if (boost::filesystem::exists(pathObj) && boost::filesystem::is_regular_file(pathObj))
      return true;
  } catch (boost::filesystem::filesystem_error &e) {
    BOOST_LOG_TRIVIAL(error) << e.what() << std::endl;
  }
  return false;
}

void remove_call_files(Call_Data_t call_info, bool plugin_failure=false) {

  if (plugin_failure) {
    std::string loghdr = log_header( call_info.short_name, call_info.call_num, call_info.talkgroup_display , call_info.freq);
    switch (call_info.archive_files_on_failure) {
      case true:
        BOOST_LOG_TRIVIAL(error) << loghdr << "Upload failed after " << call_info.retry_attempt << " attempts - " <<  Color::GRN << "Archiving files" << Color::RST;
        break;
      case false:
        BOOST_LOG_TRIVIAL(error) << loghdr << "Upload failed after " << call_info.retry_attempt << " attempts - " << Color::RED << "Removing files" << Color::RST;
        break;
    }
  }

  if (call_info.audio_archive || (plugin_failure && call_info.archive_files_on_failure)) {
    if (call_info.transmission_archive) {
      // if the files are being archived, move them to the capture directory
      for (std::vector<Transmission>::iterator it = call_info.transmission_list.begin(); it != call_info.transmission_list.end(); ++it) {
        Transmission t = *it;

        // Only move transmission wavs if they exist
        if (checkIfFile(t.filename)) {

          // Prevent "boost::filesystem::copy_file: Invalid cross-device link" errors by using std::filesystem if boost < 1.76
          // This issue exists for old boost versions OR 5.x kernels
          #if (BOOST_VERSION/100000) == 1 && ((BOOST_VERSION/100)%1000) < 76
            fs::path target_file = fs::path(fs::path(call_info.filename ).replace_filename(fs::path(t.filename).filename()));
            fs::path transmission_file = t.filename;
            fs::copy_file(transmission_file, target_file);
          #else
            boost::filesystem::path target_file = boost::filesystem::path(fs::path(call_info.filename ).replace_filename(fs::path(t.filename).filename()));
            boost::filesystem::path transmission_file = t.filename;
            boost::filesystem::copy_file(transmission_file, target_file);
          #endif
        //boost::filesystem::path target_file = boost::filesystem::path(call_info.filename).replace_filename(transmission_file.filename()); // takes the capture dir from the call file and adds the transmission filename to it
        }

      }
    }

    // remove the transmission files from the temp directory
    for (std::vector<Transmission>::iterator it = call_info.transmission_list.begin(); it != call_info.transmission_list.end(); ++it) {
      Transmission t = *it;
      if (checkIfFile(t.filename)) {
        remove(t.filename);
      }
    }



  } else {

    if (checkIfFile(call_info.filename)) {
      remove(call_info.filename);
    }
    if (checkIfFile(call_info.converted)) {
      remove(call_info.converted);
    }
    for (std::vector<Transmission>::iterator it = call_info.transmission_list.begin(); it != call_info.transmission_list.end(); ++it) {
      Transmission t = *it;
      if (checkIfFile(t.filename)) {
        remove(t.filename);
      }
    }

  }

  if (!call_info.call_log && !(plugin_failure && call_info.archive_files_on_failure)) {
    if (checkIfFile(call_info.status_filename)) {
      remove(call_info.status_filename);
    }
  }
}

Call_Data_t upload_call_worker(Call_Data_t call_info) {
  int result;

  if (call_info.status == INITIAL) {
    std::stringstream shell_command;
    std::string shell_command_string;
    std::string files;

    struct stat statbuf;
    // loop through the transmission list, pull in things to fill in totals for call_info
    // Using a for loop with iterator
    for (std::vector<Transmission>::iterator it = call_info.transmission_list.begin(); it != call_info.transmission_list.end(); ++it) {
      Transmission t = *it;

      if (stat(t.filename, &statbuf) == 0)
      {
          files.append(t.filename);
          files.append(" ");
      }
      else
      {
          BOOST_LOG_TRIVIAL(error) << "Somehow, " << t.filename << " doesn't exist, not attempting to provide it to sox";
      }
    }

    combine_wav(files, call_info.filename);

    result = create_call_json(call_info);

    if (result < 0) {
      call_info.status = FAILED;
      return call_info;
    }

    if (call_info.compress_wav) {
      // TR records files as .wav files. They need to be compressed before being upload to online services.

      char *talkgroup_title;
      if (call_info.talkgroup_alpha_tag.length() > 0) {
        talkgroup_title = (char *)call_info.talkgroup_alpha_tag.c_str();
      } else {
        talkgroup_title = (char *)std::to_string(call_info.talkgroup).c_str();
      }

      time_t start_time = static_cast<time_t>(call_info.start_time);
      result = convert_media(call_info.filename, call_info.converted, std::ctime(&start_time), call_info.short_name.c_str(), talkgroup_title);

      if (result < 0) {
        call_info.status = FAILED;
        return call_info;
      }
    }

    // Handle the Upload Script, if set
    if (call_info.upload_script.length() != 0) {
      shell_command << call_info.upload_script << " " << call_info.filename << " " << call_info.status_filename << " " << call_info.converted;
      shell_command_string = shell_command.str();
      std::string loghdr = log_header( call_info.short_name, call_info.call_num, call_info.talkgroup_display , call_info.freq);
      BOOST_LOG_TRIVIAL(info) << loghdr << "\033[0m\tRunning upload script: " << shell_command_string;

      result = system(shell_command_string.c_str());
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


// static int rec_counter=0;
Call_Data_t Call_Concluder::create_base_filename(Call *call, Call_Data_t call_info) {
  char base_filename[255];
  time_t work_start_time = call->get_start_time();
  std::stringstream base_path_stream;
  tm *ltm = localtime(&work_start_time);
  // Found some good advice on Streams and Strings here: https://blog.sensecodons.com/2013/04/dont-let-stdstringstreamstrcstr-happen.html
  base_path_stream << call->get_capture_dir() << "/" << call->get_short_name() << "/" << 1900 + ltm->tm_year << "/" << 1 + ltm->tm_mon << "/" << ltm->tm_mday;
  std::string base_path_string = base_path_stream.str();
  boost::filesystem::create_directories(base_path_string);

  int nchars;

  if (call->get_tdma_slot() == -1) {
    nchars = snprintf(base_filename, 255, "%s/%ld-%ld_%.0f", base_path_string.c_str(), call->get_talkgroup(), work_start_time, call->get_freq());
  } else {
    // this is for the case when it is a P25P2 TDMA or DMR recorder and 2 wav files are created, the slot is needed to keep them separate.
    nchars = snprintf(base_filename, 255, "%s/%ld-%ld_%.0f.%d", base_path_string.c_str(), call->get_talkgroup(), work_start_time, call->get_freq(), call->get_tdma_slot());
  }
  if (nchars >= 255) {
    BOOST_LOG_TRIVIAL(error) << "Call: Path longer than 255 charecters";
  }
  snprintf(call_info.filename, 300, "%s-call_%lu.wav", base_filename, call->get_call_num());
  snprintf(call_info.status_filename, 300, "%s-call_%lu.json", base_filename, call->get_call_num());
  snprintf(call_info.converted, 300, "%s-call_%lu.m4a", base_filename, call->get_call_num());

  return call_info;
}


Call_Data_t Call_Concluder::create_call_data(Call *call, System *sys, Config config) {
  Call_Data_t call_info;
  double total_length = 0;

  call_info = create_base_filename(call, call_info);

  call_info.status = INITIAL;
  call_info.process_call_time = time(0);
  call_info.retry_attempt = 0;
  call_info.error_count = 0;
  call_info.spike_count = 0;
  call_info.freq = call->get_freq();
  call_info.freq_error = call->get_freq_error();
  call_info.signal = call->get_signal();
  call_info.noise = call->get_noise();
  call_info.recorder_num = call->get_recorder()->get_num();
  call_info.source_num = call->get_recorder()->get_source()->get_num();
  call_info.encrypted = call->get_encrypted();
  call_info.emergency = call->get_emergency();
  call_info.priority = call->get_priority();
  call_info.mode = call->get_mode();
  call_info.duplex = call->get_duplex();
  call_info.tdma_slot = call->get_tdma_slot();
  call_info.phase2_tdma = call->get_phase2_tdma();
  call_info.transmission_list = call->get_transmissions();
  call_info.sys_num = sys->get_sys_num();
  call_info.short_name = sys->get_short_name();
  call_info.upload_script = sys->get_upload_script();
  call_info.audio_archive = sys->get_audio_archive();
  call_info.transmission_archive = sys->get_transmission_archive();
  call_info.call_log = sys->get_call_log();
  call_info.call_num = call->get_call_num();
  call_info.compress_wav = sys->get_compress_wav();
  call_info.talkgroup = call->get_talkgroup();
  call_info.talkgroup_display = call->get_talkgroup_display();
  call_info.patched_talkgroups = sys->get_talkgroup_patch(call_info.talkgroup);
  call_info.min_transmissions_removed = 0;
  call_info.color_code = 0;

  if (call->get_is_analog()) {
    call_info.audio_type = "analog";
  } else if (call->get_phase2_tdma()) {
    call_info.audio_type = "digital tdma";
  } else {
    call_info.audio_type = "digital";
  }


  // loop through the transmission list, pull in things to fill in totals for call_info
  // Using a for loop with iterator
  for (std::vector<Transmission>::iterator it = call_info.transmission_list.begin(); it != call_info.transmission_list.end();) {
    Transmission t = *it;

    if (t.length < sys->get_min_tx_duration()) {
      if (!call_info.transmission_archive) {
        std::string loghdr = log_header( call_info.short_name, call_info.call_num, call_info.talkgroup_display , call_info.freq);
        BOOST_LOG_TRIVIAL(info) << loghdr << "Removing transmission less than " << sys->get_min_tx_duration() << " seconds. Actual length: " << t.length << ".";
        call_info.min_transmissions_removed++;

        if (checkIfFile(t.filename)) {
          remove(t.filename);
        }
      }

      it = call_info.transmission_list.erase(it);
      continue;
    }

    std::string tag = sys->find_unit_tag(t.source);
    std::string display_tag = "";
    if (tag != "") {
      display_tag = " (\033[0;34m" + tag + "\033[0m)";
    }

    std::stringstream transmission_info;
    std::string loghdr = log_header( call_info.short_name, call_info.call_num, call_info.talkgroup_display , call_info.freq);
    transmission_info << loghdr << "- Transmission src: " << t.source << display_tag << " pos: " << format_time(total_length) << " length: " << format_time(t.length);

    if (t.error_count < 1) {
      BOOST_LOG_TRIVIAL(info) << transmission_info.str();
    } else {
      BOOST_LOG_TRIVIAL(info) << transmission_info.str() << "\033[0;31m errors: " << t.error_count << " spikes: " << t.spike_count << "\033[0m";
    }

    if (it == call_info.transmission_list.begin()) {
      call_info.start_time = t.start_time;
    }

    if (std::next(it) == call_info.transmission_list.end()) {
      call_info.stop_time = t.stop_time;
    }

    if (call_info.color_code == -1 && t.color_code != -1) {
      call_info.color_code = t.color_code;
      if (call_info.color_code != t.color_code) {
        BOOST_LOG_TRIVIAL(warning) << loghdr << "Call has multiple Color Codes - previous Transmission Color Code: " << call_info.color_code << " current Transmission Color Code: " << t.color_code;
      }
    }

    if (call_info.talkgroup != t.talkgroup) {
      BOOST_LOG_TRIVIAL(warning) << loghdr << "Transmission has a different Talkgroup than Call - Call Talkgroup: " << call_info.talkgroup << " Transmission Talkgroup: " << t.talkgroup;
      call_info.talkgroup = t.talkgroup;
    }


    Call_Source call_source = {t.source, t.start_time, total_length, false, "", tag};
    Call_Error call_error = {t.start_time, total_length, t.length, t.error_count, t.spike_count};
    call_info.error_count = call_info.error_count + t.error_count;
    call_info.spike_count = call_info.spike_count + t.spike_count;
    call_info.transmission_source_list.push_back(call_source);
    call_info.transmission_error_list.push_back(call_error);

    total_length = total_length + t.length;
    it++;
  }



  Talkgroup *tg = sys->find_talkgroup(call_info.talkgroup);
  if (tg != NULL) {
    call_info.talkgroup_tag = tg->tag;
    call_info.talkgroup_alpha_tag = tg->alpha_tag;
    call_info.talkgroup_description = tg->description;
    call_info.talkgroup_group = tg->group;
  } else {
    call_info.talkgroup_tag = "";
    call_info.talkgroup_alpha_tag = "";
    call_info.talkgroup_description = "";
    call_info.talkgroup_group = "";
  }

  call_info.archive_files_on_failure = config.archive_files_on_failure;
  call_info.length = total_length;

  return call_info;
}

void Call_Concluder::conclude_call(Call *call, System *sys, Config config) {
  Call_Data_t call_info = create_call_data(call, sys, config);

  std::string loghdr = log_header( call_info.short_name, call_info.call_num, call_info.talkgroup_display , call_info.freq);
  if(call->get_state() == MONITORING && call->get_monitoring_state() == SUPERSEDED){
    BOOST_LOG_TRIVIAL(info) << loghdr << "Call has been superseded. Removing files.";
    remove_call_files(call_info);
    return;
  }
  else if (call_info.transmission_list.size()== 0 && call_info.min_transmissions_removed == 0) {
    BOOST_LOG_TRIVIAL(error) << loghdr << "No Transmissions were recorded!";
    return;
  }
  else if (call_info.transmission_list.size() == 0 && call_info.min_transmissions_removed > 0) {
    BOOST_LOG_TRIVIAL(info) << loghdr << "No Transmissions were recorded! " << call_info.min_transmissions_removed << " transmissions less than " << sys->get_min_tx_duration() << " seconds were removed.";
    return;
  }

  if (call_info.length <= sys->get_min_duration()) {
    BOOST_LOG_TRIVIAL(info) << loghdr << "Call length: " << call_info.length << " is less than min duration: " << sys->get_min_duration();
    remove_call_files(call_info);
    return;
  }


  call_data_workers.push_back(std::async(std::launch::async, upload_call_worker, call_info));
}

void Call_Concluder::manage_call_data_workers() {
  for (std::list<std::future<Call_Data_t>>::iterator it = call_data_workers.begin(); it != call_data_workers.end();) {

    if (it->wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      Call_Data_t call_info = it->get();

      if (call_info.status == RETRY) {
        call_info.retry_attempt++;
        time_t start_time = call_info.start_time;
        std::string loghdr = log_header( call_info.short_name, call_info.call_num, call_info.talkgroup_display , call_info.freq);

        if (call_info.retry_attempt > Call_Concluder::MAX_RETRY) {
          remove_call_files(call_info, true);
          BOOST_LOG_TRIVIAL(error) << loghdr << "Failed to conclude call - " << std::put_time(std::localtime(&start_time), "%c %Z");
        } else {
          long jitter = rand() % 10;
          long backoff = ((1 << call_info.retry_attempt) * 60) + jitter;
          call_info.process_call_time = time(0) + backoff;
          retry_call_list.push_back(call_info);
          BOOST_LOG_TRIVIAL(error) << loghdr << std::put_time(std::localtime(&start_time), "%c %Z") << " retry attempt " << call_info.retry_attempt << " in " << backoff << "s\t retry queue: " << retry_call_list.size() << " calls";
        }
      }
      it = call_data_workers.erase(it);
    } else {
      it++;
    }
  }
  for (std::list<Call_Data_t>::iterator it = retry_call_list.begin(); it != retry_call_list.end();) {
    Call_Data_t call_info = *it;

    if (call_info.process_call_time <= time(0)) {
      call_data_workers.push_back(std::async(std::launch::async, upload_call_worker, call_info));
      it = retry_call_list.erase(it);
    } else {
      it++;
    }
  }
}
