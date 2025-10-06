#include "call_concluder.h"
#include "../plugin_manager/plugin_manager.h"
#include <boost/filesystem.hpp>
#include <filesystem>
#include <algorithm>
#include <cstdint>
#include <iomanip>

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
          {"start_time_ms", call_info.start_time_ms},
          {"stop_time_ms", call_info.stop_time_ms},
          {"emergency", int(call_info.emergency)},
          {"priority", call_info.priority},
          {"mode", int(call_info.mode)},
          {"duplex", int(call_info.duplex)},
          {"encrypted",int(call_info.encrypted)},
          {"call_length", int(std::round(call_info.length))},
          {"call_length_ms", call_info.call_length_ms},
          {"talkgroup", call_info.talkgroup},
          {"talkgroup_tag", call_info.talkgroup_alpha_tag},
          {"talkgroup_description", call_info.talkgroup_description},
          {"talkgroup_group_tag", call_info.talkgroup_tag},
          {"talkgroup_group", call_info.talkgroup_group},
          {"audio_type", call_info.audio_type},
          {"short_name", call_info.short_name}};
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
    BOOST_LOG_TRIVIAL(error) << loghdr << "C\033[0m \t Unable to create JSON file: " << call_info.status_filename;
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

void remove_call_files(Call_Data_t call_info) {

  if (!call_info.audio_archive) {
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
  } else {
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
  }

  if (!call_info.call_log) {
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
      BOOST_LOG_TRIVIAL(info) << loghdr << "C\033[0m \t Running upload script: " << shell_command_string;

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
  const std::int64_t start_ms = call->get_start_time_ms();
  time_t work_start_time = static_cast<time_t>(start_ms / 1000);
  tm *ltm = localtime(&work_start_time);

  boost::filesystem::path base_path =
      boost::filesystem::path(call->get_capture_dir()) /
      call->get_short_name() /
      boost::lexical_cast<std::string>(1900 + ltm->tm_year) /
      boost::lexical_cast<std::string>(1 + ltm->tm_mon) /
      boost::lexical_cast<std::string>(ltm->tm_mday);

  boost::filesystem::create_directories(base_path);

  // Seconds.milliseconds from call start_time_ms
  const long long sec   = start_ms / 1000;
  const int       milli = static_cast<int>(start_ms % 1000);

  std::ostringstream ts;
  ts << sec << '.' << std::setw(3) << std::setfill('0') << milli;

  char base_filename[255];
  if (call->get_tdma_slot() == -1) {
    std::snprintf(base_filename, sizeof(base_filename),
                  "%s/%ld-%s_%.0f",
                  base_path.string().c_str(),
                  call->get_talkgroup(),
                  ts.str().c_str(),
                  call->get_freq());
  } else {
    std::snprintf(base_filename, sizeof(base_filename),
                  "%s/%ld-%s_%.0f.%d",
                  base_path.string().c_str(),
                  call->get_talkgroup(),
                  ts.str().c_str(),
                  call->get_freq(),
                  call->get_tdma_slot());
  }

  std::snprintf(call_info.filename,         sizeof(call_info.filename),         "%s-call_%lu.wav", base_filename, call->get_call_num());
  std::snprintf(call_info.status_filename,  sizeof(call_info.status_filename),  "%s-call_%lu.json", base_filename, call->get_call_num());
  std::snprintf(call_info.converted,        sizeof(call_info.converted),        "%s-call_%lu.m4a", base_filename, call->get_call_num());
  return call_info;
}


Call_Data_t Call_Concluder::create_call_data(Call *call, System *sys, Config config) {
  Call_Data_t call_info;

  // ---------- Static metadata ----------
  call_info = create_base_filename(call, call_info);

  call_info.status              = INITIAL;
  call_info.process_call_time   = time(0);
  call_info.retry_attempt       = 0;
  call_info.error_count         = 0;
  call_info.spike_count         = 0;
  call_info.freq                = call->get_freq();
  call_info.freq_error          = call->get_freq_error();
  call_info.signal              = call->get_signal();
  call_info.noise               = call->get_noise();
  call_info.recorder_num        = call->get_recorder()->get_num();
  call_info.source_num          = call->get_recorder()->get_source()->get_num();
  call_info.encrypted           = call->get_encrypted();
  call_info.emergency           = call->get_emergency();
  call_info.priority            = call->get_priority();
  call_info.mode                = call->get_mode();
  call_info.duplex              = call->get_duplex();
  call_info.tdma_slot           = call->get_tdma_slot();
  call_info.phase2_tdma         = call->get_phase2_tdma();
  call_info.transmission_list   = call->get_transmissions();
  call_info.sys_num             = sys->get_sys_num();
  call_info.short_name          = sys->get_short_name();
  call_info.upload_script       = sys->get_upload_script();
  call_info.audio_archive       = sys->get_audio_archive();
  call_info.transmission_archive= sys->get_transmission_archive();
  call_info.call_log            = sys->get_call_log();
  call_info.call_num            = call->get_call_num();
  call_info.compress_wav        = sys->get_compress_wav();
  call_info.talkgroup           = call->get_talkgroup();
  call_info.talkgroup_display   = call->get_talkgroup_display();
  call_info.patched_talkgroups  = sys->get_talkgroup_patch(call_info.talkgroup);
  call_info.min_transmissions_removed = 0;

  if (Talkgroup *tg = sys->find_talkgroup(call->get_talkgroup())) {
    call_info.talkgroup_tag          = tg->tag;
    call_info.talkgroup_alpha_tag    = tg->alpha_tag;
    call_info.talkgroup_description  = tg->description;
    call_info.talkgroup_group        = tg->group;
  } else {
    call_info.talkgroup_tag.clear();
    call_info.talkgroup_alpha_tag.clear();
    call_info.talkgroup_description.clear();
    call_info.talkgroup_group.clear();
  }

  if (call->get_is_analog()) {
    call_info.audio_type = "analog";
  } else if (call->get_phase2_tdma()) {
    call_info.audio_type = "digital tdma";
  } else {
    call_info.audio_type = "digital";
  }

  // ---------- Aggregate over transmissions (ms-accurate & efficient) ----------
  const double min_tx_s = sys->get_min_tx_duration();  // seconds

  // Reserve to avoid reallocs during push_back
  call_info.transmission_source_list.reserve(call_info.transmission_list.size());
  call_info.transmission_error_list.reserve(call_info.transmission_list.size());

  double        playable_pos_s = 0.0;       // "pos" field is playable timeline
  std::int64_t  audio_sum_ms   = 0;         // sum of segment durations (playable)
  bool          have_any       = false;
  std::int64_t  min_start_ms   = 0;
  std::int64_t  max_stop_ms    = 0;

  for (auto it = call_info.transmission_list.begin();
       it != call_info.transmission_list.end(); /* manual inc */) {

    const Transmission &t = *it;

    // Canonical length from millisecond stamps
    const std::int64_t seg_ms   = std::max<std::int64_t>(0, t.stop_time_ms - t.start_time_ms);
    const double       seg_len_s = seg_ms / 1000.0;

    // Filter short segments using canonical length
    if (seg_len_s < min_tx_s) {
      if (!call_info.transmission_archive) {
        std::string loghdr = log_header(call_info.short_name, call_info.call_num,
                                        call_info.talkgroup_display, call_info.freq);
        BOOST_LOG_TRIVIAL(info) << loghdr << "Removing transmission less than "
                                << min_tx_s
                                << " seconds. Actual length: " << seg_len_s << ".";
        call_info.min_transmissions_removed++;
        if (checkIfFile(t.filename)) {
          remove(t.filename);
        }
      }
      it = call_info.transmission_list.erase(it);
      continue;
    }

    // Track true wall-clock window [min start, max stop]
    if (!have_any) {
      have_any     = true;
      min_start_ms = t.start_time_ms;
      max_stop_ms  = t.stop_time_ms;
    } else {
      if (t.start_time_ms < min_start_ms) min_start_ms = t.start_time_ms;
      if (t.stop_time_ms  > max_stop_ms)  max_stop_ms  = t.stop_time_ms;
    }

    // Unit tag (once per segment)
    std::string tag = sys->find_unit_tag(t.source);
    std::string display_tag = tag.empty() ? "" : " (\033[0;34m" + tag + "\033[0m)";

    // Log with canonical length and playable position
    {
      std::stringstream transmission_info;
      std::string loghdr = log_header(call_info.short_name, call_info.call_num,
                                      call_info.talkgroup_display, call_info.freq);
      transmission_info << loghdr << "- Transmission src: " << t.source << display_tag
                        << " pos: "    << format_time(playable_pos_s)
                        << " length: " << format_time(seg_len_s);
      if (t.error_count < 1) {
        BOOST_LOG_TRIVIAL(info) << transmission_info.str();
      } else {
        BOOST_LOG_TRIVIAL(info) << transmission_info.str()
                                << "\033[0;31m errors: " << t.error_count
                                << " spikes: " << t.spike_count << "\033[0m";
      }
    }

    // Build src/error lists aligned to playable timeline
    Call_Source call_source = { t.source, t.start_time, playable_pos_s, false, "", tag };
    Call_Error  call_error  = { t.start_time, playable_pos_s, seg_len_s,
                                t.error_count, t.spike_count };
    call_info.transmission_source_list.push_back(call_source);
    call_info.transmission_error_list.push_back(call_error);

    call_info.error_count += t.error_count;
    call_info.spike_count += t.spike_count;

    playable_pos_s += seg_len_s;
    audio_sum_ms   += seg_ms;

    ++it;
  }

  // ---------- Finalize aggregate timing ----------
  if (have_any) {
    call_info.start_time_ms  = min_start_ms;                   // earliest start (ms)
    call_info.stop_time_ms   = max_stop_ms;                    // latest stop   (ms)
    call_info.start_time     = (time_t)(min_start_ms / 1000);
    call_info.stop_time      = (time_t)(max_stop_ms  / 1000);
    call_info.call_length_ms = audio_sum_ms;                   // playable audio only
    call_info.length         = audio_sum_ms / 1000.0;          // seconds
  } else {
    call_info.length         = 0.0;
    call_info.start_time_ms  = 0;
    call_info.stop_time_ms   = 0;
    call_info.start_time     = 0;
    call_info.stop_time      = 0;
    call_info.call_length_ms = 0;
  }

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
          BOOST_LOG_TRIVIAL(error) << "[" << call_info.short_name << "]\t\033[0;34m" << call_info.call_num << "C\033[0m Failed to conclude call - TG: " << call_info.talkgroup_display << "\t" << std::put_time(std::localtime(&start_time), "%c %Z");
        } else {
          long jitter = rand() % 10;
          long backoff = (2 ^ call_info.retry_attempt * 60) + jitter;
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
