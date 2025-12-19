#include "monitor_systems.h"
#include "recorders/p25_recorder.h"
using namespace std;

volatile sig_atomic_t exit_flag = 0;
int exit_code = EXIT_SUCCESS;

void exit_interupt(int sig) { // can be called asynchronously
  exit_flag = 1;              // set flag
}

uint64_t time_since_epoch_millisec() {
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

bool start_recorder(Call *call, TrunkMessage message, Config &config, System *sys, std::vector<Source *> &sources) {
  Talkgroup *talkgroup = sys->find_talkgroup(call->get_talkgroup());

  bool source_found = false;
  bool recorder_found = false;
  bool override_record_unknown = false;

  Recorder *recorder;
  Recorder *debug_recorder;
  Recorder *sigmf_recorder;

  if (!talkgroup){
    BOOST_FOREACH (auto &TGID, sys->get_talkgroup_patch(call->get_talkgroup())) {  //for each talkgroup in the patch
      if (sys->find_talkgroup(TGID) != NULL){  //if the patched talkgroup is known
        override_record_unknown = true;
        std::string loghdr = log_header( call->get_short_name(), call->get_call_num(), call->get_talkgroup_display(), call->get_freq());
        BOOST_LOG_TRIVIAL(info) << loghdr << "\u001b[33mEnabling recording of TG not in Talkgroup File due to active supergroup patch\u001b[0m ";
      }
    }
  }

  if (!talkgroup && (sys->get_record_unknown() == false) && override_record_unknown == false) {
    call->set_state(MONITORING);
    call->set_monitoring_state(UNKNOWN_TG);
    if (sys->get_hideUnknown() == false) {
      std::string loghdr = log_header( call->get_short_name(), call->get_call_num(), call->get_talkgroup_display(), call->get_freq());
      BOOST_LOG_TRIVIAL(info) << loghdr << "\u001b[33mNot Recording: TG not in Talkgroup File\u001b[0m ";
    }
    return false;
  }

  if (talkgroup) {
    call->set_talkgroup_tag(talkgroup->alpha_tag);
  } else {
    call->set_talkgroup_tag("-");
  }

  if (call->get_encrypted() == true || (talkgroup && (talkgroup->mode.compare("E") == 0 || talkgroup->mode.compare("TE") == 0 || talkgroup->mode.compare("DE") == 0))) {
    if (talkgroup && (talkgroup->mode.compare("E") == 0 || talkgroup->mode.compare("TE") == 0 || talkgroup->mode.compare("DE") == 0)) {
      call->set_encrypted(true);
    }
    
    if (!sys->get_monitorEncrypted()) {
      call->set_state(MONITORING);
      call->set_monitoring_state(ENCRYPTED);
      if (sys->get_hideEncrypted() == false) {
        long unit_id = call->get_current_source_id();
        std::string tag = sys->find_unit_tag(unit_id);
        if (tag != "") {
          tag = " (\033[0;34m" + tag + "\033[0m)";
        }
        std::string loghdr = log_header( sys->get_short_name(), call->get_call_num(), call->get_talkgroup_display(), call->get_freq());
        BOOST_LOG_TRIVIAL(info) << loghdr << "\u001b[31mNot Recording: ENCRYPTED\u001b[0m - src: " << unit_id << tag;
      }
      return false;
    }
  }

  for (vector<Source *>::iterator it = sources.begin(); it != sources.end(); it++) {
    Source *source = *it;

    if ((source->get_min_hz() <= call->get_freq()) &&
        (source->get_max_hz() >= call->get_freq())) {
      source_found = true;

      if (talkgroup) {
        int priority = talkgroup->get_priority();
        BOOST_FOREACH (auto &TGID, sys->get_talkgroup_patch(call->get_talkgroup())) {
          if (sys->find_talkgroup(TGID) != NULL) {
            if (sys->find_talkgroup(TGID)->get_priority() < priority) {
              priority = sys->find_talkgroup(TGID)->get_priority();
              BOOST_LOG_TRIVIAL(info) << "Temporarily increased priority of talkgroup " << call->get_talkgroup() << " to " << sys->find_talkgroup(TGID)->get_priority() << " due to active patch with talkgroup " << TGID;
            }
          }
        }
        if (talkgroup->mode.compare("A") == 0) {
          recorder = source->get_analog_recorder(talkgroup, priority, call);
          call->set_is_analog(true);
        } else {
          recorder = source->get_digital_recorder(talkgroup, priority, call);
        }
      } else {
        std::string loghdr = log_header( call->get_short_name(), call->get_call_num(), call->get_talkgroup_display(), call->get_freq());
        BOOST_LOG_TRIVIAL(info) << loghdr << "TG not in Talkgroup File ";

        // A talkgroup was not found from the talkgroup file.
        // Use an analog recorder if this is a Type II trunk and defaultMode is analog.
        // All other cases use a digital recorder.
        if ((config.default_mode == "analog") && (sys->get_system_type() == "smartnet")) {
          recorder = source->get_analog_recorder(call);
          call->set_is_analog(true);
        } else {
          recorder = source->get_digital_recorder(call);
        }
      }

      if (recorder) {
        if (message.meta.length()) {
          BOOST_LOG_TRIVIAL(trace) << message.meta;
        }

        if (recorder->start(call)) {
          call->set_recorder(recorder);
          call->set_state(RECORDING);
          plugman_setup_recorder(recorder);
          recorder_found = true;
        } else {
          call->set_state(MONITORING);
          // call->set_monitoring_state(NO_SOURCE);
          recorder_found = false;
          return false;
        }
      } else {
        // not recording call either because the priority was too low or no
        // recorders were available
        return false;
      }

      debug_recorder = source->get_debug_recorder();

      if (debug_recorder) {
        debug_recorder->start(call);
        call->set_debug_recorder(debug_recorder);
        call->set_debug_recording(true);
        plugman_setup_recorder(debug_recorder);
        recorder_found = true;
      } else {
        // BOOST_LOG_TRIVIAL(info) << "\tNot debug recording call";
      }

      sigmf_recorder = source->get_sigmf_recorder();

      if (sigmf_recorder) {
        sigmf_recorder->start(call);
        call->set_sigmf_recorder(sigmf_recorder);
        call->set_sigmf_recording(true);
        plugman_setup_recorder(sigmf_recorder);
        recorder_found = true;
      } else {
        // BOOST_LOG_TRIVIAL(info) << "\tNot SIGMF recording call";
      }

      if (recorder_found) {
        // recording successfully started.
        return true;
      }
    }
  }

  if (!source_found) {
    call->set_state(MONITORING);
    call->set_monitoring_state(NO_SOURCE);
    std::string loghdr = log_header( call->get_short_name(), call->get_call_num(), call->get_talkgroup_display(), call->get_freq());
    BOOST_LOG_TRIVIAL(error) << loghdr << "\u001b[36mNot Recording: no source covering Freq\u001b[0m";
    return false;
  }
  return false;
}

void print_status(std::vector<Source *> &sources, std::vector<System *> &systems, std::vector<Call *> &calls) {
  BOOST_LOG_TRIVIAL(info) << "Active Calls: " << calls.size();

  for (vector<Call *>::iterator it = calls.begin(); it != calls.end(); it++) {
    Call *call = *it;
    Recorder *recorder = call->get_recorder();
    std::string loghdr = log_header( call->get_short_name(), call->get_call_num(), call->get_talkgroup_display(), call->get_freq());
    if (call->get_state() == MONITORING) {
      BOOST_LOG_TRIVIAL(info) << loghdr << "Elapsed: " << std::setw(4) << call->elapsed() << " State: " << format_state(call->get_state(), call->get_monitoring_state());
    } else {
      if (call->is_conventional() ) {
        bool is_enabled = call->get_recorder()->is_enabled();
         BOOST_LOG_TRIVIAL(info) << loghdr << "Elapsed: " << std::setw(4) << call->elapsed() << " State: " << format_state(call->get_state()) << " Enabled: " << is_enabled;
      } else {
        BOOST_LOG_TRIVIAL(info) << loghdr << "Elapsed: " << std::setw(4) << call->elapsed() << " State: " << format_state(call->get_state());
      }
    }

    if (recorder) {
        BOOST_LOG_TRIVIAL(info) << "\t[ " << std::setw(2) << recorder->get_num() << " ] State: " << format_state(recorder->get_state());
    }
  }

  BOOST_LOG_TRIVIAL(info) << "Active Patches: ";
  for (std::vector<System *>::iterator it = systems.begin(); it != systems.end(); ++it) {
    System_impl *sys = (System_impl *)*it;

    sys->print_active_talkgroup_patches();
  }

  BOOST_LOG_TRIVIAL(info) << "Control Channel Decode Rates: ";
  for (std::vector<System *>::iterator it = systems.begin(); it != systems.end(); ++it) {
    System_impl *sys = (System_impl *)*it;

    if ((sys->get_system_type() != "conventional") && (sys->get_system_type() != "conventionalP25") && (sys->get_system_type() != "conventionalDMR") && (sys->get_system_type() != "conventionalSIGMF")) {
      BOOST_LOG_TRIVIAL(info) << "[" << sys->get_short_name() << "]\t" << format_freq(sys->get_current_control_channel()) << "\t" << sys->get_decode_rate() << " msg/sec";
      
      if ((sys->get_source()->get_autotune_source()) && (sys->get_system_type() == "p25")) {
        // If control channel source has autotune enabled, perform autotune adjustments and log to console
        autotune_control_channel(sys);
      }
    }
  }

  BOOST_LOG_TRIVIAL(info) << "Recorders: ";

  for (vector<Source *>::iterator it = sources.begin(); it != sources.end(); it++) {
    Source *source = *it;
    source->print_recorders();
  }
}

void manage_conventional_call(Call *call, Config &config) {

  if (call->get_recorder()) {
    // if any recording has happened

    if (call->get_current_length() > 0) {

      BOOST_LOG_TRIVIAL(trace) << "[" << call->get_short_name() << "]\t\033[0;34m" << call->get_call_num() << "C\033[0m Call Length: " << call->get_current_length() << "s\t Idle: " << call->get_recorder()->is_idle() << "\t Squelched: " << call->get_recorder()->is_squelched() << " Idle Count: " << call->get_idle_count();

      // means that the squelch is on and it has stopped recording
      if (call->get_recorder()->is_idle()) {
        // increase the number of periods it has not been recording for
        call->set_noise(call->get_recorder()->get_pwr());
        call->increase_idle_count();
      } else {
        call->set_signal(call->get_recorder()->get_pwr());
        if (call->get_idle_count() > 0) {
          // if it starts recording again, then reset the idle count
          call->reset_idle_count();
        }
      }

      // if no additional recording has happened in the past X periods, stop and open new file
      if (call->get_idle_count() > config.call_timeout) {
        Recorder *recorder = call->get_recorder();
        call->conclude_call();
        call->restart_call();
        if (recorder != NULL) {
          plugman_setup_recorder(recorder);
          plugman_call_start(call);
        }
      } else if ((call->get_current_length() > call->get_system()->get_max_duration()) && (call->get_system()->get_max_duration() > 0)) {
        Recorder *recorder = call->get_recorder();
        call->conclude_call();
        call->restart_call();
        if (recorder != NULL) {
          plugman_setup_recorder(recorder);
          plugman_call_start(call);
        }
      }
    } else if (!call->get_recorder()->is_active()) {
      // P25 Conventional and DMR Recorders need a have the graph unlocked before they can start recording.
      Recorder *recorder = call->get_recorder();
      recorder->start(call);
      call->set_state(RECORDING);
      plugman_call_start(call);
      BOOST_LOG_TRIVIAL(trace) << "[" << call->get_short_name() << "]\t\033[0;34m" << call->get_call_num() << "C\033[0m Starting P25 Convetional Recorder ";

      // plugman_setup_recorder((Recorder *)recorder->get());
    }
  }
}

void manage_calls(Config &config, std::vector<Call *> &calls) {
  bool ended_call = false;
  for (vector<Call *>::iterator it = calls.begin(); it != calls.end();) {
    Call *call = *it;
    State state = call->get_state();
    // Handle Conventional Calls
    if (call->is_conventional()) {
      manage_conventional_call(call, config);
      ++it;
      continue;
    }

    // Handle Trunked Calls

    if ((state == MONITORING) && (call->since_last_update() > config.call_timeout)) {
      ended_call = true;
      it = calls.erase(it);
      delete call;
      continue;
    }

    if (state == RECORDING) {
      Recorder *recorder = call->get_recorder();

      // Stop the call if:
      // - there hasn't been an UPDATE for it on the Control Channel in X seconds AND the recorder hasn't written anything in X seconds

      if ((recorder->since_last_write() > config.call_timeout) && (call->since_last_update() > config.call_timeout)) {
        std::string loghdr = log_header( call->get_short_name(), call->get_call_num(), call->get_talkgroup_display(), call->get_freq());
        BOOST_LOG_TRIVIAL(trace) << loghdr << "\u001b[36m Stopping Call because of Recorder \u001b[0m Rec last write: " << recorder->since_last_write() << " State: " << format_state(recorder->get_state());
        call->conclude_call();
        // The State of the Recorders has changed, so lets send an update
        ended_call = true;
        if (recorder != NULL) {
          plugman_setup_recorder(recorder);
        }
        it = calls.erase(it);
        delete call;
        continue;
      }
    } else if (call->since_last_update() > config.call_timeout) {
      Recorder *recorder = call->get_recorder();
      std::string loghdr = log_header( call->get_short_name(), call->get_call_num(), call->get_talkgroup_display(), call->get_freq());
      BOOST_LOG_TRIVIAL(trace) << loghdr << "\u001b[36m  Call UPDATEs has been inactive for more than " << config.call_timeout << " Sec \u001b[0m Rec last write: " << recorder->since_last_write() << " State: " << format_state(recorder->get_state());
    }
    ++it;
  } // foreach loggers

  if (ended_call) {
    plugman_calls_active(calls);
  }
}

void current_system_status(TrunkMessage message, System *sys) {
  if (sys->update_status(message)) {
    plugman_setup_system(sys);
  }
}

void current_system_sysid(TrunkMessage message, System *sys) {
  if ((sys->get_system_type() == "p25") || (sys->get_system_type() == "conventionalP25")) {
    if (sys->update_sysid(message)) {
      plugman_setup_system(sys);
    }
  }
}

void unit_registration(System *sys, long source_id) {
  plugman_unit_registration(sys, source_id);
}

void unit_deregistration(System *sys, long source_id) {
  plugman_unit_deregistration(sys, source_id);
}

void unit_acknowledge_response(System *sys, long source_id) {
  plugman_unit_acknowledge_response(sys, source_id);
}

void unit_group_affiliation(System *sys, long source_id, long talkgroup_num) {
  plugman_unit_group_affiliation(sys, source_id, talkgroup_num);
}

void unit_data_grant(System *sys, long source_id) {
  plugman_unit_data_grant(sys, source_id);
}

void unit_answer_request(System *sys, long source_id, long talkgroup) {
  plugman_unit_answer_request(sys, source_id, talkgroup);
}

void unit_location(System *sys, long source_id, long talkgroup_num) {
  plugman_unit_location(sys, source_id, talkgroup_num);
}




void handle_call_grant(TrunkMessage message, System *sys, bool grant_message, Config &config, std::vector<Source *> &sources, std::vector<Call *> &calls) {
  bool call_found = false;
  bool duplicate_grant = false;
  bool superseding_grant = false;
  bool recording_started [[maybe_unused]] = false;

  Call *original_call;

  /* Notes: it is possible for 2 Calls to exist for the same talkgroup on different freq. This happens when a Talkgroup starts on a freq
  that current recorder can't retune to. In this case, the current orig Talkgroup reocrder will keep going on the old freq, while a new
  recorder is start on a source that can cover that freq. This makes sure any of the remaining transmission that it is in the buffer
  of the original recorder gets flushed.
  UPDATED: however if we have 2 different talkgroups on the same freq we should do a stop_call on the original call since it is being used by another TG now. This will let the recorder keep
  going until it gets a termination flag.
  */

  // BOOST_LOG_TRIVIAL(info) << "TG: " << message.talkgroup << " sys num: " << message.sys_num << " freq: " << message.freq << " TDMA Slot" << message.tdma_slot << " TDMA: " << message.phase2_tdma;

  unsigned long message_preferredNAC = 0;
  unsigned long call_rfss_site = 0;
  unsigned long sys_rfss_site = 0;

  Talkgroup *message_talkgroup = sys->find_talkgroup(message.talkgroup);
  if (message_talkgroup) {
    message_preferredNAC = message_talkgroup->get_preferredNAC();
  }

  for (vector<Call *>::iterator it = calls.begin(); it != calls.end();) {
    Call *call = *it;

    /* This is for Multi-Site support */
    // Find candidate duplicate calls with the same talkgroup and different multisite-enabled systems
    if (call->get_talkgroup() == message.talkgroup) {
      if (call->get_sys_num() != message.sys_num) {
        if (call->get_system()->get_multiSite() && sys->get_multiSite()) {
          if (call->get_system()->get_wacn() == sys->get_wacn()) {
            // Default mode to match WACN and use RFSS/Site to identify duplicate calls
            sys_rfss_site = sys->get_sys_rfss() * 10000 + sys->get_sys_site_id();
            call_rfss_site = call->get_system()->get_sys_rfss() * 10000 + call->get_system()->get_sys_site_id();
            if ((sys_rfss_site != call_rfss_site) && (call->get_system()->get_multiSiteSystemName() == "")) {
              if (call->get_state() == RECORDING) {

                duplicate_grant = true;
                original_call = call;

                unsigned long call_preferredNAC = 0;
                Talkgroup *call_talkgroup = call->get_system()->find_talkgroup(message.talkgroup);
                if (call_talkgroup) {
                  call_preferredNAC = call_talkgroup->get_preferredNAC();
                }

                // Evaluate superseding grants by comparing call NAC or RFSS-Site against preferred NAC/site in talkgroup .csv
                if ((call_preferredNAC != call->get_system()->get_nac()) && (message_preferredNAC == sys->get_nac())) {
                  superseding_grant = true;
                } else if ((call_preferredNAC != call_rfss_site) && (message_preferredNAC == sys_rfss_site)) {
                  superseding_grant = true;
                }
              }
            }

            // Secondary mode to match multiSiteSystemName and use multiSiteSystemNumber.
            // If a multiSiteSystemName has been manually entered;
            // We already know that Call's system number does not match the message system number.
            // In this case, we check that the multiSiteSystemName is present, and that the Call and System multiSiteSystemNames are the same.
            else if ((call->get_system()->get_multiSiteSystemName() != "") && (call->get_system()->get_multiSiteSystemName() == sys->get_multiSiteSystemName())) {
              if (call->get_state() == RECORDING) {

                duplicate_grant = true;
                original_call = call;

                unsigned long call_preferredNAC = 0;
                Talkgroup *call_talkgroup = call->get_system()->find_talkgroup(message.talkgroup);
                if (call_talkgroup) {
                  call_preferredNAC = call_talkgroup->get_preferredNAC();
                }

                if ((call->get_system()->get_multiSiteSystemNumber() != 0) && (sys->get_multiSiteSystemNumber() != 0)) {
                  if ((call_preferredNAC != call->get_system()->get_multiSiteSystemNumber()) && (message_preferredNAC == sys->get_multiSiteSystemNumber())) {
                    superseding_grant = true;
                  }
                }
              }
            }
          }
        }
      }
    }

    if ((call->get_talkgroup() == message.talkgroup) && (call->get_sys_num() == message.sys_num) && (call->get_freq() == message.freq) && (call->get_tdma_slot() == message.tdma_slot) && (call->get_phase2_tdma() == message.phase2_tdma)) {
      call_found = true;
      bool source_updated = call->update(message);
      if (source_updated) {
        plugman_call_start(call);
      }
    }

    // There is an existing call on freq and slot that the new call will be started on. We should stop the older call. The older recorder will
    // keep writing to the file until it hits a termination flag, so no packets should be dropped.
    if ((call->get_state() == RECORDING) && (call->get_talkgroup() != message.talkgroup) && (call->get_sys_num() == message.sys_num) && (call->get_freq() == message.freq) && (call->get_tdma_slot() == message.tdma_slot) && (call->get_phase2_tdma() == message.phase2_tdma)) {
      Recorder *recorder = call->get_recorder();
      string recorder_state = "UNKNOWN";
      if (recorder != NULL) {
        recorder_state = format_state(recorder->get_state());
      }
      std::string loghdr = log_header( call->get_short_name(), call->get_call_num(), call->get_talkgroup_display(), call->get_freq());
      BOOST_LOG_TRIVIAL(trace) << loghdr << "\u001b[36mShould be Stopping RECORDING call, Recorder State: " << recorder_state << " RX overlapping TG message Freq, TG:" << message.talkgroup << "\u001b[0m";
    }

    it++;
  }

  if (!call_found) {
    Call *call = Call::make(message, sys, config);

    Talkgroup *talkgroup = sys->find_talkgroup(call->get_talkgroup());

    if (talkgroup) {
      call->set_talkgroup_tag(talkgroup->alpha_tag);
    } else {
      call->set_talkgroup_tag("-");
    }

    boost::format original_call_data;
    boost::format grant_call_data;

    if ((superseding_grant) || (duplicate_grant)) {
      if (original_call->get_system()->get_multiSiteSystemName() == "") {
        original_call_data = boost::format("\u001b[34m%sC\u001b[0m %X/%s-%s ") % original_call->get_call_num() % original_call->get_system()->get_wacn() % original_call->get_system()->get_sys_rfss() % original_call->get_system()->get_sys_site_id();
        grant_call_data = boost::format("\u001b[34m%sC\u001b[0m %X/%s-%s ") % call->get_call_num() % sys->get_wacn() % sys->get_sys_rfss() % +sys->get_sys_site_id();
      } else {
        original_call_data = boost::format("\u001b[34m%sC\u001b[0m %s/%s ") % original_call->get_call_num() % original_call->get_system()->get_multiSiteSystemName() % original_call->get_system()->get_multiSiteSystemNumber();
        grant_call_data = boost::format("\u001b[34m%sC\u001b[0m %s/%s ") % call->get_call_num() % sys->get_multiSiteSystemName() % sys->get_multiSiteSystemNumber();
      }
    }
    if (superseding_grant) {
      std::string loghdr = log_header( call->get_short_name(), call->get_call_num(), call->get_talkgroup_display(), call->get_freq());

      BOOST_LOG_TRIVIAL(info) << loghdr << "\u001b[36mSuperseding Grant\u001b[0m - Stopping original call: " << original_call_data << "- Superseding call: " << grant_call_data;
      // Attempt to start a new call on the preferred NAC.
      recording_started = start_recorder(call, message, config, sys, sources);

      if (recording_started) {
        // Clean up the original call.
        original_call->set_state(MONITORING);
        original_call->set_monitoring_state(SUPERSEDED);
        original_call->conclude_call();
      } else {

        BOOST_LOG_TRIVIAL(info) << loghdr << "\u001b[36mCould not start Superseding recorder.\u001b[0m Continuing original call: " << original_call->get_call_num() << "C";
      }
    } else if (duplicate_grant) {
      std::string loghdr = log_header( call->get_short_name(), call->get_call_num(), call->get_talkgroup_display(), call->get_freq());
      call->set_state(MONITORING);
      call->set_monitoring_state(DUPLICATE);
      BOOST_LOG_TRIVIAL(info) << loghdr << "\u001b[36mDuplicate Grant\u001b[0m - Not recording: " << grant_call_data << "- Original call: " << original_call_data;
    } else {
      recording_started = start_recorder(call, message, config, sys, sources);
      if (recording_started && !grant_message) {
        std::string loghdr = log_header( call->get_short_name(), call->get_call_num(), call->get_talkgroup_display(), call->get_freq());
        BOOST_LOG_TRIVIAL(info) << loghdr << "\u001b[36mThis was an UPDATE\u001b[0m";
      }
    }
    calls.push_back(call);
    plugman_call_start(call);
    plugman_calls_active(calls);
  }
}

void handle_call_update(TrunkMessage message, System *sys, std::vector<Call *> &calls) {
  bool call_found = false;

  /* Notes: it is possible for 2 Calls to exist for the same talkgroup on different freq. This happens when a Talkgroup starts on a freq
  that current recorder can't retune to. In this case, the current orig Talkgroup reocrder will keep going on the old freq, while a new
  recorder is start on a source that can cover that freq. This makes sure any of the remaining transmission that it is in the buffer
  of the original recorder gets flushed.
  UPDATED: however if we have 2 different talkgroups on the same freq we should do a stop_call on the original call since it is being used by another TG now. This will let the recorder keep
  going until it gets a termination flag.
  */

  for (vector<Call *>::iterator it = calls.begin(); it != calls.end(); ++it) {
    Call *call = *it;

    // BOOST_LOG_TRIVIAL(info) << "TG: " << call->get_talkgroup() << " | " << message.talkgroup << " sys num: " << call->get_sys_num() << " | " << message.sys_num << " freq: " << call->get_freq() << " | " << message.freq << " TDMA Slot" << call->get_tdma_slot() << " | " << message.tdma_slot << " TDMA: " << call->get_phase2_tdma() << " | " << message.phase2_tdma;
    if ((call->get_talkgroup() == message.talkgroup) && (call->get_sys_num() == message.sys_num) && (call->get_freq() == message.freq) && (call->get_tdma_slot() == message.tdma_slot) && (call->get_phase2_tdma() == message.phase2_tdma)) {
      call_found = true;

      if (message.encrypted) {
        call->set_encrypted(true);
      } else {
        Talkgroup *talkgroup = sys->find_talkgroup(message.talkgroup);
        if (talkgroup && (talkgroup->mode.compare("E") == 0 || talkgroup->mode.compare("TE") == 0 || talkgroup->mode.compare("DE") == 0)) {
          call->set_encrypted(true);
        }
      }

      bool source_updated = call->update(message);
      if (source_updated) {
        plugman_call_start(call);
      }
    }
  }

  if (!call_found) {
    // Note: some calls maybe removed before the UPDATEs stop on the trunking channel if there is some GAP in the updates.
    // BOOST_LOG_TRIVIAL(info) << "Call not found for UPDATE mesg - either we missed GRANT or removed Call too soon\tFreq: " << format_freq(message.freq) << "\tTG:" << message.talkgroup << "\tSource: " << message.source << "\tSys Num: " << message.sys_num << "\tTDMA Slot: " << message.tdma_slot << "\tTDMA: " << message.phase2_tdma;
  }
}

void handle_message(std::vector<TrunkMessage> messages, System *sys, Config &config, std::vector<Source *> &sources, std::vector<Call *> &calls, gr::top_block_sptr &tb) {
  for (std::vector<TrunkMessage>::iterator it = messages.begin(); it != messages.end(); it++) {
    TrunkMessage message = *it;

    switch (message.message_type) {
    case GRANT:
      handle_call_grant(message, sys, true, config, sources, calls);
      break;

    case UPDATE:
      if (config.new_call_from_update) {
        // Treat UPDATE as a GRANT and start a new call if we don't have one for this TG
        handle_call_grant(message, sys, false, config, sources, calls);
      } else {
        // Treat UPDATE as an UPDATE and only update existing calls
        handle_call_update(message, sys, calls);
      }
      break;

    case UU_V_GRANT:
      if (config.record_uu_v_calls) {
        handle_call_grant(message, sys, true, config, sources, calls);
      }
      break;

    case UU_V_UPDATE:
      if (config.record_uu_v_calls) {
        handle_call_update(message, sys, calls);
      }
      break;

    case CONTROL_CHANNEL:
      sys->add_control_channel(message.freq);
      break;

    case REGISTRATION:
      unit_registration(sys, message.source);
      break;

    case DEREGISTRATION:
      unit_deregistration(sys, message.source);
      break;

    case AFFILIATION:
      unit_group_affiliation(sys, message.source, message.talkgroup);
      break;

    case SYSID:
      current_system_sysid(message, sys);
      break;

    case STATUS:
      current_system_status(message, sys);
      break;

    case LOCATION:
      unit_location(sys, message.source, message.talkgroup);
      break;

    case ACKNOWLEDGE:
      unit_acknowledge_response(sys, message.source);
      break;

    case PATCH_ADD:
      sys->update_active_talkgroup_patches(message.patch_data);
      break;
    case PATCH_DELETE:
      sys->delete_talkgroup_patch(message.patch_data);
      break;

    case DATA_GRANT:
      unit_data_grant(sys, message.source);
      break;

    case UU_ANS_REQ:
      unit_answer_request(sys, message.source, message.talkgroup);
      break;

    case INVALID_CC_MESSAGE:
    {
      //Do not count messages that aren't valid TSBK or MBTs.
      int msg_count = sys->get_message_count();
      if(msg_count > 1){
        sys->set_message_count(msg_count - 1);
      }
      break;
    }

    case TDULC:
      retune_system(sys,tb,sources);
      break;

    case UNKNOWN:
      break;
    }
  }
}

void retune_system(System *sys, gr::top_block_sptr &tb, std::vector<Source *> &sources) {
  System_impl *system = (System_impl *)sys;
  bool source_found = false;
  Source *current_source = system->get_source();
  double control_channel_freq = system->get_next_control_channel();

  BOOST_LOG_TRIVIAL(error) << "[" << system->get_short_name() << "] Retuning to Control Channel: " << format_freq(control_channel_freq);

  if ((current_source->get_min_hz() <= control_channel_freq) &&
      (current_source->get_max_hz() >= control_channel_freq)) {
    source_found = true;
    BOOST_LOG_TRIVIAL(info) << "\t - System Source " << current_source->get_num() << " - Min Freq: " << format_freq(current_source->get_min_hz()) << " Max Freq: " << format_freq(current_source->get_max_hz());
    // The source can cover the System's control channel, break out of the
    // For Loop
    if (system->get_system_type() == "smartnet") {
      system->smartnet_trunking->tune_freq(control_channel_freq);
      system->smartnet_trunking->reset();
    } else if (system->get_system_type() == "p25") {
      system->p25_trunking->tune_freq(control_channel_freq);
    } else {
      BOOST_LOG_TRIVIAL(error) << "\t - Unknown system type for Retune";
    }
  } else {
    for (vector<Source *>::iterator src_it = sources.begin(); src_it != sources.end(); src_it++) {
      Source *source = *src_it;

      if ((source->get_min_hz() <= control_channel_freq) &&
          (source->get_max_hz() >= control_channel_freq)) {
        source_found = true;
        BOOST_LOG_TRIVIAL(info) << "\t - System Source " << source->get_num() << " - Min Freq: " << format_freq(source->get_min_hz()) << " Max Freq: " << format_freq(source->get_max_hz());

        if (system->get_system_type() == "smartnet") {
          system->set_source(source);
          // We must lock the flow graph in order to disconnect and reconnect blocks
          tb->lock();
          tb->disconnect(current_source->get_src_block(), 0, system->smartnet_trunking, 0);
          system->smartnet_trunking = make_smartnet_trunking(control_channel_freq, source->get_center(), source->get_rate(), system->get_msg_queue(), system->get_sys_num());
          tb->connect(source->get_src_block(), 0, system->smartnet_trunking, 0);
          tb->unlock();
          system->smartnet_trunking->reset();
        } else if (system->get_system_type() == "p25") {
          system->set_source(source);
          // We must lock the flow graph in order to disconnect and reconnect blocks
          tb->stop();
          tb->disconnect(current_source->get_src_block(), 0, system->p25_trunking, 0);
          system->p25_trunking = make_p25_trunking(control_channel_freq, source->get_center(), source->get_rate(), system->get_msg_queue(), system->get_qpsk_mod(), system->get_sys_num());
          tb->connect(source->get_src_block(), 0, system->p25_trunking, 0);
          tb->start();
        } else {
          BOOST_LOG_TRIVIAL(error) << "\t - Unkown system type for Retune";
        }

        // break out of the For Loop
        break;
      }
    }
  }
  if (!source_found) {
    BOOST_LOG_TRIVIAL(error) << "\t - Unable to retune System control channel, freq not covered by any source.";
  } else {
    if ((system->get_source()->get_autotune_source()) && (system->get_system_type() == "p25")) {
      // If control channel source has autotune enabled, perform adjustments after retune completes
      // Don't store measurements since the control channel recorder just started
      autotune_control_channel(system, false);
    }
  }
}

void check_message_count(float timeDiff, Config &config, gr::top_block_sptr &tb, std::vector<Source *> &sources, std::vector<System *> &systems) {
  plugman_setup_config(sources, systems);
  plugman_system_rates(systems, timeDiff);

  for (std::vector<System *>::iterator it = systems.begin(); it != systems.end(); ++it) {
    System_impl *sys = (System_impl *)*it;

    if ((sys->get_system_type() != "conventional") && (sys->get_system_type() != "conventionalP25") && (sys->get_system_type() != "conventionalDMR") && (sys->get_system_type() != "conventionalSIGMF")) {
      int msgs_decoded_per_second = std::floor(sys->message_count / timeDiff);
      sys->set_decode_rate(msgs_decoded_per_second);

      if (msgs_decoded_per_second < 2) {

        // if it loses track of the control channel, quit after a while
        if (config.control_retune_limit > 0) {
          sys->retune_attempts++;
          if (sys->retune_attempts > config.control_retune_limit) {
            BOOST_LOG_TRIVIAL(error) << "[" << sys->get_short_name() << "]\t"
                                     << "Control channel retune limit exceeded after " << sys->retune_attempts << " tries - Terminating trunk recorder";
            exit_flag = 1;
            exit_code = EXIT_FAILURE;
            return;
          }
        }
        if (sys->control_channel_count() > 1) {
          retune_system(sys, tb, sources);
        } else {
          BOOST_LOG_TRIVIAL(error) << "[" << sys->get_short_name() << "]\tThere is only one control channel defined";
        }

      } else {
        sys->retune_attempts = 0;
      }

      if (msgs_decoded_per_second < config.control_message_warn_rate) {
        BOOST_LOG_TRIVIAL(error) << "[" << sys->get_short_name() << "]\tfreq: " << format_freq(sys->get_current_control_channel()) << "\tControl Channel Message Decode Rate: " << msgs_decoded_per_second << "/sec, count:  " << sys->message_count;
      } else if (config.control_message_warn_rate == -1) {
        BOOST_LOG_TRIVIAL(info) << "[" << sys->get_short_name() << "]\tfreq: " << format_freq(sys->get_current_control_channel()) << "\tControl Channel Message Decode Rate: " << msgs_decoded_per_second << "/sec, count:  " << sys->message_count;
      }
    }
    sys->message_count = 0;
  }
}

void check_conventional_channel_detection(std::vector<Source *> &sources) {
  Source *source = NULL;
  for (vector<Source *>::iterator src_it = sources.begin(); src_it != sources.end(); src_it++) {
    source = *src_it;
    source->enable_detected_recorders();
  }
}

// This is to handle the messages that come off the Analog recorder.
void process_message_queues(std::vector<System *> &systems) {
  for (std::vector<System *>::iterator it = systems.begin(); it != systems.end(); ++it) {
    System_impl *sys = (System_impl *)*it;

    for (std::vector<analog_recorder_sptr>::iterator arit = sys->conventional_recorders.begin(); arit != sys->conventional_recorders.end(); ++arit) {
      analog_recorder_sptr ar = (analog_recorder_sptr)*arit;
      ar->process_message_queues();
    }
  }
}

// Process message queues for recorders associated with Calls
void process_recorder_message_queues(std::vector<Call *> &calls) {
  for (vector<Call *>::iterator it = calls.begin(); it != calls.end(); ++it) {
    Call *call = *it;
    if (call->get_state() == RECORDING) {
      Recorder *recorder = call->get_recorder();
      if (recorder && (recorder->get_type() == P25 || recorder->get_type() == P25C)) {
        p25_recorder *p25_rec = dynamic_cast<p25_recorder *>(recorder);
        // Verify recorder status as conventionals calls may be in a RECORDING:IDLE state
        if (p25_rec && (p25_rec->is_active())) {
          p25_rec->process_message_queues();
        }
      }
    }
  }
}

int monitor_messages(Config &config, gr::top_block_sptr &tb, std::vector<Source *> &sources, std::vector<System *> &systems, std::vector<Call *> &calls) {
  gr::message::sptr msg;

  time_t last_status_time = time(NULL);
  time_t last_decode_rate_check = time(NULL);
  time_t management_timestamp = time(NULL);
  uint64_t last_conventional_channel_detection_check = time_since_epoch_millisec();
  time_t current_time = time(NULL);
  uint64_t current_time_ms = time_since_epoch_millisec();
  std::vector<TrunkMessage> trunk_messages;
  SmartnetParser *smartnet_parser;
  P25Parser *p25_parser;

  signal(SIGINT, exit_interupt);

  smartnet_parser = new SmartnetParser(); // this has to eventually be generic;
  p25_parser = new P25Parser();

  while (1) {

    if (exit_flag) { // my action when signal set it 1
      BOOST_LOG_TRIVIAL(info) << "Caught an Exit Signal...";
      for (vector<Call *>::iterator it = calls.begin(); it != calls.end();) {
        Call *call = *it;

        if (call->get_state() != MONITORING) {
          call->conclude_call();
        }

        it = calls.erase(it);
        delete call;
      }

      BOOST_LOG_TRIVIAL(info) << "Cleaning up & Exiting...";

      // Sleep for 5 seconds to allow for all of the Call Concluder threads to finish.
      boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
      return exit_code;
    }

    process_message_queues(systems);
    process_recorder_message_queues(calls);

    plugman_poll_one();

    for (vector<System *>::iterator sys_it = systems.begin(); sys_it != systems.end(); sys_it++) {
      System_impl *system = (System_impl *)*sys_it;

      if ((system->get_system_type() == "p25") || (system->get_system_type() == "smartnet")) {
        msg.reset();
        msg = system->get_msg_queue()->delete_head_nowait();
        while (msg != 0) {
          system->set_message_count(system->get_message_count() + 1);

          if (system->get_system_type() == "smartnet") {
            trunk_messages = smartnet_parser->parse_message(msg->to_string(), system);
            handle_message(trunk_messages, system, config, sources, calls, tb);
            plugman_trunk_message(trunk_messages, system);
          }

          if (system->get_system_type() == "p25") {
            trunk_messages = p25_parser->parse_message(msg, system);
            handle_message(trunk_messages, system, config, sources, calls, tb);
            plugman_trunk_message(trunk_messages, system);
          }

          if (msg->type() == -1) {
            BOOST_LOG_TRIVIAL(error) << "[" << system->get_short_name() << "]\t process_data_unit timeout";
          }

          msg.reset();
          msg = system->get_msg_queue()->delete_head_nowait();
        }
      }
    }
    current_time = time(NULL);
    current_time_ms = time_since_epoch_millisec();
    if ((current_time_ms - last_conventional_channel_detection_check) >= 0.1) {
      check_conventional_channel_detection(sources);
      last_conventional_channel_detection_check = current_time_ms;
    }

    if ((current_time - management_timestamp) >= 1.0) {
      manage_calls(config, calls);
      Call_Concluder::manage_call_data_workers();
      management_timestamp = current_time;
    }

    boost::this_thread::sleep(boost::posix_time::milliseconds(10));

    float decode_rate_check_time_diff = current_time - last_decode_rate_check;

    if (decode_rate_check_time_diff >= 3.0) {
      check_message_count(decode_rate_check_time_diff, config, tb, sources, systems);
      for (vector<Source *>::iterator src_it = sources.begin(); src_it != sources.end(); src_it++) {
        Source *source = *src_it;
        if (!source->got_samples()) {
          BOOST_LOG_TRIVIAL(error) << "Source " << source->get_num() << " has stopped receiving samples - Terminating trunk recorder";
          exit(1);
        }
      }
      last_decode_rate_check = current_time;
      for (vector<System *>::iterator sys_it = systems.begin(); sys_it != systems.end(); sys_it++) {
        System *system = *sys_it;
        if (system->get_system_type() == "p25") {
          system->clear_stale_talkgroup_patches();
        }
      }
    }

    float print_status_time_diff = current_time - last_status_time;

    if (print_status_time_diff > 200) {
      last_status_time = current_time;
      print_status(sources, systems, calls);
    }
  }
}
