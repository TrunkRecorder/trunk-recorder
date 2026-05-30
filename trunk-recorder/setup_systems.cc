#include "./setup_systems.h"
using namespace std;
bool setup_conventional_channel(System *system, double frequency, long channel_index, Config &config, gr::top_block_sptr &tb, std::vector<Source *> &sources, std::vector<Call *> &calls) {
  bool channel_added = false;
  Source *source = NULL;
  float tone_freq = 0.0;
  for (std::vector<Source *>::iterator src_it = sources.begin(); src_it != sources.end(); src_it++) {
    source = *src_it;

    if ((source->get_min_hz() <= frequency) && (source->get_max_hz() >= frequency)) {
      channel_added = true;
      if (system->get_squelch_db() == -160) {
        BOOST_LOG_TRIVIAL(error) << "[" << system->get_short_name() << "]\tSquelch needs to be specified for the Source for Conventional Systems";
        return false;
      } else {
        channel_added = true;
      }

      Call_conventional *call = NULL;
      if (system->has_channel_file()) {
        Talkgroup *tg = system->find_talkgroup_by_freq(frequency);
        tone_freq = tg->tone;

        // If there is a per channel squelch setting, use it, otherwise use the system squelch setting
        if (tg->squelch_db != DB_UNSET) {
          call = new Call_conventional(tg->number, tg->freq, system, config, tg->squelch_db, tg->signal_detection);
        } else {
          call = new Call_conventional(tg->number, tg->freq, system, config, system->get_squelch_db(), tg->signal_detection);
        }
        
        call->set_talkgroup_tag(tg->alpha_tag);
      } else {
        call = new Call_conventional(channel_index, frequency, system, config, system->get_squelch_db(), true);  // signal detection is always true when a channel file is not used
      }

      BOOST_LOG_TRIVIAL(info) << "[" << system->get_short_name() << "]\tMonitoring " << system->get_system_type() << " channel: " << format_freq(frequency) << " Talkgroup: " << channel_index;
      if (system->get_system_type() == "conventional") {
        analog_recorder_sptr rec;
        if (tone_freq > 0.0) {
          rec = source->create_conventional_recorder(tb, tone_freq);
        } else {
          rec = source->create_conventional_recorder(tb);
        }
        rec->start(call);
        rec->set_tau(system->get_tau()); //set the tau value for the recorder from the system config
        call->set_is_analog(true);
        call->set_recorder((Recorder *)rec.get());
        call->set_state(RECORDING);
        system->add_conventional_recorder(rec);
        calls.push_back(call);
        plugman_setup_recorder((Recorder *)rec.get());
        plugman_call_start(call);
      } else if (system->get_system_type() == "conventionalDMR") {
        // Because of dynamic mod assignment we can not start the recorder until the graph has been unlocked.
        // This has something to do with the way the Selector block works.
        // the manage_conventional_calls() function handles adding and starting the P25 Recorder
        dmr_recorder_sptr rec;
        rec = source->create_dmr_conventional_recorder(tb);
        call->set_recorder((Recorder *)rec.get());
        system->add_conventionalDMR_recorder(rec);
        calls.push_back(call);
      } else if (system->get_system_type() == "conventionalP25") { // has to be "conventional P25"
        // Because of dynamic mod assignment we can not start the recorder until the graph has been unlocked.
        // This has something to do with the way the Selector block works.
        // the manage_conventional_calls() function handles adding and starting the P25 Recorder
        p25_recorder_sptr rec;
        rec = source->create_digital_conventional_recorder(tb);
        call->set_recorder((Recorder *)rec.get());
        system->add_conventionalP25_recorder(rec);
        calls.push_back(call);
      } else if (system->get_system_type() == "conventionalSIGMF") {
        sigmf_recorder_sptr rec;
        rec = source->create_sigmf_conventional_recorder(tb);
        call->set_recorder((Recorder *)rec.get());
        system->add_conventionalSIGMF_recorder(rec);
        calls.push_back(call);
      } else {
        BOOST_LOG_TRIVIAL(error) << "Error - Unknown system type: " << system->get_system_type();
      }

      // break out of the for loop
      break;
    }
  }
  return channel_added;
}

bool setup_conventional_system(System *system, Config &config, gr::top_block_sptr &tb, std::vector<Source *> &sources, std::vector<Call *> &calls) {
  bool system_added = false;

  if (system->has_channel_file()) {
    std::vector<Talkgroup *> talkgroups = system->get_talkgroups();
    for (vector<Talkgroup *>::iterator tg_it = talkgroups.begin(); tg_it != talkgroups.end(); tg_it++) {
      Talkgroup *tg = *tg_it;

      bool channel_added = setup_conventional_channel(system, tg->freq, tg->number, config, tb, sources, calls);

      if (!channel_added) {
        BOOST_LOG_TRIVIAL(error) << "[" << system->get_short_name() << "]\t Unable to find a source for this conventional channel! Channel not added: " << format_freq(tg->freq) << " Talkgroup: " << tg->number;
        // return false;
      } else {
        system_added = true;
      }
    }
  } else {
    std::vector<double> channels = system->get_channels();
    int channel_index = 0;
    for (vector<double>::iterator chan_it = channels.begin(); chan_it != channels.end(); chan_it++) {
      double channel = *chan_it;
      ++channel_index;
      bool channel_added = setup_conventional_channel(system, channel, channel_index, config, tb, sources, calls);

      if (!channel_added) {
        BOOST_LOG_TRIVIAL(error) << "[" << system->get_short_name() << "]\t Unable to find a source for this conventional channel! Channel not added: " << format_freq(channel) << " Talkgroup: " << channel_index;
        // return false;
      } else {
        system_added = true;
      }
    }
  }
  return system_added;
}

bool setup_systems(Config &config, gr::top_block_sptr &tb, std::vector<Source *> &sources, std::vector<System *> &systems, std::vector<Call *> &calls) {

  Source *source = NULL;

  for (vector<System *>::iterator sys_it = systems.begin(); sys_it != systems.end(); sys_it++) {
    System_impl *system = (System_impl *)*sys_it;
    // bool    source_found = false;
    bool system_added = false;
    if ((system->get_system_type() == "conventional") || (system->get_system_type() == "conventionalP25") || (system->get_system_type() == "conventionalDMR")) {
      system_added = setup_conventional_system(system, config, tb, sources, calls);
    } else {
      // If it's not a conventional system, then it's a trunking system
      double control_channel_freq = system->get_current_control_channel();
      BOOST_LOG_TRIVIAL(info) << "[" << system->get_short_name() << "]\tStarted with Control Channel: " << format_freq(control_channel_freq);

      for (vector<Source *>::iterator src_it = sources.begin(); src_it != sources.end(); src_it++) {
        source = *src_it;

        if ((source->get_min_hz() <= control_channel_freq) &&
            (source->get_max_hz() >= control_channel_freq)) {
          // The source can cover the System's control channel
          system_added = true;
          system->set_source(source);

          if (system->get_system_type() == "smartnet") {
            system->smartnet_trunking = smartnet_impl::make(control_channel_freq,
                                                               source->get_center(),
                                                               source->get_rate(),
                                                               system->get_msg_queue(),
                                                               system->get_sys_num());
            tb->connect(source->get_src_block(), 0, system->smartnet_trunking, 0);
          }

          if (system->get_system_type() == "p25") {
            system->p25_trunking = make_p25_trunking(control_channel_freq,
                                                     source->get_center(),
                                                     source->get_rate(),
                                                     system->get_msg_queue(),
                                                     system->get_qpsk_mod(),
                                                     system->get_sys_num());
            tb->connect(source->get_src_block(), 0, system->p25_trunking, 0);
          }

          break;
        }
      }
      if (!system_added) {
        BOOST_LOG_TRIVIAL(error) << "[" << system->get_short_name() << "]\t Unable to find a source for this System! Control Channel Freq: " << format_freq(control_channel_freq);
        return false;
      }
    }
  }
  return true;
}
