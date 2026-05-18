#include "./setup_systems.h"
#include "talkgroup.h"
#include <map>
using namespace std;

// Common backend: given a chosen Call_conventional, its tone_config for the
// recorder, and a covering Source, build the per-system-type recorder and
// register it. Both the no-channel-file path (setup_conventional_channel)
// and the channel-file path (setup_conventional_channel_group) end up here.
static void build_recorder_for_call(
    System *system,
    Call_conventional *call,
    const Tone_Config &recorder_tone_config,
    Source *source,
    Config & /*config*/,
    gr::top_block_sptr &tb,
    std::vector<Call *> &calls) {
  if (system->get_system_type() == "conventional") {
    analog_recorder_sptr rec;
    rec = source->create_conventional_recorder(tb, recorder_tone_config);
    rec->start(call);
    rec->set_tau(system->get_tau()); // set the tau value for the recorder from the system config
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
}

// Find the first Source whose passband covers `frequency`, or nullptr.
static Source *find_source_covering(double frequency, std::vector<Source *> &sources) {
  for (Source *source : sources) {
    if (source->get_min_hz() <= frequency && source->get_max_hz() >= frequency) {
      return source;
    }
  }
  return nullptr;
}

// No-channel-file path: a single frequency, no Talkgroup metadata, no Tone
// column. Used when the system config has a "channels": [...] list instead
// of a "channelFile".
bool setup_conventional_channel(System *system, double frequency, long channel_index, Config &config, gr::top_block_sptr &tb, std::vector<Source *> &sources, std::vector<Call *> &calls) {
  Source *source = find_source_covering(frequency, sources);
  if (!source) return false;

  if (system->get_squelch_db() == -160) {
    BOOST_LOG_TRIVIAL(error) << "[" << system->get_short_name()
                              << "]\tSquelch needs to be specified for the Source for Conventional Systems";
    return false;
  }

  // signal detection is always true when a channel file is not used
  Call_conventional *call = new Call_conventional(
      channel_index, frequency, system, config, system->get_squelch_db(), true);

  BOOST_LOG_TRIVIAL(info) << "[" << system->get_short_name() << "]\tMonitoring "
                          << system->get_system_type() << " channel: "
                          << format_freq(frequency)
                          << " Talkgroup: " << channel_index;
  Tone_Config tc; // TONE_OFF
  build_recorder_for_call(system, call, tc, source, config, tb, calls);
  return true;
}

// Channel-file path: one or more Talkgroup rows all at the same frequency.
// `group[0]` is the primary row (provides squelch + signal_detection).
// Multi-row groups force the recorder into TONE_SEARCH mode so every
// transmission reaches the audio chain; the actual tone-to-metadata routing
// is done in Call_Concluder::create_call_data using the alternate_channels
// list attached to the Call_conventional. Non-matching tones become SKIPPED
// at conclusion time (no plugins fire, files removed).
bool setup_conventional_channel_group(System *system, const std::vector<Talkgroup *> &group, Config &config, gr::top_block_sptr &tb, std::vector<Source *> &sources, std::vector<Call *> &calls) {
  if (group.empty()) return false;
  Talkgroup *primary = group[0];
  const double frequency = primary->freq;

  Source *source = find_source_covering(frequency, sources);
  if (!source) return false;

  if (system->get_squelch_db() == -160 && primary->squelch_db == DB_UNSET) {
    BOOST_LOG_TRIVIAL(error) << "[" << system->get_short_name()
                              << "]\tSquelch needs to be specified for the Source for Conventional Systems";
    return false;
  }

  // Squelch + signal_detection from the primary (row 0). Per-row Squelch
  // overrides the system-wide value when set.
  const double effective_squelch =
      (primary->squelch_db != DB_UNSET) ? primary->squelch_db : system->get_squelch_db();

  // Recorder tone:
  //   • Single row → use that row's tone_config (today's existing behavior).
  //   • Multiple rows → TONE_SEARCH internally so every keyup is captured;
  //     the per-call match step decides which row to label it with (or
  //     marks it SKIPPED). This stops the previous behavior of spawning N
  //     recorders for N rows on the same freq.
  Tone_Config recorder_tone =
      (group.size() == 1) ? primary->tone_config
                          : Tone_Config{TONE_SEARCH, 0.0, 0, false};

  Call_conventional *call = new Call_conventional(
      primary->number, primary->freq, system, config,
      effective_squelch, primary->signal_detection, recorder_tone);
  call->set_talkgroup_tag(primary->alpha_tag);

  if (group.size() > 1) {
    call->set_alternate_channels(group);
  }

  if (group.size() == 1) {
    BOOST_LOG_TRIVIAL(info) << "[" << system->get_short_name() << "]\tMonitoring "
                            << system->get_system_type() << " channel: "
                            << format_freq(frequency)
                            << " Talkgroup: " << primary->number;
  } else {
    BOOST_LOG_TRIVIAL(info) << "[" << system->get_short_name() << "]\tMonitoring "
                            << system->get_system_type() << " channel: "
                            << format_freq(frequency)
                            << " — freq group (" << group.size() << " rows, primary TG "
                            << primary->number << "); recorder in search mode, "
                            << "metadata routed by detected tone";
    for (Talkgroup *tg : group) {
      const std::string tone_str =
          (tg->tone_config.mode == TONE_CTCSS) ? (std::to_string(tg->tone_config.ctcss_hz) + " Hz") :
          (tg->tone_config.mode == TONE_DCS)   ? (std::string("D") + std::to_string(tg->tone_config.dcs_code) + (tg->tone_config.dcs_inverted ? "I" : "N")) :
          (tg->tone_config.mode == TONE_SEARCH) ? std::string("S (catch-all)") :
                                                  std::string("0 (catch-all)");
      BOOST_LOG_TRIVIAL(info) << "\t  TG " << tg->number
                              << "  tone=" << tone_str
                              << "  \"" << tg->alpha_tag << "\"";
    }
  }

  build_recorder_for_call(system, call, recorder_tone, source, config, tb, calls);
  return true;
}

bool setup_conventional_system(System *system, Config &config, gr::top_block_sptr &tb, std::vector<Source *> &sources, std::vector<Call *> &calls) {
  bool system_added = false;

  if (system->has_channel_file()) {
    // Group talkgroups by frequency so we only spawn ONE analog_recorder per
    // unique freq, no matter how many CSV rows the operator wrote for it.
    // Preserve CSV order for the "primary" (row 0) selection — the first
    // row at any given freq is the one whose Squelch + Signal Detector
    // settings get used.
    std::vector<Talkgroup *> talkgroups = system->get_talkgroups();
    std::map<double, std::vector<Talkgroup *>> by_freq;
    std::vector<double> freq_order;
    for (Talkgroup *tg : talkgroups) {
      if (by_freq.find(tg->freq) == by_freq.end()) {
        freq_order.push_back(tg->freq);
      }
      by_freq[tg->freq].push_back(tg);
    }

    for (double freq : freq_order) {
      const std::vector<Talkgroup *> &group = by_freq[freq];
      bool channel_added = setup_conventional_channel_group(system, group, config, tb, sources, calls);
      if (!channel_added) {
        BOOST_LOG_TRIVIAL(error) << "[" << system->get_short_name()
                                  << "]\t Unable to find a source for this conventional channel! Channel not added: "
                                  << format_freq(freq) << " (primary TG " << group[0]->number << ")";
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
