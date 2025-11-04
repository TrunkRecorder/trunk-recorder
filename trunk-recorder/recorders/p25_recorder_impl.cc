
#include "p25_recorder_impl.h"
#include "../formatter.h"
#include "p25_recorder.h"
#include <boost/log/trivial.hpp>

p25_recorder_sptr make_p25_recorder(Source *src, Recorder_Type type) {
  p25_recorder *recorder = new p25_recorder_impl(src, type);

  return gnuradio::get_initial_sptr(recorder);
}

p25_recorder_impl::p25_recorder_impl(Source *src, Recorder_Type type)
    : gr::hier_block2("p25_recorder",
                      gr::io_signature::make(1, 1, sizeof(gr_complex)),
                      gr::io_signature::make(0, 0, sizeof(float))),
      Recorder(type) {
  if (type == P25C) {
    conventional = true;
  } else {
    conventional = false;
  }
  initialize(src);
}

void p25_recorder_impl::initialize(Source *src) {
  source = src;
  chan_freq = source->get_center();
  center_freq = source->get_center();
  config = source->get_config();
  d_soft_vocoder = config->soft_vocoder;
  input_rate = source->get_rate();
  qpsk_mod = true;
  silence_frames = source->get_silence_frames();
  squelch_db = 0;
  talkgroup = 0;
  d_phase2_tdma = false;
  rec_num = rec_counter++;
  recording_count = 0;
  recording_duration = 0;

  state = INACTIVE;

  timestamp = time(NULL);
  starttime = time(NULL);

  if (config == NULL) {
    this->set_enable_audio_streaming(false);
  } else {
    this->set_enable_audio_streaming(config->enable_audio_streaming);
  }

  prefilter = xlat_channelizer::make(input_rate, channelizer::phase1_samples_per_symbol, channelizer::phase1_symbol_rate, xlat_channelizer::channel_bandwidth, center_freq, conventional);
  // initialize_prefilter();
  //  initialize_p25();

  modulation_selector = gr::blocks::selector::make(sizeof(gr_complex), 0, 0);
  qpsk_demod = make_p25_recorder_qpsk_demod();
  qpsk_p25_decode = make_p25_recorder_decode(this, silence_frames, d_soft_vocoder);
  fsk4_demod = make_p25_recorder_fsk4_demod();
  fsk4_p25_decode = make_p25_recorder_decode(this, silence_frames, d_soft_vocoder);

  connect(self(), 0, prefilter, 0);
  connect(prefilter, 0, modulation_selector, 0);
  connect(modulation_selector, 0, fsk4_demod, 0);
  connect(fsk4_demod, 0, fsk4_p25_decode, 0);
  connect(modulation_selector, 1, qpsk_demod, 0);
  connect(qpsk_demod, 0, qpsk_p25_decode, 0);
}

void p25_recorder_impl::switch_tdma(bool phase2) {
  if (phase2) {
    d_phase2_tdma = true;
    prefilter->set_samples_per_symbol(phase2_samples_per_symbol);
  } else {
    d_phase2_tdma = false;
    prefilter->set_samples_per_symbol(phase1_samples_per_symbol);
  }

  if (qpsk_mod) {
    qpsk_p25_decode->switch_tdma(phase2);
    qpsk_demod->switch_tdma(phase2);
  }
}

void p25_recorder_impl::set_tdma(bool phase2) {
  if (phase2 != d_phase2_tdma) {
    switch_tdma(phase2);
  }
}

void p25_recorder_impl::reset_block(gr::basic_block_sptr block) {
  gr::block_detail_sptr detail;
  gr::block_sptr grblock = cast_to_block_sptr(block);
  detail = grblock->detail();
  detail->reset_nitem_counters();
}
void p25_recorder_impl::clear() {
  // This lead to weird SegFaults, but the goal was to clear out buffers inbetween transmissions.
  /*
  if (double_decim) {
    //reset_block(bandpass_filter);
    //reset_block(bfo);
  } else {
  //reset_block(lo);
  }
  reset_block(lowpass_filter);
  reset_block(mixer);

  if (arb_rate != 1.0) {
  reset_block(arb_resampler);
  }

  reset_block(cutoff_filter);
  reset_block(squelch);
  //reset_block(rms_agc); // RMS AGC cant be made into a basic block
  reset_block(fll_band_edge);
  reset_block(modulation_selector);


  //reset_block(qpsk_demod); // bad - Seg Faults
  //reset_block(qpsk_p25_decode); // bad - Seg Faults
  //reset_block(fsk4_demod); // bad - Seg Faults
  //reset_block(fsk4_p25_decode);  // bad - Seg Faults

  */
  qpsk_demod->reset();
  qpsk_p25_decode->reset();
  fsk4_demod->reset();
  fsk4_p25_decode->reset();
}

void p25_recorder_impl::autotune() {
  /*if (!qpsk_mod) {
    gr::message::sptr msg;
    msg = tune_queue->delete_head_nowait();

    if (msg != 0) {
      BOOST_LOG_TRIVIAL(info) << "p25_recorder.cc: Freq:\t" << format_freq(chan_freq) << "\t Tune: " << msg->arg1();

      // tune_offset(freq + (msg->arg1()*100));
      tune_queue->flush();
    }
  }*/
}

int p25_recorder_impl::get_freq_error() { // get frequency error from FLL and convert to Hz
  return prefilter->get_freq_error();
}

Source *p25_recorder_impl::get_source() {
  return source;
}

int p25_recorder_impl::get_num() {
  return rec_num;
}

double p25_recorder_impl::since_last_write() {
  if (qpsk_mod) {
    return qpsk_p25_decode->since_last_write();
  } else {
    return fsk4_p25_decode->since_last_write();
  }
}

State p25_recorder_impl::get_state() {
  if (qpsk_mod) {
    return qpsk_p25_decode->get_state();
  } else {
    return fsk4_p25_decode->get_state();
  }
}

bool p25_recorder_impl::is_enabled() {
  return source->is_selector_port_enabled(selector_port);
}

void p25_recorder_impl::set_enabled(bool enabled) {
  source->set_selector_port_enabled(selector_port, enabled);
}

bool p25_recorder_impl::is_active() {
  if (state == ACTIVE) {
    return true;
  } else {
    return false;
  }
}

bool p25_recorder_impl::is_squelched() {
  if (state == ACTIVE) {
    return prefilter->is_squelched();
  }
  return true;
}

double p25_recorder_impl::get_pwr() {
  return prefilter->get_pwr();
}

bool p25_recorder_impl::is_idle() {
  if (qpsk_mod) {
    if ((qpsk_p25_decode->get_state() == IDLE) || (qpsk_p25_decode->get_state() == STOPPED)) {
      return true;
    }
  } else {
    if ((fsk4_p25_decode->get_state() == IDLE) || (fsk4_p25_decode->get_state() == STOPPED)) {
      return true;
    }
  }
  return false;
}

double p25_recorder_impl::get_freq() {
  return chan_freq;
}

double p25_recorder_impl::get_current_length() {
  if (qpsk_mod) {
    return qpsk_p25_decode->get_current_length();
  } else {
    return fsk4_p25_decode->get_current_length();
  }
}

int p25_recorder_impl::lastupdate() {
  return time(NULL) - timestamp;
}

long p25_recorder_impl::elapsed() {
  return time(NULL) - starttime;
}

void p25_recorder_impl::tune_freq(double f) {
  chan_freq = f;
  float freq = (center_freq - f);
  prefilter->tune_offset(freq);
}

void p25_recorder_impl::set_source(long src) {
  if (qpsk_mod) {
    return qpsk_p25_decode->set_source(src);
  } else {
    return fsk4_p25_decode->set_source(src);
  }
}

std::vector<Transmission> p25_recorder_impl::get_transmission_list() {
  if (qpsk_mod) {
    return qpsk_p25_decode->get_transmission_list();
  } else {
    return fsk4_p25_decode->get_transmission_list();
  }
}

void p25_recorder_impl::stop() {
  if (state == ACTIVE) {
    if (qpsk_mod) {
      recording_duration += qpsk_p25_decode->get_current_length();
    } else {
      recording_duration += fsk4_p25_decode->get_current_length();
    }

    if (source->get_autotune_source()) {
      // Send last tuning measurements to autotune manager
      source->add_autotune_error_measurement(this->get_freq_error(), autotune_offset);
    }
    std::string loghdr = log_header(this->call->get_short_name(),this->call->get_call_num(),this->call->get_talkgroup_display(),chan_freq);
    BOOST_LOG_TRIVIAL(info) << loghdr << "\u001b[33mStopping P25 Recorder Num [" << rec_num << "]\u001b[0m\tTDMA: " << d_phase2_tdma << "\tSlot: " << tdma_slot << "\tTuningErr: " << std::showpos << this->get_freq_error() << std::noshowpos << " Hz";

    state = INACTIVE;
    set_enabled(false);

    clear();
    if (qpsk_mod) {
      qpsk_p25_decode->stop();
    } else {
      fsk4_p25_decode->stop();
    }
  } else {
    BOOST_LOG_TRIVIAL(error) << "p25_recorder.cc: Trying to Stop an Inactive Logger!!!";
  }
}

void p25_recorder_impl::set_tdma_slot(int slot) {
  if (qpsk_mod) {
    qpsk_p25_decode->set_tdma_slot(slot);
  } else {
    fsk4_p25_decode->set_tdma_slot(slot);
  }
  tdma_slot = slot;
}

bool p25_recorder_impl::start(Call *call) {
  if (state == INACTIVE) {
    System *system = call->get_system();
    qpsk_mod = system->get_qpsk_mod();
    set_tdma(call->get_phase2_tdma());
    if (call->get_phase2_tdma()) {
      if (!qpsk_mod) {
        BOOST_LOG_TRIVIAL(error) << "Error - Modulation is FSK4 but receiving Phase 2 call, this will not work";
        return false;
      }
      set_tdma_slot(call->get_tdma_slot());

      if (call->get_xor_mask()) {
        qpsk_p25_decode->set_xor_mask(call->get_xor_mask());
      } else {
        BOOST_LOG_TRIVIAL(info) << "Error - can't set XOR Mask for TDMA";
        return false;
      }
    } else {
      set_tdma_slot(0);
    }

    timestamp = time(NULL);
    starttime = time(NULL);

    talkgroup = call->get_talkgroup();
    short_name = call->get_short_name();
    chan_freq = call->get_freq();
    this->call = call;

    std::string loghdr = log_header(this->call->get_short_name(),this->call->get_call_num(),this->call->get_talkgroup_display(),chan_freq);
    autotune_offset = 0;
    std::ostringstream autotune_info;

    if (source->get_autotune_source()) {
      // Retrieve current autotune offset from the source's autotune manager
      autotune_offset = source->get_source_error();
      autotune_info << " AutoTune: " << std::showpos << autotune_offset << std::noshowpos << " Hz";
    }

    BOOST_LOG_TRIVIAL(info) << loghdr << "\u001b[32mStarting P25 Recorder Num [" << rec_num << "]\u001b[0m\tTDMA: " << call->get_phase2_tdma() << "\tSlot: " << call->get_tdma_slot() << "\tQPSK: " << qpsk_mod << autotune_info.str();

    int offset_amount = (center_freq - chan_freq + autotune_offset);

    prefilter->tune_offset(offset_amount);

    if (qpsk_mod) {
      modulation_selector->set_output_index(1);
      qpsk_p25_decode->start(call);
    } else {
      modulation_selector->set_output_index(0);
      fsk4_p25_decode->start(call);
    }
    state = ACTIVE;

    if (conventional) {
      Call_conventional *conventional_call = dynamic_cast<Call_conventional *>(call);
      squelch_db = conventional_call->get_squelch_db();
      if (conventional_call->get_signal_detection()) {
        set_enabled(false);
      } else {
        set_enabled(true); // If signal detection is not being used, open up the Value/Selector from the start
      }
    } else {
      squelch_db = system->get_squelch_db();
      set_enabled(true);
    }
    prefilter->set_squelch_db(squelch_db);


    recording_count++;
  } else {
    BOOST_LOG_TRIVIAL(error) << "p25_recorder.cc: Trying to Start an already Active Logger!!!";
    return false;
  }
  return true;
}

int Recorder::rec_counter = 0;
