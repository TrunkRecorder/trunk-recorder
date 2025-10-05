
#include "dmr_recorder_impl.h"

#include "../formatter.h"
#include "../gr_blocks/plugin_wrapper_impl.h"
#include "../plugin_manager/plugin_manager.h"
#include <boost/log/trivial.hpp>

dmr_recorder_sptr make_dmr_recorder(Source *src, Recorder_Type type) {
  dmr_recorder *recorder = new dmr_recorder_impl(src, type);

  return gnuradio::get_initial_sptr(recorder);
}

dmr_recorder_impl::dmr_recorder_impl(Source *src, Recorder_Type type)
    : gr::hier_block2("dmr_recorder",
                      gr::io_signature::make(1, 1, sizeof(gr_complex)),
                      gr::io_signature::make(0, 0, sizeof(float))),
      Recorder(type) {
  conventional = true;
  initialize(src);
}

void dmr_recorder_impl::initialize(Source *src) {
  source = src;
  chan_freq = source->get_center();
  center_freq = source->get_center();
  config = source->get_config();
  d_soft_vocoder = config->soft_vocoder;
  input_rate = source->get_rate();
  silence_frames = source->get_silence_frames();
  squelch_db = 0;

  talkgroup = 0;
  d_phase2_tdma = true;
  rec_num = rec_counter++;
  recording_count = 0;
  recording_duration = 0;

  bool use_streaming = false;

  if (config != NULL) {
    use_streaming = config->enable_audio_streaming;
  }

  state = INACTIVE;

  timestamp = time(NULL);
  starttime = time(NULL);

  prefilter = xlat_channelizer::make(input_rate, channelizer::phase1_samples_per_symbol, channelizer::phase1_symbol_rate, xlat_channelizer::channel_bandwidth, center_freq, conventional);

  /* FSK4 Demod */
  const double phase1_channel_rate = phase1_symbol_rate * phase1_samples_per_symbol;
  const double pi = M_PI;

  // FSK4: Phase Loop Lock - can only be Phase 1, so locking at that rate.
  double freq_to_norm_radians = pi / (phase1_channel_rate / 2.0);
  double fc = 0.0;
  double fd = 600.0;
  double pll_demod_gain = 1.0 / (fd * freq_to_norm_radians);
  double samples_per_symbol = 5;
  pll_freq_lock = gr::analog::pll_freqdet_cf::make((phase1_symbol_rate / 2.0 * 1.2) * freq_to_norm_radians, (fc + (3 * fd * 1.9)) * freq_to_norm_radians, (fc + (-3 * fd * 1.9)) * freq_to_norm_radians);
  pll_amp = gr::blocks::multiply_const_ff::make(pll_demod_gain * 1.0);

  // FSK4: noise filter - can only be Phase 1, so locking at that rate.
#if GNURADIO_VERSION < 0x030900
  baseband_noise_filter_taps = gr::filter::firdes::low_pass_2(1.0, phase1_channel_rate, phase1_symbol_rate / 2.0 * 1.175, phase1_symbol_rate / 2.0 * 0.125, 20.0, gr::filter::firdes::WIN_KAISER, 6.76);
#else
  baseband_noise_filter_taps = gr::filter::firdes::low_pass_2(1.0, phase1_channel_rate, phase1_symbol_rate / 2.0 * 1.175, phase1_symbol_rate / 2.0 * 0.125, 20.0, gr::fft::window::WIN_KAISER, 6.76);

#endif
  noise_filter = gr::filter::fft_filter_fff::make(1.0, baseband_noise_filter_taps);

  // FSK4: Symbol Taps
  double symbol_decim = 1;

  for (int i = 0; i < samples_per_symbol; i++) {
    sym_taps.push_back(1.0 / samples_per_symbol);
  }
  sym_filter = gr::filter::fir_filter_fff::make(symbol_decim, sym_taps);

  // FSK4: FSK4 Demod - locked at Phase 1 rates, since it can only be Phase 1
  tune_queue = gr::msg_queue::make(20);
  fsk4_demod = gr::op25_repeater::fsk4_demod_ff::make(tune_queue, phase1_channel_rate, phase1_symbol_rate);

  /* P25 Decode */
  // OP25 Slicer
  const float l[] = {-2.0, 0.0, 2.0, 4.0};
  const int msgq_id = 0;
  const int debug = 0;
  std::vector<float> slices(l, l + sizeof(l) / sizeof(l[0]));
  slicer = gr::op25_repeater::fsk4_slicer_fb::make(msgq_id, debug, slices);
  wav_sink_slot0 = gr::blocks::transmission_sink::make(1, 8000, 16);
  wav_sink_slot1 = gr::blocks::transmission_sink::make(1, 8000, 16);

  // OP25 Frame Assembler
  traffic_queue = gr::msg_queue::make(2);
  rx_queue = gr::msg_queue::make(100);
  int verbosity = 0; // 10 = lots of debug messages

  framer = gr::op25_repeater::frame_assembler::make("file:///tmp/out1.raw", verbosity, 1, rx_queue, d_soft_vocoder);
  levels = gr::blocks::multiply_const_ff::make(1);
  plugin_sink_slot0 = gr::blocks::plugin_wrapper_impl::make(std::bind(&dmr_recorder_impl::plugin_callback_handler, this, std::placeholders::_1, std::placeholders::_2));
  plugin_sink_slot1 = gr::blocks::plugin_wrapper_impl::make(std::bind(&dmr_recorder_impl::plugin_callback_handler, this, std::placeholders::_1, std::placeholders::_2));

  connect(self(), 0, prefilter, 0);
  connect(prefilter, 0, pll_freq_lock, 0);
  connect(pll_freq_lock, 0, pll_amp, 0);
  connect(pll_amp, 0, noise_filter, 0);
  connect(noise_filter, 0, sym_filter, 0);
  connect(sym_filter, 0, fsk4_demod, 0);
  connect(fsk4_demod, 0, slicer, 0);
  connect(slicer, 0, framer, 0);
  connect(framer, 0, wav_sink_slot0, 0);
  connect(framer, 1, wav_sink_slot1, 0);

  if (use_streaming) {
    connect(framer, 0, plugin_sink_slot0, 0);
    connect(framer, 1, plugin_sink_slot1, 0);
  }
}

void dmr_recorder_impl::plugin_callback_handler(int16_t *samples, int sampleCount) {
  plugman_audio_callback(call, this, samples, sampleCount);
}

void dmr_recorder_impl::switch_tdma(bool phase2) {
}

void dmr_recorder_impl::set_tdma(bool phase2) {
  if (phase2 != d_phase2_tdma) {
    switch_tdma(phase2);
  }
}

Source *dmr_recorder_impl::get_source() {
  return source;
}

int dmr_recorder_impl::get_num() {
  return rec_num;
}

double dmr_recorder_impl::since_last_write() {
  time_t now = time(NULL);
  return now - wav_sink_slot0->get_stop_time();
}

State dmr_recorder_impl::get_state() {
  return wav_sink_slot0->get_state();
}

bool dmr_recorder_impl::is_active() {
  if (state == ACTIVE) {
    return true;
  } else {
    return false;
  }
}
bool dmr_recorder_impl::is_enabled() {
  return source->is_selector_port_enabled(selector_port);
}

void dmr_recorder_impl::set_enabled(bool enabled) {
  source->set_selector_port_enabled(selector_port, enabled);
}

bool dmr_recorder_impl::is_squelched() {
  if (state == ACTIVE) {
    return prefilter->is_squelched();
  }
  return true;
}

double dmr_recorder_impl::get_pwr() {
  return prefilter->get_pwr();
}

bool dmr_recorder_impl::is_idle() {
  /*
    if ((wav_sink_slot0->get_state() == IDLE) || (wav_sink_slot0->get_state() == STOPPED)) {
      return true;
    }

    return false;*/
  if (state == ACTIVE) {
    return prefilter->is_squelched();
  }
  return true;
}

double dmr_recorder_impl::get_freq() {
  return chan_freq;
}

int dmr_recorder_impl::get_freq_error() { // get frequency error from FLL and convert to Hz
  return prefilter->get_freq_error();
}

double dmr_recorder_impl::get_current_length() {
  return wav_sink_slot0->total_length_in_seconds();
}

int dmr_recorder_impl::lastupdate() {
  return time(NULL) - timestamp;
}

long dmr_recorder_impl::elapsed() {
  return time(NULL) - starttime;
}

void dmr_recorder_impl::tune_freq(double f) {
  chan_freq = f;
  float freq = (center_freq - f);
  prefilter->tune_offset(freq);
}

bool compareTransmissions(Transmission t1, Transmission t2) {
  return (t1.start_time < t2.start_time);
}

std::vector<Transmission> dmr_recorder_impl::get_transmission_list() {
  std::vector<Transmission> return_list = wav_sink_slot0->get_transmission_list();
  std::vector<Transmission> second_list = wav_sink_slot1->get_transmission_list();
  BOOST_LOG_TRIVIAL(info) << "Slot 0: " << return_list.size() << " Slot 1: " << second_list.size();
  return_list.insert(return_list.end(), second_list.begin(), second_list.end());
  BOOST_LOG_TRIVIAL(info) << "Combined: " << return_list.size();
  sort(return_list.begin(), return_list.end(), compareTransmissions);
  BOOST_LOG_TRIVIAL(info) << "Sorted: " << return_list.size();
  return return_list;
}

void dmr_recorder_impl::stop() {
  if (state == ACTIVE) {

    recording_duration += wav_sink_slot0->total_length_in_seconds();
    
    //std::string loghdr = log_header(this->call->get_short_name(),this->call->get_call_num(),this->call->get_talkgroup_display(),chan_freq);
    // BOOST_LOG_TRIVIAL(info) << loghdr << "Stopping P25 Recorder Num [" << rec_num << "]\tTDMA: " << d_phase2_tdma << "\tSlot: " << tdma_slot;

    state = INACTIVE;
    set_enabled(false);
    wav_sink_slot0->stop_recording();
    wav_sink_slot1->stop_recording();
  } else {
    BOOST_LOG_TRIVIAL(error) << "dmr_recorder.cc: Trying to Stop an Inactive Logger!!!";
  }
}

void dmr_recorder_impl::set_tdma_slot(int slot) {
  tdma_slot = slot;
}

bool dmr_recorder_impl::start(Call *call) {
  if (state == INACTIVE) {
    System *system = call->get_system();
    set_tdma_slot(0);

    timestamp = time(NULL);
    starttime = time(NULL);

    talkgroup = call->get_talkgroup();
    short_name = call->get_short_name();
    chan_freq = call->get_freq();
    this->call = call;
    std::string loghdr = log_header(this->call->get_short_name(),this->call->get_call_num(),this->call->get_talkgroup_display(),chan_freq);
    BOOST_LOG_TRIVIAL(info) << loghdr << "\u001b[32mStarting DMR Recorder Num [" << rec_num << "]\u001b[0m\tTDMA: " << call->get_phase2_tdma() << "\tSlot: " << call->get_tdma_slot();

    int offset_amount = (center_freq - chan_freq);

    prefilter->tune_offset(offset_amount);
    levels->set_k(call->get_system()->get_digital_levels());
    wav_sink_slot0->start_recording(call, 0);
    wav_sink_slot1->start_recording(call, 1);
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
    BOOST_LOG_TRIVIAL(error) << "dmr_recorder.cc: Trying to Start an already Active Logger!!!";
    return false;
  }
  return true;
}
