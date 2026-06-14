#include "dmr_trunked_recorder_impl.h"

#include "../formatter.h"
#include "../gr_blocks/plugin_wrapper_impl.h"
#include "../plugin_manager/plugin_manager.h"
#include <boost/log/trivial.hpp>
#include <gnuradio/blocks/null_sink.h>

dmr_trunked_recorder_sptr make_dmr_trunked_recorder(Source *src) {
  dmr_trunked_recorder *recorder = new dmr_trunked_recorder_impl(src);
  return gnuradio::get_initial_sptr(recorder);
}

dmr_trunked_recorder_impl::dmr_trunked_recorder_impl(Source *src)
    : gr::hier_block2("dmr_trunked_recorder",
                      gr::io_signature::make(1, 1, sizeof(gr_complex)),
                      gr::io_signature::make(0, 0, sizeof(float))),
      Recorder(DMR_TRUNKED) {
  conventional = false;
  initialize(src);
}

void dmr_trunked_recorder_impl::initialize(Source *src) {
  source = src;
  chan_freq = source->get_center();
  center_freq = source->get_center();
  config = source->get_config();
  d_soft_vocoder = config ? config->soft_vocoder : false;
  input_rate = source->get_rate();
  silence_frames = source->get_silence_frames();
  squelch_db = 0;

  talkgroup = 0;
  tdma_slot = 0;
  rec_num = rec_counter++;
  recording_count = 0;
  recording_duration = 0;

  if (config != NULL) {
    set_enable_audio_streaming(config->enable_audio_streaming);
  } else {
    set_enable_audio_streaming(false);
  }

  state = INACTIVE;
  timestamp = time(NULL);
  starttime = time(NULL);

  // Same FSK4 chain dmr_recorder uses (xlat_channelizer -> PLL freqdet ->
  // noise low-pass -> 5-tap boxcar matched filter -> op25 FSK4 demod ->
  // slicer -> frame_assembler).
  prefilter = xlat_channelizer::make(input_rate,
                                     channelizer::phase1_samples_per_symbol,
                                     channelizer::phase1_symbol_rate,
                                     xlat_channelizer::channel_bandwidth,
                                     center_freq,
                                     /*conventional=*/false);

  const double phase1_channel_rate = phase1_symbol_rate * phase1_samples_per_symbol;
  const double pi = M_PI;
  double freq_to_norm_radians = pi / (phase1_channel_rate / 2.0);
  double fc = 0.0;
  double fd = 600.0;
  double pll_demod_gain = 1.0 / (fd * freq_to_norm_radians);
  double samples_per_symbol = 5;

  pll_freq_lock = gr::analog::pll_freqdet_cf::make(
      (phase1_symbol_rate / 2.0 * 1.2) * freq_to_norm_radians,
      (fc + (3 * fd * 1.9)) * freq_to_norm_radians,
      (fc + (-3 * fd * 1.9)) * freq_to_norm_radians);
  pll_amp = gr::blocks::multiply_const_ff::make(pll_demod_gain * 1.0);

#if GNURADIO_VERSION < 0x030900
  baseband_noise_filter_taps = gr::filter::firdes::low_pass_2(
      1.0, phase1_channel_rate,
      phase1_symbol_rate / 2.0 * 1.175,
      phase1_symbol_rate / 2.0 * 0.125,
      20.0, gr::filter::firdes::WIN_KAISER, 6.76);
#else
  baseband_noise_filter_taps = gr::filter::firdes::low_pass_2(
      1.0, phase1_channel_rate,
      phase1_symbol_rate / 2.0 * 1.175,
      phase1_symbol_rate / 2.0 * 0.125,
      20.0, gr::fft::window::WIN_KAISER, 6.76);
#endif
  noise_filter = gr::filter::fft_filter_fff::make(1.0, baseband_noise_filter_taps);

  for (int i = 0; i < samples_per_symbol; i++) {
    sym_taps.push_back(1.0 / samples_per_symbol);
  }
  sym_filter = gr::filter::fir_filter_fff::make(1, sym_taps);

  tune_queue = gr::msg_queue::make(20);
  fsk4_demod = gr::op25_repeater::fsk4_demod_ff::make(tune_queue, phase1_channel_rate, phase1_symbol_rate);

  const float l[] = {-2.0, 0.0, 2.0, 4.0};
  std::vector<float> slices(l, l + sizeof(l) / sizeof(l[0]));
  const int msgq_id = 0;
  const int debug = 0;
  slicer = gr::op25_repeater::fsk4_slicer_fb::make(msgq_id, debug, slices);

  // frame_assembler in default (auto-detect) mode picks up DMR sync magics.
  rx_queue = gr::msg_queue::make(100);
  int verbosity = 0;
  framer = gr::op25_repeater::frame_assembler::make("", verbosity, 1, rx_queue, d_soft_vocoder);
  framer->set_voice_codec_callback(voice_codec_cb_handler, this);

  wav_sink = gr::blocks::transmission_sink::make(1, 8000, 16);
  null_slot_sink = gr::blocks::null_sink::make(sizeof(int16_t));
  plugin_sink = gr::blocks::plugin_wrapper_impl::make(
      std::bind(&dmr_trunked_recorder_impl::plugin_callback_handler, this,
                std::placeholders::_1, std::placeholders::_2));

  connect(self(), 0, prefilter, 0);
  connect(prefilter, 0, pll_freq_lock, 0);
  connect(pll_freq_lock, 0, pll_amp, 0);
  connect(pll_amp, 0, noise_filter, 0);
  connect(noise_filter, 0, sym_filter, 0);
  connect(sym_filter, 0, fsk4_demod, 0);
  connect(fsk4_demod, 0, slicer, 0);
  connect(slicer, 0, framer, 0);

  // frame_assembler always emits 2 audio outputs (slot 0, slot 1). start()
  // picks which one feeds wav_sink based on the call's TDMA slot. We need
  // both ports connected at all times — re-wire on tune is not safe inside
  // a running flow graph. The selection is done by toggling the OP25
  // set_slotid() slot mask, which mutes the unused output to silence.
  // Default: slot 0 active.
  connect(framer, 0, wav_sink, 0);
  connect(framer, 1, null_slot_sink, 0);

  if (get_enable_audio_streaming()) {
    connect(framer, 0, plugin_sink, 0);
  }
}

void dmr_trunked_recorder_impl::plugin_callback_handler(int16_t *samples, int sampleCount) {
  plugman_audio_callback(call, this, samples, sampleCount);
}

void dmr_trunked_recorder_impl::voice_codec_cb_handler(int codec_type, long tgid, uint32_t src_id, const uint32_t *params, int param_count, int errs, void *user_data) {
  dmr_trunked_recorder_impl *self = static_cast<dmr_trunked_recorder_impl *>(user_data);
  if (self->call) {
    plugman_voice_codec_data(self->call, codec_type, tgid, src_id, params, param_count, errs);
  }
}

Source *dmr_trunked_recorder_impl::get_source() {
  return source;
}

int dmr_trunked_recorder_impl::get_num() {
  return rec_num;
}

double dmr_trunked_recorder_impl::since_last_write() {
  time_t now = time(NULL);
  return now - wav_sink->get_stop_time();
}

State dmr_trunked_recorder_impl::get_state() {
  return wav_sink->get_state();
}

bool dmr_trunked_recorder_impl::is_active() {
  return state == ACTIVE;
}

bool dmr_trunked_recorder_impl::is_enabled() {
  return source->is_selector_port_enabled(selector_port);
}

void dmr_trunked_recorder_impl::set_enabled(bool enabled) {
  source->set_selector_port_enabled(selector_port, enabled);
}

bool dmr_trunked_recorder_impl::is_squelched() {
  if (state == ACTIVE) {
    return prefilter->is_squelched();
  }
  return true;
}

double dmr_trunked_recorder_impl::get_pwr() {
  return prefilter->get_pwr();
}

bool dmr_trunked_recorder_impl::is_idle() {
  if ((wav_sink->get_state() == IDLE) || (wav_sink->get_state() == STOPPED)) {
    return true;
  }
  return false;
}

double dmr_trunked_recorder_impl::get_freq() {
  return chan_freq;
}

int dmr_trunked_recorder_impl::get_freq_error() {
  return prefilter->get_freq_error();
}

double dmr_trunked_recorder_impl::get_current_length() {
  return wav_sink->total_length_in_seconds();
}

int dmr_trunked_recorder_impl::lastupdate() {
  return time(NULL) - timestamp;
}

long dmr_trunked_recorder_impl::elapsed() {
  return time(NULL) - starttime;
}

void dmr_trunked_recorder_impl::tune_freq(double f) {
  chan_freq = f;
  float freq = (center_freq - f);
  prefilter->tune_offset(freq);
}

int dmr_trunked_recorder_impl::get_tdma_slot() {
  return tdma_slot;
}

void dmr_trunked_recorder_impl::set_tdma_slot(int slot) {
  tdma_slot = slot;
  // OP25 slot mask: bit 0 = slot 0, bit 1 = slot 1, bit 2 = "no slot" (idle).
  // We always demodulate exactly one slot for trunked DMR.
  if (slot == 0) {
    framer->set_slotid(1);
  } else {
    framer->set_slotid(2);
  }
}

std::vector<Transmission> dmr_trunked_recorder_impl::get_transmission_list() {
  return wav_sink->get_transmission_list();
}

void dmr_trunked_recorder_impl::stop() {
  if (state == ACTIVE) {
    recording_duration += wav_sink->total_length_in_seconds();

    std::string loghdr = log_header(this->call->get_short_name(), this->call->get_call_num(),
                                    this->call->get_talkgroup_display(), chan_freq);
    BOOST_LOG_TRIVIAL(info) << loghdr << "[33mStopping DMR Trunked Recorder Num [" << rec_num << "][0m\tSlot: " << tdma_slot;

    state = INACTIVE;
    set_enabled(false);
    wav_sink->stop_recording();
    // Park the framer with mask=4 (both slots dropped) while idle so it does
    // not produce noise into the wav_sink between calls.
    framer->set_slotid(4);
  } else {
    BOOST_LOG_TRIVIAL(error) << "dmr_trunked_recorder.cc: Trying to Stop an Inactive Logger!!!";
  }
}

bool dmr_trunked_recorder_impl::start(Call *call) {
  if (state == INACTIVE) {
    System *system = call->get_system();
    set_tdma_slot(call->get_tdma_slot());

    timestamp = time(NULL);
    starttime = time(NULL);

    talkgroup = call->get_talkgroup();
    short_name = call->get_short_name();
    chan_freq = call->get_freq();
    this->call = call;
    std::string loghdr = log_header(call->get_short_name(), call->get_call_num(),
                                    call->get_talkgroup_display(), chan_freq);
    BOOST_LOG_TRIVIAL(info) << loghdr << "[32mStarting DMR Trunked Recorder Num [" << rec_num << "][0m\tSlot: " << call->get_tdma_slot();

    int offset_amount = (center_freq - chan_freq);
    prefilter->tune_offset(offset_amount);

    wav_sink->start_recording(call, /*slot=*/tdma_slot);
    state = ACTIVE;

    squelch_db = system->get_squelch_db();
    prefilter->set_squelch_db(squelch_db);
    set_enabled(true);

    recording_count++;
    return true;
  }

  BOOST_LOG_TRIVIAL(error) << "dmr_trunked_recorder.cc: Trying to Start an already Active Logger!!!";
  return false;
}
