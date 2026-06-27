#include "dmr_recorder_impl.h"

#include "../call_conventional.h"
#include "../formatter.h"
#include "../gr_blocks/plugin_wrapper_impl.h"
#include "../plugin_manager/plugin_manager.h"
#include <algorithm>
#include <boost/log/trivial.hpp>

dmr_recorder_sptr make_dmr_recorder(Source *src, bool conventional) {
  dmr_recorder *recorder = new dmr_recorder_impl(src, conventional);
  return gnuradio::get_initial_sptr(recorder);
}

dmr_recorder_impl::dmr_recorder_impl(Source *src, bool conv)
    : gr::hier_block2("dmr_recorder",
                      gr::io_signature::make(1, 1, sizeof(gr_complex)),
                      gr::io_signature::make(0, 0, sizeof(float))),
      Recorder(DMR) {
  conventional = conv;
  initialize(src);
}

void dmr_recorder_impl::initialize(Source *src) {
  source = src;
  chan_freq = source->get_center();
  center_freq = source->get_center();
  config = source->get_config();
  d_soft_vocoder = config ? config->soft_vocoder : false;
  input_rate = source->get_rate();
  silence_frames = source->get_silence_frames();
  squelch_db = 0;

  rec_num = rec_counter++;
  recording_count = 0;
  recording_duration = 0;

  set_enable_audio_streaming(config ? config->enable_audio_streaming : false);

  timestamp = time(NULL);
  starttime = time(NULL);

  for (int i = 0; i < 2; i++) {
    slots[i].call = nullptr;
    slots[i].active = false;
  }

  prefilter = xlat_channelizer::make(input_rate,
                                     channelizer::phase1_samples_per_symbol,
                                     channelizer::phase1_symbol_rate,
                                     xlat_channelizer::channel_bandwidth,
                                     center_freq, conventional);

  // FSK4 demod chain — locked at Phase 1 rates because DMR voice is always 4-FSK
  // at 4800 sym/s.
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
  slicer = gr::op25_repeater::fsk4_slicer_fb::make(0, 0, slices);

  rx_queue = gr::msg_queue::make(100);
  framer = gr::op25_repeater::frame_assembler::make("", 0, 1, rx_queue, d_soft_vocoder);
  framer->set_voice_codec_callback(voice_codec_cb_handler, this);

  // Per-slot sinks. Both are always connected — the framer always emits both
  // slot outputs, and a sink with no Call attached drops samples (see
  // transmission_sink::work() when d_current_call is null). This means we never
  // need to rewire the flowgraph when slots come and go.
  for (int i = 0; i < 2; i++) {
    slots[i].wav_sink = gr::blocks::transmission_sink::make(1, 8000, 16);
    slots[i].plugin_sink = gr::blocks::plugin_wrapper_impl::make(
        std::bind(&dmr_recorder_impl::plugin_callback_handler, this, i,
                  std::placeholders::_1, std::placeholders::_2));
  }

  connect(self(), 0, prefilter, 0);
  connect(prefilter, 0, pll_freq_lock, 0);
  connect(pll_freq_lock, 0, pll_amp, 0);
  connect(pll_amp, 0, noise_filter, 0);
  connect(noise_filter, 0, sym_filter, 0);
  connect(sym_filter, 0, fsk4_demod, 0);
  connect(fsk4_demod, 0, slicer, 0);
  connect(slicer, 0, framer, 0);
  connect(framer, 0, slots[0].wav_sink, 0);
  connect(framer, 1, slots[1].wav_sink, 0);

  if (get_enable_audio_streaming()) {
    connect(framer, 0, slots[0].plugin_sink, 0);
    connect(framer, 1, slots[1].plugin_sink, 0);
  }
}

void dmr_recorder_impl::plugin_callback_handler(int slot, int16_t *samples, int sampleCount) {
  if (slots[slot].call) {
    plugman_audio_callback(slots[slot].call, this, samples, sampleCount);
  }
}

void dmr_recorder_impl::voice_codec_cb_handler(int codec_type, long tgid, uint32_t src_id,
                                               const uint32_t *params, int param_count,
                                               int errs, void *user_data) {
  // The framer doesn't tell us which slot the codec data came from, so we route
  // to whichever slot currently has the matching talkgroup. If both slots are
  // active on the same TG (unusual) we attribute to slot 0.
  dmr_recorder_impl *self = static_cast<dmr_recorder_impl *>(user_data);
  for (int i = 0; i < 2; i++) {
    if (self->slots[i].call && self->slots[i].call->get_talkgroup() == tgid) {
      plugman_voice_codec_data(self->slots[i].call, codec_type, tgid, src_id, params, param_count, errs);
      return;
    }
  }
}

Source *dmr_recorder_impl::get_source() {
  return source;
}

int dmr_recorder_impl::get_num() {
  return rec_num;
}

double dmr_recorder_impl::get_freq() {
  return chan_freq;
}

int dmr_recorder_impl::get_freq_error() {
  return prefilter->get_freq_error();
}

bool dmr_recorder_impl::is_squelched() {
  if (slots[0].active || slots[1].active) {
    return prefilter->is_squelched();
  }
  return true;
}

double dmr_recorder_impl::get_pwr() {
  return prefilter->get_pwr();
}

int dmr_recorder_impl::lastupdate() {
  return time(NULL) - timestamp;
}

long dmr_recorder_impl::elapsed() {
  return time(NULL) - starttime;
}

void dmr_recorder_impl::tune_freq(double f) {
  chan_freq = f;
  prefilter->tune_offset(center_freq - f);
}

bool dmr_recorder_impl::is_enabled() {
  return source->is_selector_port_enabled(selector_port);
}

void dmr_recorder_impl::set_enabled(bool enabled) {
  source->set_selector_port_enabled(selector_port, enabled);
}

// ---- Per-slot lifecycle ---------------------------------------------------

bool dmr_recorder_impl::is_slot_available(int slot) {
  return !slots[slot].active;
}

bool dmr_recorder_impl::is_fully_available() {
  return !slots[0].active && !slots[1].active;
}

bool dmr_recorder_impl::start(Call *call) {
  int slot = call->get_tdma_slot();
  if (slots[slot].active) {
    BOOST_LOG_TRIVIAL(error) << "dmr_recorder: tried to start slot " << slot
                             << " on recorder " << rec_num << " but slot is busy";
    return false;
  }

  bool was_fully_idle = is_fully_available();
  System *system = call->get_system();
  double call_freq = call->get_freq();

  // If the recorder was fully idle, this is the first slot starting — tune to
  // the call's freq. If the other slot is already active, the freqs must match
  // (the allocator guarantees this); we never retune a recorder out from under
  // an active slot.
  if (was_fully_idle) {
    chan_freq = call_freq;
    prefilter->tune_offset(center_freq - chan_freq);
    if (call->is_conventional()) {
      Call_conventional *conv = dynamic_cast<Call_conventional *>(call);
      squelch_db = conv ? conv->get_squelch_db() : system->get_squelch_db();
    } else {
      squelch_db = system->get_squelch_db();
    }
    prefilter->set_squelch_db(squelch_db);
  } else if (call_freq != chan_freq) {
    BOOST_LOG_TRIVIAL(error) << "dmr_recorder: rec " << rec_num
                             << " slot " << slot << " freq " << call_freq
                             << " does not match active recorder freq " << chan_freq;
    return false;
  }

  slots[slot].call = call;
  slots[slot].active = true;
  slots[slot].wav_sink->start_recording(call, slot);

  timestamp = time(NULL);
  starttime = time(NULL);

  std::string loghdr = log_header(call->get_short_name(), call->get_call_num(),
                                  call->get_talkgroup_display(), chan_freq);
  BOOST_LOG_TRIVIAL(info) << loghdr << "[32mStarting DMR Recorder Num ["
                          << rec_num << "][0m\tSlot: " << slot;

  // Conventional with signal detection waits for the detector to open the gate;
  // everything else opens immediately.
  bool enable_now = true;
  if (call->is_conventional()) {
    Call_conventional *conv = dynamic_cast<Call_conventional *>(call);
    if (conv && conv->get_signal_detection()) {
      enable_now = false;
    }
  }
  if (enable_now) {
    set_enabled(true);
  }

  recording_count++;
  return true;
}

void dmr_recorder_impl::stop(int slot) {
  if (!slots[slot].active) {
    BOOST_LOG_TRIVIAL(error) << "dmr_recorder: stop on inactive slot " << slot
                             << " of rec " << rec_num;
    return;
  }

  recording_duration += slots[slot].wav_sink->total_length_in_seconds();
  slots[slot].wav_sink->stop_recording();
  slots[slot].active = false;
  slots[slot].call = nullptr;

  std::string short_name;
  if (slots[0].call) short_name = slots[0].call->get_short_name();
  else if (slots[1].call) short_name = slots[1].call->get_short_name();
  BOOST_LOG_TRIVIAL(info) << "[" << short_name << "]\t[33mStopping DMR Recorder Num ["
                          << rec_num << "][0m\tSlot: " << slot;

  // Only disable the pipeline when both slots have released — otherwise we'd
  // cut samples to the active slot's sink.
  if (is_fully_available()) {
    set_enabled(false);
  }
}

void dmr_recorder_impl::stop() {
  for (int i = 0; i < 2; i++) {
    if (slots[i].active) stop(i);
  }
}

// ---- Per-slot queries -----------------------------------------------------

State dmr_recorder_impl::get_state(int slot) {
  return slots[slot].wav_sink->get_state();
}

bool dmr_recorder_impl::is_active(int slot) {
  return slots[slot].active;
}

bool dmr_recorder_impl::is_idle(int slot) {
  // Per-slot idle is sink-state. The prefilter squelch is per-channel — it tells
  // us about RF on the repeater, not about voice on a specific slot.
  State s = slots[slot].wav_sink->get_state();
  return (s == IDLE) || (s == STOPPED);
}

double dmr_recorder_impl::since_last_write(int slot) {
  return time(NULL) - slots[slot].wav_sink->get_stop_time();
}

double dmr_recorder_impl::get_current_length(int slot) {
  return slots[slot].wav_sink->total_length_in_seconds();
}

std::vector<Transmission> dmr_recorder_impl::get_transmission_list(int slot) {
  return slots[slot].wav_sink->get_transmission_list();
}

// ---- Aggregate (slot-less) overrides --------------------------------------

State dmr_recorder_impl::get_state() {
  // For status reporting: prefer "more active" of the two slots.
  State s0 = slots[0].wav_sink->get_state();
  State s1 = slots[1].wav_sink->get_state();
  if (s0 == RECORDING || s1 == RECORDING) return RECORDING;
  if (s0 == IDLE || s1 == IDLE) return IDLE;
  if (s0 == STOPPED || s1 == STOPPED) return STOPPED;
  return AVAILABLE;
}

bool dmr_recorder_impl::is_active() {
  return slots[0].active || slots[1].active;
}

bool dmr_recorder_impl::is_idle() {
  return is_idle(0) && is_idle(1);
}

double dmr_recorder_impl::since_last_write() {
  return std::min(since_last_write(0), since_last_write(1));
}

double dmr_recorder_impl::get_current_length() {
  return std::max(get_current_length(0), get_current_length(1));
}

std::vector<Transmission> dmr_recorder_impl::get_transmission_list() {
  std::vector<Transmission> out = slots[0].wav_sink->get_transmission_list();
  std::vector<Transmission> s1 = slots[1].wav_sink->get_transmission_list();
  out.insert(out.end(), s1.begin(), s1.end());
  std::sort(out.begin(), out.end(),
            [](const Transmission &a, const Transmission &b) { return a.start_time < b.start_time; });
  return out;
}
