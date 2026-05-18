
#include "analog_recorder.h"
#include "../formatter.h"
#include "../gr_blocks/decoder_wrapper_impl.h"
#include "../gr_blocks/plugin_wrapper_impl.h"
#include "../gr_blocks/transmission_sink.h"
#include "../plugin_manager/plugin_manager.h"
#include "../recorder_globals.h"

#include <iomanip>

using namespace std;

bool analog_recorder::logging = false;
// static int rec_counter = 0;

std::vector<float> design_filter(double interpolation, double deci) {
  float beta = 5.0;
  float trans_width = 0.5 - 0.4;
  float mid_transition_band = 0.5 - trans_width / 2;

#if GNURADIO_VERSION < 0x030900
  std::vector<float> result = gr::filter::firdes::low_pass(
      interpolation,
      1,
      mid_transition_band / interpolation,
      trans_width / interpolation,
      gr::filter::firdes::WIN_KAISER,
      beta);
#else
  std::vector<float> result = gr::filter::firdes::low_pass(
      interpolation,
      1,
      mid_transition_band / interpolation,
      trans_width / interpolation,
      gr::fft::window::WIN_KAISER,
      beta);
#endif
  return result;
}

analog_recorder_sptr make_analog_recorder(Source *src, Recorder_Type type) {
  return gnuradio::get_initial_sptr(new analog_recorder(src, static_cast<System*>(nullptr), type, Tone_Config{}));
}

analog_recorder_sptr make_analog_recorder(Source *src, Recorder_Type type, const Tone_Config &tone_config) {
  return gnuradio::get_initial_sptr(new analog_recorder(src, static_cast<System*>(nullptr), type, tone_config));
}

void analog_recorder::set_tau(float tau) {
  d_tau = tau;
  calculate_iir_taps(d_tau);
  if (deemph) {
    deemph->set_taps(d_fftaps, d_fbtaps);
  }
}

float analog_recorder::get_tau() const {
  return d_tau;
}


/*! \brief Calculate taps for FM de-emph IIR filter. */
void analog_recorder::calculate_iir_taps(float tau) {
  // copied from fm_emph.py in gr-analog
  double w_c;  // Digital corner frequency
  double w_ca; // Prewarped analog corner frequency
  double k, z1, p1, b0;
  double fs = static_cast<float>(system_channel_rate);

  w_c = 1.0f / tau;
  w_ca = 2.0f * fs * std::tan(w_c / (2.0f * fs));

  // Resulting digital pole, zero, and gain term from the bilinear
  // transformation of H(s) = w_ca / (s + w_ca) to
  // H(z) = b0 (1 - z1 z^-1)/(1 - p1 z^-1)
  k = -w_ca / (2.0f * fs);
  z1 = -1.0f;
  p1 = (1.0f + k) / (1.0f - k);
  b0 = -k / (1.0f - k);

  d_fftaps[0] = b0;
  d_fftaps[1] = -z1 * b0;
  d_fbtaps[0] = 1.0f;
  d_fbtaps[1] = -p1;
}

analog_recorder::analog_recorder(Source *src, System *system, Recorder_Type type, const Tone_Config &tone_config)
    : gr::hier_block2("analog_recorder",
                      gr::io_signature::make(1, 1, sizeof(gr_complex)),
                      gr::io_signature::make(0, 0, sizeof(float))),
      Recorder(type) {
  // int nchars;

  source = src;
  this->system = system;
  chan_freq = source->get_center();
  center_freq = source->get_center();
  config = source->get_config();
  input_rate = source->get_rate();
  squelch_db = 0;
  talkgroup = 0;
  recording_count = 0;
  recording_duration = 0;

  rec_num = rec_counter++;
  state = INACTIVE;

  timestamp = time(NULL);
  starttime = time(NULL);

  bool use_streaming = false;

  this->tone_config = tone_config;

  if (config != NULL) {
    use_streaming = config->enable_audio_streaming;
  }

  if (type == ANALOGC) {
    conventional = true;
  } else {
    conventional = false;
  }

  int samp_per_sym        = 2;
  double bandwidth = 12000;
  system_channel_rate = 96000; // 4800 * samp_per_sym;
  wav_sample_rate = 16000;     // Must be an integer decimation of system_channel_rate

  // The Prefilter provides the initial squelch for the channel
  prefilter = xlat_channelizer::make(input_rate, samp_per_sym, system_channel_rate / samp_per_sym, bandwidth, center_freq, true);
  prefilter->set_analog_squelch(true);

  //  based on squelch code form ham2mon
  // set low -200 since its after demod and its just gate for previous squelch so that the audio
  // recording doesn't contain blank spaces between transmissions
  squelch_two = gr::analog::pwr_squelch_ff::make(-200, 0.01, 0, true);

  // Sub-audible tone gating / identification blocks (new design).
  // The audio chain below taps these in at the wav_sample_rate point (16 kHz)
  // rather than the system_channel_rate point — CTCSS is 67-254 Hz and DCS
  // is ~134 baud, both well within 16 kHz Nyquist, and the blocks decimate
  // internally to ~1 kHz so running them at 16 kHz vs 96 kHz costs only a
  // few extra taps in the internal anti-alias LPF.
  switch (tone_config.mode) {
  case TONE_OFF:
    // No gate, no detector. Audio passes through unchanged from decim_audio.
    break;
  case TONE_CTCSS:
    // CTCSS gates the audio path; DCS detector runs as a side-chain so we
    // still spot a wrong-agency DCS keyup at end-of-call.
    ctcss_block = gr::blocks::ctcss_squelch_ff::make(wav_sample_rate, static_cast<float>(tone_config.ctcss_hz), true);
    dcs_block   = gr::blocks::dcs_squelch_ff::make(wav_sample_rate, 0, false, false);
    ctcss_block_in_path = true;
    dcs_block_in_path   = false;
    break;
  case TONE_DCS:
    // DCS gates the audio path; CTCSS detector runs as a side-chain.
    dcs_block   = gr::blocks::dcs_squelch_ff::make(wav_sample_rate, tone_config.dcs_code, tone_config.dcs_inverted, true);
    ctcss_block = gr::blocks::ctcss_squelch_ff::make(wav_sample_rate, 0.0f, false);
    ctcss_block_in_path = false;
    dcs_block_in_path   = true;
    break;
  case TONE_SEARCH:
    // No gating; both detectors run as side-chains so end-of-call can
    // report whichever scored higher.
    ctcss_block = gr::blocks::ctcss_squelch_ff::make(wav_sample_rate, 0.0f, false);
    dcs_block   = gr::blocks::dcs_squelch_ff::make(wav_sample_rate, 0, false, false);
    ctcss_block_in_path = false;
    dcs_block_in_path   = false;
    break;
  }
  // k = quad_rate/(2*math.pi*max_dev) = 48k / (6.283185*5000) = 1.527

  int d_max_dev = 5000;
  /* demodulator gain */
  quad_gain = system_channel_rate / (2.0 * M_PI * d_max_dev);
  demod = gr::analog::quadrature_demod_cf::make(quad_gain);
  levels = gr::blocks::multiply_const_ff::make(1); // 33);
  converter = gr::blocks::float_to_short::make(1, 32767);

  /* de-emphasis */
  d_tau = (system != nullptr) ? system->get_tau() : 0.000075f;  // Default to 75us if system is not provided
  d_fftaps.resize(2);
  d_fbtaps.resize(2);
  calculate_iir_taps(d_tau);
  deemph = gr::filter::iir_filter_ffd::make(d_fftaps, d_fbtaps, false);

  audio_resampler_taps = design_filter(1, (system_channel_rate / wav_sample_rate)); // Calculated to make sample rate changable -- must be an integer

  BOOST_LOG_TRIVIAL(info) << "Audio Resampler Taps: " << audio_resampler_taps.size() << " Decimation: " << (system_channel_rate / wav_sample_rate);
  // downsample from 48k to 8k
  decim_audio = gr::filter::fir_filter_fff::make((system_channel_rate / wav_sample_rate), audio_resampler_taps); // Calculated to make sample rate changable

  // tm *ltm = localtime(&starttime);

  wav_sink = gr::blocks::transmission_sink::make(1, wav_sample_rate, 16); //  Configurable

  if (use_streaming) {
    BOOST_LOG_TRIVIAL(info) << "\t Creating plugin sink..." << std::endl;
    plugin_sink = gr::blocks::plugin_wrapper_impl::make(std::bind(&analog_recorder::plugin_callback_handler, this, std::placeholders::_1, std::placeholders::_2));
    BOOST_LOG_TRIVIAL(info) << "\t Plugin sink created!" << std::endl;
  }

  BOOST_LOG_TRIVIAL(info) << "\t Creating decoder sink..." << std::endl;
  decoder_sink = gr::blocks::decoder_wrapper_impl::make(wav_sample_rate, std::bind(&analog_recorder::decoder_callback_handler, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  BOOST_LOG_TRIVIAL(info) << "\t Decoder sink created!" << std::endl;

  // Analog audio band pass from 300 to 3000 Hz
  // can't use gnuradio.filter.firdes.band_pass since we have different transition widths
  // 300 Hz high pass (275-325 Hz): removes CTCSS/DCS and Type II 150 bps Low Speed Data (LSD), or "FSK wobble"
#if GNURADIO_VERSION < 0x030900
  high_f_taps = gr::filter::firdes::high_pass(1, wav_sample_rate, 300, 50, gr::filter::firdes::WIN_HANN); // Configurable
  low_f_taps = gr::filter::firdes::low_pass(1, wav_sample_rate, 3250, 500, gr::filter::firdes::WIN_HANN);
#else
  high_f_taps = gr::filter::firdes::high_pass(1, wav_sample_rate, 300, 50, gr::fft::window::WIN_HANN); // Configurable
  low_f_taps = gr::filter::firdes::low_pass(1, wav_sample_rate, 3250, 500, gr::fft::window::WIN_HANN);
#endif

  high_f = gr::filter::fir_filter_fff::make(1, high_f_taps);
  // 3000 Hz low pass (3000-3500 Hz)

  low_f = gr::filter::fir_filter_fff::make(1, low_f_taps);

  // Always: RF → channelized → demod → deemph → decim to 16 kHz audio.
  connect(self(), 0, prefilter, 0);
  connect(prefilter, 0, demod, 0);
  connect(demod, 0, deemph, 0);
  connect(deemph, 0, decim_audio, 0);

  // Audio gate. The tone block (if any) sits between decim_audio and the
  // downstream branches so both the wav writer and the decoder_sink see the
  // gated signal — matches the prior behaviour where ctcss_squelch_ff sat
  // before decim_audio. The side-chain detector (if present) taps
  // decim_audio directly so it can still see ungated audio and identify
  // wrong-agency keyups even when the configured gate is closed.
  if (ctcss_block_in_path) {
    connect(decim_audio, 0, ctcss_block, 0);
    connect(ctcss_block, 0, decoder_sink, 0);
    connect(ctcss_block, 0, high_f, 0);
  } else if (dcs_block_in_path) {
    connect(decim_audio, 0, dcs_block, 0);
    connect(dcs_block, 0, decoder_sink, 0);
    connect(dcs_block, 0, high_f, 0);
  } else {
    connect(decim_audio, 0, decoder_sink, 0);
    connect(decim_audio, 0, high_f, 0);
  }

  // Side-chain detect-only blocks. Their output is terminated in a
  // null_sink so the flow graph is well-formed; only their internal
  // get_verdict() state is consumed (at end-of-call by stop()).
  if (ctcss_block && !ctcss_block_in_path) {
    connect(decim_audio, 0, ctcss_block, 0);
    ctcss_null_sink = gr::blocks::null_sink::make(sizeof(float));
    connect(ctcss_block, 0, ctcss_null_sink, 0);
  }
  if (dcs_block && !dcs_block_in_path) {
    connect(decim_audio, 0, dcs_block, 0);
    dcs_null_sink = gr::blocks::null_sink::make(sizeof(float));
    connect(dcs_block, 0, dcs_null_sink, 0);
  }

  connect(high_f, 0, low_f, 0);
  connect(low_f, 0, squelch_two, 0);
  connect(squelch_two, 0, levels, 0);
  connect(levels, 0, converter, 0);
  connect(converter, 0, wav_sink, 0);

  if (use_streaming) {
    connect(converter, 0, plugin_sink, 0);
  }
}

analog_recorder::~analog_recorder() {}

long analog_recorder::get_wav_hz() { return wav_sample_rate; };

State analog_recorder::get_state() {
  return wav_sink->get_state();
}

double analog_recorder::since_last_write() {
  time_t now = time(NULL);
  return now - wav_sink->get_stop_time();
}

int analog_recorder::get_num() {
  return rec_num;
}

std::vector<Transmission> analog_recorder::get_transmission_list() {
  return wav_sink->get_transmission_list();
}

void analog_recorder::stop() {
  if (state == ACTIVE) {
    recording_duration += wav_sink->length_in_seconds();
    state = INACTIVE;
    set_enabled(false);
    wav_sink->stop_recording();
  } else {

    BOOST_LOG_TRIVIAL(error) << "analog_recorder.cc: Stopping an inactive Logger \t[ " << rec_num << " ] - freq[ " << format_freq(chan_freq) << "] \t talkgroup[ " << talkgroup << " ]";
  }

  decoder_sink->set_mdc_enabled(false);
  decoder_sink->set_fsync_enabled(false);
  decoder_sink->set_star_enabled(false);
  decoder_sink->set_tps_enabled(false);

  // ----- Tone identification verdict ------------------------------------
  // Pull verdicts from whichever new sub-audible blocks ran and pick the
  // higher-confidence one. The configured (gated) block has the strongest
  // claim when both fire; in TONE_SEARCH the higher confidence wins.
  tone_result = Tone_Result{};
  switch (tone_config.mode) {
  case TONE_OFF:    tone_result.mode = "off";    break;
  case TONE_CTCSS:  tone_result.mode = "ctcss";  break;
  case TONE_DCS:    tone_result.mode = "dcs";    break;
  case TONE_SEARCH: tone_result.mode = "search"; break;
  }

  float ctcss_conf = 0.0f;
  float dcs_conf   = 0.0f;
  std::string ctcss_det, dcs_det;

  if (ctcss_block) {
    auto v = ctcss_block->get_verdict();
    if (v.detected_hz > 0.0f && v.confidence > 0.0f) {
      char buf[16];
      // One decimal place matches scanner/CHIRP display convention for CTCSS.
      snprintf(buf, sizeof(buf), "%.1f", v.detected_hz);
      ctcss_det = buf;
      ctcss_conf = v.confidence;
    }
  }
  if (dcs_block) {
    auto v = dcs_block->get_verdict();
    if (v.detected_code != 0 && v.confidence > 0.0f) {
      // In search mode the receiver genuinely can't tell apart cyclic-class
      // members — e.g. D703I and D565N are literally the same shift-register
      // pattern at different rotations on the air, so neither is "wrong" and
      // we report all class members so the operator can see what the
      // transmission could be. In configured mode the rewrite picked the
      // configured form so we just print that single name.
      if (tone_config.mode == TONE_SEARCH && v.aliases.size() > 1) {
        std::string s;
        for (const auto &kv : v.aliases) {
          if (!s.empty()) s += "/";
          char b[8];
          snprintf(b, sizeof(b), "D%03d%c", kv.first, kv.second ? 'I' : 'N');
          s += b;
        }
        dcs_det = s;
      } else {
        char buf[16];
        snprintf(buf, sizeof(buf), "D%03d%c", v.detected_code, v.detected_inverted ? 'I' : 'N');
        dcs_det = buf;
      }
      dcs_conf = v.confidence;
    }
  }

  // Pick the verdict to report. Prefer the configured-gate block when both
  // fire (operator's expectation: a CTCSS-configured channel should show
  // CTCSS unless DCS clearly won the call). For SEARCH mode it's purely
  // higher confidence.
  if (tone_config.mode == TONE_CTCSS) {
    if (!ctcss_det.empty()) { tone_result.detected = ctcss_det; tone_result.confidence = ctcss_conf; }
    else if (!dcs_det.empty()) { tone_result.detected = dcs_det; tone_result.confidence = dcs_conf; }
  } else if (tone_config.mode == TONE_DCS) {
    if (!dcs_det.empty()) { tone_result.detected = dcs_det; tone_result.confidence = dcs_conf; }
    else if (!ctcss_det.empty()) { tone_result.detected = ctcss_det; tone_result.confidence = ctcss_conf; }
  } else if (tone_config.mode == TONE_SEARCH) {
    // Search-mode tiebreak: simple max-confidence comparison. With the
    // phase-diversity DCS confidence now in place, a CTCSS-aliased false
    // DCS lock scores ~0.05-0.15 while a real DCS lock scores ~0.9-1.0;
    // a CTCSS detector lock with all four verdict guards passed scores
    // ~0.85-0.95. So whichever returns the higher confidence is genuinely
    // the better identification — no need for a CTCSS priority override
    // (an earlier version had one, which incorrectly forced CTCSS to win
    // even when a real DCS keyup was correctly identified at conf 1.0).
    const bool have_ctcss = !ctcss_det.empty();
    const bool have_dcs   = !dcs_det.empty();
    if (have_ctcss && have_dcs) {
      if (ctcss_conf >= dcs_conf) {
        tone_result.detected   = ctcss_det;
        tone_result.confidence = ctcss_conf;
      } else {
        tone_result.detected   = dcs_det;
        tone_result.confidence = dcs_conf;
      }
    } else if (have_ctcss) {
      tone_result.detected   = ctcss_det;
      tone_result.confidence = ctcss_conf;
    } else if (have_dcs) {
      tone_result.detected   = dcs_det;
      tone_result.confidence = dcs_conf;
    }
  }
  // TONE_OFF leaves tone_result.detected empty.

  if (tone_config.mode != TONE_OFF) {
    std::string loghdr;
    if (call != NULL) {
      loghdr = log_header(call->get_short_name(), call->get_call_num(), call->get_talkgroup_display(), chan_freq);
    } else {
      loghdr = "[?]\t[0m\t";
    }
    if (!tone_result.detected.empty()) {
      BOOST_LOG_TRIVIAL(info) << loghdr << "[36mTone Result:[0m " << tone_result.detected
                              << " (mode=" << tone_result.mode
                              << " conf=" << std::fixed << std::setprecision(2) << tone_result.confidence << ")";
    } else {
      BOOST_LOG_TRIVIAL(info) << loghdr << "[36mTone Result:[0m none (mode=" << tone_result.mode << ")";
    }
  }
}

void analog_recorder::process_message_queues() {
  decoder_sink->process_message_queues();
}

bool analog_recorder::is_analog() {
  return true;
}

bool analog_recorder::is_active() {
  if (state == ACTIVE) {
    return true;
  } else {
    return false;
  }
}

bool analog_recorder::is_enabled() {
  return source->is_selector_port_enabled(selector_port);
}

void analog_recorder::set_enabled(bool enabled) {
  source->set_selector_port_enabled(selector_port, enabled);
}

bool analog_recorder::is_squelched() {
  // Combine the RF power squelch (prefilter) with whichever sub-audible gate
  // (CTCSS or DCS) is sitting in the audio path. The recorder counts as
  // "squelched" when either gate is closed — RF gone, OR carrier present but
  // configured tone not detected. Side-chain detect-only blocks don't gate
  // audio so they are intentionally ignored here. In TONE_OFF / TONE_SEARCH
  // neither block is in the path, so behaviour matches the old prefilter-
  // only semantics.
  if (prefilter->is_squelched()) return true;
  if (ctcss_block_in_path && ctcss_block && !ctcss_block->is_unmuted()) return true;
  if (dcs_block_in_path   && dcs_block   && !dcs_block->is_unmuted())   return true;
  return false;
}

double analog_recorder::get_pwr() {
  return prefilter->get_pwr();
}

bool analog_recorder::is_idle() {
  // Same combined semantics as is_squelched: a configured-gate tone block
  // counts as a closer. Without this, a wrong-tone keyup (RF present, tone
  // wrong → audio gated to zeros and dropped at squelch_two) would never
  // increment manage_conventional_call's idle_count and the call would
  // sit open until RF eventually dropped. Pre-existing behaviour did the
  // same thing because the stock ctcss_squelch_ff didn't expose its
  // muted state to the recorder either.
  if (state != ACTIVE) return true;
  if (prefilter->is_squelched()) return true;
  if (ctcss_block_in_path && ctcss_block && !ctcss_block->is_unmuted()) return true;
  if (dcs_block_in_path   && dcs_block   && !dcs_block->is_unmuted())   return true;
  return false;
}

long analog_recorder::get_talkgroup() {
  return talkgroup;
}

double analog_recorder::get_freq() {
  return chan_freq;
}

int analog_recorder::get_freq_error() { // get frequency error from FLL and convert to Hz
  return prefilter->get_freq_error();
}

void analog_recorder::set_source(long src) {
  wav_sink->set_source(src);
}

Source *analog_recorder::get_source() {
  return source;
}

int analog_recorder::lastupdate() {
  return time(NULL) - timestamp;
}

long analog_recorder::elapsed() {
  return time(NULL) - starttime;
}

time_t analog_recorder::get_start_time() {
  return starttime;
}

double analog_recorder::get_current_length() {
  return wav_sink->total_length_in_seconds();
}

void analog_recorder::tune_freq(double f) {
  chan_freq = f;
  int offset_amount = (center_freq - f);

  prefilter->tune_offset(offset_amount);
}

void analog_recorder::decoder_callback_handler(long unitId, const char *signaling_type, gr::blocks::SignalType signal) {
  if (call != NULL) {
    wav_sink->set_source(unitId);
    plugman_signal(unitId, signaling_type, signal, call, call->get_system(), this);
  } else {
    plugman_signal(unitId, signaling_type, signal, NULL, NULL, this);
  }
}

void analog_recorder::plugin_callback_handler(int16_t *samples, int sampleCount) {
  plugman_audio_callback(call, this, samples, sampleCount);
}

void analog_recorder::setup_decoders_for_system(System *system) {
  decoder_sink->set_mdc_enabled(system->get_mdc_enabled());
  decoder_sink->set_fsync_enabled(system->get_fsync_enabled());
  decoder_sink->set_star_enabled(system->get_star_enabled());
  decoder_sink->set_tps_enabled(system->get_tps_enabled());
}

bool analog_recorder::start(Call *call) {
  starttime = time(NULL);
  System *system = call->get_system();
  this->call = call;

  setup_decoders_for_system(call->get_system());

  // Reset sub-audible block state per transmission so the verdict reflects
  // only this call. Both blocks (when instantiated) maintain cumulative
  // scoring across the audio they see; without reset() a quiet call after
  // a confidently-identified earlier one would still report the prior tone.
  if (ctcss_block) ctcss_block->reset();
  if (dcs_block)   dcs_block->reset();
  tone_result = Tone_Result{};

  talkgroup = call->get_talkgroup();
  chan_freq = call->get_freq();



  // BOOST_LOG_TRIVIAL(error) << "Setting squelch to: " << squelch_db << " block says: " << squelch->threshold();
  
  levels->set_k(system->get_analog_levels());
  int d_max_dev = system->get_max_dev();
  prefilter->set_max_dev(d_max_dev);
  quad_gain = system_channel_rate / (2.0 * M_PI * (d_max_dev + 1000));
  demod->set_gain(quad_gain);
  int offset_amount = (center_freq - chan_freq);
  prefilter->tune_offset(offset_amount);

  wav_sink->start_recording(call);

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
  
  std::string loghdr = log_header(call->get_short_name(),call->get_call_num(),this->call->get_talkgroup_display(),chan_freq);
  BOOST_LOG_TRIVIAL(info) << loghdr << "\u001b[32mStarting Analog Recorder Num [" << rec_num << "]\u001b[0m \tSquelch: " << squelch_db << " Max Dev: " << d_max_dev << " Gain: " << quad_gain;
  prefilter->set_squelch_db(squelch_db);
  return true;
}

double analog_recorder::get_output_sample_rate() {
  return wav_sample_rate;
}
