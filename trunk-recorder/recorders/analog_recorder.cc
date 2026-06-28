
#include "analog_recorder.h"
#include "../formatter.h"
#include "../gr_blocks/decoder_wrapper_impl.h"
#include "../gr_blocks/plugin_wrapper_impl.h"
#include "../gr_blocks/transmission_sink.h"
#include "../plugin_manager/plugin_manager.h"
#include "../recorder_globals.h"

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
  return gnuradio::get_initial_sptr(new analog_recorder(src, static_cast<System*>(nullptr), type, -1, false, 0.0));
}

analog_recorder_sptr make_analog_recorder(Source *src, Recorder_Type type, float tone_freq) {
  return gnuradio::get_initial_sptr(new analog_recorder(src, static_cast<System*>(nullptr), type, tone_freq, false, 0.0));
}

analog_recorder_sptr make_analog_recorder(Source *src, Recorder_Type type, float tone_freq, bool am_mode, double channel_bandwidth) {
  return gnuradio::get_initial_sptr(new analog_recorder(src, static_cast<System*>(nullptr), type, tone_freq, am_mode, channel_bandwidth));
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

analog_recorder::analog_recorder(Source *src, System *system, Recorder_Type type, float tone_freq, bool am_mode, double channel_bandwidth)
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

  this->am_mode = am_mode;
  this->d_channel_bandwidth = channel_bandwidth;
  // Defaults are populated later, after we know whether we're AM or FM, so
  // we can pick a sensible audio passband.
  d_audio_high_hz = 0;
  d_audio_low_hz = 0;

  // CTCSS tone squelch is FM-only. Silently ignore a configured tone for an AM
  // channel rather than misconfigure the audio chain.
  if (tone_freq > 0 && !am_mode) {
    use_tone_squelch = true;
    this->tone_freq = tone_freq;
  } else {
    use_tone_squelch = false;
    this->tone_freq = 0;
  }

  if (config != NULL) {
    use_streaming = config->enable_audio_streaming;
  }

  if (type == ANALOGC) {
    conventional = true;
  } else {
    conventional = false;
  }

  int samp_per_sym        = 2;
  // Airband AM uses ~8 kHz channel spacing (25 kHz raster, but the carrier
  // occupies ~6 kHz of useful audio). FM voice channels are ~12.5 kHz wide.
  double default_bandwidth_fm = 12000;
  double default_bandwidth_am = 8000;
  double bandwidth = channel_bandwidth > 0 ? channel_bandwidth
                                           : (am_mode ? default_bandwidth_am : default_bandwidth_fm);
  system_channel_rate = 96000; // 4800 * samp_per_sym;
  wav_sample_rate = 16000;     // Must be an integer decimation of system_channel_rate

  // The Prefilter provides the initial squelch for the channel
  prefilter = xlat_channelizer::make(input_rate, samp_per_sym, system_channel_rate / samp_per_sym, bandwidth, center_freq, true);
  prefilter->set_analog_squelch(true);

  //  based on squelch code form ham2mon
  // set low -200 since its after demod and its just gate for previous squelch so that the audio
  // recording doesn't contain blank spaces between transmissions
  squelch_two = gr::analog::pwr_squelch_ff::make(-200, 0.01, 0, true);

  if (use_tone_squelch) {
    tone_squelch = gr::analog::ctcss_squelch_ff::make(system_channel_rate, this->tone_freq, 0.01, 0, 0, false);
  }
  // k = quad_rate/(2*math.pi*max_dev) = 48k / (6.283185*5000) = 1.527

  int d_max_dev = 5000;
  /* demodulator gain */
  quad_gain = system_channel_rate / (2.0 * M_PI * d_max_dev);

  levels = gr::blocks::multiply_const_ff::make(1); // 33);
  converter = gr::blocks::float_to_short::make(1, 32767);

  if (am_mode) {
    // Standard AM-DSB envelope detection: |z| - 1.0. The channelizer's
    // rms_agc upstream normalizes the carrier so the average magnitude
    // sits near 1.0; subtracting that constant leaves just the audio.
    // Any residual DC bias is mopped up by the downstream high_f HPF.
    am_envelope = gr::blocks::complex_to_mag::make(1);
    am_dc_remove = gr::blocks::add_const_ff::make(-1.0f);
  } else {
    demod = gr::analog::quadrature_demod_cf::make(quad_gain);
  }

  /* de-emphasis (FM only) */
  d_tau = (system != nullptr) ? system->get_tau() : 0.000075f;  // Default to 75us if system is not provided
  d_fftaps.resize(2);
  d_fbtaps.resize(2);
  if (!am_mode) {
    calculate_iir_taps(d_tau);
    deemph = gr::filter::iir_filter_ffd::make(d_fftaps, d_fbtaps, false);
  }

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

  // Analog audio band pass. Defaults: FM voice 300 Hz HPF (removes CTCSS/DCS
  // and Type II 150 bps LSD/FSK wobble) and 3250 Hz LPF. Airband AM voice
  // is bandlimited to ~300-3000 Hz at the transmit end, so the AM defaults
  // match. Both cutoffs are exposed as system config options and applied at
  // start() time.
  int default_high_hz = 300;
  int default_low_hz = am_mode ? 3000 : 3250;
  d_audio_high_hz = default_high_hz;
  d_audio_low_hz = default_low_hz;
#if GNURADIO_VERSION < 0x030900
  high_f_taps = gr::filter::firdes::high_pass(1, wav_sample_rate, d_audio_high_hz, 50, gr::filter::firdes::WIN_HANN);
  low_f_taps = gr::filter::firdes::low_pass(1, wav_sample_rate, d_audio_low_hz, 500, gr::filter::firdes::WIN_HANN);
#else
  high_f_taps = gr::filter::firdes::high_pass(1, wav_sample_rate, d_audio_high_hz, 50, gr::fft::window::WIN_HANN);
  low_f_taps = gr::filter::firdes::low_pass(1, wav_sample_rate, d_audio_low_hz, 500, gr::fft::window::WIN_HANN);
#endif

  high_f = gr::filter::fir_filter_fff::make(1, high_f_taps);
  low_f = gr::filter::fir_filter_fff::make(1, low_f_taps);

  // using squelch
  connect(self(), 0, prefilter, 0);
  if (am_mode) {
    // AM-DSB envelope detection. No de-emphasis (FM-only), no CTCSS gate.
    connect(prefilter, 0, am_envelope, 0);
    connect(am_envelope, 0, am_dc_remove, 0);
    connect(am_dc_remove, 0, decim_audio, 0);
  } else {
    connect(prefilter, 0, demod, 0);
    connect(demod, 0, deemph, 0);
    if (use_tone_squelch) {
      connect(deemph, 0, tone_squelch, 0);
      connect(tone_squelch, 0, decim_audio, 0);
    } else {
      connect(deemph, 0, decim_audio, 0);
    }
  }

  connect(decim_audio, 0, decoder_sink, 0);
  connect(decim_audio, 0, high_f, 0);
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
  return prefilter->is_squelched();
}

double analog_recorder::get_pwr() {
  return prefilter->get_pwr();
}

bool analog_recorder::is_idle() {
  if (state == ACTIVE) {
    return prefilter->is_squelched();
  }
  return true;
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

  talkgroup = call->get_talkgroup();
  chan_freq = call->get_freq();



  // BOOST_LOG_TRIVIAL(error) << "Setting squelch to: " << squelch_db << " block says: " << squelch->threshold();
  
  levels->set_k(system->get_analog_levels());
  int d_max_dev = system->get_max_dev();
  prefilter->set_max_dev(d_max_dev);
  if (!am_mode) {
    quad_gain = system_channel_rate / (2.0 * M_PI * (d_max_dev + 1000));
    demod->set_gain(quad_gain);
  }
  int offset_amount = (center_freq - chan_freq);
  prefilter->tune_offset(offset_amount);

  // Honor per-system audio passband overrides. 0 means "keep the recorder's
  // mode-appropriate default" set at construction time. We re-derive taps
  // here so the recorder can be reconfigured without rebuilding the graph.
  int sys_high = system->get_audio_passband_low();
  int sys_low = system->get_audio_passband_high();
  if (sys_high > 0 && sys_high != d_audio_high_hz) {
    d_audio_high_hz = sys_high;
#if GNURADIO_VERSION < 0x030900
    high_f_taps = gr::filter::firdes::high_pass(1, wav_sample_rate, d_audio_high_hz, 50, gr::filter::firdes::WIN_HANN);
#else
    high_f_taps = gr::filter::firdes::high_pass(1, wav_sample_rate, d_audio_high_hz, 50, gr::fft::window::WIN_HANN);
#endif
    high_f->set_taps(high_f_taps);
  }
  if (sys_low > 0 && sys_low != d_audio_low_hz) {
    d_audio_low_hz = sys_low;
#if GNURADIO_VERSION < 0x030900
    low_f_taps = gr::filter::firdes::low_pass(1, wav_sample_rate, d_audio_low_hz, 500, gr::filter::firdes::WIN_HANN);
#else
    low_f_taps = gr::filter::firdes::low_pass(1, wav_sample_rate, d_audio_low_hz, 500, gr::fft::window::WIN_HANN);
#endif
    low_f->set_taps(low_f_taps);
  }

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
  if (am_mode) {
    BOOST_LOG_TRIVIAL(info) << loghdr << "\u001b[32mStarting Analog Recorder Num [" << rec_num << "]\u001b[0m \tMode: AM \tSquelch: " << squelch_db << " Audio HPF: " << d_audio_high_hz << " LPF: " << d_audio_low_hz;
  } else {
    BOOST_LOG_TRIVIAL(info) << loghdr << "\u001b[32mStarting Analog Recorder Num [" << rec_num << "]\u001b[0m \tSquelch: " << squelch_db << " Max Dev: " << d_max_dev << " Gain: " << quad_gain;
  }
  prefilter->set_squelch_db(squelch_db);
  return true;
}

double analog_recorder::get_output_sample_rate() {
  return wav_sample_rate;
}
