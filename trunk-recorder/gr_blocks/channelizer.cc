#include "channelizer.h"

channelizer::sptr channelizer::make(double input_rate, int samples_per_symbol, double symbol_rate, double center_freq, bool conventional) {

  return gnuradio::get_initial_sptr(new channelizer(input_rate, samples_per_symbol, symbol_rate, center_freq, conventional));
}

const int channelizer::smartnet_samples_per_symbol;
const int channelizer::phase1_samples_per_symbol;
const int channelizer::phase2_samples_per_symbol;
const double channelizer::phase1_symbol_rate;
const double channelizer::phase2_symbol_rate;
const double channelizer::smartnet_symbol_rate;

channelizer::DecimSettings channelizer::get_decim(long speed) {
  long s = speed;
  long if_freqs[] = {24000, 25000, 32000};
  DecimSettings decim_settings = {-1, -1};
  for (int i = 0; i < 3; i++) {
    long if_freq = if_freqs[i];
    if (s % if_freq != 0) {
      continue;
    }
    long q = s / if_freq;
    if (q & 1) {
      continue;
    }

    if ((q >= 40) && ((q & 3) == 0)) {
      decim_settings.decim = q / 4;
      decim_settings.decim2 = 4;
    } else {
      decim_settings.decim = q / 2;
      decim_settings.decim2 = 2;
    }
    BOOST_LOG_TRIVIAL(debug) << "P25 recorder Decim: " << decim_settings.decim << " Decim2:  " << decim_settings.decim2;
    return decim_settings;
  }
  BOOST_LOG_TRIVIAL(error) << "P25 recorder Decim: Nothing found";
  return decim_settings;
}

channelizer::channelizer(double input_rate, int samples_per_symbol, double symbol_rate, double center_freq, bool conventional)
    : gr::hier_block2("channelizer_ccf",
                      gr::io_signature::make(1, 1, sizeof(gr_complex)),
                      gr::io_signature::make(1, 1, sizeof(gr_complex))),
      d_input_rate(input_rate),
      d_samples_per_symbol(samples_per_symbol),
      d_symbol_rate(symbol_rate),
      d_center_freq(center_freq),
      d_conventional(conventional) {

  long channel_rate = d_symbol_rate * d_samples_per_symbol;
  // long if_rate = 12500;
  double resampled_rate;

  const float pi = M_PI;

  lo = gr::analog::sig_source_c::make(d_input_rate, gr::analog::GR_SIN_WAVE, 0, 1.0, 0.0);
  mixer = gr::blocks::multiply_cc::make();

  DecimSettings decim_settings = get_decim(input_rate);

  if (decim_settings.decim != -1) {
    double_decim = true;
    decim = decim_settings.decim;
    if1 = input_rate / decim_settings.decim;
    if2 = if1 / decim_settings.decim2;
    long fa = 6250;
    long fb = if2 / 2;

    bandpass_filter_coeffs = gr::filter::firdes::complex_band_pass(1.0, input_rate, -if1 / 2, if1 / 2, if1 / 2);
#if GNURADIO_VERSION < 0x030900
    lowpass_filter_coeffs = gr::filter::firdes::low_pass(1.0, if1, (fb + fa) / 2, fb - fa, gr::filter::firdes::WIN_HAMMING);
#else
    lowpass_filter_coeffs = gr::filter::firdes::low_pass(1.0, if1, (fb + fa) / 2, fb - fa, gr::fft::window::WIN_HAMMING);
#endif
    bandpass_filter = gr::filter::fft_filter_ccc::make(decim_settings.decim, bandpass_filter_coeffs);
    lowpass_filter = gr::filter::fft_filter_ccf::make(decim_settings.decim2, lowpass_filter_coeffs);
    resampled_rate = if2;
    BOOST_LOG_TRIVIAL(info) << "\t Channelizer two-stage decimator - Initial decimated rate: " << if1 << " Second decimated rate: " << if2 << " Resampled Rate: " << resampled_rate << " Bandpass Size: " << bandpass_filter_coeffs.size() << " Lowpass Size: " << lowpass_filter_coeffs.size();

    bfo = gr::analog::sig_source_c::make(if1, gr::analog::GR_SIN_WAVE, 0, 1.0, 0.0);
  } else {
    double_decim = false;
    long fa = 6250;
    long fb = fa + 1250;
    lo = gr::analog::sig_source_c::make(input_rate, gr::analog::GR_SIN_WAVE, 0, 1.0, 0.0);

#if GNURADIO_VERSION < 0x030900
    lowpass_filter_coeffs = gr::filter::firdes::low_pass(1.0, input_rate, (fb + fa) / 2, fb - fa, gr::filter::firdes::WIN_HAMMING);
#else
    lowpass_filter_coeffs = gr::filter::firdes::low_pass(1.0, input_rate, (fb + fa) / 2, fb - fa, gr::fft::window::WIN_HAMMING);
#endif
    decim = floor(input_rate / channel_rate);
    resampled_rate = input_rate / decim;
    lowpass_filter = gr::filter::fft_filter_ccf::make(decim, lowpass_filter_coeffs);
    BOOST_LOG_TRIVIAL(info) << "\t Channelizer single-stage decimator - Decim: " << decim << " Resampled Rate: " << resampled_rate << " Lowpass Size: " << lowpass_filter_coeffs.size();
  }

  // ARB Resampler
  double arb_rate = channel_rate / resampled_rate;
  BOOST_LOG_TRIVIAL(info) << "\t Channelizer ARB - Symbol Rate: " << channel_rate << " Resampled Rate: " << resampled_rate << " ARB Rate: " << arb_rate;
  double arb_size = 32;
  double arb_atten = 30; // was originally 100
  // Create a filter that covers the full bandwidth of the output signal

  // If rate >= 1, we need to prevent images in the output,
  // so we have to filter it to less than half the channel
  // width of 0.5.  If rate < 1, we need to filter to less
  // than half the output signal's bw to avoid aliasing, so
  // the half-band here is 0.5*rate.
  double percent = 0.80;

  if (arb_rate <= 1) {
    double halfband = 0.5 * arb_rate;
    double bw = percent * halfband;
    double tb = (percent / 2.0) * halfband;

// BOOST_LOG_TRIVIAL(info) << "Arb Rate: " << arb_rate << " Half band: " << halfband << " bw: " << bw << " tb: " <<
// tb;

// As we drop the bw factor, the optfir filter has a harder time converging;
// using the firdes method here for better results.
#if GNURADIO_VERSION < 0x030900
    arb_taps = gr::filter::firdes::low_pass_2(arb_size, arb_size, bw, tb, arb_atten, gr::filter::firdes::WIN_BLACKMAN_HARRIS);
#else
    arb_taps = gr::filter::firdes::low_pass_2(arb_size, arb_size, bw, tb, arb_atten, gr::fft::window::WIN_BLACKMAN_HARRIS);
#endif
  } else {
    BOOST_LOG_TRIVIAL(error) << "Something is probably wrong! Resampling rate too low";
    exit(1);
  }
  arb_resampler = gr::filter::pfb_arb_resampler_ccf::make(arb_rate, arb_taps);
  double sps = d_samples_per_symbol;
  double def_excess_bw = 0.2;
  // Squelch DB
  // on a trunked network where you know you will have good signal, a carrier
  // power squelch works well. real FM receviers use a noise squelch, where
  // the received audio is high-passed above the cutoff and then fed to a
  // reverse squelch. If the power is then BELOW a threshold, open the squelch.

  squelch = gr::analog::pwr_squelch_cc::make(squelch_db, 0.0001, 0, true);

  rms_agc = gr::blocks::rms_agc::make(0.45, 0.85);
  // Legacy FLL loop-filter bandwidth (OP25 historically uses 350 instead of 250).
  double fll_loop_bw_legacy = (2.0 * pi) / sps / 250;
#if GNURADIO_VERSION >= 0x030a0d
  // gnuradio PR #7890 reworked the FLL loop filter. The improved filter expects
  // bandwidth as BL*T (loop noise bandwidth * symbol period). Convert the legacy
  // gain so closed-loop dynamics roughly match the prior behavior.
  double fll_loop_bw = (fll_loop_bw_legacy * fll_loop_bw_legacy * sps) /
                       ((1.0 + M_SQRT2 * fll_loop_bw_legacy + fll_loop_bw_legacy * fll_loop_bw_legacy) * 2.0 * pi);
#if GNURADIO_VERSION >= 0x030b00
  // 3.11+: the improved filter is the only behavior; the opt-in bool was removed.
  fll_band_edge = gr::digital::fll_band_edge_cc::make(sps, def_excess_bw, 2 * sps + 1, fll_loop_bw);
#else
  // 3.10 backport: opt in via the new `improved_loop_filter` bool to silence the legacy warning.
  fll_band_edge = gr::digital::fll_band_edge_cc::make(sps, def_excess_bw, 2 * sps + 1, fll_loop_bw, true);
#endif
#else
  fll_band_edge = gr::digital::fll_band_edge_cc::make(sps, def_excess_bw, 2 * sps + 1, fll_loop_bw_legacy);
#endif

 
  if (double_decim) {
    connect(self(), 0, bandpass_filter, 0);
    connect(bandpass_filter, 0, mixer, 0);
    connect(bfo, 0, mixer, 1);
  } else {
    connect(self(), 0, mixer, 0);
    connect(lo, 0, mixer, 1);
  }
  connect(mixer, 0, lowpass_filter, 0);

  if (d_conventional) {
    if (arb_rate == 1.0) {
      connect(lowpass_filter, 0, squelch, 0);
      connect(squelch, 0, rms_agc, 0);
    } else {
      connect(lowpass_filter, 0, squelch, 0);
      connect(squelch, 0, arb_resampler, 0);
      connect(arb_resampler, 0, rms_agc, 0);
    }

    // connect(squelch, 0, self(), 0);
  } else {
    if (arb_rate == 1.0) {
      connect(lowpass_filter, 0, rms_agc, 0);
    } else {
      connect(lowpass_filter, 0, arb_resampler, 0);
      connect(arb_resampler, 0, rms_agc, 0);
    }
  }

  connect(rms_agc, 0, fll_band_edge, 0);
  connect(fll_band_edge, 0, self(), 0);
}

int channelizer::get_freq_error() { // get frequency error from FLL and convert to Hz
  const float pi = M_PI;
  long if_rate = 24000;
  return int((fll_band_edge->get_frequency() / (2 * pi)) * if_rate);
}

bool channelizer::is_squelched() {

  return !squelch->unmuted();
}

void channelizer::tune_offset(double f) {

  float freq = static_cast<float>(f);

  if (abs(freq) > ((d_input_rate / 2) - (if1 / 2))) {
    BOOST_LOG_TRIVIAL(info) << "Channelizer - Tune Offset: Freq exceeds limit: " << abs(freq) << " compared to: " << ((d_input_rate / 2) - (if1 / 2));
  }
  if (double_decim) {
    bandpass_filter_coeffs = gr::filter::firdes::complex_band_pass(1.0, d_input_rate, -freq - if1 / 2, -freq + if1 / 2, if1 / 2);
    bandpass_filter->set_taps(bandpass_filter_coeffs);
    float bfz = (static_cast<float>(decim) * -freq) / (float)d_input_rate;
    bfz = bfz - static_cast<int>(bfz);
    if (bfz < -0.5) {
      bfz = bfz + 1.0;
    }
    if (bfz > 0.5) {
      bfz = bfz - 1.0;
    }
    bfo->set_frequency(-bfz * if1);

  } else {
    lo->set_frequency(freq);
  }
}

void channelizer::set_squelch_db(double squelch_db) {
  squelch->set_threshold(squelch_db);
}

double channelizer::get_pwr() {
  if (d_conventional) {
    return squelch->get_pwr();
  } else {
    return 0;
  }
}

void channelizer::set_analog_squelch(bool analog_squelch) {
  if (analog_squelch) {
    squelch->set_alpha(0.01);
    squelch->set_ramp(10);
    squelch->set_gate(false);
  } else {
    squelch->set_alpha(0.0001);
    squelch->set_ramp(0);
    squelch->set_gate(true);
  }
}

void channelizer::set_samples_per_symbol(int samples_per_symbol) {
  fll_band_edge->set_samples_per_symbol(samples_per_symbol);
}