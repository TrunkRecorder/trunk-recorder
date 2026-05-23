#include "xlat_channelizer.h"

xlat_channelizer::sptr xlat_channelizer::make(double input_rate, int samples_per_symbol, double symbol_rate, double bandwidth, double center_freq, bool use_squelch, double excess_bw, bool use_fll) {
  return gnuradio::get_initial_sptr(new xlat_channelizer(input_rate, samples_per_symbol, symbol_rate, bandwidth, center_freq, use_squelch, excess_bw, use_fll));
}

const int xlat_channelizer::smartnet_samples_per_symbol;
const int xlat_channelizer::phase1_samples_per_symbol;
const int xlat_channelizer::phase2_samples_per_symbol;
const double xlat_channelizer::phase1_symbol_rate;
const double xlat_channelizer::phase2_symbol_rate;
const double xlat_channelizer::smartnet_symbol_rate;

xlat_channelizer::xlat_channelizer(double input_rate, int samples_per_symbol, double symbol_rate, double bandwidth, double center_freq, bool use_squelch, double excess_bw, bool use_fll)
    : gr::hier_block2("xlat_channelizer_ccf",
                      gr::io_signature::make(1, 1, sizeof(gr_complex)),
                      gr::io_signature::make(1, 1, sizeof(gr_complex))),
      d_center_freq(center_freq),
      d_input_rate(input_rate),
      d_bandwidth(bandwidth),
      d_samples_per_symbol(samples_per_symbol),
      d_symbol_rate(symbol_rate),
      d_use_squelch(use_squelch),
      d_use_fll(use_fll) {

  long channel_rate = d_symbol_rate * d_samples_per_symbol;
  const float pi = M_PI;

  // Input has already been frequency-translated and coarsely decimated by
  // the shared channelizer upstream. From here we just narrow to the
  // channel passband, resample to channel_rate, then run squelch/AGC/FLL.
  int decim = std::max(1, static_cast<int>(std::floor(d_input_rate / channel_rate)));
  double resampled_rate = d_input_rate / static_cast<double>(decim);

  std::vector<float> channel_lpf_taps = gr::filter::firdes::low_pass_2(1.0, d_input_rate, d_bandwidth / 2, d_bandwidth / 4, 60);
  channel_lpf = gr::filter::fft_filter_ccf::make(decim, channel_lpf_taps);

  // The shared channelizer puts the tuned bin at DC of its K-rate output.
  // Any signal energy at exactly the carrier (residual transmitter carrier,
  // a constant-amplitude artifact, or USRP DC bias when tuned near IF DC)
  // lands in the IFFT's DC bin and shows up as a strong tone at 0 Hz of the
  // baseband stream — right where the FLL needs to lock. The old
  // freq_xlating_fft_filter avoided this by translating to a continuous
  // (non-bin-quantized) frequency, but we don't have that luxury here.
  // A delay-line DC blocker punches a narrow notch at DC with minimal
  // effect on the in-band signal. D=32 gives a ~3 kHz notch at 96 kHz
  // and group delay ~62 samples, fine for 4800-baud P25.
  dc_blocker = gr::filter::dc_blocker_cc::make(32, true);

  double arb_rate = channel_rate / resampled_rate;
  double arb_size = 32;
  double arb_atten = 30;
  double percent = 0.80;

  BOOST_LOG_TRIVIAL(info) << "\t Xlating Channelizer post-filter - input_rate: " << d_input_rate << " channel_rate: " << channel_rate << " decim: " << decim << " resampled_rate: " << resampled_rate << " arb_rate: " << arb_rate << " channel_lpf_taps: " << channel_lpf_taps.size();

  if (arb_rate < 1.0) {
    double halfband = 0.5 * arb_rate;
    double bw = percent * halfband;
    double tb = (percent / 2.0) * halfband;
#if GNURADIO_VERSION < 0x030900
    arb_taps = gr::filter::firdes::low_pass_2(arb_size, arb_size, bw, tb, arb_atten, gr::filter::firdes::WIN_BLACKMAN_HARRIS);
#else
    arb_taps = gr::filter::firdes::low_pass_2(arb_size, arb_size, bw, tb, arb_atten, gr::fft::window::WIN_BLACKMAN_HARRIS);
#endif
    arb_resampler = gr::filter::pfb_arb_resampler_ccf::make(arb_rate, arb_taps);
  } else if (arb_rate > 1.0) {
    BOOST_LOG_TRIVIAL(error) << "xlat_channelizer: arb_rate > 1 (" << arb_rate << "); upstream channelizer rate (" << d_input_rate << ") is below channel_rate (" << channel_rate << ")";
    exit(1);
  }

  squelch = gr::analog::pwr_squelch_cc::make(squelch_db, 0.0001, 0, true);
  rms_agc = gr::blocks::rms_agc::make(0.45, 0.85);

  if (d_use_fll) {
    double fll_loop_bw_legacy = (2.0 * pi) / d_samples_per_symbol / 250;
#if GNURADIO_VERSION >= 0x030a0d
    double fll_loop_bw = (fll_loop_bw_legacy * fll_loop_bw_legacy * d_samples_per_symbol) /
                         ((1.0 + M_SQRT2 * fll_loop_bw_legacy + fll_loop_bw_legacy * fll_loop_bw_legacy) * 2.0 * pi);
#if GNURADIO_VERSION >= 0x030b00
    fll_band_edge = gr::digital::fll_band_edge_cc::make(d_samples_per_symbol, excess_bw, 2 * d_samples_per_symbol + 1, fll_loop_bw);
#else
    fll_band_edge = gr::digital::fll_band_edge_cc::make(d_samples_per_symbol, excess_bw, 2 * d_samples_per_symbol + 1, fll_loop_bw, true);
#endif
#else
    fll_band_edge = gr::digital::fll_band_edge_cc::make(d_samples_per_symbol, excess_bw, 2 * d_samples_per_symbol + 1, fll_loop_bw_legacy);
#endif
  }

  connect(self(), 0, dc_blocker, 0);
  connect(dc_blocker, 0, channel_lpf, 0);
  if (arb_rate == 1.0) {
    if (d_use_squelch) {
      connect(channel_lpf, 0, squelch, 0);
      connect(squelch, 0, rms_agc, 0);
    } else {
      connect(channel_lpf, 0, rms_agc, 0);
    }
  } else {
    connect(channel_lpf, 0, arb_resampler, 0);
    if (d_use_squelch) {
      connect(arb_resampler, 0, squelch, 0);
      connect(squelch, 0, rms_agc, 0);
    } else {
      connect(arb_resampler, 0, rms_agc, 0);
    }
  }

  if (d_use_fll) {
    connect(rms_agc, 0, fll_band_edge, 0);
    connect(fll_band_edge, 0, self(), 0);
  } else {
    // Band-edge FLL is matched to shaped (RRC) signals like P25; for SmartNet
    // NRZ FSK it doesn't lock cleanly. Callers opting out get the AGC output
    // directly and are expected to track the carrier downstream.
    connect(rms_agc, 0, self(), 0);
  }
}

int xlat_channelizer::get_freq_error() {
  if (!fll_band_edge) {
    return 0;
  }
  const float pi = M_PI;
  // The FLL operates at the recorder's channel rate (symbol_rate * sps).
  long channel_rate = static_cast<long>(d_symbol_rate * d_samples_per_symbol);
  return int((fll_band_edge->get_frequency() / (2 * pi)) * channel_rate);
}

bool xlat_channelizer::is_squelched() {
  return !squelch->unmuted();
}

double xlat_channelizer::get_pwr() {
  if (d_use_squelch) {
    return squelch->get_pwr();
  } else {
    return DB_UNSET;
  }
}

void xlat_channelizer::set_max_dev(double max_dev) {
  std::vector<float> channel_lpf_taps = gr::filter::firdes::low_pass_2(1.0, d_input_rate, max_dev, d_bandwidth / 2, 60);
  channel_lpf->set_taps(channel_lpf_taps);
}

void xlat_channelizer::set_squelch_db(double squelch_db) {
  squelch->set_threshold(squelch_db);
}

void xlat_channelizer::set_analog_squelch(bool analog_squelch) {
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

void xlat_channelizer::set_samples_per_symbol(int samples_per_symbol) {
  fll_band_edge->set_samples_per_symbol(samples_per_symbol);
}
