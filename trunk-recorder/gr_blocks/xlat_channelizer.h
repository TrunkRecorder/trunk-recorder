#ifndef XLAT_CHANNELIZER_H
#define XLAT_CHANNELIZER_H

#include <boost/log/trivial.hpp>
#include <iomanip>

#include "./rms_agc.h"
#include "./pwr_squelch_cc.h"
#include <gnuradio/blocks/copy.h>
#include <gnuradio/digital/fll_band_edge_cc.h>
#include <gnuradio/filter/fft_filter_ccf.h>
#include <gnuradio/filter/firdes.h>
#include <gnuradio/filter/pfb_arb_resampler_ccf.h>
#include <gnuradio/hier_block2.h>

#include "../formatter.h"
#include "../global_structs.h"

// Post-channelization filter chain for a single recorder. Input arrives
// already frequency-translated by the Source's shared_channelizer (so this
// block's input rate is the channelizer's output rate, NOT the SDR rate).
// Responsibilities that remain here: final per-channel low-pass, optional
// fractional resample to the target symbol×sps rate, power squelch, AGC,
// and the band-edge FLL for shaped-pulse digital signals.
//
// The original name "xlat" (translating) is retained for source-tree
// continuity; the translation step has moved up to the shared channelizer.
class xlat_channelizer : public gr::hier_block2 {
public:
#if GNURADIO_VERSION < 0x030900
  typedef boost::shared_ptr<xlat_channelizer> sptr;
#else
  typedef std::shared_ptr<xlat_channelizer> sptr;
#endif

  // input_rate is the rate the upstream shared_channelizer produces for
  // this port (typically ~96 kHz), not the SDR rate.
  static sptr make(double input_rate, int samples_per_symbol, double symbol_rate, double bandwidth, double center_freq, bool use_squelch, double excess_bw = default_excess_bw, bool use_fll = true);
  xlat_channelizer(double input_rate, int samples_per_symbol, double symbol_rate, double bandwidth, double center_freq, bool use_squelch, double excess_bw, bool use_fll);

  static constexpr float default_excess_bw = 0.2;
  static constexpr float smartnet_excess_bw = 0.35;
  static const int smartnet_samples_per_symbol = 5;
  static const int phase1_samples_per_symbol = 5;
  static const int phase2_samples_per_symbol = 4;
  static constexpr double phase1_symbol_rate = 4800;
  static constexpr double phase2_symbol_rate = 6000;
  static constexpr double smartnet_symbol_rate = 3600;
  static constexpr double channel_bandwidth = 12500;

  int get_freq_error();
  bool is_squelched();
  double get_pwr();
  void set_samples_per_symbol(int samples_per_symbol);
  void set_squelch_db(double squelch_db);
  void set_analog_squelch(bool analog_squelch);
  void set_max_dev(double max_dev);

private:
  double d_center_freq;
  double d_input_rate;
  double d_bandwidth;
  int d_samples_per_symbol;
  double d_symbol_rate;
  bool d_use_squelch;
  bool d_use_fll;
  double squelch_db;

  std::vector<float> arb_taps;

  gr::analog::pwr_squelch_cc::sptr squelch;
  gr::digital::fll_band_edge_cc::sptr fll_band_edge;
  gr::blocks::rms_agc::sptr rms_agc;

  gr::filter::fft_filter_ccf::sptr channel_lpf;
  gr::filter::pfb_arb_resampler_ccf::sptr arb_resampler;
};

#endif
