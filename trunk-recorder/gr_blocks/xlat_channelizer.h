#ifndef XLAT_CHANNELIZER_H
#define XLAT_CHANNELIZER_H

#include <boost/log/trivial.hpp>
#include <iomanip>

#include "./rms_agc.h"
#include "./freq_xlating_fft_filter.h"
#include "./pwr_squelch_cc.h"
#include <gnuradio/blocks/copy.h>
#include <gnuradio/digital/fll_band_edge_cc.h>
#include <gnuradio/filter/fft_filter_ccc.h>
#include <gnuradio/filter/fft_filter_ccf.h>
#include <gnuradio/filter/firdes.h>
#include <gnuradio/filter/freq_xlating_fir_filter.h>
#include <gnuradio/filter/pfb_arb_resampler_ccf.h>
#include <gnuradio/hier_block2.h>

#if GNURADIO_VERSION < 0x030800
#include <gnuradio/analog/sig_source_c.h>
#include <gnuradio/blocks/multiply_cc.h>
#include <gnuradio/blocks/multiply_const_ff.h>
#include <gnuradio/blocks/multiply_const_ss.h>

#else
#include <gnuradio/analog/sig_source.h>
#include <gnuradio/blocks/multiply.h>
#include <gnuradio/blocks/multiply_const.h>
#endif

#include "../formatter.h"
#include "../global_structs.h"

class xlat_channelizer : public gr::hier_block2 {
public:
#if GNURADIO_VERSION < 0x030900
  typedef boost::shared_ptr<xlat_channelizer> sptr;
#else
  typedef std::shared_ptr<xlat_channelizer> sptr;
#endif

  static sptr make(double input_rate, int samples_per_symbol, double symbol_rate, double bandwidth, double center_freq, bool use_squelch, double excess_bw=default_excess_bw);
  xlat_channelizer(double input_rate, int samples_per_symbol, double symbol_rate, double bandwidth, double center_freq, bool use_squelch, double excess_bw);

  struct DecimSettings {
    long decim;
    long decim2;
  };

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
  void tune_offset(double f);
  void set_samples_per_symbol(int samples_per_symbol);
  void set_squelch_db(double squelch_db);
  void set_analog_squelch(bool analog_squelch);
  void set_max_dev(double max_dev); 

private:
  bool double_decim;
  long if1;
  long if2;
  double d_center_freq;
  double d_input_rate;
  double d_bandwidth;
  double d_system_channel_rate;
  int d_samples_per_symbol;
  double d_symbol_rate;

  bool d_use_squelch;
  long symbol_rate;
  double initial_rate;
  double squelch_db;
  long decim;

  // gr::filter::freq_xlating_fir_filter<gr_complex, gr_complex, float>::sptr freq_xlat;
  freq_xlating_fft_filter_sptr freq_xlat;
  std::vector<float> arb_taps;
  std::vector<gr_complex> bandpass_filter_coeffs;
  std::vector<float> lowpass_filter_coeffs;
  std::vector<float> cutoff_filter_coeffs;

  gr::analog::pwr_squelch_cc::sptr squelch;
  gr::digital::fll_band_edge_cc::sptr fll_band_edge;
  gr::blocks::rms_agc::sptr rms_agc;

  gr::analog::sig_source_c::sptr lo;
  gr::analog::sig_source_c::sptr bfo;
  gr::blocks::multiply_cc::sptr mixer;

  gr::filter::fft_filter_ccc::sptr bandpass_filter;
  gr::filter::fft_filter_ccf::sptr lowpass_filter;
  gr::filter::fft_filter_ccf::sptr channel_lpf;
  gr::filter::fft_filter_ccf::sptr cutoff_filter;

  gr::filter::pfb_arb_resampler_ccf::sptr arb_resampler;

  static DecimSettings get_decim(long speed);
};

#endif