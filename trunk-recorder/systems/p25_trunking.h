#ifndef P25_TRUNKING_H
#define P25_TRUNKING_H

#define _USE_MATH_DEFINES

#include <cstdio>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/log/trivial.hpp>
#include <boost/shared_ptr.hpp>

#include <gnuradio/hier_block2.h>
#include <gnuradio/io_signature.h>

#include <gnuradio/block.h>

#if GNURADIO_VERSION < 0x030800
#include <gnuradio/analog/sig_source_c.h>
#include <gnuradio/blocks/multiply_cc.h>
#include <gnuradio/blocks/multiply_const_ff.h>
#include <gnuradio/filter/fir_filter_ccf.h>
#include <gnuradio/filter/fir_filter_fff.h>
#else
#include <gnuradio/analog/sig_source.h>
#include <gnuradio/blocks/multiply.h>
#include <gnuradio/blocks/multiply_const.h>
#include <gnuradio/filter/fir_filter_blk.h>
#endif

#include <gnuradio/blocks/complex_to_arg.h>
#include <gnuradio/blocks/short_to_float.h>

#include <gnuradio/filter/fft_filter_ccf.h>
#include <gnuradio/filter/fft_filter_fff.h>
#include <gnuradio/filter/firdes.h>
#include <gnuradio/filter/pfb_arb_resampler_ccf.h>

#include <gnuradio/analog/feedforward_agc_cc.h>
#include <gnuradio/analog/pll_freqdet_cf.h>
#include <gnuradio/analog/quadrature_demod_cf.h>

#include <gnuradio/digital/diff_phasor_cc.h>

#include <op25_repeater/fsk4_demod_ff.h>
#include <op25_repeater/fsk4_slicer_fb.h>
#include <op25_repeater/costas_loop_cc.h>
#include <op25_repeater/gardner_cc.h>
#include <op25_repeater/include/op25_repeater/p25_frame_assembler.h>
#include <gnuradio/digital/fll_band_edge_cc.h>
#include <gnuradio/message.h>
#include <gnuradio/msg_queue.h>

#include "../gr_blocks/rms_agc.h"
//#include <op25_repeater/include/op25_repeater/rmsagc_ff.h>
#include "../gr_blocks/freq_xlating_fft_filter.h"
#include "../gr_blocks/channelizer.h"
#include "../gr_blocks/xlat_channelizer.h"

class p25_trunking;

#if GNURADIO_VERSION < 0x030900
typedef boost::shared_ptr<p25_trunking> p25_trunking_sptr;
#else
typedef std::shared_ptr<p25_trunking> p25_trunking_sptr;
#endif

p25_trunking_sptr make_p25_trunking(double f,
                                    double c,
                                    long s,
                                    gr::msg_queue::sptr queue,
                                    bool qpsk,
                                    int sys_num);

class p25_trunking : public gr::hier_block2 {
  struct DecimSettings {
    long decim;
    long decim2;
  };
  friend p25_trunking_sptr make_p25_trunking(double f,
                                             double c,
                                             long s,
                                             gr::msg_queue::sptr queue,
                                             bool qpsk,
                                             int sys_num);

protected:
  p25_trunking(double f,
               double c,
               long s,
               gr::msg_queue::sptr queue,
               bool qpsk,
               int sys_num);

public:
  ~p25_trunking();

  void set_center(double c);
  void set_rate(long s);
  void tune_freq(double f);
  double get_freq();
  void enable();
  int get_freq_error();
  void finetune_control_freq(double f);
  int autotune_offset;

  gr::msg_queue::sptr tune_queue;
  gr::msg_queue::sptr traffic_queue;
  gr::msg_queue::sptr rx_queue;

private:
  void initialize_qpsk();
  void initialize_fsk4();
  void initialize_p25();

  double system_channel_rate;
  double arb_rate;
  double samples_per_symbol;
  double symbol_rate;
  double resampled_rate;
  double center_freq, chan_freq;
  long input_rate;
  long decim;
  bool double_decim;
  long if1;
  long if2;
  bool qpsk_mod;
  int sys_num;
  const int phase1_samples_per_symbol = 5;
  const int phase2_samples_per_symbol = 4;
  const double phase1_symbol_rate = 4800;
  const double phase2_symbol_rate = 6000;

  std::vector<float> arb_taps;
  std::vector<float> sym_taps;
  std::vector<float> baseband_noise_filter_taps;
  std::vector<gr_complex> bandpass_filter_coeffs;
  std::vector<float> lowpass_filter_coeffs;
  std::vector<float> cutoff_filter_coeffs;


  //channelizer::sptr prefilter;
  xlat_channelizer::sptr prefilter;
  gr::analog::sig_source_c::sptr lo;
  gr::analog::sig_source_c::sptr bfo;
  gr::blocks::multiply_cc::sptr mixer;

  gr::filter::fft_filter_ccc::sptr bandpass_filter;
  gr::filter::fft_filter_ccf::sptr lowpass_filter;
  gr::filter::fft_filter_ccf::sptr cutoff_filter;

  /* GR blocks */
  gr::filter::fir_filter_fff::sptr sym_filter;
  gr::filter::fft_filter_fff::sptr noise_filter;
  gr::filter::pfb_arb_resampler_ccf::sptr arb_resampler;

  gr::digital::diff_phasor_cc::sptr diffdec;

  gr::analog::quadrature_demod_cf::sptr fm_demod;
  gr::analog::feedforward_agc_cc::sptr agc;
  gr::blocks::multiply_const_ff::sptr pll_amp;
  gr::analog::pll_freqdet_cf::sptr pll_freq_lock;

  gr::blocks::short_to_float::sptr converter;
  gr::blocks::multiply_const_ff::sptr rescale;
  gr::blocks::complex_to_arg::sptr to_float;

  gr::op25_repeater::fsk4_demod_ff::sptr fsk4_demod;
  gr::op25_repeater::p25_frame_assembler::sptr op25_frame_assembler;
  gr::op25_repeater::fsk4_slicer_fb::sptr slicer;
  gr::op25_repeater::gardner_cc::sptr clock;
  gr::op25_repeater::costas_loop_cc::sptr costas;
  gr::digital::fll_band_edge_cc::sptr fll_band_edge;
  gr::blocks::rms_agc::sptr rms_agc;
  //gr::op25_repeater::rmsagc_ff::sptr rms_agc;
};

#endif // ifndef P25_TRUNKING_H
