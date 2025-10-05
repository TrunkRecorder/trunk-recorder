#ifndef DMR_RECORDER_IMPL_H
#define DMR_RECORDER_IMPL_H

#define _USE_MATH_DEFINES

#include <cstdio>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/shared_ptr.hpp>

#include <gnuradio/filter/firdes.h>
#include <gnuradio/hier_block2.h>
#include <gnuradio/io_signature.h>

#include <gnuradio/analog/pll_freqdet_cf.h>
#include <gnuradio/analog/pwr_squelch_cc.h>
#include <gnuradio/blocks/short_to_float.h>
#include <gnuradio/filter/fft_filter_fff.h>
#include <gnuradio/filter/pfb_arb_resampler_ccf.h>

#include <gnuradio/block.h>
#include <gnuradio/blocks/copy.h>

#if GNURADIO_VERSION < 0x030800
#include <gnuradio/analog/sig_source_c.h>
#include <gnuradio/blocks/multiply_cc.h>
#include <gnuradio/blocks/multiply_const_ff.h>
#include <gnuradio/blocks/multiply_const_ss.h>
#include <gnuradio/filter/fir_filter_ccc.h>
#include <gnuradio/filter/fir_filter_ccf.h>
#include <gnuradio/filter/fir_filter_fff.h>
#else
#include <gnuradio/analog/sig_source.h>
#include <gnuradio/blocks/multiply.h>
#include <gnuradio/blocks/multiply_const.h>
#include <gnuradio/filter/fir_filter_blk.h>
#endif

#include <boost/shared_ptr.hpp>
#include <gnuradio/analog/pll_freqdet_cf.h>
#include <gnuradio/block.h>
#include <gnuradio/filter/fft_filter_fff.h>
#include <gnuradio/filter/firdes.h>
#include <gnuradio/hier_block2.h>
#include <gnuradio/io_signature.h>
#include <gnuradio/msg_queue.h>

#include <gnuradio/filter/fft_filter_ccf.h>

#if GNURADIO_VERSION < 0x030800
#include <gnuradio/blocks/multiply_const_ff.h>
#include <gnuradio/filter/fir_filter_fff.h>
#else
#include <gnuradio/blocks/multiply_const.h>
#include <gnuradio/filter/fir_filter_blk.h>
#endif

#include <op25_repeater/costas_loop_cc.h>
#include <op25_repeater/fsk4_slicer_fb.h>
#include <op25_repeater/gardner_cc.h>
#include <op25_repeater/include/op25_repeater/frame_assembler.h>
#include <op25_repeater/include/op25_repeater/fsk4_demod_ff.h>
#include <op25_repeater/include/op25_repeater/p25_frame_assembler.h>

#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/blocks/head.h>
#include <gnuradio/message.h>
#include <gnuradio/msg_queue.h>

#include "../gr_blocks/channelizer.h"
#include "../gr_blocks/plugin_wrapper_impl.h"
#include "../gr_blocks/selector.h"
#include "../gr_blocks/transmission_sink.h"
#include "../gr_blocks/xlat_channelizer.h"
#include "../source.h"
#include "../call_conventional.h"
#include "dmr_recorder.h"
#include "recorder.h"

class dmr_recorder_impl : public dmr_recorder {

protected:
  void initialize(Source *src);

public:
  dmr_recorder_impl(Source *src, Recorder_Type type);
  void tune_freq(double f);
  bool start(Call *call);
  void stop();
  double get_freq();
  int get_freq_error();
  int get_num();
  void set_tdma(bool phase2);
  void switch_tdma(bool phase2);
  void set_tdma_slot(int slot);
  double since_last_write();
  double get_current_length();
  void set_enabled(bool enabled);
  bool is_enabled();
  bool is_active();
  bool is_idle();
  bool is_squelched();
  double get_pwr();
  std::vector<Transmission> get_transmission_list();
  State get_state();
  int lastupdate();
  long elapsed();
  Source *get_source();

  void plugin_callback_handler(int16_t *samples, int sampleCount);

protected:
  State state;
  time_t timestamp;
  time_t starttime;
  long talkgroup;
  std::string short_name;
  Call *call;
  Config *config;
  Source *source;
  double chan_freq;
  double center_freq;
  double squelch_db;

  // gr::blocks::multiply_const_ss::sptr levels;
  // channelizer::sptr prefilter;
  xlat_channelizer::sptr prefilter;
  gr::op25_repeater::gardner_cc::sptr clock;
  gr::op25_repeater::costas_loop_cc::sptr costas;

private:
  int silence_frames;
  int tdma_slot;
  bool d_phase2_tdma;
  bool d_soft_vocoder;
  long input_rate;
  const int phase1_samples_per_symbol = 5;
  const double phase1_symbol_rate = 4800;

  gr::blocks::multiply_const_ff::sptr rescale;

  /* FSK4 Stuff */

  std::vector<float> baseband_noise_filter_taps;
  std::vector<float> sym_taps;
  gr::msg_queue::sptr tune_queue;

  gr::filter::fft_filter_fff::sptr noise_filter;
  gr::filter::fir_filter_fff::sptr sym_filter;

  gr::blocks::multiply_const_ff::sptr pll_amp;
  gr::analog::pll_freqdet_cf::sptr pll_freq_lock;
  gr::op25_repeater::fsk4_demod_ff::sptr fsk4_demod;
  gr::op25_repeater::fsk4_slicer_fb::sptr slicer;

  /* P25 Decoder */
  gr::op25_repeater::frame_assembler::sptr framer;
  gr::op25_repeater::p25_frame_assembler::sptr op25_frame_assembler;
  gr::msg_queue::sptr traffic_queue;
  gr::msg_queue::sptr rx_queue;

  gr::blocks::short_to_float::sptr converter_slot0;
  gr::blocks::short_to_float::sptr converter_slot1;
  gr::blocks::multiply_const_ff::sptr levels;
  gr::blocks::transmission_sink::sptr wav_sink_slot0;
  gr::blocks::transmission_sink::sptr wav_sink_slot1;
  gr::blocks::plugin_wrapper::sptr plugin_sink_slot0;
  gr::blocks::plugin_wrapper::sptr plugin_sink_slot1;
};

#endif // ifndef dmr_recorder_H
