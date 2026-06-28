#ifndef DMR_TRUNKING_H
#define DMR_TRUNKING_H

#define _USE_MATH_DEFINES

#include <cstdio>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <boost/log/trivial.hpp>
#include <boost/shared_ptr.hpp>

#include <gnuradio/analog/pll_freqdet_cf.h>
#include <gnuradio/block.h>
#include <gnuradio/blocks/multiply_const.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/filter/fft_filter_fff.h>
#include <gnuradio/filter/fir_filter_blk.h>
#include <gnuradio/filter/firdes.h>
#include <gnuradio/hier_block2.h>
#include <gnuradio/io_signature.h>
#include <gnuradio/message.h>
#include <gnuradio/msg_queue.h>

#include <op25_repeater/fsk4_slicer_fb.h>
#include <op25_repeater/include/op25_repeater/frame_assembler.h>
#include <op25_repeater/include/op25_repeater/fsk4_demod_ff.h>

#include "../gr_blocks/channelizer.h"
#include "../gr_blocks/xlat_channelizer.h"

class dmr_trunking;

#if GNURADIO_VERSION < 0x030900
typedef boost::shared_ptr<dmr_trunking> dmr_trunking_sptr;
#else
typedef std::shared_ptr<dmr_trunking> dmr_trunking_sptr;
#endif

dmr_trunking_sptr make_dmr_trunking(double f, double c, long s,
                                    gr::msg_queue::sptr queue, int sys_num);

// dmr_trunking is the control-channel hier block for trunked DMR systems
// (MOTOTRBO Capacity Plus, Capacity Max, Connect Plus, ETSI Tier III).
//
// It mirrors p25_trunking: a freq-xlat channelizer followed by the FSK4
// demod chain, terminating in OP25's protocol-multiplexing frame_assembler.
// Decoded CSBK / MBC / CACH messages are pushed to the supplied msg_queue
// for the system's DmrParser to consume.
class dmr_trunking : public gr::hier_block2 {
  friend dmr_trunking_sptr make_dmr_trunking(double f, double c, long s,
                                             gr::msg_queue::sptr queue, int sys_num);

protected:
  dmr_trunking(double f, double c, long s, gr::msg_queue::sptr queue, int sys_num);

public:
  ~dmr_trunking();

  void set_center(double c);
  void set_rate(long s);
  void tune_freq(double f);
  double get_freq();
  int get_freq_error();
  void finetune_control_freq(double f);
  int autotune_offset;

  gr::msg_queue::sptr tune_queue;
  gr::msg_queue::sptr rx_queue;

private:
  void initialize_fsk4();

  double system_channel_rate;
  double samples_per_symbol;
  double symbol_rate;
  double center_freq, chan_freq;
  long input_rate;
  int sys_num;

  const int phase1_samples_per_symbol = 5;
  const double phase1_symbol_rate = 4800;

  std::vector<float> sym_taps;
  std::vector<float> baseband_noise_filter_taps;

  xlat_channelizer::sptr prefilter;
  gr::filter::fir_filter_fff::sptr sym_filter;
  gr::filter::fft_filter_fff::sptr noise_filter;

  gr::blocks::multiply_const_ff::sptr pll_amp;
  gr::analog::pll_freqdet_cf::sptr pll_freq_lock;

  gr::op25_repeater::fsk4_demod_ff::sptr fsk4_demod;
  gr::op25_repeater::fsk4_slicer_fb::sptr slicer;
  gr::op25_repeater::frame_assembler::sptr framer;

  // frame_assembler always emits two int16 audio streams (slot 0 + slot 1).
  // The control channel doesn't need audio — but every output port must be
  // connected for the flat-graph to start, so we route both to null sinks.
  gr::blocks::null_sink::sptr null_slot0;
  gr::blocks::null_sink::sptr null_slot1;
};

#endif // DMR_TRUNKING_H
