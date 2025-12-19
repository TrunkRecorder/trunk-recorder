#ifndef P25_RECORDER_IMPL_H
#define P25_RECORDER_IMPL_H

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
#include <gnuradio/filter/fft_filter_fff.h>
#include <gnuradio/filter/pfb_arb_resampler_ccf.h>

#include <gnuradio/block.h>
#include <gnuradio/blocks/copy.h>

// #include <gnuradio/blocks/selector.h>

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

#include <gnuradio/block_detail.h>
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/blocks/head.h>
#include <gnuradio/digital/fll_band_edge_cc.h>
#include <gnuradio/message.h>
#include <gnuradio/msg_queue.h>
#include <gnuradio/runtime_types.h>

#include "../gr_blocks/channelizer.h"
#include "../gr_blocks/selector.h"
#include "../gr_blocks/transmission_sink.h"
#include "../gr_blocks/xlat_channelizer.h"

// #include <op25_repeater/include/op25_repeater/rmsagc_ff.h>
#include "../../lib/gr-latency-manager/include/latency_manager.h"
#include "../../lib/gr-latency-manager/include/tag_to_msg.h"
#include "../../lib/gr-latency/latency_probe.h"
#include "../../lib/gr-latency/latency_tagger.h"
#include "../gr_blocks/rms_agc.h"
#include "../call_conventional.h"
#include "p25_recorder.h"
#include "p25_recorder_decode.h"
#include "p25_recorder_fsk4_demod.h"
#include "p25_recorder_qpsk_demod.h"
#include "recorder.h"

class Source;
class p25_recorder;

#include "../source.h"

class p25_recorder_impl : public p25_recorder {

protected:
  void initialize(Source *src);

public:
  p25_recorder_impl(Source *src, Recorder_Type type);

  void initialize_qpsk();
  void initialize_fsk4();
  void initialize_p25();
  void tune_freq(double f);
  bool start(Call *call);
  void stop();
  void clear();
  double get_freq();
  int get_num();
  int get_freq_error();
  void set_tdma(bool phase2);
  void switch_tdma(bool phase2);
  void set_tdma_slot(int slot);
  void set_source(long src);
  double since_last_write();
  void process_message_queues();
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
  void autotune();

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
  bool qpsk_mod;
  double squelch_db;
  gr::blocks::selector::sptr modulation_selector;

  p25_recorder_fsk4_demod_sptr fsk4_demod;
  p25_recorder_decode_sptr fsk4_p25_decode;
  p25_recorder_qpsk_demod_sptr qpsk_demod;
  p25_recorder_decode_sptr qpsk_p25_decode;
  // channelizer::sptr prefilter;
  xlat_channelizer::sptr prefilter;

private:
  int silence_frames;
  int tdma_slot;
  bool d_phase2_tdma;
  bool d_soft_vocoder;
  long input_rate;
  const int phase1_samples_per_symbol = 5;
  const int phase2_samples_per_symbol = 4;
  const double phase1_symbol_rate = 4800;
  const double phase2_symbol_rate = 6000;

  gr::blocks::multiply_const_ff::sptr rescale;
  void reset_block(gr::basic_block_sptr block);
};

#endif // ifndef P25_RECORDER_H
