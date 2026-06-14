#ifndef DMR_TRUNKED_RECORDER_IMPL_H
#define DMR_TRUNKED_RECORDER_IMPL_H

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

#include <gnuradio/analog/pll_freqdet_cf.h>
#include <gnuradio/blocks/multiply_const.h>
#include <gnuradio/blocks/short_to_float.h>
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

#include "../gr_blocks/plugin_wrapper_impl.h"
#include "../gr_blocks/transmission_sink.h"
#include "../gr_blocks/xlat_channelizer.h"
#include "../source.h"
#include "dmr_trunked_recorder.h"
#include "recorder.h"

class dmr_trunked_recorder_impl : public dmr_trunked_recorder {

protected:
  void initialize(Source *src);

public:
  dmr_trunked_recorder_impl(Source *src);
  void tune_freq(double f) override;
  bool start(Call *call) override;
  void stop() override;
  double get_freq() override;
  int get_freq_error() override;
  int get_num() override;
  void set_tdma_slot(int slot) override;
  int get_tdma_slot() override;
  double since_last_write() override;
  double get_current_length() override;
  void set_enabled(bool enabled) override;
  bool is_enabled() override;
  bool is_active() override;
  bool is_idle() override;
  bool is_squelched() override;
  double get_pwr() override;
  std::vector<Transmission> get_transmission_list() override;
  State get_state() override;
  int lastupdate() override;
  long elapsed() override;
  Source *get_source() override;

  void plugin_callback_handler(int16_t *samples, int sampleCount);
  static void voice_codec_cb_handler(int codec_type, long tgid, uint32_t src_id, const uint32_t *params, int param_count, int errs, void *user_data);

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
  xlat_channelizer::sptr prefilter;

private:
  int silence_frames;
  int tdma_slot;
  bool d_soft_vocoder;
  long input_rate;
  const int phase1_samples_per_symbol = 5;
  const double phase1_symbol_rate = 4800;

  std::vector<float> baseband_noise_filter_taps;
  std::vector<float> sym_taps;
  gr::msg_queue::sptr tune_queue;
  gr::msg_queue::sptr rx_queue;

  gr::filter::fft_filter_fff::sptr noise_filter;
  gr::filter::fir_filter_fff::sptr sym_filter;
  gr::blocks::multiply_const_ff::sptr pll_amp;
  gr::analog::pll_freqdet_cf::sptr pll_freq_lock;
  gr::op25_repeater::fsk4_demod_ff::sptr fsk4_demod;
  gr::op25_repeater::fsk4_slicer_fb::sptr slicer;
  gr::op25_repeater::frame_assembler::sptr framer;

  // Single transmission sink — one slot per recorder. The frame_assembler still
  // emits both slot outputs; the unused one is routed to a null sink because
  // GR hier blocks require every output port to be connected.
  gr::blocks::transmission_sink::sptr wav_sink;
  gr::blocks::null_sink::sptr null_slot_sink;
  gr::blocks::plugin_wrapper::sptr plugin_sink;
};

#endif // DMR_TRUNKED_RECORDER_IMPL_H
