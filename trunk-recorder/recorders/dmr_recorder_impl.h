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

#include <gnuradio/analog/pll_freqdet_cf.h>
#include <gnuradio/block.h>
#include <gnuradio/filter/fft_filter_fff.h>
#include <gnuradio/filter/firdes.h>
#include <gnuradio/hier_block2.h>
#include <gnuradio/io_signature.h>
#include <gnuradio/msg_queue.h>

#include <gnuradio/filter/fft_filter_ccf.h>

#include <op25_repeater/fsk4_slicer_fb.h>
#include <op25_repeater/include/op25_repeater/frame_assembler.h>
#include <op25_repeater/include/op25_repeater/fsk4_demod_ff.h>

#include <gnuradio/message.h>
#include <gnuradio/msg_queue.h>

#include "../gr_blocks/plugin_wrapper_impl.h"
#include "../gr_blocks/transmission_sink.h"
#include "../gr_blocks/xlat_channelizer.h"
#include "../source.h"
#include "dmr_recorder.h"
#include "recorder.h"

class dmr_recorder_impl : public dmr_recorder {
public:
  dmr_recorder_impl(Source *src, bool conventional);

  // From dmr_recorder
  void tune_freq(double f) override;
  double get_freq() override;
  int get_freq_error() override;
  int get_num() override;
  Source *get_source() override;

  bool start(Call *call) override;
  void stop(int slot) override;
  void stop() override;

  State get_state(int slot) override;
  bool is_active(int slot) override;
  bool is_idle(int slot) override;
  double since_last_write(int slot) override;
  double get_current_length(int slot) override;
  std::vector<Transmission> get_transmission_list(int slot) override;

  bool is_slot_available(int slot) override;
  bool is_fully_available() override;

  void set_enabled(bool enabled) override;
  bool is_enabled() override;
  bool is_squelched() override;
  double get_pwr() override;
  int lastupdate() override;
  long elapsed() override;

  // Slot-less overrides from Recorder. These aggregate across both slots and
  // are only used by reporting code (print_recorders, get_stats). Never use
  // them from Call lifecycle code — they cannot tell which slot a Call owns.
  State get_state() override;
  bool is_active() override;
  bool is_idle() override;
  double since_last_write() override;
  double get_current_length() override;
  std::vector<Transmission> get_transmission_list() override;

  // Plugin audio callback. Slot is bound at construction time so the framer
  // demuxes voice to the right Call.
  void plugin_callback_handler(int slot, int16_t *samples, int sampleCount);
  static void voice_codec_cb_handler(int codec_type, long tgid, uint32_t src_id, const uint32_t *params, int param_count, int errs, void *user_data);

protected:
  void initialize(Source *src);

  Config *config;
  Source *source;
  double chan_freq;
  double center_freq;
  double squelch_db;
  time_t timestamp;
  time_t starttime;
  bool d_soft_vocoder;
  long input_rate;
  int silence_frames;

  const int phase1_samples_per_symbol = 5;
  const double phase1_symbol_rate = 4800;

  // Each TDMA slot is an independent recording stream. The pipeline above the
  // framer is shared; below it, each slot has its own transmission_sink and
  // optional plugin_wrapper. `call` is the Call currently attached to the slot
  // (nullptr when free). The slot is "active" when a Call has been started on
  // it; the wav_sink's own State (IDLE/RECORDING/STOPPED) tracks voice activity
  // and is driven by tags from the framer.
  struct Slot {
    gr::blocks::transmission_sink::sptr wav_sink;
    gr::blocks::plugin_wrapper::sptr plugin_sink;
    Call *call;
    bool active;
  };
  Slot slots[2];

  // Shared pipeline
  xlat_channelizer::sptr prefilter;
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
};

#endif // DMR_RECORDER_IMPL_H
