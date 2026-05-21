#ifndef ANALOG_RECORDER_H
#define ANALOG_RECORDER_H

#include <cstdio>
#include <fstream>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/shared_ptr.hpp>

#include <gnuradio/hier_block2.h>
#include <gnuradio/io_signature.h>

#include <gnuradio/blocks/float_to_short.h>

#include <gnuradio/block.h>
#include <gnuradio/blocks/copy.h>
#include <gnuradio/blocks/null_sink.h>
#if GNURADIO_VERSION < 0x030800
#include <gnuradio/filter/fir_filter_fff.h>

#include <gnuradio/blocks/multiply_const_ff.h>
#else
#include <gnuradio/blocks/multiply_const.h>
#include <gnuradio/filter/fir_filter_blk.h>
#endif
#include <gnuradio/filter/fft_filter_ccf.h>
#include <gnuradio/filter/firdes.h>
#include <gnuradio/filter/iir_filter_ffd.h>

#include <gnuradio/analog/pwr_squelch_cc.h>
#include <gnuradio/analog/pwr_squelch_ff.h>
#include <gnuradio/analog/quadrature_demod_cf.h>

#include <gnuradio/blocks/float_to_short.h>

#include <gnuradio/filter/pfb_arb_resampler_ccf.h>

class Source;
class analog_recorder;

#include "../gr_blocks/ctcss_squelch_ff.h"
#include "../gr_blocks/dcs_squelch_ff.h"
#include "../gr_blocks/channelizer.h"
#include "../gr_blocks/decoder_wrapper.h"
#include "../gr_blocks/freq_xlating_fft_filter.h"
#include "../gr_blocks/plugin_wrapper.h"
#include "../gr_blocks/transmission_sink.h"
#include "../gr_blocks/xlat_channelizer.h"
#include "../systems/system.h"
#include "../call_conventional.h"
#include "recorder.h"

#if GNURADIO_VERSION < 0x030900
typedef boost::shared_ptr<analog_recorder> analog_recorder_sptr;
#else
typedef std::shared_ptr<analog_recorder> analog_recorder_sptr;
#endif

#include "../source.h"

int plugman_signal(long unitId, const char *signaling_type, gr::blocks::SignalType sig_type, Call *call, System *system, Recorder *recorder);

analog_recorder_sptr make_analog_recorder(Source *src, Recorder_Type type);
analog_recorder_sptr make_analog_recorder(Source *src, Recorder_Type type, const Tone_Config &tone_config);
class analog_recorder : public gr::hier_block2, public Recorder {
  friend analog_recorder_sptr make_analog_recorder(Source *src, Recorder_Type type);
  friend analog_recorder_sptr make_analog_recorder(Source *src, Recorder_Type type, const Tone_Config &tone_config);

protected:
  analog_recorder(Source *src, System *system, Recorder_Type type, const Tone_Config &tone_config);

public:
  ~analog_recorder();
  void tune_freq(double f);
  bool start(Call *call);
  void stop();
  double get_freq();
  int get_freq_error();
  void set_source(long src);
  Source *get_source();
  long get_talkgroup();
  time_t get_start_time();
  double get_current_length();
  long get_wav_hz();
  void set_enabled(bool enabled);
  bool is_enabled();
  bool is_active();
  bool is_analog();
  bool is_idle();
  bool is_squelched();
  double get_pwr();
  std::vector<Transmission> get_transmission_list();
  State get_state();
  int get_num();
  int lastupdate();
  long elapsed();
  static bool logging;
  void process_message_queues(void);
  void decoder_callback_handler(long unitId, const char *signaling_type, gr::blocks::SignalType signal);
  void plugin_callback_handler(int16_t *samples, int sampleCount);
  double get_output_sample_rate();
  double since_last_write();
  void set_tau(float tau);
  float get_tau() const;

  // Result of the end-of-call tone identification pass. Populated in stop()
  // by polling get_verdict() on whichever new ctcss_squelch_ff / dcs_squelch_ff
  // blocks ran (configured-gate, side-chain, or search mode). Empty when
  // the channel was Tone=0 / OFF mode and no detector ran.
  struct Tone_Result {
    std::string mode;        // "off" | "ctcss" | "dcs" | "search" (configured)
    std::string detected;    // "" | "173.8" | "D023N" | etc.
    float       confidence = 0.0f;
  };
  Tone_Result get_tone_result() const { return tone_result; }

private:
  double center_freq, chan_freq;
  long talkgroup;
  long input_rate;
  Tone_Config tone_config;          // copy of the channel's tone configuration
  double system_channel_rate;
  double initial_rate;
  float quad_gain;
  double wav_sample_rate;
  double squelch_db;
  time_t timestamp;
  time_t starttime;
  Tone_Result tone_result;

  State state;
  std::vector<float> channel_lpf_taps;
  std::vector<float> lpf_taps;
  std::vector<float> audio_resampler_taps;
  std::vector<float> sym_taps;
  std::vector<float> high_f_taps;
  std::vector<float> low_f_taps;
  /* De-emph IIR filter taps */
  std::vector<double> d_fftaps; /*! Feed forward taps. */
  std::vector<double> d_fbtaps; /*! Feed back taps. */
  float d_tau;                 /*! De-emphasis time constant. */

  Call *call;
  Config *config;
  Source *source;
  System *system;
  void calculate_iir_taps(float tau);

  /* GR blocks */
  // channelizer::sptr prefilter;
  xlat_channelizer::sptr prefilter;
  gr::filter::iir_filter_ffd::sptr deemph;
  gr::filter::fir_filter_fff::sptr sym_filter;
  gr::filter::fft_filter_ccf::sptr channel_lpf;

  gr::blocks::multiply_const_ff::sptr levels;
  gr::filter::pfb_arb_resampler_ccf::sptr arb_resampler;
  gr::filter::fir_filter_fff::sptr decim_audio;
  gr::filter::fir_filter_fff::sptr high_f;
  gr::filter::fir_filter_fff::sptr low_f;
  gr::analog::pwr_squelch_ff::sptr squelch_two;

  // Sub-audible squelch / identification blocks.
  // CTCSS: only ctcss_block, wired as the main-path gate.
  // DCS:   only dcs_block, wired as the main-path gate.
  // SEARCH: both blocks, neither gating — each taps decim_audio and drains
  //         into its own null_sink; get_verdict() is called at end-of-call
  //         and the higher-confidence result wins.
  // OFF:   neither instantiated.
  // See analog_recorder.cc constructor for the wiring.
  gr::trunkrecorder::ctcss_squelch_ff::sptr ctcss_block;
  gr::trunkrecorder::dcs_squelch_ff::sptr   dcs_block;
  gr::blocks::null_sink::sptr               ctcss_null_sink;
  gr::blocks::null_sink::sptr               dcs_null_sink;
  bool ctcss_block_in_path = false; // true when ctcss_block is the main-path gate
  bool dcs_block_in_path   = false; // true when dcs_block is the main-path gate

  gr::analog::quadrature_demod_cf::sptr demod;
  gr::blocks::float_to_short::sptr converter;

  gr::blocks::transmission_sink::sptr wav_sink;
  gr::blocks::decoder_wrapper::sptr decoder_sink;
  gr::blocks::plugin_wrapper::sptr plugin_sink;

  void setup_decoders_for_system(System *system);
};

#endif // ifndef ANALOG_RECORDER_H
