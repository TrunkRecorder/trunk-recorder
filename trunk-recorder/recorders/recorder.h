#ifndef RECORDER_H
#define RECORDER_H

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

#include <gnuradio/filter/firdes.h>
#include <gnuradio/hier_block2.h>
#include <gnuradio/io_signature.h>

#if GNURADIO_VERSION < 0x030800
#include <gnuradio/analog/sig_source_c.h>
#include <gnuradio/analog/sig_source_f.h>
#include <gnuradio/blocks/multiply_cc.h>
#include <gnuradio/blocks/multiply_const_ff.h>
#include <gnuradio/filter/fir_filter_ccf.h>
#include <gnuradio/filter/fir_filter_fff.h>
#include <gnuradio/filter/freq_xlating_fir_filter_ccf.h>
#include <gnuradio/filter/rational_resampler_base_ccc.h>
#include <gnuradio/filter/rational_resampler_base_ccf.h>
#include <gnuradio/filter/rational_resampler_base_fff.h>
#else
#include <gnuradio/analog/sig_source.h>
#include <gnuradio/blocks/multiply.h>
#include <gnuradio/blocks/multiply_const.h>
#include <gnuradio/filter/fir_filter_blk.h>
#include <gnuradio/filter/freq_xlating_fir_filter.h>
#if GNURADIO_VERSION < 0x030900
#include <gnuradio/filter/rational_resampler_base.h>
#else
#include <gnuradio/filter/rational_resampler.h>
#endif
#endif

#include <gnuradio/analog/quadrature_demod_cf.h>

#include <gnuradio/blocks/file_sink.h>

#include <gnuradio/block.h>
#include <gnuradio/blocks/copy.h>
#include <gnuradio/blocks/null_sink.h>

#include <gnuradio/blocks/head.h>

#include "../call.h"
#include "../state.h"
#include <gnuradio/blocks/file_sink.h>

#include "../gr_blocks/transmission_sink.h"

class Recorder {

public:
  struct DecimSettings {
    long decim;
    long decim2;
  };

  int rec_num;
  int rssi=0;
  static int rec_counter;
  std::string get_type_string();
  bool conventional;
  unsigned int selector_port;

  int get_selector_port() { return selector_port;}
  void set_selector_port(unsigned int port) {selector_port = port;}
  Recorder(Recorder_Type  type);
  int get_num() { return rec_num; };
  Recorder_Type get_type() { return type; };
  virtual double get_pwr() { return 0; };

  bool is_conventional() { return conventional; };

  virtual void tune_offset(double f){};
  virtual void tune_freq(double f){};
  virtual bool start(Call *call) { return false; };
  virtual void stop(){};
  virtual void set_tdma_slot(int slot){};
  virtual double get_freq() { return 0; };
  virtual int get_freq_error() { return 0; };
  virtual Source *get_source() { return NULL; };
  virtual std::vector<Transmission> get_transmission_list() { return {}; };
  virtual void set_source(long src){};
  virtual Call_Source *get_source_list() { return NULL; };
  virtual long get_source_count() { return 0; };
  virtual long get_wav_hz() { return 8000; };
  virtual long get_talkgroup() { return 0; };
  virtual State get_state() { return INACTIVE; };
  virtual void set_enabled(bool enabled) {};
  virtual bool is_enabled() { return false; };
  virtual bool is_active() { return false; };
  virtual bool is_analog() { return false; };
  virtual bool is_idle() { return true; };
  virtual bool is_squelched() { return true; };
  virtual double get_current_length() { return 0; };
  virtual double since_last_write() { return 0; };
  virtual void clear(){};
  virtual boost::property_tree::ptree get_stats();
  virtual int get_recording_count() { return recording_count; }
  virtual double get_recording_duration() { return recording_duration; }
  virtual void process_message_queues(void){};
  virtual double get_output_sample_rate() { return 0; }
  virtual int get_output_channels() { return 1; }
  virtual bool get_enable_audio_streaming() { return d_enable_audio_streaming; };
  virtual void set_enable_audio_streaming(bool enable_audio_streaming) { d_enable_audio_streaming = enable_audio_streaming; };

protected:
  int recording_count;
  bool d_enable_audio_streaming;
  double recording_duration;
  Recorder_Type  type;
  int autotune_offset = 0;
};

#endif
