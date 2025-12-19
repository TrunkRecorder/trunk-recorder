#ifndef P25_RECORDER_H
#define P25_RECORDER_H

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

#include "recorder.h"

class Source;
class p25_recorder;

#if GNURADIO_VERSION < 0x030900
typedef boost::shared_ptr<p25_recorder> p25_recorder_sptr;
#else
typedef std::shared_ptr<p25_recorder> p25_recorder_sptr;
#endif

p25_recorder_sptr make_p25_recorder(Source *src, Recorder_Type type);
#include "../source.h"

class p25_recorder : virtual public gr::hier_block2, virtual public Recorder {
  static p25_recorder_sptr make_p25_recorder(Source *src);

public:
  p25_recorder(){};
  virtual ~p25_recorder(){};
  virtual void tune_freq(double f) = 0;
  virtual bool start(Call *call) = 0;
  virtual void stop() = 0;
  virtual void clear() = 0;
  virtual double get_freq() = 0;
  virtual int get_num() = 0;
  virtual void set_tdma(bool phase2) = 0;
  virtual void switch_tdma(bool phase2) = 0;
  virtual void set_tdma_slot(int slot) = 0;
  virtual void set_source(long src) = 0;
  virtual double since_last_write() = 0;
  virtual void process_message_queues() = 0;
  virtual double get_current_length() = 0;
  virtual void set_enabled(bool enabled) {};
  virtual bool is_enabled() { return false; };
  virtual bool is_active() = 0;
  virtual bool is_idle() = 0;
  virtual bool is_squelched() = 0;
  virtual double get_pwr() = 0;
  virtual std::vector<Transmission> get_transmission_list() = 0;
  virtual State get_state() = 0;
  virtual int lastupdate() = 0;
  virtual long elapsed() = 0;
  virtual Source *get_source() = 0;
  virtual void autotune() = 0;
};

#endif // ifndef P25_RECORDER_H
