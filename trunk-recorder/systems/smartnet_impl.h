#ifndef SMARTNET_IMPL_H
#define SMARTNET_IMPL_H

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

#include <gnuradio/message.h>
#include <gnuradio/msg_queue.h>
#include <gnuradio/blocks/null_sink.h>

#include "../gr_blocks/xlat_channelizer.h"
#include "smartnet_fsk2_demod.h"

class smartnet_impl;



class smartnet_impl : public gr::hier_block2 {
    public:
    #if GNURADIO_VERSION < 0x030900
    typedef boost::shared_ptr<smartnet_impl> sptr;
    #else
    typedef std::shared_ptr<smartnet_impl> sptr;
    #endif
    
    static sptr make(double f,
                                        double c,
                                        long s,
                                        gr::msg_queue::sptr queue,
                                        int sys_num);
  smartnet_impl(double f,
               double c,
               long s,
               gr::msg_queue::sptr queue,
               int sys_num);


  ~smartnet_impl();

  void set_center(double c);
  void set_rate(long s);
  void tune_freq(double f);
  double get_pwr();
  double get_freq();
  void enable();
  int get_freq_error();
  void finetune_control_freq(double f);
  int autotune_offset;

  gr::msg_queue::sptr rx_queue;

private:
  void initialize(double freq, double center, long s, gr::msg_queue::sptr queue, int sys_num);

  double center_freq, chan_freq;
  long input_rate;
  int sys_num;


  //channelizer::sptr prefilter;
  xlat_channelizer::sptr prefilter;

  smartnet_fsk2_demod::sptr fsk2_demod;

};

#endif // ifndef P25_TRUNKING_H
