#ifndef SMARTNET_FSK2_DEMOD_H
#define SMARTNET_FSK2_DEMOD_H

#include <boost/shared_ptr.hpp>
#include <gnuradio/analog/pll_freqdet_cf.h>
#include <gnuradio/analog/quadrature_demod_cf.h>
#include <gnuradio/block.h>

#include <gnuradio/block_detail.h>
#include <gnuradio/filter/fft_filter_fff.h>
#include <gnuradio/filter/firdes.h>
#include <gnuradio/hier_block2.h>
#include <gnuradio/io_signature.h>
#include <gnuradio/msg_queue.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/filter/fft_filter_ccf.h>

#if GNURADIO_VERSION < 0x030800
#include <gnuradio/blocks/multiply_const_ff.h>
#include <gnuradio/filter/fir_filter_fff.h>
#else
#include <gnuradio/blocks/multiply_const.h>
#include <gnuradio/filter/fir_filter_blk.h>
#endif
#include "../gr_blocks/rms_agc.h"
#include <op25_repeater/rmsagc_ff.h>
#include <op25_repeater/include/op25_repeater/fsk4_demod_ff.h>
#include <op25_repeater/include/op25_repeater/frame_assembler.h>
#include <gnuradio/digital/clock_recovery_mm_ff.h>
#include <gnuradio/digital/binary_slicer_fb.h>




class smartnet_fsk2_demod : public gr::hier_block2 {
    public:
  smartnet_fsk2_demod(gr::msg_queue::sptr queue);
  virtual ~smartnet_fsk2_demod();
  void reset();
    #if GNURADIO_VERSION < 0x030900
typedef boost::shared_ptr<smartnet_fsk2_demod> sptr;
#else
typedef std::shared_ptr<smartnet_fsk2_demod> sptr;
#endif

static sptr make(gr::msg_queue::sptr queue);


protected:
  virtual void initialize();



private:
  const int samples_per_symbol = 5;
  const double symbol_rate = 3600;
  gr::msg_queue::sptr rx_queue;
  std::vector<float> baseband_noise_filter_taps;
  std::vector<float> sym_taps;
  gr::msg_queue::sptr tune_queue;
  std::vector<float> cutoff_filter_coeffs;
  gr::filter::fft_filter_fff::sptr noise_filter;
  gr::filter::fir_filter_fff::sptr sym_filter;
  gr::filter::fft_filter_ccf::sptr cutoff_filter;
  gr::blocks::multiply_const_ff::sptr pll_amp;
  gr::analog::pll_freqdet_cf::sptr pll_demod;
  gr::analog::quadrature_demod_cf::sptr fm_demod;
  gr::op25_repeater::rmsagc_ff::sptr baseband_amp;
  gr::op25_repeater::fsk4_demod_ff::sptr fsk4_demod;
  gr::digital::clock_recovery_mm_ff::sptr fsk4_demod_mm;
  gr::digital::clock_recovery_mm_ff::sptr clock_recovery;
  gr::digital::binary_slicer_fb::sptr slicer;
  gr::op25_repeater::frame_assembler::sptr framer;
  gr::blocks::null_sink::sptr null_sink1;
  gr::blocks::null_sink::sptr null_sink2;
  gr::blocks::file_sink::sptr debug_sink_in;
  gr::blocks::file_sink::sptr debug_sink_sym;
  gr::blocks::file_sink::sptr debug_sink_demod;
  void reset_block(gr::basic_block_sptr block); 
};
#endif