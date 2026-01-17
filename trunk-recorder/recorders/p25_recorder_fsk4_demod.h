#ifndef P25_RECORDER_FSK4_DEMOD_H
#define P25_RECORDER_FSK4_DEMOD_H

#include <boost/shared_ptr.hpp>
#include <gnuradio/analog/quadrature_demod_cf.h>
#include <gnuradio/block.h>
#include <gnuradio/block_detail.h>
#include <gnuradio/hier_block2.h>
#include <gnuradio/io_signature.h>
#include <gnuradio/msg_queue.h>

#if GNURADIO_VERSION < 0x030800
#include <gnuradio/blocks/multiply_const_ff.h>
#include <gnuradio/filter/fir_filter_fff.h>
#else
#include <gnuradio/blocks/multiply_const.h>
#include <gnuradio/filter/fir_filter_blk.h>
#endif
#include <op25_repeater/include/op25_repeater/fsk4_demod_ff.h>

class p25_recorder_fsk4_demod;

#if GNURADIO_VERSION < 0x030900
typedef boost::shared_ptr<p25_recorder_fsk4_demod> p25_recorder_fsk4_demod_sptr;
#else
typedef std::shared_ptr<p25_recorder_fsk4_demod> p25_recorder_fsk4_demod_sptr;
#endif

p25_recorder_fsk4_demod_sptr make_p25_recorder_fsk4_demod();

class p25_recorder_fsk4_demod : public gr::hier_block2 {
  friend p25_recorder_fsk4_demod_sptr make_p25_recorder_fsk4_demod();

protected:
  virtual void initialize();
  std::vector<float> generate_c4fm_taps(double sample_rate, double symbol_rate, int span);

public:
  p25_recorder_fsk4_demod();
  virtual ~p25_recorder_fsk4_demod();
  void reset();

private:
  const int phase1_samples_per_symbol = 5;
  const double phase1_symbol_rate = 4800;
  std::vector<float> c4fm_taps;
  gr::msg_queue::sptr tune_queue;
  gr::filter::fir_filter_fff::sptr sym_filter;
  gr::analog::quadrature_demod_cf::sptr fm_demod;
  gr::blocks::multiply_const_ff::sptr baseband_amp;
  gr::op25_repeater::fsk4_demod_ff::sptr fsk4_demod;
  void reset_block(gr::basic_block_sptr block);
};
#endif