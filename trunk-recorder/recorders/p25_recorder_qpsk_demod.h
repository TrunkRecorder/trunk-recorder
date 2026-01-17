#ifndef P25_RECORDER_QPSK_DEMOD_H
#define P25_RECORDER_QPSK_DEMOD_H

#include <boost/shared_ptr.hpp>
#include <gnuradio/block.h>
#include <gnuradio/block_detail.h>
#include <gnuradio/hier_block2.h>
#include <gnuradio/io_signature.h>

#include <gnuradio/blocks/complex_to_arg.h>
#include <gnuradio/digital/diff_phasor_cc.h>
#include <op25_repeater/costas_loop_cc.h>
#include <op25_repeater/gardner_cc.h>

#if GNURADIO_VERSION < 0x030800
#include <gnuradio/blocks/multiply_const_ff.h>
#else
#include <gnuradio/blocks/multiply_const.h>
#endif

class p25_recorder_qpsk_demod;

#if GNURADIO_VERSION < 0x030900
typedef boost::shared_ptr<p25_recorder_qpsk_demod> p25_recorder_qpsk_demod_sptr;
#else
typedef std::shared_ptr<p25_recorder_qpsk_demod> p25_recorder_qpsk_demod_sptr;
#endif

p25_recorder_qpsk_demod_sptr make_p25_recorder_qpsk_demod();

class p25_recorder_qpsk_demod : public gr::hier_block2 {
  friend p25_recorder_qpsk_demod_sptr make_p25_recorder_qpsk_demod();

protected:
  virtual void initialize();

   gr::op25_repeater::gardner_cc::sptr clock;
  gr::op25_repeater::costas_loop_cc::sptr costas;

public:
  p25_recorder_qpsk_demod();
  virtual ~p25_recorder_qpsk_demod();
  void switch_tdma(bool phase2);
  void reset();

private:
  double system_channel_rate;
  double symbol_rate;
  int samples_per_symbol;
  const double phase1_symbol_rate = 4800;
  bool tdma_mode;

  gr::digital::diff_phasor_cc::sptr diffdec;
  gr::blocks::complex_to_arg::sptr to_float;
  gr::blocks::multiply_const_ff::sptr rescale;

    void reset_block(gr::basic_block_sptr block); 
};
#endif