#ifndef SMARTNET_TRUNKING
#define SMARTNET_TRUNKING

#include <boost/log/trivial.hpp>
#include <boost/math/constants/constants.hpp>

#include <gnuradio/gr_complex.h>
#include <gnuradio/hier_block2.h>
#include <gnuradio/message.h>
#include <gnuradio/msg_queue.h>

#include <gnuradio/filter/firdes.h>

#if GNURADIO_VERSION < 0x030800
#include <gnuradio/analog/sig_source_c.h>
#include <gnuradio/blocks/multiply_cc.h>
#include <gnuradio/filter/fir_filter_fff.h>
#else
#include <gnuradio/analog/sig_source.h>
#include <gnuradio/blocks/multiply.h>
#include <gnuradio/filter/fir_filter_blk.h>
#endif

#include <gnuradio/digital/binary_slicer_fb.h>
#include <gnuradio/digital/clock_recovery_mm_ff.h>
#include <gnuradio/digital/correlate_access_code_tag_bb.h>
#include <gnuradio/digital/fll_band_edge_cc.h>
#include <gnuradio/filter/fft_filter_ccc.h>
#include <gnuradio/filter/fft_filter_ccf.h>
#include <gnuradio/filter/pfb_arb_resampler_ccf.h>

#include <gnuradio/analog/pll_freqdet_cf.h>
#include "../gr_blocks/channelizer.h"
#include "../gr_blocks/xlat_channelizer.h"
#include "smartnet_decode.h"

class smartnet_trunking;

#if GNURADIO_VERSION < 0x030900
typedef boost::shared_ptr<smartnet_trunking> smartnet_trunking_sptr;
#else
typedef std::shared_ptr<smartnet_trunking> smartnet_trunking_sptr;
#endif
smartnet_trunking_sptr make_smartnet_trunking(float f,
                                              float c,
                                              long s,
                                              gr::msg_queue::sptr queue,
                                              int sys_num);

class smartnet_trunking : public gr::hier_block2 {
  friend smartnet_trunking_sptr make_smartnet_trunking(float f,
                                                       float c,
                                                       long s,
                                                       gr::msg_queue::sptr queue,
                                                       int sys_num);

public:
  struct DecimSettings {
    long decim;
    long decim2;
  };
  void set_center(double c);
  void set_rate(long s);
  void tune_offset(double f);
  void tune_freq(double f);
  void reset();

protected:
  gr::filter::pfb_arb_resampler_ccf::sptr arb_resampler;
  gr::analog::pll_freqdet_cf::sptr pll_demod;
  gr::digital::clock_recovery_mm_ff::sptr softbits;
  gr::digital::binary_slicer_fb::sptr slicer;
  gr::digital::correlate_access_code_tag_bb::sptr start_correlator;

  smartnet_trunking(float f,
                    float c,
                    long s,
                    gr::msg_queue::sptr queue,
                    int sys_num);
  double chan_freq, center_freq;
  double system_channel_rate;
  double arb_rate;
  double samples_per_symbol;
  double symbol_rate;
  long input_rate;
  int sys_num;
  //channelizer::sptr prefilter;
    xlat_channelizer::sptr prefilter;
};

#endif // ifndef SMARTNET_TRUNKING
