#include "smartnet_trunking.h"
#include "../formatter.h"

using namespace std;

smartnet_trunking_sptr make_smartnet_trunking(float freq,
                                              float center,
                                              long samp,
                                              gr::msg_queue::sptr queue,
                                              int sys_num) {
  return gnuradio::get_initial_sptr(new smartnet_trunking(freq, center, samp,
                                                          queue, sys_num));
}


smartnet_trunking::smartnet_trunking(float f,
                                     float c,
                                     long s,
                                     gr::msg_queue::sptr queue,
                                     int sys_num)
    : gr::hier_block2("smartnet_trunking",
                      gr::io_signature::make(1, 1, sizeof(gr_complex)),
                      gr::io_signature::make(0, 0, sizeof(float))) {
  center_freq = c;
  chan_freq = f;
  input_rate = s;
    symbol_rate = 3600;
  samples_per_symbol = 5; // was 10
  system_channel_rate = symbol_rate * samples_per_symbol;

  this->sys_num = sys_num;

  float gain_mu = 0.01;
  float mu = 0.5;
  float omega_relative_limit = 0.3;

  const double pi = boost::math::constants::pi<double>();
  BOOST_LOG_TRIVIAL(info) << "SmartNet Trunking - SysNum: " << sys_num;

  BOOST_LOG_TRIVIAL(info) << "Control channel: " << format_freq(chan_freq);

  //initialize_prefilter();

  //carriertrack = gr::digital::fll_band_edge_cc::make(system_channel_rate, 0.6, 64, 0.35);

  pll_demod = gr::analog::pll_freqdet_cf::make(
      2.0 / samples_per_symbol,
      1 * pi / samples_per_symbol,
      -1 * pi / samples_per_symbol);

  softbits = gr::digital::clock_recovery_mm_ff::make(samples_per_symbol,
                                                     0.25 * gain_mu * gain_mu,
                                                     mu,
                                                     gain_mu,
                                                     omega_relative_limit);

  slicer = gr::digital::binary_slicer_fb::make();

  start_correlator = gr::digital::correlate_access_code_tag_bb::make("10101100", 0, "smartnet_preamble");

  smartnet_decode_sptr decode = smartnet_make_decode(queue, sys_num);

  prefilter = xlat_channelizer::make(input_rate, channelizer::smartnet_samples_per_symbol, channelizer::smartnet_symbol_rate, xlat_channelizer::channel_bandwidth, center_freq, false);

  connect(self(), 0, prefilter, 0);
  connect(prefilter, 0, pll_demod, 0);
  connect(pll_demod, 0, softbits, 0);
  connect(softbits, 0, slicer, 0);
  connect(slicer, 0, start_correlator, 0);
  connect(start_correlator, 0, decode, 0);
  tune_freq(chan_freq);
}

void smartnet_trunking::reset() {
  BOOST_LOG_TRIVIAL(info) << "Pll Phase: " << pll_demod->get_phase() << " min Freq: " << format_freq(pll_demod->get_min_freq()) << " Max Freq: " << format_freq(pll_demod->get_max_freq());
  //carriertrack->set_rolloff(0.6);
  pll_demod->update_gains();
  // pll_demod->frequency_limit();
  pll_demod->phase_wrap();
  // softbits->set_verbose(true);
  // pll_demod->set_phase(0);
}

void smartnet_trunking::set_center(double c) {
  center_freq = c;
}

void smartnet_trunking::set_rate(long s) {
  input_rate = s;
  // TODO: Update/remake blocks that depend on input_rate
}

void smartnet_trunking::tune_freq(double f) {
  chan_freq = f;
  int offset_amount = (center_freq - f);
  prefilter->tune_offset(offset_amount);

}

