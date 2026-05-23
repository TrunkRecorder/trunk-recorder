
#include "smartnet_impl.h"
#include "../formatter.h"
#include "../source.h"
#include "smartnet_fsk2_demod.h"
#include <boost/log/trivial.hpp>

smartnet_impl::sptr smartnet_impl::make(Source *src, unsigned int port, double freq, gr::msg_queue::sptr queue, int sys_num) {
  smartnet_impl *smartnet = new smartnet_impl(src, port, freq, queue, sys_num);

  return gnuradio::get_initial_sptr(smartnet);
}

smartnet_impl::smartnet_impl(Source *src, unsigned int port, double freq, gr::msg_queue::sptr queue, int sys_num)
    : gr::hier_block2("smartnet_impl",
                      gr::io_signature::make(1, 1, sizeof(gr_complex)),
                      gr::io_signature::make(0, 0, sizeof(float)))
{
  source = src;
  channelizer_port = port;
  center_freq = src->get_center();
  input_rate = src->get_intermediate_rate();
  initialize(freq, queue, sys_num);
}

smartnet_impl::~smartnet_impl() {

}

void smartnet_impl::reset() {
  if (fsk2_demod) {
    fsk2_demod->reset();
  }
}

void smartnet_impl::initialize(double freq, gr::msg_queue::sptr queue, int sys_num) {
  chan_freq = freq;
  rx_queue = queue;
  this->sys_num = sys_num;


  // use_fll=false: band-edge FLL is matched to RRC-shaped signals and doesn't
  // lock cleanly on SmartNet's NRZ 2FSK. The PLL inside smartnet_fsk2_demod
  // (pll_freqdet_cf) handles carrier tracking instead.
  prefilter = xlat_channelizer::make(input_rate, xlat_channelizer::smartnet_samples_per_symbol, xlat_channelizer::smartnet_symbol_rate, xlat_channelizer::channel_bandwidth, center_freq, false, xlat_channelizer::smartnet_excess_bw, false);

  double offset_amount = (center_freq - chan_freq);
  source->set_recorder_port_offset(channelizer_port, offset_amount);

  fsk2_demod = smartnet_fsk2_demod::make(rx_queue);

  connect(self(), 0, prefilter, 0);
  connect(prefilter, 0, fsk2_demod, 0);

}



int smartnet_impl::get_freq_error() { // get frequency error from FLL and convert to Hz
  return prefilter->get_freq_error();
}


double smartnet_impl::get_pwr() {
  return prefilter->get_pwr();
}


double smartnet_impl::get_freq() {
  return chan_freq;
}


void smartnet_impl::tune_freq(double f) {
  chan_freq = f;
  source->set_recorder_port_offset(channelizer_port, center_freq - f);
}

void smartnet_impl::set_center(double c) {
  center_freq = c;
  source->set_recorder_port_offset(channelizer_port, center_freq - chan_freq);
}

void smartnet_impl::set_rate(long s) {
  input_rate = s;
}

void smartnet_impl::enable() {
  
}

void smartnet_impl::finetune_control_freq(double f) {
   tune_freq(f);
}
