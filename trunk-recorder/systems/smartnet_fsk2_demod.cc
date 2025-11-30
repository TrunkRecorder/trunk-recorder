#include "smartnet_fsk2_demod.h"

smartnet_fsk2_demod::sptr smartnet_fsk2_demod::make(gr::msg_queue::sptr queue) {
  smartnet_fsk2_demod *recorder = new smartnet_fsk2_demod(queue);

  recorder->initialize();
  return gnuradio::get_initial_sptr(recorder);
}

smartnet_fsk2_demod::smartnet_fsk2_demod(gr::msg_queue::sptr queue)
    : gr::hier_block2("smartnet_fsk2_demod",
                      gr::io_signature::make(1, 1, sizeof(gr_complex)),
                      gr::io_signature::make(0, 0, sizeof(float))) {

    rx_queue = queue;
}

smartnet_fsk2_demod::~smartnet_fsk2_demod() {
}
void smartnet_fsk2_demod::reset_block(gr::basic_block_sptr block) {
  gr::block_detail_sptr detail;
  gr::block_sptr grblock = cast_to_block_sptr(block);
  detail = grblock->detail();
  detail->reset_nitem_counters();
}
void smartnet_fsk2_demod::reset() {
  /*reset_block(pll_freq_lock);
  reset_block(pll_amp);
  reset_block(noise_filter);
  reset_block(sym_filter);
  reset_block(fsk4_demod);*/
  /*
  pll_freq_lock->update_gains();
  pll_freq_lock->frequency_limit();
  pll_freq_lock->phase_wrap();
  pll_demod->set_phase(0);*/

  //  fsk4_demod->reset(); This one may have been working but removing for now to be safe
}

void smartnet_fsk2_demod::initialize() {
  const double channel_rate = symbol_rate * samples_per_symbol;
  const double pi = M_PI;
  double samples_per_symbol = 5;
 double ntaps = 7 * samples_per_symbol;

 sym_taps = gr::filter::firdes::root_raised_cosine(1.0, channel_rate, symbol_rate, 0.35, ntaps);


  baseband_amp = gr::op25_repeater::rmsagc_ff::make(0.01, 1.00);

  // FSK4: Symbol Taps
  double symbol_decim = 1;

  for (int i = 0; i < samples_per_symbol; i++) {
    sym_taps.push_back(1.0 / samples_per_symbol);
  }
  sym_filter = gr::filter::fir_filter_fff::make(symbol_decim, sym_taps);

  slicer = gr::digital::binary_slicer_fb::make( );
  float gain_mu = 0.01;
  float mu = 0.5;
  float omega_relative_limit = 0.3;

  pll_demod = gr::analog::pll_freqdet_cf::make(
    2.0 / samples_per_symbol,
    1 * pi / samples_per_symbol,
    -1 * pi / samples_per_symbol);
  fsk4_demod_mm = gr::digital::clock_recovery_mm_ff::make(samples_per_symbol,
    0.25 * gain_mu * gain_mu,
    mu,
    gain_mu,
    omega_relative_limit); //gr::digital::clock_recovery_mm_ff::make(samples_per_symbol, 0.1, 0.5, 0.05, 0.005);
  //fsk4_demod = gr::op25_repeater::fsk4_demod_ff::make(tune_queue, channel_rate, symbol_rate);
  float fm_demod_gain = channel_rate / (2 * pi * symbol_rate);
  fm_demod = gr::analog::quadrature_demod_cf::make(fm_demod_gain);
  framer = gr::op25_repeater::frame_assembler::make("smartnet", 11, 1, rx_queue, false);
  null_sink1 = gr::blocks::null_sink::make(sizeof(uint16_t));
  null_sink2 = gr::blocks::null_sink::make(sizeof(uint16_t));

  debug_sink_in = gr::blocks::file_sink::make(sizeof(gr_complex), "smartnet_debug_in.cfile");
  debug_sink_sym = gr::blocks::file_sink::make(sizeof(float), "smartnet_debug_sym.float");
  debug_sink_demod = gr::blocks::file_sink::make(sizeof(float), "smartnet_debug_demod.float");
  
  // This is the original Approach
//   connect(self(), 0, pll_freq_lock, 0);
//   connect(pll_freq_lock, 0, pll_amp, 0);
//   connect(pll_amp, 0, noise_filter, 0);
//   connect(noise_filter, 0, sym_filter, 0);
//   connect(sym_filter, 0,fsk4_demod, 0);
//   connect(fsk4_demod, 0, self(), 0);


  // This is the current approch in OP25, but they generate some of the filters differently
  // connect(self(), 0, fm_demod,0);
  // connect(fm_demod, 0, baseband_amp,0);
  // connect(baseband_amp, 0, sym_filter, 0);
  // connect(sym_filter, 0,fsk4_demod_mm, 0);
  connect(self(), 0, pll_demod, 0);
  connect(pll_demod, 0, fsk4_demod_mm, 0);
  connect(fsk4_demod_mm, 0, slicer, 0);
  connect(slicer, 0, framer, 0);

  connect(framer, 0, null_sink1, 0);
  connect(framer, 1, null_sink2, 0);

  connect(self(), 0, debug_sink_in, 0);
  //connect(sym_filter, 0, debug_sink_sym, 0);
  connect(fsk4_demod_mm, 0, debug_sink_demod, 0);
}