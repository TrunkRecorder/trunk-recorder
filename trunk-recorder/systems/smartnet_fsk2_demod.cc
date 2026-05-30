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

void smartnet_fsk2_demod::reset() {
  // Clear stale tracking state on the carrier PLL and the symbol-tracking
  // loops so reacquisition on a new control channel doesn't start from
  // whatever state things converged to while staring at noise.
  if (pll_demod) {
    pll_demod->set_phase(0);
    pll_demod->set_frequency(0);
  }
  if (fsk4_demod) {
    fsk4_demod->reset();
  }
}

void smartnet_fsk2_demod::initialize() {
  const double channel_rate = symbol_rate * samples_per_symbol;
  const double pi = M_PI;

  // Bounded queue for fsk4_demod's AFC messages — nothing drains it, so cap it
  // small to let fsk4_demod drop on full instead of growing without bound.
  tune_queue = gr::msg_queue::make(2);

  // Baseband AGC
  baseband_amp = gr::op25_repeater::rmsagc_ff::make(0.01, 1.00);

  // Symbol filter - simple averaging filter
  std::vector<float> sym_taps;
  for (int i = 0; i < samples_per_symbol; i++) {
    sym_taps.push_back(1.0 / samples_per_symbol);
  }
  sym_filter = gr::filter::fir_filter_fff::make(1, sym_taps);

  // FSK4 demodulator — SmartNet is 2FSK, so enable the bfsk path so the
  // symbol tracking loop expects two levels instead of four.
  fsk4_demod = gr::op25_repeater::fsk4_demod_ff::make(tune_queue, channel_rate, symbol_rate, true);

  // Binary slicer
  slicer = gr::digital::binary_slicer_fb::make();

  // Carrier-tracking PLL used as the FM frequency detector. Same
  // configuration as the pre-#1085 SmartNet chain: a fast loop (2/sps) wide
  // enough to track the FSK modulation, with the lock range set to ±pi/sps —
  // ~±1800 Hz at the 18 kHz channel rate, enough headroom for the ±1.2 kHz
  // FSK deviation plus typical receiver oscillator drift. Replaces the
  // open-loop quadrature_demod_cf so any residual carrier offset gets pulled
  // in rather than appearing as a DC bias on the slicer input.
  const float loop_bw = 2.0f / samples_per_symbol;
  const float max_freq = (float)(pi / samples_per_symbol);
  pll_demod = gr::analog::pll_freqdet_cf::make(loop_bw, max_freq, -max_freq);

  // Frame assembler and null sinks
  framer = gr::op25_repeater::frame_assembler::make("smartnet", 1, 1, rx_queue, false);
  null_sink1 = gr::blocks::null_sink::make(sizeof(uint16_t));
  null_sink2 = gr::blocks::null_sink::make(sizeof(uint16_t));

  // Signal flow: Input -> PLL Freq Det -> Baseband AGC -> Symbol Filter -> FSK4 Demod -> Slicer -> Framer
  connect(self(), 0, pll_demod, 0);
  connect(pll_demod, 0, baseband_amp, 0);
  connect(baseband_amp, 0, sym_filter, 0);
  connect(sym_filter, 0, fsk4_demod, 0);
  connect(fsk4_demod, 0, slicer, 0);
  connect(slicer, 0, framer, 0);

  connect(framer, 0, null_sink1, 0);
  connect(framer, 1, null_sink2, 0);
}