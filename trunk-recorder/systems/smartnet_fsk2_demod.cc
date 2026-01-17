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
}

void smartnet_fsk2_demod::initialize() {
  const double channel_rate = symbol_rate * samples_per_symbol;
  const double pi = M_PI;

  // Baseband AGC
  baseband_amp = gr::op25_repeater::rmsagc_ff::make(0.01, 1.00);

  // Symbol filter - simple averaging filter
  std::vector<float> sym_taps;
  for (int i = 0; i < samples_per_symbol; i++) {
    sym_taps.push_back(1.0 / samples_per_symbol);
  }
  sym_filter = gr::filter::fir_filter_fff::make(1, sym_taps);

  // FSK4 demodulator
  fsk4_demod = gr::op25_repeater::fsk4_demod_ff::make(tune_queue, channel_rate, symbol_rate);

  // Binary slicer
  slicer = gr::digital::binary_slicer_fb::make();

  // FM demod gain based on deviation
  // SmartNet uses ±1.2kHz deviation for FSK
  const double deviation = 1200.0;
  float fm_demod_gain = channel_rate / (2 * pi * deviation);
  fm_demod = gr::analog::quadrature_demod_cf::make(fm_demod_gain);

  // Frame assembler and null sinks
  framer = gr::op25_repeater::frame_assembler::make("smartnet", 1, 1, rx_queue, false);
  null_sink1 = gr::blocks::null_sink::make(sizeof(uint16_t));
  null_sink2 = gr::blocks::null_sink::make(sizeof(uint16_t));

  // Signal flow: Input -> FM Demod -> Baseband AGC -> Symbol Filter -> FSK4 Demod -> Slicer -> Framer
  connect(self(), 0, fm_demod, 0);
  connect(fm_demod, 0, baseband_amp, 0);
  connect(baseband_amp, 0, sym_filter, 0);
  connect(sym_filter, 0, fsk4_demod, 0);
  connect(fsk4_demod, 0, slicer, 0);
  connect(slicer, 0, framer, 0);

  connect(framer, 0, null_sink1, 0);
  connect(framer, 1, null_sink2, 0);
}