#include "p25_recorder_fsk4_demod.h"
#include <cmath>

p25_recorder_fsk4_demod_sptr make_p25_recorder_fsk4_demod() {
  p25_recorder_fsk4_demod *recorder = new p25_recorder_fsk4_demod();

  recorder->initialize();
  return gnuradio::get_initial_sptr(recorder);
}

p25_recorder_fsk4_demod::p25_recorder_fsk4_demod()
    : gr::hier_block2("p25_recorder_fsk4_demod",
                      gr::io_signature::make(1, 1, sizeof(gr_complex)),
                      gr::io_signature::make(1, 1, sizeof(float))) {
}

p25_recorder_fsk4_demod::~p25_recorder_fsk4_demod() {
}

void p25_recorder_fsk4_demod::reset_block(gr::basic_block_sptr block) {
  gr::block_detail_sptr detail;
  gr::block_sptr grblock = cast_to_block_sptr(block);
  detail = grblock->detail();
  detail->reset_nitem_counters();
}

void p25_recorder_fsk4_demod::reset() {
}

/**
 * Generate C4FM matched filter taps using the OP25 approach.
 * This implements the P25 C4FM de-emphasis filter.
 *
 * These taps are pre-computed for sample_rate=24000, symbol_rate=4800, span=9
 * using the OP25 algorithm: transfer_function_rx -> irfft -> fftshift -> window -> normalize
 *
 * The transfer function is: D(f) = sin(pi*f/rate) / (pi*f/rate) for f in [0, symbol_rate)
 */
std::vector<float> p25_recorder_fsk4_demod::generate_c4fm_taps(double sample_rate, double symbol_rate, int span) {
  // Pre-computed C4FM taps for sample_rate=24000, symbol_rate=4800, span=9
  // Generated using OP25's c4fm_taps algorithm with transfer_function_rx
  static const float c4fm_taps_24000[] = {
    4.0312432198e-04f,
    -2.2037895551e-04f,
    -6.3724152930e-04f,
    -1.6166325335e-04f,
    6.7865385656e-04f,
    6.6618120420e-04f,
    -4.0420558050e-04f,
    -1.1383620250e-03f,
    -2.6115800979e-04f,
    1.3417428514e-03f,
    1.3060668391e-03f,
    -9.6070100493e-04f,
    -2.5975062956e-03f,
    -4.3705610688e-04f,
    3.8328930923e-03f,
    3.6392152810e-03f,
    -4.3930370271e-03f,
    -1.1280588508e-02f,
    2.8130577702e-03f,
    5.2971011626e-02f,
    1.3150365306e-01f,
    2.0542515300e-01f,
    2.3582229080e-01f,
    2.0542515300e-01f,
    1.3150365306e-01f,
    5.2971011626e-02f,
    2.8130577702e-03f,
    -1.1280588508e-02f,
    -4.3930370271e-03f,
    3.6392152810e-03f,
    3.8328930923e-03f,
    -4.3705610688e-04f,
    -2.5975062956e-03f,
    -9.6070100493e-04f,
    1.3060668391e-03f,
    1.3417428514e-03f,
    -2.6115800979e-04f,
    -1.1383620250e-03f,
    -4.0420558050e-04f,
    6.6618120420e-04f,
    6.7865385656e-04f,
    -1.6166325335e-04f,
    -6.3724152930e-04f,
    -2.2037895551e-04f,
    4.0312432198e-04f
  };

  const int ntaps_24000 = 45;

  // For the standard P25 Phase 1 rate, use pre-computed taps
  if (std::abs(sample_rate - 24000.0) < 1.0 && std::abs(symbol_rate - 4800.0) < 1.0) {
    return std::vector<float>(c4fm_taps_24000, c4fm_taps_24000 + ntaps_24000);
  }

  // Fallback: simple averaging filter for non-standard rates
  int sps = static_cast<int>(sample_rate / symbol_rate);
  std::vector<float> taps(sps);
  float tap_value = 1.0f / sps;
  for (int i = 0; i < sps; i++) {
    taps[i] = tap_value;
  }
  return taps;
}

void p25_recorder_fsk4_demod::initialize() {
  const double phase1_channel_rate = phase1_symbol_rate * phase1_samples_per_symbol;
  const double pi = M_PI;

  // OP25 FSK4 Demodulation Chain:
  // fm_demod -> baseband_amp -> sym_filter (C4FM) -> fsk4_demod

  // FM Demodulator: gain = sample_rate / (2 * pi * symbol_deviation)
  const double def_symbol_deviation = 600.0;
  float fm_demod_gain = static_cast<float>(phase1_channel_rate / (2.0 * pi * def_symbol_deviation));
  fm_demod = gr::analog::quadrature_demod_cf::make(fm_demod_gain);

  // Baseband Amplifier: unity gain (matches OP25 default)
  baseband_amp = gr::blocks::multiply_const_ff::make(1.0f);

  // C4FM Matched Filter
  c4fm_taps = generate_c4fm_taps(phase1_channel_rate, phase1_symbol_rate, 9);
  sym_filter = gr::filter::fir_filter_fff::make(1, c4fm_taps);

  // FSK4 Demodulator: symbol timing recovery
  fsk4_demod = gr::op25_repeater::fsk4_demod_ff::make(tune_queue, phase1_channel_rate, phase1_symbol_rate);

  // Connect the chain
  connect(self(), 0, fm_demod, 0);
  connect(fm_demod, 0, baseband_amp, 0);
  connect(baseband_amp, 0, sym_filter, 0);
  connect(sym_filter, 0, fsk4_demod, 0);
  connect(fsk4_demod, 0, self(), 0);
}
