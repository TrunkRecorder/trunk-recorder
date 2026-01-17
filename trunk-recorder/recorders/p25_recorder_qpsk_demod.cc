#include "p25_recorder_qpsk_demod.h"

p25_recorder_qpsk_demod_sptr make_p25_recorder_qpsk_demod() {
  p25_recorder_qpsk_demod *recorder = new p25_recorder_qpsk_demod();
  recorder->initialize();
  return gnuradio::get_initial_sptr(recorder);
}

p25_recorder_qpsk_demod::p25_recorder_qpsk_demod()
    : gr::hier_block2("p25_recorder_qpsk_demod",
                      gr::io_signature::make(1, 1, sizeof(gr_complex)),
                      gr::io_signature::make(1, 1, sizeof(float))) {
  symbol_rate = 4800;
  tdma_mode = false;
  samples_per_symbol = 5;
  symbol_rate = phase1_symbol_rate;
  system_channel_rate = symbol_rate * samples_per_symbol;
}

p25_recorder_qpsk_demod::~p25_recorder_qpsk_demod() {
}

void p25_recorder_qpsk_demod::reset_block(gr::basic_block_sptr block) {
  gr::block_detail_sptr detail;
  gr::block_sptr grblock = cast_to_block_sptr(block);
  detail = grblock->detail();
  detail->reset_nitem_counters();
}

void p25_recorder_qpsk_demod::reset() {
    costas->set_phase(0);
    costas->set_frequency(0);
    clock->reset();
/*
    reset_block(clock);
    reset_block(costas);
    reset_block(diffdec);
    reset_block(to_float);
    reset_block(rescale);*/
}

void p25_recorder_qpsk_demod::switch_tdma(bool phase2) {
  double omega;
  double fmax;
  const double pi = M_PI;
  tdma_mode = phase2;

  if (phase2) {
    symbol_rate = 6000;
    samples_per_symbol = 4; // 5;//4;
  } else {
    symbol_rate = 4800;
    samples_per_symbol = 5;
  }

  system_channel_rate = symbol_rate * samples_per_symbol;

  omega = double(system_channel_rate) / double(symbol_rate);
  fmax = symbol_rate / 2; // Hz
  fmax = 2 * pi * fmax / double(system_channel_rate);
  clock->set_omega(omega);
  //costas_clock->update_fmax(fmax);
  this->reset();
  // op25_frame_assembler->set_phase2_tdma(d_phase2_tdma);
}

void p25_recorder_qpsk_demod::initialize() {
  const double pi = M_PI;

  // Gardner Costas Clock
  double gain_mu = 0.025; // 0.025
  double costas_alpha = 0.008;
  double omega = double(system_channel_rate) / symbol_rate; // set to 6000 for TDMA, should be symbol_rate
  double gain_omega = 0.1 * gain_mu * gain_mu;
  double fmax = 3000; // Hz
  fmax = 2 * pi * fmax / double(system_channel_rate);

  costas = gr::op25_repeater::costas_loop_cc::make(costas_alpha,  4, (2 * pi)/4 );
  clock = gr::op25_repeater::gardner_cc::make(omega, gain_mu, gain_omega);
  // QPSK: Perform Differential decoding on the constellation
  diffdec = gr::digital::diff_phasor_cc::make();

  // QPSK: take angle of the difference (in radians)
  to_float = gr::blocks::complex_to_arg::make();

  // QPSK: convert from radians such that signal is in -3/-1/+1/+3
  rescale = gr::blocks::multiply_const_ff::make((1 / (pi / 4)));


  connect(self(), 0, clock, 0);
  connect(clock, 0, diffdec, 0);
  connect(diffdec, 0, costas,0);
  connect(costas,0, to_float, 0);
  connect(to_float, 0, rescale, 0);
  connect(rescale, 0, self(), 0);
}
