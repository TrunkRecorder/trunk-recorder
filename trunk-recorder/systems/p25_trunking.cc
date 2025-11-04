
#include "p25_trunking.h"
#include <boost/log/trivial.hpp>

p25_trunking_sptr make_p25_trunking(double freq, double center, long s, gr::msg_queue::sptr queue, bool qpsk, int sys_num) {
  return gnuradio::get_initial_sptr(new p25_trunking(freq, center, s, queue, qpsk, sys_num));
}

void p25_trunking::initialize_fsk4() {

  double phase1_channel_rate = phase1_symbol_rate * phase1_samples_per_symbol;
  system_channel_rate = phase1_symbol_rate * phase1_samples_per_symbol;
  samples_per_symbol = phase1_samples_per_symbol;
  symbol_rate = phase1_symbol_rate;
  // double phase2_channel_rate = phase2_symbol_rate * phase2_samples_per_symbol;
  const double pi = M_PI;

  // FSK4: Phase Loop Lock - can only be Phase 1, so locking at that rate.
  double freq_to_norm_radians = pi / (phase1_channel_rate / 2.0);
  double fc = 0.0;
  double fd = 600.0;
  double pll_demod_gain = 1.0 / (fd * freq_to_norm_radians);
  pll_freq_lock = gr::analog::pll_freqdet_cf::make((phase1_symbol_rate / 2.0 * 1.2) * freq_to_norm_radians, (fc + (3 * fd * 1.9)) * freq_to_norm_radians, (fc + (-3 * fd * 1.9)) * freq_to_norm_radians);
  pll_amp = gr::blocks::multiply_const_ff::make(pll_demod_gain * 1.0); // source->get_());

  // FSK4: noise filter - can only be Phase 1, so locking at that rate.
#if GNURADIO_VERSION < 0x030900
  baseband_noise_filter_taps = gr::filter::firdes::low_pass_2(1.0, phase1_channel_rate, phase1_symbol_rate / 2.0 * 1.175, phase1_symbol_rate / 2.0 * 0.125, 20.0, gr::filter::firdes::WIN_KAISER, 6.76);
#else
  baseband_noise_filter_taps = gr::filter::firdes::low_pass_2(1.0, phase1_channel_rate, phase1_symbol_rate / 2.0 * 1.175, phase1_symbol_rate / 2.0 * 0.125, 20.0, gr::fft::window::WIN_KAISER, 6.76);

#endif
  noise_filter = gr::filter::fft_filter_fff::make(1.0, baseband_noise_filter_taps);

  // FSK4: Symbol Taps
  double symbol_decim = 1;
  for (int i = 0; i < samples_per_symbol; i++) {
    sym_taps.push_back(1.0 / samples_per_symbol);
  }
  sym_filter = gr::filter::fir_filter_fff::make(symbol_decim, sym_taps);

  // FSK4: FSK4 Demod - locked at Phase 1 rates, since it can only be Phase 1
  tune_queue = gr::msg_queue::make(20);
  fsk4_demod = gr::op25_repeater::fsk4_demod_ff::make(tune_queue, phase1_channel_rate, phase1_symbol_rate);

  connect(prefilter, 0, pll_freq_lock, 0);
  connect(pll_freq_lock, 0, pll_amp, 0);
  connect(pll_amp, 0, noise_filter, 0);
  connect(noise_filter, 0, sym_filter, 0);
  connect(sym_filter, 0, fsk4_demod, 0);
  connect(fsk4_demod, 0, slicer, 0);
}

void p25_trunking::initialize_qpsk() {
  const double pi = M_PI;

  agc = gr::analog::feedforward_agc_cc::make(16, 1.0);
  system_channel_rate = phase1_symbol_rate * phase1_samples_per_symbol;
  samples_per_symbol = phase1_samples_per_symbol;
  symbol_rate = phase1_symbol_rate;
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




  connect(prefilter, 0, clock, 0);
  connect(clock, 0, diffdec, 0);
  connect(diffdec, 0, costas,0);
  connect(costas,0, to_float, 0);
  connect(to_float, 0, rescale, 0);
  connect(rescale, 0, slicer, 0);
}

void p25_trunking::initialize_p25() {
  // OP25 Slicer
  const float l[] = {-2.0, 0.0, 2.0, 4.0};
  const int msgq_id = 0;
  const int debug = 0;
  std::vector<float> slices(l, l + sizeof(l) / sizeof(l[0]));

  slicer = gr::op25_repeater::fsk4_slicer_fb::make(msgq_id, debug, slices);

  // OP25 Frame Assembler
  traffic_queue = gr::msg_queue::make(2);
  tune_queue = gr::msg_queue::make(2);

  int udp_port = 0;
  int verbosity = 0;
  const char *wireshark_host = "0";
  bool do_imbe = 0;
  int silence_frames = 0;
  bool do_output = 0;
  bool do_msgq = 1;
  bool do_audio_output = 0;
  bool do_tdma = 0;
  bool do_nocrypt = 1;
  bool soft_vocoder = false;
  op25_frame_assembler = gr::op25_repeater::p25_frame_assembler::make(silence_frames, soft_vocoder, wireshark_host, udp_port, verbosity, do_imbe, do_output, do_msgq, rx_queue, do_audio_output, do_tdma, do_nocrypt);
  autotune_offset = 0;

  connect(slicer, 0, op25_frame_assembler, 0);
}

p25_trunking::p25_trunking(double f, double c, long s, gr::msg_queue::sptr queue, bool qpsk, int sys_num)
    : gr::hier_block2("p25_trunking",
                      gr::io_signature::make(1, 1, sizeof(gr_complex)),
                      gr::io_signature::make(0, 0, sizeof(float))) {

  this->sys_num = sys_num;
  chan_freq = f;
  center_freq = c;
  input_rate = s;
  rx_queue = queue;
  qpsk_mod = qpsk;

  prefilter = xlat_channelizer::make(input_rate, channelizer::phase1_samples_per_symbol, channelizer::phase1_symbol_rate, xlat_channelizer::channel_bandwidth, center_freq, false);

  initialize_p25();

  connect(self(),0, prefilter, 0);
  if (!qpsk_mod) {
    initialize_fsk4();
  } else {
    initialize_qpsk();
  }
  tune_freq(chan_freq);
}

p25_trunking::~p25_trunking() {}

void p25_trunking::set_center(double c) {
  center_freq = c;
}

void p25_trunking::set_rate(long s) {
  input_rate = s;
  // TODO: Update/remake blocks that depend on input_rate
}

double p25_trunking::get_freq() {
  return chan_freq;
}

void p25_trunking::tune_freq(double f) {
  autotune_offset = 0;
  chan_freq = f;
  int offset_amount = (center_freq - f);
  prefilter->tune_offset(offset_amount);
  if (qpsk_mod) {
    costas->set_phase(0);
    costas->set_frequency(0);
  } else {
    //fsk4_demod->reset();
  }
}

int p25_trunking::get_freq_error() { 
  // Get current tuningerror from FLL
  return prefilter->get_freq_error();
}

void p25_trunking::finetune_control_freq(double f) {
  // Minor tuning adjustment without resetting costas or phase
  chan_freq = f;
  int offset_amount = (center_freq - f);
  prefilter->tune_offset(offset_amount);
}
