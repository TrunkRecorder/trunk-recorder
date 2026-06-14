#include "dmr_trunking.h"
#include <boost/log/trivial.hpp>

dmr_trunking_sptr make_dmr_trunking(double freq, double center, long s,
                                    gr::msg_queue::sptr queue, int sys_num) {
  return gnuradio::get_initial_sptr(new dmr_trunking(freq, center, s, queue, sys_num));
}

dmr_trunking::dmr_trunking(double f, double c, long s,
                           gr::msg_queue::sptr queue, int sys_num)
    : gr::hier_block2("dmr_trunking",
                      gr::io_signature::make(1, 1, sizeof(gr_complex)),
                      gr::io_signature::make(0, 0, sizeof(float))) {
  this->sys_num = sys_num;
  chan_freq = f;
  center_freq = c;
  input_rate = s;
  rx_queue = queue;
  autotune_offset = 0;

  // xlat_channelizer down-converts/decimates input_rate -> 24 kHz
  // (channelizer::phase1_samples_per_symbol * phase1_symbol_rate). Mark
  // "non-conventional" so its bandwidth profile matches a trunked CC.
  prefilter = xlat_channelizer::make(input_rate,
                                     channelizer::phase1_samples_per_symbol,
                                     channelizer::phase1_symbol_rate,
                                     xlat_channelizer::channel_bandwidth,
                                     center_freq, false);

  // OP25 fsk4_slicer expects symbols around -3/-1/+1/+3 — same slice table
  // dmr_recorder uses.
  const float l[] = {-2.0, 0.0, 2.0, 4.0};
  std::vector<float> slices(l, l + sizeof(l) / sizeof(l[0]));
  const int msgq_id = 0;
  const int debug = 0;
  slicer = gr::op25_repeater::fsk4_slicer_fb::make(msgq_id, debug, slices);

  // frame_assembler in default (no protocol forced) mode auto-detects DMR
  // sync words and routes parsed CSBK/MBC/CACH messages to rx_queue.
  // d_soft_vocoder=false because we never need voice off the control channel.
  const char *debug_env = std::getenv("TR_DMR_TRUNKING_VERBOSITY");
  int verbosity = debug_env ? std::atoi(debug_env) : 0;
  framer = gr::op25_repeater::frame_assembler::make("", verbosity, sys_num, rx_queue, false);

  connect(self(), 0, prefilter, 0);
  initialize_fsk4();
  connect(slicer, 0, framer, 0);

  // frame_assembler always has 2 audio outputs even when we don't want voice.
  null_slot0 = gr::blocks::null_sink::make(sizeof(int16_t));
  null_slot1 = gr::blocks::null_sink::make(sizeof(int16_t));
  connect(framer, 0, null_slot0, 0);
  connect(framer, 1, null_slot1, 0);

  tune_freq(chan_freq);
}

void dmr_trunking::initialize_fsk4() {
  const double phase1_channel_rate = phase1_symbol_rate * phase1_samples_per_symbol;
  system_channel_rate = phase1_channel_rate;
  samples_per_symbol = phase1_samples_per_symbol;
  symbol_rate = phase1_symbol_rate;
  const double pi = M_PI;

  double freq_to_norm_radians = pi / (phase1_channel_rate / 2.0);
  double fc = 0.0;
  double fd = 600.0;
  double pll_demod_gain = 1.0 / (fd * freq_to_norm_radians);
  pll_freq_lock = gr::analog::pll_freqdet_cf::make(
      (phase1_symbol_rate / 2.0 * 1.2) * freq_to_norm_radians,
      (fc + (3 * fd * 1.9)) * freq_to_norm_radians,
      (fc + (-3 * fd * 1.9)) * freq_to_norm_radians);
  pll_amp = gr::blocks::multiply_const_ff::make(pll_demod_gain * 1.0);

#if GNURADIO_VERSION < 0x030900
  baseband_noise_filter_taps = gr::filter::firdes::low_pass_2(
      1.0, phase1_channel_rate,
      phase1_symbol_rate / 2.0 * 1.175,
      phase1_symbol_rate / 2.0 * 0.125,
      20.0, gr::filter::firdes::WIN_KAISER, 6.76);
#else
  baseband_noise_filter_taps = gr::filter::firdes::low_pass_2(
      1.0, phase1_channel_rate,
      phase1_symbol_rate / 2.0 * 1.175,
      phase1_symbol_rate / 2.0 * 0.125,
      20.0, gr::fft::window::WIN_KAISER, 6.76);
#endif
  noise_filter = gr::filter::fft_filter_fff::make(1.0, baseband_noise_filter_taps);

  for (int i = 0; i < samples_per_symbol; i++) {
    sym_taps.push_back(1.0 / samples_per_symbol);
  }
  sym_filter = gr::filter::fir_filter_fff::make(1, sym_taps);

  tune_queue = gr::msg_queue::make(20);
  fsk4_demod = gr::op25_repeater::fsk4_demod_ff::make(tune_queue, phase1_channel_rate, phase1_symbol_rate);

  connect(prefilter, 0, pll_freq_lock, 0);
  connect(pll_freq_lock, 0, pll_amp, 0);
  connect(pll_amp, 0, noise_filter, 0);
  connect(noise_filter, 0, sym_filter, 0);
  connect(sym_filter, 0, fsk4_demod, 0);
  connect(fsk4_demod, 0, slicer, 0);
}

dmr_trunking::~dmr_trunking() {}

void dmr_trunking::set_center(double c) {
  center_freq = c;
}

void dmr_trunking::set_rate(long s) {
  input_rate = s;
}

double dmr_trunking::get_freq() {
  return chan_freq;
}

void dmr_trunking::tune_freq(double f) {
  autotune_offset = 0;
  chan_freq = f;
  int offset_amount = (center_freq - f);
  prefilter->tune_offset(offset_amount);
}

int dmr_trunking::get_freq_error() {
  return prefilter->get_freq_error();
}

void dmr_trunking::finetune_control_freq(double f) {
  chan_freq = f;
  int offset_amount = (center_freq - f);
  prefilter->tune_offset(offset_amount);
}
