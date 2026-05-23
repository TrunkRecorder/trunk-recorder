#ifndef SHARED_CHANNELIZER_IMPL_H
#define SHARED_CHANNELIZER_IMPL_H

#include "shared_channelizer.h"

#include <gnuradio/fft/fft.h>
#include <gnuradio/gr_complex.h>

#include <atomic>
#include <memory>
#include <vector>

namespace gr {
namespace blocks {

class shared_channelizer_impl : public shared_channelizer {
public:
  shared_channelizer_impl(double input_rate, double target_output_rate,
                          unsigned int max_channels, double channel_bandwidth);
  ~shared_channelizer_impl() override;

  double get_input_rate() const override { return d_input_rate; }
  double get_output_rate() const override { return d_output_rate; }
  int get_decimation() const override { return d_decim; }
  int get_fft_size() const override { return d_fft_size; }
  int get_channel_size() const override { return d_channel_size; }

  void set_channel_offset(unsigned int port, double offset_hz) override;
  void set_channel_enabled(unsigned int port, bool enabled) override;
  bool is_channel_enabled(unsigned int port) const override;
  void set_diagnostic_interval(uint64_t interval) override { d_diagnostic_interval = interval; }
  void set_phase_rotation_mode(int mode) override { d_phase_rotation_mode = mode; }

  void forecast(int noutput_items, gr_vector_int &ninput_items_required) override;
  bool check_topology(int ninputs, int noutputs) override;
  int general_work(int noutput_items,
                   gr_vector_int &ninput_items,
                   gr_vector_const_void_star &input_items,
                   gr_vector_void_star &output_items) override;

private:
  double d_input_rate;
  double d_target_output_rate;
  double d_output_rate;
  double d_channel_bandwidth;
  int d_decim;
  int d_fft_size;     // N
  int d_channel_size; // K
  int d_overlap_in;   // M
  int d_overlap_out;  // M / decim
  int d_step_in;      // N - M
  int d_step_out;     // K - M_K
  unsigned int d_max_channels;
  unsigned int d_num_outputs;

  // Shared forward FFT, used once per block.
#if GNURADIO_VERSION >= 0x030900
  std::unique_ptr<gr::fft::fft_complex_fwd> d_fwd_fft;
  std::unique_ptr<gr::fft::fft_complex_rev> d_inv_fft;
#else
  std::unique_ptr<gr::fft::fft_complex> d_fwd_fft;
  std::unique_ptr<gr::fft::fft_complex> d_inv_fft;
#endif

  // Frequency-domain channel filter, length K. Window applied to the K
  // selected bins to suppress spectral leakage into the overlap-discard
  // region. Includes the 1/N amplitude normalization.
  std::vector<gr_complex> d_channel_filter_freq;

  // Per-port state. Atomic so tune/enable can be called from any thread
  // without locking. Indexed [0, d_max_channels).
  std::unique_ptr<std::atomic<int>[]> d_channel_bin_offset;
  std::unique_ptr<std::atomic<bool>[]> d_channel_enabled;

  // Per-port residual (sub-bin) rotation. When a recorder requests an offset
  // that doesn't land exactly on an FFT bin, the bin-selection puts the
  // signal at residual_hz away from DC in the K-rate output. Applying a
  // continuous-frequency rotator at the output rate brings the signal to
  // exact DC — analogous to the rotator the old freq_xlating_fft_filter had
  // after its FIR. Increment is computed from residual_hz in
  // set_channel_offset; the accumulator advances in work().
  std::unique_ptr<std::atomic<double>[]> d_channel_residual_phase_inc;
  // Running rotor per port (not atomic — only touched from work()).
  std::vector<gr_complex> d_channel_residual_rotor;
  // Counter to occasionally renormalize the rotor magnitude back to 1.0.
  std::vector<int> d_channel_renorm_counter;

  // Running count of total input samples consumed across all work() calls,
  // taken mod d_fft_size. Used to compute the per-block phase correction
  // that keeps the channelized output coherent across FFT blocks.
  uint64_t d_input_sample_counter;

  // Diagnostic: how often (in FFT blocks) to log per-port bin power. 0 = off.
  // Set with set_diagnostic_interval(); read in general_work().
  uint64_t d_diagnostic_interval;
  uint64_t d_diagnostic_block_count;

  // Runtime-switchable phase rotation mode. Lets us bisect bugs without
  // recompiling: env var TR_CHANNELIZER_ROT={0,1,2} at process start.
  std::atomic<int> d_phase_rotation_mode;

  static int choose_fft_size(int decim);
  void design_channel_filter();
};

} // namespace blocks
} // namespace gr

#endif
