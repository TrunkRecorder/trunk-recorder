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

// Per-port channelization via FFT-based fast convolution. The forward FFT
// is shared across all output ports (the win); each port has its own
// frequency-domain band-pass filter centered on its tuned carrier, its own
// inverse FFT, decimation, and output-rate rotator that brings the carrier
// to DC. This is essentially gr::filter::fft_filter_ccc + rotator (i.e. the
// old freq_xlating_fft_filter) replicated N times, with the forward FFT
// computed once and reused.
class shared_channelizer_impl : public shared_channelizer {
public:
  shared_channelizer_impl(double input_rate, double target_output_rate,
                          unsigned int max_channels, double channel_bandwidth);
  ~shared_channelizer_impl() override;

  double get_input_rate() const override { return d_input_rate; }
  double get_output_rate() const override { return d_output_rate; }
  int get_decimation() const override { return d_decim; }
  int get_fft_size() const override { return d_fft_size; }
  int get_channel_size() const override { return d_decim; }

  void set_channel_offset(unsigned int port, double offset_hz) override;
  void set_channel_enabled(unsigned int port, bool enabled) override;
  bool is_channel_enabled(unsigned int port) const override;
  void set_diagnostic_interval(uint64_t interval) override { d_diagnostic_interval = interval; }
  void set_phase_rotation_mode(int /*mode*/) override {}  // no-op (was a debug knob in the bin-select version)

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
  int d_decim;          // input_rate / output_rate
  int d_fft_size;       // N: forward and inverse FFT size
  int d_overlap_in;     // M: input overlap (= filter taps length, roughly)
  int d_step_in;        // N - M: input samples consumed per block
  int d_step_out;       // step_in / decim: output samples produced per block per port
  unsigned int d_max_channels;
  unsigned int d_num_outputs;

  // Shared forward FFT.
#if GNURADIO_VERSION >= 0x030900
  std::unique_ptr<gr::fft::fft_complex_fwd> d_fwd_fft;
  // Single inverse FFT instance used per-port in sequence (the input buffer
  // is overwritten before each port's execute() call).
  std::unique_ptr<gr::fft::fft_complex_rev> d_inv_fft;
#else
  std::unique_ptr<gr::fft::fft_complex> d_fwd_fft;
  std::unique_ptr<gr::fft::fft_complex> d_inv_fft;
#endif

  // Baseband prototype filter, in frequency domain, length N. Designed once
  // at construction (the band-pass shape). Per-port filters are this taken
  // with a circular shift indexed by the port's bin offset.
  std::vector<gr_complex> d_prototype_freq;

  // Per-port state.
  std::unique_ptr<std::atomic<int>[]> d_channel_bin_offset;
  std::unique_ptr<std::atomic<bool>[]> d_channel_enabled;
  // Output-rate rotator phase increment per port. Set when offset changes;
  // read in general_work().
  std::unique_ptr<std::atomic<double>[]> d_channel_rotator_phase_inc;
  // Running rotator state (touched only from work()).
  std::vector<gr_complex> d_channel_rotator;
  std::vector<int> d_channel_rotator_renorm;

  // Diagnostic counters.
  uint64_t d_diagnostic_interval;
  uint64_t d_diagnostic_block_count;

  void design_prototype_filter();
};

} // namespace blocks
} // namespace gr

#endif
