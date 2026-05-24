#ifndef SHARED_CHANNELIZER_H
#define SHARED_CHANNELIZER_H

#include <gnuradio/block.h>
#include <gnuradio/blocks/api.h>

#if GNURADIO_VERSION < 0x030900
#include <boost/shared_ptr.hpp>
#else
#include <memory>
#endif

namespace gr {
namespace blocks {

// shared_channelizer: a 1-input / N-output GNU Radio block that performs
// overlap-save fast-convolution channelization, sharing a single forward
// FFT across all output channels. Each output port is independently tunable
// to an arbitrary frequency offset within the input passband and can be
// enabled/disabled. All output ports run at the same sample rate.
//
// The algorithm is the same one ka9q-radio uses (see Mark Borgerding,
// "Turning Overlap-Save into a Multiband Mixing, Downsampling Filter Bank"):
//
//   for each block of N input samples (with M-sample overlap from previous):
//     X = FFT(N) of input
//     for each enabled channel c with bin offset b_c:
//       Y_c[k] = X[b_c + k - K/2] * H[k]      (for k in 0..K-1)
//       y_c   = IFFT(K) of Y_c
//       emit (K - M_K) samples of y_c (discard M_K-sample overlap)
//
// Where:
//   N             = forward FFT size
//   K             = N / decim  (per-channel IFFT size)
//   M             = N / 4      (input-domain overlap, 25%)
//   M_K           = M / decim  (output-domain overlap)
//   decim         = round(input_rate / target_output_rate)
//   output_rate   = input_rate / decim  (returned by get_output_rate())
//
// Tuning a channel is just a matter of updating its integer bin offset, which
// is atomic and lock-free. Enabling/disabling skips the per-channel IFFT.
class BLOCKS_API shared_channelizer : virtual public gr::block {
public:
#if GNURADIO_VERSION < 0x030900
  typedef boost::shared_ptr<shared_channelizer> sptr;
#else
  typedef std::shared_ptr<shared_channelizer> sptr;
#endif

  // input_rate           SDR sample rate (must be integer multiple of resulting output_rate).
  // target_output_rate   Desired per-channel sample rate (Hz). Actual rate is
  //                      input_rate / round(input_rate / target_output_rate).
  // max_channels         Maximum number of output ports to allocate state for.
  // channel_bandwidth    Per-channel passband (currently unused for filtering;
  //                      retained for future per-port filter shaping).
  static sptr make(double input_rate, double target_output_rate,
                   unsigned int max_channels, double channel_bandwidth);

  virtual double get_input_rate() const = 0;
  virtual double get_output_rate() const = 0;
  virtual int get_decimation() const = 0;
  virtual int get_fft_size() const = 0;
  virtual int get_channel_size() const = 0;

  // Per-channel control. Safe to call from any thread.
  // offset_hz: tune the channel's center frequency to this offset (in Hz)
  //            relative to the SDR's center frequency. Positive = above center.
  virtual void set_channel_offset(unsigned int port, double offset_hz) = 0;
  virtual void set_channel_enabled(unsigned int port, bool enabled) = 0;
  virtual bool is_channel_enabled(unsigned int port) const = 0;

  // Diagnostic: log per-port FFT bin power every `interval` blocks. 0 = off.
  virtual void set_diagnostic_interval(uint64_t interval) = 0;

  // Phase rotation mode for the per-block phase correction.
  // 0 = disabled (no rotation; output has block-discontinuities)
  // 1 = -2π·k·t/N (current derivation; this is what theory says)
  // 2 = +2π·k·t/N (sign-flipped)
  virtual void set_phase_rotation_mode(int mode) = 0;

  // Snapshot of the most recent forward-FFT *power* spectrum (|X[k]|²),
  // resized to the caller's vector with length get_fft_size(). Returned in
  // raw FFTW bin order: bin 0 = DC, bins [1..N/2-1] are positive freqs,
  // bins [N/2..N-1] are negative freqs. Used by signal_detector to avoid
  // duplicating the wideband FFT it would otherwise need to compute.
  // Thread-safe; copies under a lock.
  virtual void get_spectrum_snapshot(std::vector<float> &out) = 0;
};

} // namespace blocks
} // namespace gr

#endif
