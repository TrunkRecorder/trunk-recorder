#include "shared_channelizer_impl.h"

#include <gnuradio/io_signature.h>

#include <boost/log/trivial.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace gr {
namespace blocks {

shared_channelizer::sptr shared_channelizer::make(double input_rate,
                                                  double target_output_rate,
                                                  unsigned int max_channels,
                                                  double channel_bandwidth) {
  return gnuradio::get_initial_sptr(
      new shared_channelizer_impl(input_rate, target_output_rate, max_channels, channel_bandwidth));
}

int shared_channelizer_impl::choose_fft_size(int decim) {
  // Pick K (per-channel IFFT size) such that K * decim lands in a sweet spot
  // for FFTW (a few thousand to a few tens of thousands). Larger N means
  // better adjacent-channel rejection but more latency and more cost per
  // block. We don't insist on a power of two; FFTW handles arbitrary sizes
  // and trunk-recorder's SDR rates rarely produce power-of-two N anyway.
  int K = 64;
  int N = K * decim;
  while (N < 4096 && K < 1024) {
    K *= 2;
    N = K * decim;
  }
  while (N > 32768 && K > 8) {
    K /= 2;
    N = K * decim;
  }
  return K;
}

void shared_channelizer_impl::design_channel_filter() {
  // Hann window across the K passband bins with 1/N amplitude normalization
  // folded in. This shapes the time-domain impulse response so its effective
  // length stays inside the overlap region, avoiding wraparound artifacts.
  d_channel_filter_freq.resize(d_channel_size);
  const double scale = 1.0 / static_cast<double>(d_fft_size);
  for (int k = 0; k < d_channel_size; k++) {
    double w = 0.5 * (1.0 - std::cos(2.0 * M_PI * k / (d_channel_size - 1)));
    d_channel_filter_freq[k] = gr_complex(static_cast<float>(w * scale), 0.0f);
  }
}

shared_channelizer_impl::shared_channelizer_impl(double input_rate,
                                                 double target_output_rate,
                                                 unsigned int max_channels,
                                                 double channel_bandwidth)
    : gr::block("shared_channelizer",
                gr::io_signature::make(1, 1, sizeof(gr_complex)),
                gr::io_signature::make(1, -1, sizeof(gr_complex))),
      d_input_rate(input_rate),
      d_target_output_rate(target_output_rate),
      d_channel_bandwidth(channel_bandwidth),
      d_max_channels(max_channels),
      d_num_outputs(0),
      d_input_sample_counter(0) {
  if (max_channels == 0) {
    throw std::invalid_argument("shared_channelizer: max_channels must be > 0");
  }
  if (target_output_rate <= 0 || input_rate <= 0) {
    throw std::invalid_argument("shared_channelizer: rates must be positive");
  }

  d_decim = static_cast<int>(std::round(input_rate / target_output_rate));
  if (d_decim < 2) {
    throw std::invalid_argument("shared_channelizer: input_rate / target_output_rate must be >= 2");
  }
  d_output_rate = input_rate / static_cast<double>(d_decim);

  d_channel_size = choose_fft_size(d_decim);
  d_fft_size = d_channel_size * d_decim;
  // 25% overlap. This gives the channel filter ~K/4 samples of effective
  // impulse-response length to settle, which is comfortable for the Hann
  // shape we apply in frequency domain.
  d_overlap_out = d_channel_size / 4;
  if (d_overlap_out < 1) d_overlap_out = 1;
  d_overlap_in = d_overlap_out * d_decim;
  d_step_in = d_fft_size - d_overlap_in;
  d_step_out = d_channel_size - d_overlap_out;

  BOOST_LOG_TRIVIAL(info) << "shared_channelizer: input_rate=" << d_input_rate
                          << " output_rate=" << d_output_rate
                          << " decim=" << d_decim
                          << " N=" << d_fft_size
                          << " K=" << d_channel_size
                          << " M=" << d_overlap_in
                          << " max_channels=" << d_max_channels;

  // Per-port state. atomic<int>/atomic<bool> are not move-constructible so
  // we allocate as raw arrays; std::vector won't work here.
  d_channel_bin_offset.reset(new std::atomic<int>[d_max_channels]);
  d_channel_enabled.reset(new std::atomic<bool>[d_max_channels]);
  for (unsigned int i = 0; i < d_max_channels; i++) {
    d_channel_bin_offset[i].store(0, std::memory_order_relaxed);
    d_channel_enabled[i].store(false, std::memory_order_relaxed);
  }

  design_channel_filter();

  // FFTW plan creation isn't thread-safe, but GR's fft_complex constructors
  // already serialize on gr::fft::planner::mutex() internally. Acquiring it
  // here would self-deadlock on the (non-recursive) std::mutex.
#if GNURADIO_VERSION >= 0x030900
  d_fwd_fft.reset(new gr::fft::fft_complex_fwd(d_fft_size, 1));
  d_inv_fft.reset(new gr::fft::fft_complex_rev(d_channel_size, 1));
#else
  d_fwd_fft.reset(new gr::fft::fft_complex(d_fft_size, true, 1));
  d_inv_fft.reset(new gr::fft::fft_complex(d_channel_size, false, 1));
#endif

  // Ensure we're called with output sizes aligned to whole FFT blocks. GR
  // will then hand us noutput_items that's a multiple of d_step_out.
  set_output_multiple(d_step_out);
  // Keep the previous M input samples around so each FFT block can read N
  // samples (M of which come from the prior block).
  set_history(d_overlap_in + 1);
}

shared_channelizer_impl::~shared_channelizer_impl() = default;

void shared_channelizer_impl::set_channel_offset(unsigned int port, double offset_hz) {
  if (port >= d_max_channels) {
    BOOST_LOG_TRIVIAL(error) << "shared_channelizer: set_channel_offset port " << port
                             << " >= max_channels " << d_max_channels;
    return;
  }
  // Map Hz offset to integer FFT bin number, modulo d_fft_size, with negative
  // frequencies wrapping to the upper half of the spectrum. Sub-bin tuning
  // is left to the recorder's downstream resampler/squelch chain.
  double bin_spacing = d_input_rate / static_cast<double>(d_fft_size);
  int signed_bin = static_cast<int>(std::lround(offset_hz / bin_spacing));
  int bin = ((signed_bin % d_fft_size) + d_fft_size) % d_fft_size;
  d_channel_bin_offset[port].store(bin, std::memory_order_relaxed);
  BOOST_LOG_TRIVIAL(debug) << "shared_channelizer: port=" << port
                           << " offset_hz=" << offset_hz
                           << " bin_spacing=" << bin_spacing
                           << " signed_bin=" << signed_bin
                           << " stored_bin=" << bin
                           << " (effective_hz=" << (signed_bin * bin_spacing) << ")";
}

void shared_channelizer_impl::set_channel_enabled(unsigned int port, bool enabled) {
  if (port >= d_max_channels) {
    BOOST_LOG_TRIVIAL(error) << "shared_channelizer: set_channel_enabled port " << port
                             << " >= max_channels " << d_max_channels;
    return;
  }
  d_channel_enabled[port].store(enabled, std::memory_order_relaxed);
}

bool shared_channelizer_impl::is_channel_enabled(unsigned int port) const {
  if (port >= d_max_channels) {
    return false;
  }
  return d_channel_enabled[port].load(std::memory_order_relaxed);
}

void shared_channelizer_impl::forecast(int noutput_items, gr_vector_int &ninput_items_required) {
  for (auto &n : ninput_items_required) {
    n = noutput_items * d_decim;
  }
}

bool shared_channelizer_impl::check_topology(int ninputs, int noutputs) {
  if (noutputs > static_cast<int>(d_max_channels)) {
    BOOST_LOG_TRIVIAL(error) << "shared_channelizer: " << noutputs
                             << " output ports connected, but only " << d_max_channels
                             << " allocated";
    return false;
  }
  d_num_outputs = static_cast<unsigned int>(noutputs);
  return ninputs == 1;
}

int shared_channelizer_impl::general_work(int noutput_items,
                                          gr_vector_int &ninput_items,
                                          gr_vector_const_void_star &input_items,
                                          gr_vector_void_star &output_items) {
  const gr_complex *in = reinterpret_cast<const gr_complex *>(input_items[0]);
  const int nblocks = noutput_items / d_step_out;
  if (nblocks <= 0) {
    return 0;
  }

  gr_complex *fwd_in = d_fwd_fft->get_inbuf();
  gr_complex *fwd_out = d_fwd_fft->get_outbuf();
  gr_complex *inv_in = d_inv_fft->get_inbuf();
  gr_complex *inv_out = d_inv_fft->get_outbuf();
  const int K = d_channel_size;
  const int N = d_fft_size;
  const int half_K = K / 2;

  for (int b = 0; b < nblocks; b++) {
    // Copy N input samples (including M overlap from prior block) into the
    // forward FFT input buffer. With set_history(M+1), input[0..N-1] for
    // b=0 is M old samples + step_in new ones; subsequent blocks step
    // forward by step_in samples.
    std::memcpy(fwd_in, in + b * d_step_in, N * sizeof(gr_complex));
    d_fwd_fft->execute();

    // Global time index (in input samples, mod N) at the start of this
    // block. The mod-N is what we need for the per-block phase correction
    // and keeps the math in a range where double precision is exact.
    uint64_t t_b = (d_input_sample_counter + static_cast<uint64_t>(b) * d_step_in) % static_cast<uint64_t>(N);

    for (unsigned int port = 0; port < d_num_outputs; port++) {
      if (!d_channel_enabled[port].load(std::memory_order_relaxed)) {
        continue;
      }

      // Center the K-bin window on the tuned bin. Negative offsets are
      // already wrapped into [0, N) by set_channel_offset, so we just
      // index modulo N.
      int center_bin = d_channel_bin_offset[port].load(std::memory_order_relaxed);
      int start_bin = center_bin - half_K;
      // Pre-wrap into [0, N) so the inner loop avoids a branch.
      start_bin = ((start_bin % N) + N) % N;

      // Bin-select + frequency-domain filter into the IFFT input buffer.
      // We also fftshift so that the channel's tuned bin lands at DC after
      // IFFT — i.e. the output is at baseband.
      for (int k = 0; k < K; k++) {
        int src = start_bin + k;
        if (src >= N) src -= N;
        // Shift: bins below DC of the extracted band map to negative time
        // frequencies (upper half of IFFT input).
        int dst = (k + half_K) % K;
        inv_in[dst] = fwd_out[src] * d_channel_filter_freq[k];
      }
      d_inv_fft->execute();

      // Per-block phase correction. Without this, each IFFT output block
      // is rotated by exp(2πi · center_bin · t_b / N) relative to block 0
      // — a discontinuity that prevents downstream FLLs from locking and
      // injects spurs at the block rate. Applying exp(-2πi · ...) un-does
      // it so consecutive blocks concatenate into a phase-coherent stream.
      double phase = -2.0 * M_PI * static_cast<double>(center_bin) *
                     static_cast<double>(t_b) / static_cast<double>(N);
      gr_complex rotor(static_cast<float>(std::cos(phase)),
                       static_cast<float>(std::sin(phase)));

      // Discard the first d_overlap_out samples (corrupted by circular
      // convolution wraparound) and emit the rest, rotated.
      gr_complex *out_port = reinterpret_cast<gr_complex *>(output_items[port]);
      gr_complex *dst_ptr = out_port + b * d_step_out;
      const gr_complex *src_ptr = inv_out + d_overlap_out;
      for (int m = 0; m < d_step_out; m++) {
        dst_ptr[m] = src_ptr[m] * rotor;
      }
    }
  }

  d_input_sample_counter = (d_input_sample_counter +
                            static_cast<uint64_t>(nblocks) * d_step_in) %
                           static_cast<uint64_t>(N);

  const int produced_per_port = nblocks * d_step_out;
  for (unsigned int port = 0; port < d_num_outputs; port++) {
    if (d_channel_enabled[port].load(std::memory_order_relaxed)) {
      produce(static_cast<int>(port), produced_per_port);
    }
  }

  consume_each(nblocks * d_step_in);
  return WORK_CALLED_PRODUCE;
}

} // namespace blocks
} // namespace gr
