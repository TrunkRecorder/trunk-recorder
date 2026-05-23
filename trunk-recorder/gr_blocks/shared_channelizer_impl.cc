#include "shared_channelizer_impl.h"

#include <gnuradio/io_signature.h>
#include <gnuradio/filter/firdes.h>

#include <boost/log/trivial.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
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
      d_diagnostic_interval(0),
      d_diagnostic_block_count(0) {
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

  // FFT size N must be a multiple of decim so decimation lands cleanly. Pick
  // N to be large enough that the per-block step gives a few hundred output
  // samples (~5 ms of audio), which keeps the FFT cost low and FLL-relevant
  // dynamics not averaged across too many symbols. We err small here vs the
  // bin-select version because we no longer need K = N/decim to be useful.
  int target_step_out = 96;
  d_step_out = target_step_out;
  d_step_in = d_step_out * d_decim;
  // Overlap = filter length. We design a band-pass with ~12 kHz transition
  // width, which yields ~256 taps at 8 MHz input. Round up to a comfy power
  // of two over that.
  d_overlap_in = 512;
  d_fft_size = d_step_in + d_overlap_in;
  // Round FFT size up to multiple of decim so we can decimate cleanly later.
  // (Decimation here is applied AFTER IFFT, not via bin selection — only
  // sample alignment matters.)

  BOOST_LOG_TRIVIAL(info) << "shared_channelizer: input_rate=" << d_input_rate
                          << " output_rate=" << d_output_rate
                          << " decim=" << d_decim
                          << " N=" << d_fft_size
                          << " step_in=" << d_step_in
                          << " step_out=" << d_step_out
                          << " M=" << d_overlap_in
                          << " max_channels=" << d_max_channels;

  d_channel_bin_offset.reset(new std::atomic<int>[d_max_channels]);
  d_channel_enabled.reset(new std::atomic<bool>[d_max_channels]);
  d_channel_rotator_phase_inc.reset(new std::atomic<double>[d_max_channels]);
  d_channel_rotator.assign(d_max_channels, gr_complex(1.0f, 0.0f));
  d_channel_rotator_renorm.assign(d_max_channels, 0);
  for (unsigned int i = 0; i < d_max_channels; i++) {
    d_channel_bin_offset[i].store(0, std::memory_order_relaxed);
    d_channel_enabled[i].store(false, std::memory_order_relaxed);
    d_channel_rotator_phase_inc[i].store(0.0, std::memory_order_relaxed);
  }

  design_prototype_filter();

#if GNURADIO_VERSION >= 0x030900
  d_fwd_fft.reset(new gr::fft::fft_complex_fwd(d_fft_size, 1));
  d_inv_fft.reset(new gr::fft::fft_complex_rev(d_fft_size, 1));
#else
  d_fwd_fft.reset(new gr::fft::fft_complex(d_fft_size, true, 1));
  d_inv_fft.reset(new gr::fft::fft_complex(d_fft_size, false, 1));
#endif

  set_output_multiple(d_step_out);
  set_history(d_overlap_in + 1);

  if (const char *env = std::getenv("TR_CHANNELIZER_DIAG")) {
    d_diagnostic_interval = static_cast<uint64_t>(std::atoi(env));
  }
}

shared_channelizer_impl::~shared_channelizer_impl() = default;

void shared_channelizer_impl::design_prototype_filter() {
  // Design a real low-pass at baseband with cutoff ~channel_bandwidth/2 and
  // a transition band wider than that. We'll FFT it to N bins; per-port
  // filters are this prototype circularly shifted to the port's carrier
  // bin. Real-domain filter is fine because we apply the carrier shift
  // separately.
  //
  // The wide pre-filter here (channel_bandwidth) is intentionally generous —
  // the recorder's downstream channel_lpf does the tight selection. We just
  // need to suppress out-of-band aliases before decimation.
  double cutoff = std::max(d_channel_bandwidth, 24000.0);  // ≥ 24 kHz
  double transition = std::max(d_channel_bandwidth * 0.5, 12000.0);
  std::vector<float> taps = gr::filter::firdes::low_pass_2(
      1.0, d_input_rate, cutoff, transition, 60.0
#if GNURADIO_VERSION >= 0x030900
      , gr::fft::window::WIN_HAMMING
#else
      , gr::filter::firdes::WIN_HAMMING
#endif
      );

  if (static_cast<int>(taps.size()) >= d_fft_size) {
    BOOST_LOG_TRIVIAL(warning) << "shared_channelizer: filter taps (" << taps.size()
                               << ") >= FFT size (" << d_fft_size << "); truncating";
    taps.resize(d_fft_size - 1);
  }

  // Zero-pad taps to N and FFT to get the prototype's frequency-domain
  // representation. Also fold in the 1/N scaling so the IFFT output is
  // amplitude-correct.
  std::vector<gr_complex> tap_time(d_fft_size, gr_complex(0.0f, 0.0f));
  for (size_t i = 0; i < taps.size(); i++) {
    tap_time[i] = gr_complex(taps[i] / static_cast<float>(d_fft_size), 0.0f);
  }

#if GNURADIO_VERSION >= 0x030900
  gr::fft::fft_complex_fwd tmp_fft(d_fft_size, 1);
#else
  gr::fft::fft_complex tmp_fft(d_fft_size, true, 1);
#endif
  std::memcpy(tmp_fft.get_inbuf(), tap_time.data(), d_fft_size * sizeof(gr_complex));
  tmp_fft.execute();
  d_prototype_freq.assign(tmp_fft.get_outbuf(), tmp_fft.get_outbuf() + d_fft_size);

  BOOST_LOG_TRIVIAL(info) << "shared_channelizer: prototype filter taps=" << taps.size()
                          << " (cutoff=" << cutoff << " Hz, transition=" << transition << " Hz)";
}

void shared_channelizer_impl::set_channel_offset(unsigned int port, double offset_hz) {
  if (port >= d_max_channels) {
    BOOST_LOG_TRIVIAL(error) << "shared_channelizer: set_channel_offset port " << port
                             << " >= max_channels " << d_max_channels;
    return;
  }
  // The per-port filter in the freq domain is the prototype filter
  // circularly shifted by the carrier bin. We use that bin index to compute
  // the shift in work(). The continuous part of the offset (sub-bin) is
  // applied by the output-rate rotator (= what the old freq_xlating did
  // after its fft_filter_ccc).
  double bin_spacing = d_input_rate / static_cast<double>(d_fft_size);
  int signed_bin = static_cast<int>(std::lround(offset_hz / bin_spacing));
  int bin = ((signed_bin % d_fft_size) + d_fft_size) % d_fft_size;
  d_channel_bin_offset[port].store(bin, std::memory_order_relaxed);
  // Output-rate rotator phase increment: -2π · offset_hz / output_rate
  // (matches the rotator in gr_blocks/freq_xlating_fft_filter).
  double phase_inc = -2.0 * M_PI * offset_hz / d_output_rate;
  d_channel_rotator_phase_inc[port].store(phase_inc, std::memory_order_relaxed);
  BOOST_LOG_TRIVIAL(debug) << "shared_channelizer: port=" << port
                           << " offset_hz=" << offset_hz
                           << " bin_spacing=" << bin_spacing
                           << " signed_bin=" << signed_bin
                           << " stored_bin=" << bin
                           << " rotator_phase_inc=" << phase_inc << " rad/sample";
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
  if (port >= d_max_channels) return false;
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
  if (nblocks <= 0) return 0;

  gr_complex *fwd_in = d_fwd_fft->get_inbuf();
  const gr_complex *fwd_out = d_fwd_fft->get_outbuf();
  gr_complex *inv_in = d_inv_fft->get_inbuf();
  const gr_complex *inv_out = d_inv_fft->get_outbuf();
  const int N = d_fft_size;

  for (int b = 0; b < nblocks; b++) {
    std::memcpy(fwd_in, in + b * d_step_in, N * sizeof(gr_complex));
    d_fwd_fft->execute();

    for (unsigned int port = 0; port < d_num_outputs; port++) {
      if (!d_channel_enabled[port].load(std::memory_order_relaxed)) {
        continue;
      }

      int carrier_bin = d_channel_bin_offset[port].load(std::memory_order_relaxed);

      // Per-port filter response = prototype shifted by carrier_bin. That is,
      // H_port[k] = H_proto[(k - carrier_bin) mod N]. Multiply with input
      // spectrum and place in IFFT input.
      for (int k = 0; k < N; k++) {
        int proto_idx = ((k - carrier_bin) % N + N) % N;
        inv_in[k] = fwd_out[k] * d_prototype_freq[proto_idx];
      }
      d_inv_fft->execute();

      // Discard the first M samples (overlap-save corruption region), then
      // decimate by d_decim and apply the output-rate rotator that shifts
      // the carrier from its post-decimation alias back to DC.
      gr_complex *out_port = reinterpret_cast<gr_complex *>(output_items[port]);
      gr_complex *dst_ptr = out_port + b * d_step_out;
      double phase_inc = d_channel_rotator_phase_inc[port].load(std::memory_order_relaxed);
      gr_complex one_step(static_cast<float>(std::cos(phase_inc)),
                          static_cast<float>(std::sin(phase_inc)));
      gr_complex rotator = d_channel_rotator[port];

      for (int m = 0; m < d_step_out; m++) {
        int src_idx = d_overlap_in + m * d_decim;
        dst_ptr[m] = inv_out[src_idx] * rotator;
        rotator *= one_step;
      }
      d_channel_rotator[port] = rotator;

      if (++d_channel_rotator_renorm[port] >= 256) {
        d_channel_rotator_renorm[port] = 0;
        float mag = std::abs(d_channel_rotator[port]);
        if (mag > 1e-6f) {
          d_channel_rotator[port] /= mag;
        } else {
          d_channel_rotator[port] = gr_complex(1.0f, 0.0f);
        }
      }

      if (d_diagnostic_interval > 0 &&
          (d_diagnostic_block_count + b) % d_diagnostic_interval == 0) {
        gr_complex tuned = fwd_out[carrier_bin];
        float tuned_mag = std::sqrt(tuned.real() * tuned.real() + tuned.imag() * tuned.imag()) / d_fft_size;
        double out_sum2 = 0.0;
        for (int m = 0; m < d_step_out; m++) {
          out_sum2 += dst_ptr[m].real() * dst_ptr[m].real() + dst_ptr[m].imag() * dst_ptr[m].imag();
        }
        float out_rms = std::sqrt(out_sum2 / d_step_out);
        BOOST_LOG_TRIVIAL(debug) << "shared_channelizer: port=" << port
                                 << " carrier_bin=" << carrier_bin
                                 << " tuned_amp=" << tuned_mag
                                 << " output_rms=" << out_rms
                                 << " block_count=" << (d_diagnostic_block_count + b);
      }
    }
  }

  const int produced_per_port = nblocks * d_step_out;
  for (unsigned int port = 0; port < d_num_outputs; port++) {
    if (d_channel_enabled[port].load(std::memory_order_relaxed)) {
      produce(static_cast<int>(port), produced_per_port);
    }
  }

  consume_each(nblocks * d_step_in);
  d_diagnostic_block_count += nblocks;
  return WORK_CALLED_PRODUCE;
}

} // namespace blocks
} // namespace gr
