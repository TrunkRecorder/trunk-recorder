/* -*- c++ -*- */
/*
 * Copyright 2019 Free Software Foundation Inc..
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include "signal_detector_cvf_impl.h"

#include <algorithm>
#include <chrono>
#include <cmath>

signal_detector_cvf::sptr signal_detector_cvf::make(
    gr::blocks::shared_channelizer::sptr channelizer,
    double samp_rate,
    int /*window_type*/,
    float threshold,
    float sensitivity,
    bool auto_threshold,
    float average,
    float quantization,
    float min_bw,
    float max_bw) {
  return std::make_shared<signal_detector_cvf_impl>(
      channelizer, samp_rate, threshold, sensitivity, auto_threshold, average,
      quantization, min_bw, max_bw);
}

uint64_t signal_detector_cvf_impl::time_since_epoch_millisec() {
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

signal_detector_cvf_impl::signal_detector_cvf_impl(
    gr::blocks::shared_channelizer::sptr channelizer,
    double samp_rate,
    float threshold,
    float sensitivity,
    bool auto_threshold,
    float average,
    float quantization,
    float min_bw,
    float max_bw)
    : d_channelizer(channelizer),
      d_auto_threshold(auto_threshold),
      d_fft_len(channelizer->get_fft_size()),
      d_threshold(threshold),
      d_sensitivity(sensitivity),
      d_average(average),
      d_quantization(quantization),
      d_min_bw(min_bw),
      d_max_bw(max_bw),
      d_samp_rate(samp_rate),
      d_last_update_ms(0) {
  rebuild_for_fft_size(d_fft_len);
  BOOST_LOG_TRIVIAL(info) << "signal_detector: shared FFT bin size: " << (samp_rate / d_fft_len)
                          << " Hz, fft_len=" << d_fft_len
                          << ", quantization=" << static_cast<int>(std::floor(d_quantization * d_samp_rate))
                          << " Hz, threshold=" << threshold
                          << ", auto=" << auto_threshold;
}

void signal_detector_cvf_impl::rebuild_for_fft_size(unsigned int fft_len) {
  d_fft_len = fft_len;
  d_pxx_avg.assign(fft_len, 0.0f);
  d_snapshot.assign(fft_len, 0.0f);
  d_tmp_pxx.assign(fft_len, 0.0f);
  d_avg_filter.clear();
  d_avg_filter.resize(fft_len);
  for (unsigned int i = 0; i < fft_len; i++) {
    d_avg_filter[i].set_taps(d_average);
  }
  // fftshifted frequency vector: from -fs/2 to +fs/2.
  d_freq.resize(fft_len);
  double step = d_samp_rate / fft_len;
  double point = -d_samp_rate / 2;
  for (unsigned int i = 0; i < fft_len; i++) {
    d_freq[i] = static_cast<float>(point);
    point += step;
  }
}

void signal_detector_cvf_impl::update() {
  uint64_t now = time_since_epoch_millisec();
  if (now - d_last_update_ms < 100) {
    return;  // throttle to 10 Hz
  }
  d_last_update_ms = now;

  // Get the latest |X[k]|² snapshot from the channelizer (raw FFTW order).
  d_channelizer->get_spectrum_snapshot(d_snapshot);
  if (d_snapshot.size() != d_fft_len) {
    // Channelizer's FFT size changed under us. Rebuild internal state.
    rebuild_for_fft_size(static_cast<unsigned int>(d_snapshot.size()));
  }

  // Convert raw power to dB and apply fftshift in one pass:
  //   raw layout: [DC, +Δf, +2Δf, ..., +Nyquist, -Nyquist+Δf, ..., -Δf]
  //   shifted:    [-fs/2, ..., -Δf, DC, +Δf, ..., +fs/2-Δf]
  // 10·log10 with a tiny floor to avoid log(0).
  const unsigned int half = d_fft_len / 2;
  const float scale = 1.0f / static_cast<float>(d_fft_len * d_fft_len);  // matches old norm factor
  for (unsigned int i = 0; i < d_fft_len; i++) {
    unsigned int src = (i + half) % d_fft_len;
    float p = d_snapshot[src] * scale;
    float db = (p > 1e-20f) ? 10.0f * std::log10(p) : -200.0f;
    d_pxx_avg[i] = static_cast<float>(d_avg_filter[i].filter(db));
  }

  if (d_auto_threshold) {
    build_threshold();
  }

  {
    std::lock_guard<std::mutex> lock(d_mutex);
    d_detected_signals = find_signal_edges();
  }
}

void signal_detector_cvf_impl::build_threshold() {
  // Replicate the old algorithm: sort the (averaged, dB-scale) power spectrum,
  // search for a large jump in the upper half — the jump indicates the
  // transition from noise floor to signal energy.
  std::copy(d_pxx_avg.begin(), d_pxx_avg.end(), d_tmp_pxx.begin());
  std::sort(d_tmp_pxx.begin(), d_tmp_pxx.end());

  float range = d_tmp_pxx[d_fft_len - 1] - d_tmp_pxx[0];
  d_threshold = 500.0f;  // sentinel
  for (unsigned int i = d_fft_len / 2; i < d_fft_len - 1; i++) {
    if ((d_tmp_pxx[i + 1] - d_tmp_pxx[i]) / range > 1.0f - d_sensitivity) {
      d_threshold = d_tmp_pxx[i];
      break;
    }
  }
  if (d_threshold == 500.0f) {
    d_threshold = d_tmp_pxx[d_fft_len - 1];
  }
}

std::vector<Detected_Signal> signal_detector_cvf_impl::find_signal_edges() {
  Detected_Signal signal;
  std::vector<Detected_Signal> detected_signals;
  bool signal_started = false;
  int quantization = static_cast<int>(std::floor(d_quantization * d_samp_rate));

  for (unsigned int i = 0; i < d_fft_len; i++) {
    if (d_pxx_avg[i] > d_threshold) {
      if (!signal_started) {
        signal_started = true;
        signal.avg_rssi = static_cast<int>(d_pxx_avg[i]);
        signal.max_rssi = static_cast<int>(d_pxx_avg[i]);
        signal.min_rssi = static_cast<int>(d_pxx_avg[i]);
        signal.start_bin = i;
        signal.end_bin = i;
        signal.rssi.clear();
        signal.rssi.push_back(static_cast<int>(d_pxx_avg[i]));
      } else {
        signal.avg_rssi = (signal.avg_rssi + static_cast<int>(d_pxx_avg[i])) / 2;
        signal.rssi.push_back(static_cast<int>(d_pxx_avg[i]));
        if (d_pxx_avg[i] > signal.max_rssi) signal.max_rssi = static_cast<int>(d_pxx_avg[i]);
        if (d_pxx_avg[i] < signal.min_rssi) signal.min_rssi = static_cast<int>(d_pxx_avg[i]);
        signal.end_bin = i;
      }
    } else if (signal_started) {
      signal_started = false;
      double bandwidth = d_freq[signal.end_bin] - d_freq[signal.start_bin];
      double quantized_bandwidth = quantization * std::round(bandwidth / quantization);
      signal.bandwidth = quantized_bandwidth;
      if (quantized_bandwidth >= d_min_bw && quantized_bandwidth <= d_max_bw) {
        signal.center_freq = (d_freq[signal.start_bin] + d_freq[signal.end_bin]) / 2;
        signal.threshold = static_cast<int>(d_threshold);
        detected_signals.push_back(signal);
      }
    }
  }
  if (signal_started) {
    double bandwidth = d_freq[signal.end_bin] - d_freq[signal.start_bin];
    signal.bandwidth = quantization * std::round(bandwidth / quantization);
    signal.center_freq = (d_freq[signal.start_bin] + d_freq[signal.end_bin]) / 2;
    signal.threshold = static_cast<int>(d_threshold);
    detected_signals.push_back(signal);
  }
  return detected_signals;
}

std::vector<Detected_Signal> signal_detector_cvf_impl::get_detected_signals() {
  std::lock_guard<std::mutex> lock(d_mutex);
  return d_detected_signals;
}
