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

#ifndef INCLUDED_INSPECTOR_SIGNAL_DETECTOR_CVF_IMPL_H
#define INCLUDED_INSPECTOR_SIGNAL_DETECTOR_CVF_IMPL_H

#include "signal_detector_cvf.h"

#include <gnuradio/filter/single_pole_iir.h>

#include <boost/log/trivial.hpp>
#include <mutex>
#include <vector>

class signal_detector_cvf_impl : public signal_detector_cvf {
private:
  gr::blocks::shared_channelizer::sptr d_channelizer;
  std::mutex d_mutex;
  bool d_auto_threshold;
  unsigned int d_fft_len;
  float d_threshold, d_sensitivity, d_average, d_quantization, d_min_bw, d_max_bw;
  double d_samp_rate;

  // Per-bin power values (after averaging) used for detection. Sized to
  // match d_fft_len.
  std::vector<float> d_pxx_avg;
  // Per-bin IIR smoothers.
  std::vector<gr::filter::single_pole_iir<float, float, double>> d_avg_filter;
  // Scratch for the snapshot copy and sort.
  std::vector<float> d_snapshot;
  std::vector<float> d_tmp_pxx;
  // Bin → Hz map (fftshifted: -fs/2 to +fs/2).
  std::vector<float> d_freq;
  // Latest detection list.
  std::vector<Detected_Signal> d_detected_signals;
  uint64_t d_last_update_ms;

  uint64_t time_since_epoch_millisec();
  void rebuild_for_fft_size(unsigned int fft_len);
  void build_threshold();
  std::vector<Detected_Signal> find_signal_edges();

public:
  signal_detector_cvf_impl(gr::blocks::shared_channelizer::sptr channelizer,
                           double samp_rate,
                           float threshold,
                           float sensitivity,
                           bool auto_threshold,
                           float average,
                           float quantization,
                           float min_bw,
                           float max_bw);
  ~signal_detector_cvf_impl() override = default;

  void update() override;
  std::vector<Detected_Signal> get_detected_signals() override;

  void set_threshold(float threshold) override {
    d_auto_threshold = false;
    d_threshold = threshold;
  }
  void set_sensitivity(float sensitivity) override { d_sensitivity = sensitivity; }
  void set_auto_threshold(bool auto_threshold) override { d_auto_threshold = auto_threshold; }
  void set_average(float average) override {
    d_average = average;
    for (unsigned int i = 0; i < d_fft_len; i++) {
      d_avg_filter[i].set_taps(d_average);
    }
  }
};

#endif /* INCLUDED_INSPECTOR_SIGNAL_DETECTOR_CVF_IMPL_H */
