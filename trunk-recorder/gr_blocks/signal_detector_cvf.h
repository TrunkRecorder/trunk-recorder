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

#ifndef INCLUDED_INSPECTOR_SIGNAL_DETECTOR_CVF_H
#define INCLUDED_INSPECTOR_SIGNAL_DETECTOR_CVF_H

#include <memory>
#include <vector>

#include "shared_channelizer.h"

// signal_detector: Energy-detection scanner used by Source to enable
// conventional recorders when their channel's RF energy goes above a
// (possibly auto-calibrated) threshold.
//
// In the new architecture this is no longer a GR block in the flowgraph —
// it consumes the shared_channelizer's already-computed wideband forward
// FFT via get_spectrum_snapshot(). Source calls update() periodically
// (currently from enable_detected_recorders()) before reading the
// detected-signal list. This eliminates the second wideband FFT that the
// old GR block was running on the SDR stream.

struct Detected_Signal {
  int avg_rssi;
  int max_rssi;
  int min_rssi;
  int threshold;
  std::vector<int> rssi;
  double center_freq;
  double bandwidth;
  unsigned int start_bin;
  unsigned int end_bin;
};

class signal_detector_cvf {
public:
  typedef std::shared_ptr<signal_detector_cvf> sptr;

  // Construct a signal detector that reads spectrum snapshots from the
  // given shared_channelizer. samp_rate is needed to map bin index to Hz
  // when building Detected_Signal entries.
  //
  // The threshold/sensitivity/average/quantization/min_bw/max_bw arguments
  // are the same as before; threshold is initial only when auto_threshold
  // is true. window_type is currently ignored (the shared_channelizer's FFT
  // is unwindowed).
  static sptr make(gr::blocks::shared_channelizer::sptr channelizer,
                   double samp_rate,
                   int /*window_type*/ = 0,
                   float threshold = -45.0f,
                   float sensitivity = 0.9f,
                   bool auto_threshold = true,
                   float average = 0.8f,
                   float quantization = 0.01f,
                   float min_bw = 0.0f,
                   float max_bw = 50000.0f);

  virtual ~signal_detector_cvf() = default;

  // Pull a spectrum snapshot from the channelizer and run detection.
  // Internally throttled to ~100 ms between snapshots so it's cheap to
  // call from a hot loop.
  virtual void update() = 0;

  // Latest detection results. Returns a copy (thread-safe).
  virtual std::vector<Detected_Signal> get_detected_signals() = 0;

  // Knobs (same semantics as old code).
  virtual void set_threshold(float threshold) = 0;
  virtual void set_sensitivity(float sensitivity) = 0;
  virtual void set_auto_threshold(bool auto_threshold) = 0;
  virtual void set_average(float average) = 0;
};

#endif /* INCLUDED_INSPECTOR_SIGNAL_DETECTOR_CVF_H */
