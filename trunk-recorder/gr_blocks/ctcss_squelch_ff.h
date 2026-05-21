/*
 * CTCSS detector + audio gate (single block, replaces gr::analog::ctcss_squelch_ff
 * AND the CTCSS portion of trunk-recorder's tone_detector_block).
 *
 * Float-in / float-out passthrough block. Maintains per-CTCSS-bin sliding-window
 * Goertzel power estimates on an internally-decimated sub-audible-band copy of
 * the input. Gates audio output to zeros when the configured tone is not heard
 * (or, in free-scan mode, when no tone above threshold). Exposes a verdict
 * structure that callers can read at end-of-call to identify what tone was
 * actually on air (with diagnostic top-N for the wrong-agency case).
 *
 * Why we wrote our own (vs. gr::analog::ctcss_squelch_ff):
 *   - Stock block opens/closes on opaque thresholds (level, len, ramp), making
 *     bench tuning into trial-and-error.
 *   - Stock block can stay closed for 1.5+ s on the very call we're trying to
 *     detect (bench evidence: 5 NOT-heard events on calls <1.5 s with strong
 *     tones in raw audio; see DECISIONS_*.md).
 *   - Stock block lives in the audio path AND requires a parallel detector
 *     for state truth — two sources of truth, polling races.
 *   - Stock block has no concept of an adaptive sub-audible noise floor;
 *     it treats voice low-frequency harmonics the same as a real tone.
 *
 * What this block does differently:
 *   1. Sliding 200 ms Goertzel window evaluated every 50 ms — ~100 ms time
 *      to lock for any call ≥ 100 ms with a real tone. (vs the 1.5 s minimum
 *      our block-based detector currently needs.)
 *   2. Adaptive noise floor = median of all 50 bin powers. Real CTCSS
 *      tone reads 20-50 dB above; voice harmonics typically only 5-10 dB.
 *      Single SNR threshold cleanly separates them.
 *   3. Energy-weighted cumulative scoring across the call. Voice harmonics
 *      stealing 30 % of windows can't outvote a tone that's the dominant
 *      energy contributor across the whole call.
 *   4. Hysteresis-gated squelch: open threshold > close threshold, with
 *      configurable hangtimes. No chatter.
 */

#ifndef INCLUDED_CTCSS_SQUELCH_FF_H
#define INCLUDED_CTCSS_SQUELCH_FF_H

#include <boost/log/trivial.hpp>
#include <boost/thread/mutex.hpp>
#include <gnuradio/blocks/api.h>
#include <gnuradio/sync_block.h>
#include <utility>
#include <vector>

namespace gr {
namespace trunkrecorder {

struct ctcss_squelch_verdict {
  // 0.0 if no confident lock during the call, else the detected CTCSS
  // frequency in Hz. For configured channels this normally equals the
  // configured tone (matches), but in shared-frequency multi-PL setups
  // it can equal a *different* CTCSS standard frequency — that's the
  // wrong-agency-keyed-up case operators need to see.
  float detected_hz;
  // Fraction of evaluation ticks (0-1) where the detected bin was the
  // winner above threshold. Above ~0.5 is a confident detection.
  float confidence;
  // Mean SNR (dB above sub-audible noise floor) of the detected bin
  // across the ticks where it was the winner.
  float dominant_snr_db;
  // Top three winning bins (frequency, mean SNR) for diagnostics —
  // useful when the detected tone wasn't the configured one.
  std::vector<std::pair<float, float>> top_three;
};

class BLOCKS_API ctcss_squelch_ff : virtual public sync_block {
public:
#if GNURADIO_VERSION < 0x030900
  typedef boost::shared_ptr<ctcss_squelch_ff> sptr;
#else
  typedef std::shared_ptr<ctcss_squelch_ff> sptr;
#endif

  // sample_rate: full input audio sample rate (e.g. 16000 Hz).
  // configured_pl_freq: CTCSS frequency the recorder is configured for.
  //   When > 0, the block gates audio open/closed based on whether THIS
  //   tone is currently above its open/close thresholds — same role as
  //   ctcss_squelch_ff for live audio gating.
  //   When 0 (free-scan mode), the block gates open whenever ANY tone
  //   above the open threshold is the winner, and the verdict reports
  //   whichever tone scored highest cumulatively over the call.
  // gate_audio: when true (default), work() zeros output samples whenever
  //   the squelch is muted (live audio-path gating). When false, output
  //   is a passthrough of input regardless of detection state — used
  //   when this block runs as a side-chain detector alongside a separate
  //   gating block (or in search mode where audio passes through
  //   ungated).
  static sptr make(double sample_rate, float configured_pl_freq = 0.0f, bool gate_audio = true);

  ctcss_squelch_ff(double sample_rate, float configured_pl_freq, bool gate_audio);
  ~ctcss_squelch_ff();

  int work(int noutput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items) override;

  // Polled at end-of-transmission by analog_recorder for the Tone Result
  // log line and the JSON `tone` field. Safe to call from any thread.
  ctcss_squelch_verdict get_verdict() const;

  // True when the squelch is currently open (tone above open threshold +
  // hysteresis). Useful for the recorder's tone_was_ever_unmuted()
  // shortcut when polling is needed before end-of-call.
  bool is_unmuted() const;

  // Reset per-call state (cumulative scores, squelch state, sliding
  // window). Called from analog_recorder::start() per transmission.
  void reset();

  // Tunables. Defaults are calibrated for the bench but configurable.
  void set_open_threshold_db(float db);
  void set_close_threshold_db(float db);
  void set_open_hangtime_ms(int ms);
  void set_close_hangtime_ms(int ms);

private:
  void evaluate_goertzels();    // called every eval_tick_samples
  void update_squelch_state(int winner_idx, float winner_snr_db);

  // Configuration (read-mostly; protected by d_mutex when changed).
  double  d_sample_rate;        // full input rate, e.g. 16000
  float   d_configured_pl_freq; // 0 = free-scan
  int     d_configured_bin;     // index into d_bins; -1 if not in table
  bool    d_gate_audio;         // true: zero output when muted; false: passthrough

  // Pre-built immutable tables.
  static constexpr int N_CTCSS = 50;
  struct bin_state {
    double freq;        // standard CTCSS Hz
    double coeff;       // 2 * cos(2*pi*f/decim_rate)
    double cum_score;   // sum of SNR-dB while this bin was the winner above threshold
    int    win_ticks;   // how many evaluation ticks this bin was the winner above threshold
    double energy_sum;  // sum of sqrt(Goertzel power) across ALL ticks (loudness,
                        // not just winning ticks). Used by get_verdict()'s
                        // cross-metric agreement guard — see the four-way
                        // check there.
  };
  std::vector<bin_state> d_bins;

  // Decimation: full rate → ~1 kHz sub-audible.
  int                d_decim_factor;     // e.g. 16 (16k/1k)
  std::vector<float> d_decim_taps;       // anti-alias LPF, ~32 taps, 300 Hz cutoff
  std::vector<float> d_decim_state;      // FIR delay line
  int                d_decim_state_idx;
  int                d_decim_phase;      // counts 0..decim_factor-1

  // Sliding 500 ms window of decimated samples for Goertzel.
  int                d_window_samples;   // e.g. 500 at 1 kHz
  std::vector<float> d_window;           // ring buffer
  int                d_window_write_idx; // 0..window_samples-1
  bool               d_window_filled;

  // ---- Notched-reference path (verify mode only) ----
  // OpenAudio-style detector: pass the same decimated audio through a
  // 4-stage cascaded biquad notch centered on the configured tone (Q=60,
  // staggered ±0.18%/±0.45% per stage → ~2 Hz total null width). The
  // post-notch signal contains everything in the sub-audible band EXCEPT
  // the configured tone, so its power is a "voice / noise reference."
  // Per evaluation tick: target_pwr / notched_band_pwr >= NOTCH_REL_THRESH
  // is required in addition to the SNR-vs-median test for the gate to
  // open and the bin to score. A real sine survives the notch chain
  // (target_pwr stays high, notched_pwr collapses → ratio huge); voice
  // fundamentals at the configured frequency raise BOTH target_pwr and
  // notched_pwr (because voice has broad spectral spread inside the
  // 67-254 Hz band) so the ratio stays small.
  //
  // d_notch_chain is sized 4 (cascaded biquads) when d_configured_pl_freq>0
  // and is empty otherwise — search mode bypasses the notch path entirely
  // since "which tone" decisions can't share a single notch.
  struct biquad_state {
    double b0, b1, b2;   // numerator (feed-forward)
    double a1, a2;       // denominator (feedback), pre-negated form:
                         //   y = b0*x + b1*x1 + b2*x2 + a1*y1 + a2*y2
    double x1, x2;       // input delay line
    double y1, y2;       // output delay line
  };
  std::vector<biquad_state> d_notch_chain;
  std::vector<float>        d_notched_window;  // parallel ring buffer, same indexing as d_window

  // Evaluation cadence: every ~50 ms of decimated samples.
  int                d_eval_tick_samples;  // e.g. 50 at 1 kHz
  int                d_samples_since_eval;
  long               d_total_eval_ticks;

  // Squelch hysteresis state.
  float              d_open_threshold_db;
  float              d_close_threshold_db;
  int                d_open_hangtime_ticks;   // # of evals above open thresh required
  int                d_close_hangtime_ticks;  // # of evals below close thresh required
  int                d_open_streak;
  int                d_close_streak;
  bool               d_unmuted;

  // Synchronization for cross-thread accessor reads.
  mutable boost::mutex d_mutex;
};

} // namespace trunkrecorder
} // namespace gr

#endif // INCLUDED_CTCSS_SQUELCH_FF_H
