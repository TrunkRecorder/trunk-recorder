/*
 * DCS / DPL squelch + identification.
 *
 * Single-block source of truth for DCS:
 *   - Detects on-air DCS bit-stream (16 parallel bit-clock phases, 112-code
 *     Golay-(23,12) table, both polarities, frame-anchor + cyclic-class
 *     disambiguation).
 *   - Gates audio with a smooth EMA fade. In configured mode the gate opens
 *     when the detected code is cyclically equivalent to the configured one
 *     (so D031I and D627N — same on-air bit pattern at different rotations —
 *     both open the gate of a "D031I" channel). In search mode (constructed
 *     with code=0) the gate opens on the first locked code.
 *   - Exposes an end-of-call verdict: detected code, polarity, and the full
 *     cyclic-class alias list.
 *
 * Replaces the DCS portion of the old tone_detector_block. CTCSS is handled
 * separately by ctcss_squelch_ff — the two signal types use very different
 * DSP (Goertzel sweep vs Golay-(23,12) bit-stream) and stay in their own
 * blocks for clarity and so the analog_recorder chain can pick which one is
 * in the audio path based on channel configuration.
 */

#ifndef INCLUDED_DCS_SQUELCH_FF_H
#define INCLUDED_DCS_SQUELCH_FF_H

#include <atomic>
#include <boost/log/trivial.hpp>
#include <boost/thread/mutex.hpp>
#include <gnuradio/blocks/api.h>
#include <gnuradio/sync_block.h>
#include <cstdint>
#include <map>
#include <utility>
#include <vector>

namespace gr {
namespace trunkrecorder {

// DCS_CONFIDENCE_FULL_MATCHES: how many post-lock re-confirmations to call
// a DCS detection "fully confident" (verdict.confidence = 1.0). At ~134
// baud, one re-confirm fires every 23 bits ≈ 170 ms; 10 re-confirms = ~1.7 s
// of sustained lock. A spurious one-shot lock (CTCSS bit-clock aliasing,
// burst noise) seldom produces even 2 re-confirms — those calls show
// confidence ≤ 0.1 and lose to any decent CTCSS verdict in search-mode
// reporting.
static constexpr long DCS_CONFIDENCE_FULL_MATCHES = 10;

// End-of-call DCS verdict. Caller (analog_recorder) reads this once the
// recording closes to populate the Tone Result log line and the JSON
// `tone` field.
struct dcs_squelch_verdict {
  // 0 if no DCS was confidently observed during the call, else the
  // detected octal code (decimal-encoded, e.g. 31, 162, 754).
  int  detected_code;
  // Polarity of the detected code (true = inverted / "I" / "R" suffix).
  // Meaningful only when detected_code != 0.
  bool detected_inverted;
  // Every cyclic-class sibling of the detected code in the standard 112-
  // code DCS table. Golay (23,12) is cyclic so several (code, polarity)
  // pairs share the same on-air bit pattern at different rotations; the
  // detector picks one (preferring the configured form when there's an
  // operator hint) but the full class is reported here for diagnostics.
  // Empty when no DCS was detected.
  std::vector<std::pair<int, bool>> aliases;
  // Confidence in the detection (0.0 = no lock, 1.0 = fully confirmed).
  // Used by analog_recorder's search-mode reporting to choose between
  // CTCSS and DCS verdicts when both fire. Defined as
  //   min(1.0, frames_matched_since_lock / DCS_CONFIDENCE_FULL_MATCHES)
  // where DCS_CONFIDENCE_FULL_MATCHES≈10. A real DCS stream re-matches
  // every 23 bits ≈ 170 ms, so a 2 s call reaches ≈1.0. A spurious
  // single-frame false lock (CTCSS bit-clock aliasing, etc.) caps out
  // near 0.05-0.10 — naturally loses to a strong CTCSS verdict in the
  // search-mode comparison.
  float confidence;
};

class BLOCKS_API dcs_squelch_ff : virtual public sync_block {
public:
#if GNURADIO_VERSION < 0x030900
  typedef boost::shared_ptr<dcs_squelch_ff> sptr;
#else
  typedef std::shared_ptr<dcs_squelch_ff> sptr;
#endif

  // sample_rate:               input audio rate (Hz, typically wav rate 16k)
  // configured_dcs_code:       octal code as a decimal integer (0 = search mode)
  // configured_dcs_inverted:   polarity of the configured code (meaningful
  //                            only when configured_dcs_code != 0)
  // gate_audio:                when true (default), work() applies the
  //                            smoothed gate to output samples. When false,
  //                            input is passed through unchanged while the
  //                            detector still runs — used for side-chain
  //                            "what tone is this?" identification next to
  //                            a separate gating block, and for SEARCH-mode
  //                            channels where audio is not gated.
  static sptr make(double sample_rate,
                   int    configured_dcs_code     = 0,
                   bool   configured_dcs_inverted = false,
                   bool   gate_audio              = true);

  dcs_squelch_ff(double sample_rate,
                 int    configured_dcs_code,
                 bool   configured_dcs_inverted,
                 bool   gate_audio);
  ~dcs_squelch_ff();

  int work(int noutput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items) override;

  // True when the audio gate is currently open (configured DCS or any
  // cyclic-class sibling actively matching, plus the close-hold window).
  bool is_unmuted();

  // End-of-call verdict. Safe to call from any thread.
  dcs_squelch_verdict get_verdict() const;

  // Reset per-call state (latched lock, per-phase shift registers, gate
  // smoother). Called from analog_recorder::start() per transmission.
  void reset();

private:
  double d_sample_rate;
  int    d_configured_code;        // 0 = search mode
  bool   d_configured_inverted;
  bool   d_gate_audio;             // false = passthrough, true = apply d_gate to output

  // Sub-audible detection rate: bit-clock recovery and codeword matching
  // run at this rate, audio I/O stays at d_sample_rate. The LPF is a
  // polyphase anti-alias filter — its taps span the full input window
  // but only one output is computed per d_detect_decim input samples.
  double d_detect_rate;
  int    d_detect_decim;
  int    d_decim_count;

  // Anti-alias low-pass filter (Hamming, ~250 Hz cutoff at d_sample_rate).
  std::vector<double> d_lpf_taps;
  std::vector<double> d_lpf_state;
  int                 d_lpf_state_idx;

  // One-pole IIR high-pass at ~30 Hz strips DC bias and slow drift. Runs
  // at d_detect_rate, applied to the polyphase LPF output.
  double d_hpf_a;
  double d_hpf_prev_x;
  double d_hpf_prev_y;

  // 16 parallel integrate-and-dump bit clocks. With ~7.4 samples/bit at
  // 1 kHz detection rate, 16 phases bound max phase error to ~3% — at
  // least one phase aligns cleanly within the first 23-bit codeword so
  // sub-1s DCS bursts can lock on a single frame.
  double d_decim_step;
  struct phase_state {
    double   decim_phase;     // [0, 1)
    double   bit_integ;
    int      bit_integ_n;
    uint32_t shift;           // last 23 bits, LSB = newest
    long     bits_seen;
    // Frame-alignment anchor: the first sync-aligned valid match locks
    // the expected frame position (bits_seen % 23). Subsequent sync
    // detections only count if they occur at the same modular position;
    // otherwise they're cyclic-alias false-syncs at other positions in
    // the codeword that happen to look like "100" or "011".
    int      frame_anchor;       // -1 = not yet locked
    bool     frame_anchor_inv;   // polarity of the locked frame

    // Tracking the candidate code being accumulated on this phase.
    int      detected_code;
    bool     detected_inverted;
    int      match_run;
  };
  std::vector<phase_state> d_phases;

  // Standard DCS code table (112 codes covering the standard Motorola
  // DPL 83 plus the extended set used by Uniden BCD-series scanners).
  struct dcs_entry {
    int      code;     // octal as decimal integer
    uint32_t pattern;  // 23-bit shift-register pattern (LSB = newest bit)
  };
  std::vector<dcs_entry> d_dcs_table;

  // Cyclic equivalence class key per (code, polarity) pair. Two pairs
  // share a key iff their on-air bit patterns are rotations of each
  // other — i.e. they look identical on-air at different sync alignments.
  std::map<std::pair<int, bool>, uint32_t> d_class_key;
  // Polarity-strict cyclic equivalence class key for the configured
  // (code, polarity) pair.  Two on-air patterns share this key iff they
  // are rotations of each other at the SAME polarity.  D023N and D047I
  // share one key; D023I and D047N share a different key.  The gate
  // opens only for matches against this single key — the complementary
  // polarity class is a distinct code set and is intentionally rejected.
  uint32_t d_configured_class_key;

  // Latched detection — once any phase confirms a code, we keep it for
  // the rest of the recording window so a brief mid-call signal dropout
  // doesn't erase a confident earlier identification.
  int  d_locked_code;
  bool d_locked_inverted;

  // Snapshot of the strongest confident lock seen so far this call. When
  // the stale-lock watchdog fires (a transmission ended cleanly OR a
  // false lock drifted out), the active state in d_locked_* is cleared
  // so the detector can search again — but if it had accumulated real
  // evidence we save it here. Updated only when d_post_lock_matches at
  // unlock time exceeds DCS_BEST_MIN_MATCHES (so brief false locks
  // aren't preserved). get_verdict() falls back to this when d_locked_*
  // is zero, which fixes the regression where a watchdog clearing a
  // good lock at end-of-transmission caused the call verdict to lose
  // all DCS evidence.
  int  d_best_code;
  bool d_best_inverted;
  long d_best_matches;
  int  d_best_phases;

  // Search-mode confidence counter: increments once per detection tick
  // where a phase re-confirms the latched d_locked_code.  Unused in
  // decode mode.  get_verdict() converts to [0,1] via
  // DCS_CONFIDENCE_FULL_MATCHES.
  long d_post_lock_matches;

  // Decode-mode match counter: increments once per detection tick where
  // any phase confirms a code in the configured cyclic class.  Analogous
  // to ctcss_squelch_ff::win_ticks.  Zero in search mode.
  long d_configured_match_count;
  // One-shot diagnostic log guard for decode mode. Member (not
  // thread_local static) so each block instance has its own flag and
  // GR's thread-pool scheduler cannot cause one recorder to suppress
  // another's first-match log. Reset by reset().
  bool d_first_decode_match_logged;

  // Stale-lock watchdog (search mode only). Counts detection-rate ticks
  // since the last successful re-confirmation while locked. If it exceeds
  // a timeout (~500 ms), the lock is treated as stale: d_locked_code is
  // cleared, all per-phase frame anchors and match_runs are reset, and
  // the detector starts searching again. This mirrors what a real
  // scanner does — it never permanently latches on a guess that stops
  // being confirmed. CTCSS-aliased false locks fail re-confirmation
  // within a few hundred ms because the 134.4/118.8 ratio is irrational
  // (the alignment drifts out of phase quickly), so the watchdog catches
  // them and lets the detector recover. Disabled in configured mode so a
  // real D023N gate isn't briefly forced closed by a transient.
  long d_samples_since_last_match;

  // Audio gate state. Smoothed [0,1] level applied per output sample.
  double d_gate;             // current gate value
  double d_gate_attack;      // per-sample EMA alpha (rising)
  double d_gate_decay;       // per-sample EMA alpha (falling)
  // Written by work(); read by is_unmuted() from the control thread.
  // Atomic so is_unmuted() needs no lock.
  std::atomic<bool> d_open;  // logical open/closed (drives target gate)
  long   d_samples_since_match;
  long   d_hold_samples;

  mutable boost::mutex d_mutex;

  void build_dcs_table();
  void build_lpf();
  void try_dcs_match(phase_state &p);
  // Process one input sample. Returns true if this sample updated
  // d_locked_code OR if d_locked_code already qualifies the gate to be
  // open for the current channel's configuration.
  void process_one(float sample, bool &gate_should_open);
};

} /* namespace trunkrecorder */
} /* namespace gr */

#endif /* INCLUDED_DCS_SQUELCH_FF_H */
