/*
 * DCS / DPL squelch + identification implementation. See dcs_squelch_ff.h
 * for the design overview.
 *
 * Polynomial choice: g(x) = x^11 + x^10 + x^6 + x^5 + x^4 + x^2 + 1 (0xC75)
 * — the one DCS / DPL transmitters use in practice. Verified end-to-end
 * against the canonical W9CR D023 codeword.
 */

#include "dcs_squelch_ff.h"

#include <gnuradio/io_signature.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>

namespace gr {
namespace trunkrecorder {

namespace {

// Full 112-code DCS table matching Uniden BCD-series scanner coverage:
// the original Motorola DPL 83 codes + 29 extended codes used by modern
// scanners. Same table the old tone_detector_block used; kept here as
// the single source of truth now that tone_detector_block is gone.
const int DCS_CODES[] = {
     6,   7,  15,  17,  21,  23,  25,  26,  27,  31,  32,  36,  43,  47,
    50,  51,  53,  54,  65,  71,  72,  73,  74, 114, 115, 116, 122, 125,
   131, 132, 134, 143, 145, 152, 155, 156, 162, 165, 172, 174, 205, 212,
   223, 225, 226, 232, 243, 244, 245, 246, 251, 252, 255, 261, 263, 265,
   266, 271, 274, 306, 311, 315, 325, 331, 332, 343, 346, 351, 356, 364,
   365, 371, 411, 412, 413, 423, 431, 432, 445, 446, 452, 454, 455, 462,
   464, 465, 466, 503, 506, 516, 523, 526, 532, 546, 565, 606, 612, 624,
   627, 631, 632, 654, 662, 664, 703, 712, 723, 731, 732, 734, 743, 754
};

constexpr double DCS_BIT_RATE     = 134.4;
constexpr int    N_DCS_PHASES     = 16;    // ~7.4 samples/bit at 1 kHz detect
                                            // rate → 16 phases bound max phase
                                            // error to ~3%, so at least one
                                            // aligns cleanly within the first
                                            // 23-bit codeword.
constexpr int    DCS_DIST_THRESH  = 1;     // max bit errors per frame
// Sync-aligned matching: per-frame false-positive probability for any of
// 112 codes × 2 polarities at Hamming distance ≤1 is ~0.07% on random
// bits. Two consecutive same-code matches drives per-call false-lock to
// ~0.005%. Each phase tracks match_run independently — only one phase
// needs to see 2 consecutive on the same code+polarity to confirm.
constexpr int    DCS_RUN_REQUIRED = 2;
// Stricter criterion for search mode (no operator hint). A continuous
// CTCSS tone sampled at 134.4 baud produces a quasi-periodic bit pattern
// that can land at Hamming distance 1 of a real codeword for several
// consecutive frames; 2 in a row is well within reach. The bit-clock vs
// CTCSS frequency ratio (134.4/118.8 = 1.131…) is irrational, so the
// pattern *eventually* drifts — typically within 4-5 frames the
// alignment shifts enough that Hamming distance exceeds 1. Requiring 4
// consecutive same-code matches in search mode reduces the CTCSS-aliased
// false-positive rate to near zero without hurting real DCS bursts
// (which lock all 16 phases within 2-3 frames anyway).
constexpr int    DCS_RUN_REQUIRED_SEARCH = 4;
constexpr double DCS_HOLD_SECONDS = 0.3;   // close-hold after last match
constexpr double DCS_ATTACK_MS    = 5.0;
constexpr double DCS_DECAY_MS     = 25.0;
// Search-mode stale-lock timeout. At the 1 kHz detection rate, 500 samples
// = 500 ms ≈ 3 frame periods at 134.4 baud. A real DCS signal re-confirms
// roughly every 170 ms per phase × 16 phases = much faster than this, so
// 500 ms of silence in the post-lock matches counter is strong evidence
// that whatever pattern triggered the lock has drifted out of agreement.
constexpr long   DCS_STALE_LOCK_SAMPLES = 500;
// Minimum post-lock re-matches required to preserve a lock in
// d_best_* when the watchdog fires. A real transmission produces
// many tens-hundreds of re-matches; a fleeting false lock the
// watchdog catches typically has 0-3 by the time it gets cleared.
// 5 splits these cleanly.
constexpr long   DCS_BEST_MIN_MATCHES   = 5;

uint16_t dcs_code_to_bits9(int octal_code) {
  int hundreds = (octal_code / 100) % 10;
  int tens     = (octal_code / 10)  % 10;
  int ones     = (octal_code / 1)   % 10;
  return static_cast<uint16_t>(((hundreds & 0x7) << 6) |
                               ((tens     & 0x7) << 3) |
                               ((ones     & 0x7)));
}

// Golay (23,12) systematic encoding using generator polynomial
// g(x) = x^11 + x^10 + x^6 + x^5 + x^4 + x^2 + 1 (binary 0xC75). This
// is the polynomial DCS / DPL transmitters use in practice — the other
// degree-11 factor of x^23+1 (0xAE3) generates the bit-reverse code and
// would identify every transmission as the wrong code. Verified against
// the canonical D023 codeword from the W9CR DPL/DCS wiki and eccpage.com.
uint32_t golay_23_12_encode(uint16_t message12) {
  const uint32_t gen = 0xC75;
  uint32_t reg = static_cast<uint32_t>(message12 & 0xFFF) << 11;
  for (int i = 22; i >= 11; --i) {
    if (reg & (1u << i)) {
      reg ^= (gen << (i - 11));
    }
  }
  uint32_t parity = reg & 0x7FF;
  return (static_cast<uint32_t>(message12 & 0xFFF) << 11) | parity;
}

// Build the 23-bit shift-register pattern for one DCS code. Layout
// follows the canonical W9CR/Motorola DCS frame:
//   shift bits  0..10 = parity, MSB-first   (parity bit 10 → shift bit 0)
//   shift bits 11..13 = sync "100"          ("1" → bit 11)
//   shift bits 14..22 = data, MSB-first     (data bit 8 → bit 14)
uint32_t dcs_code_to_pattern(int octal_code) {
  uint16_t data9     = dcs_code_to_bits9(octal_code);
  uint16_t message12 = static_cast<uint16_t>((0x4u << 9) | (data9 & 0x1FF));
  uint32_t parity    = golay_23_12_encode(message12) & 0x7FF;

  uint32_t pat = 0;
  for (int i = 0; i < 11; ++i) {
    if (parity & (1u << i)) pat |= (1u << (10 - i));
  }
  pat |= (1u << 11);
  for (int i = 0; i < 9; ++i) {
    if (data9 & (1u << i)) pat |= (1u << (22 - i));
  }
  return pat;
}

int popcount23(uint32_t v) {
  v &= 0x7FFFFFu;
#if defined(__GNUC__) || defined(__clang__)
  return __builtin_popcount(v);
#else
  int c = 0; while (v) { c += v & 1u; v >>= 1; } return c;
#endif
}

// Smallest 23-bit value across all rotations of `bits`. Used as a cyclic-
// equivalence key — two codeword patterns share a key iff one is a
// rotation of the other.
uint32_t smallest_rotation_23(uint32_t bits) {
  bits &= 0x7FFFFFu;
  uint32_t best = bits;
  uint32_t cur  = bits;
  for (int k = 1; k < 23; ++k) {
    cur = (((cur >> 1) | ((cur & 1u) << 22)) & 0x7FFFFFu);
    if (cur < best) best = cur;
  }
  return best;
}

// Per-sample EMA coefficient for a one-pole IIR with time constant tau_s.
double smoothing_alpha(double sample_rate, double tau_seconds) {
  if (tau_seconds <= 0.0) return 1.0;
  return 1.0 - std::exp(-1.0 / (sample_rate * tau_seconds));
}

} // anonymous namespace

dcs_squelch_ff::sptr dcs_squelch_ff::make(double sample_rate,
                                          int    configured_dcs_code,
                                          bool   configured_dcs_inverted,
                                          bool   gate_audio) {
  return gnuradio::get_initial_sptr(new dcs_squelch_ff(sample_rate,
                                                       configured_dcs_code,
                                                       configured_dcs_inverted,
                                                       gate_audio));
}

dcs_squelch_ff::dcs_squelch_ff(double sample_rate,
                               int    configured_dcs_code,
                               bool   configured_dcs_inverted,
                               bool   gate_audio)
    : sync_block("dcs_squelch_ff",
                 io_signature::make(1, 1, sizeof(float)),
                 io_signature::make(1, 1, sizeof(float))),
      d_sample_rate(sample_rate),
      d_configured_code(configured_dcs_code),
      d_configured_inverted(configured_dcs_inverted),
      d_gate_audio(gate_audio),
      d_detect_rate(1000.0),
      d_detect_decim(std::max(1, static_cast<int>(std::round(sample_rate / 1000.0)))),
      d_decim_count(0),
      d_lpf_state_idx(0),
      d_decim_step(DCS_BIT_RATE / 1000.0),
      d_configured_class_key(0),
      d_configured_match_count(0),
      d_first_decode_match_logged(false),
      d_locked_code(0),
      d_locked_inverted(false),
      d_best_code(0),
      d_best_inverted(false),
      d_best_matches(0),
      d_best_phases(0),
      d_post_lock_matches(0),
      d_samples_since_last_match(0),
      d_gate(0.0),
      d_gate_attack(smoothing_alpha(sample_rate, DCS_ATTACK_MS / 1000.0)),
      d_gate_decay(smoothing_alpha(sample_rate, DCS_DECAY_MS  / 1000.0)),
      d_open(false),
      d_samples_since_match(0),
      d_hold_samples(static_cast<long>(sample_rate * DCS_HOLD_SECONDS)) {
  build_dcs_table();
  build_lpf();

  // HPF runs at the detection rate (1 kHz), applied to the polyphase
  // LPF output. 30 Hz cutoff strips DC bias and slow drift; tighter
  // cutoffs (tried 80 Hz to kill mains hum) distorted the DCS NRZ
  // shape too much — phase diversity handles hum-induced jitter.
  d_hpf_a      = std::exp(-2.0 * M_PI * 30.0 / d_detect_rate);
  d_hpf_prev_x = 0.0;
  d_hpf_prev_y = 0.0;

  d_phases.resize(N_DCS_PHASES);
  reset();

  char cfg_buf[24];
  if (d_configured_code == 0) {
    snprintf(cfg_buf, sizeof(cfg_buf), "search-mode (any code)");
  } else {
    snprintf(cfg_buf, sizeof(cfg_buf), "D%03d%c",
             d_configured_code, d_configured_inverted ? 'I' : 'N');
  }
  BOOST_LOG_TRIVIAL(debug)
      << "dcs_squelch_ff: configured DCS = " << cfg_buf
      << " gate_audio=" << (d_gate_audio ? "true" : "false")
      << " cyclic_key=0x" << std::hex << d_configured_class_key << std::dec;
}

dcs_squelch_ff::~dcs_squelch_ff() = default;

void dcs_squelch_ff::build_dcs_table() {
  const int n = sizeof(DCS_CODES) / sizeof(DCS_CODES[0]);
  d_dcs_table.resize(n);
  for (int i = 0; i < n; ++i) {
    d_dcs_table[i].code    = DCS_CODES[i];
    d_dcs_table[i].pattern = dcs_code_to_pattern(DCS_CODES[i]);
  }
  // Pre-compute cyclic equivalence class keys.
  for (int i = 0; i < n; ++i) {
    const int code = DCS_CODES[i];
    const uint32_t pN = dcs_code_to_pattern(code);
    const uint32_t pI = (~pN) & 0x7FFFFFu;
    d_class_key[ {code, false} ] = smallest_rotation_23(pN);
    d_class_key[ {code, true } ] = smallest_rotation_23(pI);
  }
  // Single polarity-strict class key. Cyclic rotations at the configured
  // polarity are accepted (e.g. D047I when configured D023N because they
  // are the same physical bit sequence at different frame offsets).
  // The complementary polarity class is a distinct code set — intentionally
  // rejected. See d_configured_class_key in the header.
  if (d_configured_code != 0) {
    auto it = d_class_key.find( {d_configured_code, d_configured_inverted} );
    if (it != d_class_key.end()) d_configured_class_key = it->second;
  }
}

void dcs_squelch_ff::build_lpf() {
  // Hamming-windowed low-pass at ~250 Hz cutoff. Transition is 500 Hz so
  // the stopband starts at the d_detect_rate Nyquist — alias-free
  // decimation by d_detect_decim.
  const double fc       = 250.0 / d_sample_rate;
  const double trans_hz = 500.0;
  int n_taps = static_cast<int>(std::ceil(4.0 * d_sample_rate / trans_hz));
  if (n_taps < 31) n_taps = 31;
  if ((n_taps & 1) == 0) n_taps += 1;

  d_lpf_taps.assign(n_taps, 0.0);
  double sum = 0.0;
  for (int i = 0; i < n_taps; ++i) {
    int    k = i - n_taps / 2;
    double sinc = (k == 0) ? (2.0 * fc)
                            : (std::sin(2.0 * M_PI * fc * k) / (M_PI * k));
    double window = 0.54 - 0.46 * std::cos(2.0 * M_PI * i / (n_taps - 1));
    d_lpf_taps[i] = sinc * window;
    sum += d_lpf_taps[i];
  }
  if (sum != 0.0) {
    for (auto &t : d_lpf_taps) t /= sum;
  }
  d_lpf_state.assign(n_taps, 0.0);
}

void dcs_squelch_ff::reset() {
  boost::mutex::scoped_lock lock(d_mutex);
  std::fill(d_lpf_state.begin(), d_lpf_state.end(), 0.0);
  d_lpf_state_idx       = 0;
  d_decim_count         = 0;
  d_hpf_prev_x          = 0.0;
  d_hpf_prev_y          = 0.0;
  d_locked_code         = 0;
  d_locked_inverted     = false;
  d_best_code           = 0;
  d_best_inverted       = false;
  d_best_matches        = 0;
  d_best_phases         = 0;
  d_post_lock_matches           = 0;
  d_configured_match_count      = 0;
  d_first_decode_match_logged   = false;
  d_samples_since_last_match    = 0;
  d_open                = false;
  d_samples_since_match = 0;
  d_gate                = 0.0;
  for (int i = 0; i < static_cast<int>(d_phases.size()); ++i) {
    auto &p = d_phases[i];
    p.decim_phase       = static_cast<double>(i) / d_phases.size();
    p.bit_integ         = 0.0;
    p.bit_integ_n       = 0;
    p.shift             = 0;
    p.bits_seen         = 0;
    p.frame_anchor      = -1;
    p.frame_anchor_inv  = false;
    p.detected_code     = 0;
    p.detected_inverted = false;
    p.match_run         = 0;
  }
}

void dcs_squelch_ff::try_dcs_match(phase_state &p) {
  // Sync-aligned matching. DCS frame has fixed "100" sync at logical
  // positions 12-14 → shift bits 11..13 = (1, 0, 0). Inverted-polarity
  // codewords present "011" instead. Accept both.
  const uint32_t shift     = p.shift & 0x7FFFFFu;
  const uint32_t sync_bits = (shift >> 11) & 0x7u;
  bool sync_inverted;
  uint32_t cmp;
  if      (sync_bits == 0x1u) { sync_inverted = false; cmp = shift; }
  else if (sync_bits == 0x6u) { sync_inverted = true;  cmp = (~shift) & 0x7FFFFFu; }
  else return;

  // Frame-alignment gate: the codeword cycles every 23 bits and naive
  // "100"/"011" subpatterns appear at multiple positions in any cyclic
  // codeword. The real sync fires at the same (bits_seen % 23) every
  // codeword; cyclic-alias sync detections land at consistent but
  // different modular positions. Lock the first match's modular
  // position as the frame anchor; ignore sync detections at other
  // positions.
  const int frame_pos = static_cast<int>(p.bits_seen % 23);
  if (p.frame_anchor >= 0) {
    if (frame_pos != p.frame_anchor || sync_inverted != p.frame_anchor_inv) {
      return;
    }
  }

  int best_code = 0;
  int best_dist = 24;
  for (const auto &entry : d_dcs_table) {
    const int d = popcount23(cmp ^ entry.pattern);
    if (d < best_dist) { best_dist = d; best_code = entry.code; }
  }
  if (best_dist > DCS_DIST_THRESH || best_code == 0) return;

  // First valid match on this phase locks the frame anchor.
  if (p.frame_anchor < 0) {
    p.frame_anchor     = frame_pos;
    p.frame_anchor_inv = sync_inverted;
  }
  const bool best_inverted = sync_inverted;
  if (best_code == p.detected_code && best_inverted == p.detected_inverted) {
    ++p.match_run;
  } else {
    p.detected_code     = best_code;
    p.detected_inverted = best_inverted;
    p.match_run         = 1;
  }

  // Fast-lock for decode mode: any match within the acceptance threshold
  // (dist ≤ DCS_DIST_THRESH = 1) against the configured cyclic class
  // immediately elevates this phase to gate-open eligible without waiting
  // for a second consecutive frame.  In decode mode the code is already
  // known, so a single dist≤1 frame is strong evidence (~2.9×10⁻⁶
  // false-positive probability per frame for a random 23-bit window).
  // This guarantees gate-open within one frame period (~171 ms) — well
  // under the ETSI TS 103 236 §4.3 350 ms DCS decoder response limit.
  // Search mode uses the two-frame accumulator below (no fast-lock) so
  // the stricter DCS_RUN_REQUIRED_SEARCH guard still applies there.
  if (best_dist <= DCS_DIST_THRESH && d_configured_code != 0) {
    auto it = d_class_key.find( {best_code, best_inverted} );
    if (it != d_class_key.end() && it->second == d_configured_class_key) {
      p.match_run = DCS_RUN_REQUIRED;  // gate-open eligible this tick
    }
  }

  // Search mode: latch the first confirmed code so the gate can open and
  // post-lock re-confirmation tracking can begin.  Decode mode uses a
  // live phase scan in process_one() — no latch needed or used here.
  if (d_configured_code != 0) return;

  if (p.match_run >= DCS_RUN_REQUIRED_SEARCH && d_locked_code == 0) {
    BOOST_LOG_TRIVIAL(debug)
        << "dcs_squelch_ff[search]: first lock on D" << std::setw(3) << std::setfill('0')
        << p.detected_code << (p.detected_inverted ? "I" : "N")
        << " (sync=" << (sync_inverted ? "011/inverted" : "100/normal")
        << ", dist=" << best_dist
        << ", run=" << p.match_run
        << ")" << std::setfill(' ');

    // N-canonical disambiguation for search-mode display (matches
    // Uniden BCD scanner convention: always report the N-polarity member
    // of the cyclic class).
    int  out_code     = p.detected_code;
    bool out_inverted = p.detected_inverted;
    auto it = d_class_key.find( {p.detected_code, p.detected_inverted} );
    if (it != d_class_key.end()) {
      const uint32_t key = it->second;
      for (const auto &kv : d_class_key) {
        if (kv.second == key && !kv.first.second) {
          out_code     = kv.first.first;
          out_inverted = false;
          break;
        }
      }
    }
    d_locked_code     = out_code;
    d_locked_inverted = out_inverted;
  }
}

void dcs_squelch_ff::process_one(float sample, bool &gate_should_open) {
  gate_should_open = false;

  // LPF state ring (cheap per-sample work).
  const int n_taps = static_cast<int>(d_lpf_taps.size());
  d_lpf_state[d_lpf_state_idx] = static_cast<double>(sample);
  d_lpf_state_idx = (d_lpf_state_idx + 1) % n_taps;

  ++d_decim_count;
  if (d_decim_count < d_detect_decim) return;
  d_decim_count = 0;

  // ---- Stale-lock watchdog (search mode only) -------------------------
  // Drives the "abandon a hypothesis that stops being confirmed" behaviour
  // expected from a real scanner. Increments every detection-rate tick
  // while locked; if no re-confirmation arrives within
  // DCS_STALE_LOCK_SAMPLES (~500 ms), the lock is treated as stale and the
  // detector starts looking again. Limited to search mode because in
  // configured-mode the gate logic depends on d_locked_code and we don't
  // want a transient gap to close the audio gate mid-transmission.
  if (d_configured_code == 0 && d_locked_code != 0) {
    ++d_samples_since_last_match;
    if (d_samples_since_last_match > DCS_STALE_LOCK_SAMPLES) {
      // Before clearing, preserve the lock in d_best_* IF it had
      // accumulated enough re-matches to be trustworthy. This makes the
      // watchdog safe at end-of-transmission: a clean DCS keyup that
      // ended naturally still produces a verdict at end-of-call even
      // though no further matches are arriving. Fleeting false locks
      // (caught by the watchdog before they accumulated evidence) are
      // not preserved.
      int locked_phases = 0;
      for (const auto &p : d_phases) {
        if (p.frame_anchor >= 0) ++locked_phases;
      }
      const bool worth_saving = d_post_lock_matches >= DCS_BEST_MIN_MATCHES;
      const bool better_than_existing = d_post_lock_matches > d_best_matches;
      if (worth_saving && better_than_existing) {
        d_best_code     = d_locked_code;
        d_best_inverted = d_locked_inverted;
        d_best_matches  = d_post_lock_matches;
        d_best_phases   = locked_phases;
      }

      BOOST_LOG_TRIVIAL(debug)
          << "dcs_squelch_ff: stale lock cleared — was D"
          << std::setw(3) << std::setfill('0') << d_locked_code
          << (d_locked_inverted ? "I" : "N")
          << " with " << d_post_lock_matches << " re-matches, "
          << locked_phases << "/" << N_DCS_PHASES << " phases, "
          << "no new confirmation in " << d_samples_since_last_match
          << " ms; "
          << (worth_saving ? "preserved as best-of-call" : "discarded (below threshold)")
          << "; resetting all phase anchors to search again"
          << std::setfill(' ');
      d_locked_code              = 0;
      d_locked_inverted          = false;
      d_post_lock_matches        = 0;
      d_samples_since_last_match = 0;
      // Reset per-phase tracking but keep shift register / bits_seen —
      // we want the search to continue with fresh frame_anchor decisions
      // but not lose the bit-clock recovery state.
      for (auto &p : d_phases) {
        p.frame_anchor      = -1;
        p.frame_anchor_inv  = false;
        p.detected_code     = 0;
        p.detected_inverted = false;
        p.match_run         = 0;
      }
    }
  }

  // Anti-alias LPF: convolve taps with delay line (oldest first).
  int idx = (d_lpf_state_idx == 0) ? (n_taps - 1) : (d_lpf_state_idx - 1);
  double y_lpf = 0.0;
  for (int k = 0; k < n_taps; ++k) {
    y_lpf += d_lpf_taps[k] * d_lpf_state[idx];
    idx = (idx == 0) ? (n_taps - 1) : (idx - 1);
  }

  // HPF strips DC bias / slow drift.
  const double hpf_y = d_hpf_a * (d_hpf_prev_y + y_lpf - d_hpf_prev_x);
  d_hpf_prev_x = y_lpf;
  d_hpf_prev_y = hpf_y;

  // Distribute to every parallel bit-clock phase.
  bool cfg_match_this_tick = false;  // decode mode gate trigger
  for (auto &p : d_phases) {
    p.bit_integ   += hpf_y;
    p.bit_integ_n += 1;
    p.decim_phase += d_decim_step;
    if (p.decim_phase < 1.0) continue;
    p.decim_phase -= 1.0;

    if (p.bit_integ_n <= 0) {
      p.bit_integ = 0.0;
      continue;
    }
    const double avg = p.bit_integ / p.bit_integ_n;
    const uint32_t bit = (avg >= 0.0) ? 1u : 0u;
    p.shift       = ((p.shift << 1) | bit) & 0x7FFFFFu;
    p.bits_seen++;
    p.bit_integ   = 0.0;
    p.bit_integ_n = 0;

    if (p.bits_seen < 23) continue;

    const int  locked_before     = d_locked_code;
    const int  phase_run_before  = p.match_run;
    const int  phase_code_before = p.detected_code;
    const bool phase_inv_before  = p.detected_inverted;
    try_dcs_match(p);
    if (d_configured_code == 0) {
      // Search mode: track d_locked_code changes for confidence + watchdog.
      if (d_locked_code != 0 && d_locked_code != locked_before) {
        d_samples_since_last_match = 0;
      } else if (d_locked_code != 0
                 && p.match_run > phase_run_before
                 && p.detected_code == phase_code_before
                 && p.detected_inverted == phase_inv_before) {
        ++d_post_lock_matches;
        d_samples_since_last_match = 0;
      }
    } else {
      // Decode mode: check if this phase now confirms the configured class.
      if (p.match_run >= DCS_RUN_REQUIRED) {
        auto ck = d_class_key.find({p.detected_code, p.detected_inverted});
        if (ck != d_class_key.end() && ck->second == d_configured_class_key)
          cfg_match_this_tick = true;
      }
    }
  }

  if (cfg_match_this_tick) {
    ++d_configured_match_count;
    if (!d_first_decode_match_logged) {
      d_first_decode_match_logged = true;
      BOOST_LOG_TRIVIAL(debug)
          << "dcs_squelch_ff[decode]: first gate-open match for D"
          << std::setw(3) << std::setfill('0') << d_configured_code
          << (d_configured_inverted ? 'I' : 'N') << std::setfill(' ');
    }
  }

  // Gate decision.
  //   Decode mode: live phase scan — does any phase currently confirm a
  //     code in the configured cyclic class? No latch, no state from a
  //     prior tick; just: "is the configured code present right now?"
  //   Search mode: gate follows d_locked_code (any confirmed code).
  if (d_configured_code != 0) {
    if (cfg_match_this_tick) gate_should_open = true;
  } else {
    if (d_locked_code != 0) gate_should_open = true;
  }
}

int dcs_squelch_ff::work(int noutput_items,
                         gr_vector_const_void_star &input_items,
                         gr_vector_void_star &output_items) {
  const float *in  = static_cast<const float *>(input_items[0]);
  float       *out = static_cast<float *>(output_items[0]);


  for (int i = 0; i < noutput_items; ++i) {
    bool gate_should_open = false;
    process_one(in[i], gate_should_open);

    if (gate_should_open) {
      d_open                = true;
      d_samples_since_match = 0;
    } else if (d_open) {
      ++d_samples_since_match;
      if (d_samples_since_match >= d_hold_samples) {
        d_open = false;
      }
    }

    const double target = d_open ? 1.0 : 0.0;
    const double alpha  = d_open ? d_gate_attack : d_gate_decay;
    d_gate += alpha * (target - d_gate);

    // Apply the smoothed gate only when audio-gating is enabled. Detect-only
    // / search-mode side-chains pass input through unchanged while still
    // running the detector so get_verdict()/is_unmuted() report correctly.
    if (d_gate_audio) {
      out[i] = in[i] * static_cast<float>(d_gate);
    } else {
      out[i] = in[i];
    }
  }

  return noutput_items;
}

bool dcs_squelch_ff::is_unmuted() {
  return d_open.load(std::memory_order_acquire);
}

dcs_squelch_verdict dcs_squelch_ff::get_verdict() const {
  boost::mutex::scoped_lock lock(d_mutex);
  dcs_squelch_verdict v{};

  // Decode mode: the gate either opened (d_configured_match_count > 0) or
  // did not.  Confidence scales with how many detection ticks confirmed the
  // configured class, analogous to ctcss_squelch_ff's win_ticks / total ratio.
  if (d_configured_code != 0) {
    if (d_configured_match_count > 0) {
      v.detected_code     = d_configured_code;
      v.detected_inverted = d_configured_inverted;
      v.confidence        = std::min(1.0f,
          static_cast<float>(d_configured_match_count) /
          static_cast<float>(DCS_CONFIDENCE_FULL_MATCHES));
      auto it = d_class_key.find({d_configured_code, d_configured_inverted});
      if (it != d_class_key.end()) {
        const uint32_t key = it->second;
        for (const auto &kv : d_class_key) {
          if (kv.second == key) v.aliases.push_back(kv.first);
        }
      }
    }
    return v;
  }

  // Search mode: composite confidence from phase diversity + match rate.
  //
  //   phase_diversity = min(1, locked_phases / EFFECTIVE_PHASES)
  //     A real DCS NRZ transmission only engages ~half the 16 polyphase
  //     bit-clock trackers (the half whose decim_phase offset puts the
  //     integration window in the MIDDLE of each bit period; the other
  //     half straddle bit boundaries and never accumulate clean bits).
  //     So EFFECTIVE_PHASES = 8 — a signal that engages ≥8 phases scores
  //     1.0. CTCSS-aliased false locks typically only engage 1-3 phases
  //     so they score 0.1-0.4. This is the single strongest discriminator.
  //
  //   match_rate = min(1, d_post_lock_matches / DCS_CONFIDENCE_FULL_MATCHES)
  //     How many post-lock re-confirmations of the locked code have
  //     accumulated, saturating at ~1.7 s of clean reception. With the
  //     counter fix in process_one this only counts actual same-code
  //     frame re-matches, not per-tick increments.
  //
  // Composite: real DCS quickly reaches both factors ≈ 1.0 → conf ≈ 1.0.
  // CTCSS false: phase_diversity stays low (1-3/16 = 0.06-0.19) and
  // match_rate may also stay modest (the false pattern usually drifts
  // out before reaching 10 re-confirms) → conf typically < 0.15.
  // Pick the lock to report from. If we have an active lock, use it.
  // Otherwise fall back to the watchdog-preserved best-of-call (so a
  // transmission that ended cleanly still produces a verdict). If
  // neither exists, no DCS was found.
  int  reported_code;
  bool reported_inverted;
  long matches;
  int  phases;
  if (d_locked_code != 0) {
    reported_code     = d_locked_code;
    reported_inverted = d_locked_inverted;
    matches           = d_post_lock_matches;
    phases            = 0;
    for (const auto &p : d_phases) {
      if (p.frame_anchor >= 0) ++phases;
    }
  } else if (d_best_code != 0) {
    reported_code     = d_best_code;
    reported_inverted = d_best_inverted;
    matches           = d_best_matches;
    phases            = d_best_phases;
  } else {
    v.confidence = 0.0f;
    return v;
  }
  v.detected_code     = reported_code;
  v.detected_inverted = reported_inverted;
  // Phase diversity: how many of the 16 polyphase bit-clock trackers
  // independently locked their frame anchor on the same code. Empirically
  // only ~half the phases ever lock for a real DCS transmission — the
  // other half happen to sample near bit boundaries (where the integrator
  // gets noisy mid-transition bits) and never accumulate a confident
  // pattern. So divide by EFFECTIVE_PHASES rather than the full N_DCS_PHASES
  // — a real signal that engages 8-10 phases should read ~1.0, not 0.5.
  // CTCSS-aliased false locks typically engage 1-3 phases so they still
  // score low (0.1-0.4).
  static constexpr int EFFECTIVE_PHASES = 8;
  const float phase_diversity = std::min(1.0f,
      static_cast<float>(phases) / static_cast<float>(EFFECTIVE_PHASES));
  const float match_rate = std::min(1.0f,
      static_cast<float>(matches) /
      static_cast<float>(DCS_CONFIDENCE_FULL_MATCHES));
  v.confidence = phase_diversity * match_rate;
  auto it = d_class_key.find( {reported_code, reported_inverted} );
  if (it == d_class_key.end()) {
    v.aliases.emplace_back(d_locked_code, d_locked_inverted);
    return v;
  }
  const uint32_t key = it->second;
  for (const auto &kv : d_class_key) {
    if (kv.second == key) v.aliases.push_back(kv.first);
  }
  return v;
}

} /* namespace trunkrecorder */
} /* namespace gr */
