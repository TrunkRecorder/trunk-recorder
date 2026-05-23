/*
 * CTCSS detector + audio gate implementation.
 *
 * Two operating modes selected at construction time:
 *
 * DECODE (configured_pl_freq maps to a known CTCSS standard, d_configured_bin >= 0):
 *   Single-bin path. Only the configured CTCSS frequency and its two
 *   immediate table-neighbours are computed per tick. The neighbours
 *   provide a local spectral floor reference (not a pass/fail guard).
 *   Window: 150 ms (7 Hz BW — fine; target freq is known).
 *   Tick:   50 ms → first eval at ~150 ms → gate-open possible by ~200 ms.
 *   Verdict: win_ticks > 0 → return configured Hz + confidence; no guards.
 *
 * SEARCH (configured_pl_freq == 0):
 *   Full 50-bin path. Every CTCSS standard frequency is computed each tick.
 *   Adaptive noise floor = median of all 50 bin powers.
 *   Adjacent-bin guard rejects voice-fundamental smear.
 *   Window: 500 ms (2 Hz BW — needed to separate adjacent CTCSS standards).
 *   Tick:   100 ms → first eval at ~500 ms.
 *   Verdict: four-guard test (agree, enough_wins, win_dominant, energy_dominant).
 */

#include "ctcss_squelch_ff.h"

#include <gnuradio/io_signature.h>

#include <algorithm>
#include <cmath>
#include <numeric>

namespace gr {
namespace trunkrecorder {

namespace {

// Standard 50-tone EIA CTCSS frequency plan (Hz). Same as the table in
// tone_detector_block.cc — kept here as a self-contained copy so this
// block doesn't depend on tone_detector_block (we want them
// independently buildable / testable).
constexpr double CTCSS_FREQS[ctcss_squelch_ff::N_CTCSS] = {
    67.0,  69.3,  71.9,  74.4,  77.0,  79.7,  82.5,  85.4,  88.5,  91.5,
    94.8,  97.4, 100.0, 103.5, 107.2, 110.9, 114.8, 118.8, 123.0, 127.3,
   131.8, 136.5, 141.3, 146.2, 151.4, 156.7, 159.8, 162.2, 165.5, 167.9,
   171.3, 173.8, 177.3, 179.9, 183.5, 186.2, 189.9, 192.8, 196.6, 199.5,
   203.5, 206.5, 210.7, 218.1, 225.7, 229.1, 233.6, 241.8, 250.3, 254.1
};

// Decimation target: ~1 kHz so each CTCSS bin's Goertzel window is small
// and cheap. 1 kHz also guarantees the entire CTCSS band (67-254 Hz) is
// well within Nyquist for the LPF.
constexpr double DECIM_TARGET_RATE = 1000.0;

// Search mode: 500 ms window / 100 ms tick (2 Hz noise BW).
// Required to separate adjacent CTCSS standards (min spacing 2.3 Hz at
// the low end). Previous 200 ms / 5 Hz blurred voice fundamental into
// neighbouring bins and caused the winner to bounce.
constexpr double WINDOW_DURATION_SEARCH_S = 0.500;
constexpr double EVAL_TICK_SEARCH_S       = 0.100;

// Decode mode: 150 ms window / 50 ms tick (7 Hz noise BW).
// The target frequency is already known so fine bin separation is not
// needed. Shorter window means the first eval fires at ~150 ms, enabling
// gate-open on sub-500 ms transmissions that would previously never reach
// the first eval tick and return verdict=none.
constexpr double WINDOW_DURATION_DECODE_S = 0.150;
constexpr double EVAL_TICK_DECODE_S       = 0.050;

// Score-above-threshold bar for the cumulative tally. A bin must beat
// floor by this many dB to "win" a tick. Significantly above the open
// threshold so the verdict requires sustained dominance, not just brief
// edge cases.
constexpr float SCORING_THRESHOLD_DB = 6.0f;

// Defaults for the squelch hysteresis. Open is generous enough that
// fast TX peeling (radio keys mid-syllable) still locks; close is well
// below open to give clean hysteresis with no chatter.
constexpr float DEFAULT_OPEN_THRESH_DB  = 8.0f;
constexpr float DEFAULT_CLOSE_THRESH_DB = 4.0f;
constexpr int   DEFAULT_OPEN_HANG_MS    = 100;
constexpr int   DEFAULT_CLOSE_HANG_MS   = 150;  // TIA-603 reverse-burst spec

// Designs a small Hann-windowed low-pass FIR for the internal decimation
// stage. We keep this self-contained so the block has no external filter
// dependencies. cutoff_hz = ~300 Hz, transition ~150 Hz, ~32 taps gives
// >40 dB stopband — good enough to keep voice formants from aliasing
// into the sub-audible band.
std::vector<float> design_lpf(double rate, double cutoff_hz, int n_taps) {
  std::vector<float> taps(n_taps);
  const double fc_norm = cutoff_hz / rate;        // normalized cutoff (cycles/sample)
  const double M       = n_taps - 1;
  double sum = 0.0;
  for (int n = 0; n < n_taps; ++n) {
    const double k = n - M / 2.0;
    const double sinc = (k == 0.0) ? (2.0 * fc_norm)
                                   : (std::sin(2.0 * M_PI * fc_norm * k) / (M_PI * k));
    const double hann = 0.5 - 0.5 * std::cos(2.0 * M_PI * n / M);
    const double v    = sinc * hann;
    taps[n] = static_cast<float>(v);
    sum    += v;
  }
  // Normalize for unity DC gain
  for (auto &t : taps) t /= static_cast<float>(sum);
  return taps;
}

} // anonymous namespace

ctcss_squelch_ff::sptr ctcss_squelch_ff::make(double sample_rate, float configured_pl_freq, bool gate_audio) {
  return gnuradio::get_initial_sptr(new ctcss_squelch_ff(sample_rate, configured_pl_freq, gate_audio));
}

ctcss_squelch_ff::ctcss_squelch_ff(double sample_rate, float configured_pl_freq, bool gate_audio)
    : sync_block("ctcss_squelch_ff",
                 io_signature::make(1, 1, sizeof(float)),
                 io_signature::make(1, 1, sizeof(float))),
      d_sample_rate(sample_rate),
      d_configured_pl_freq(configured_pl_freq),
      d_configured_bin(-1),
      d_gate_audio(gate_audio),
      d_open_threshold_db(DEFAULT_OPEN_THRESH_DB),
      d_close_threshold_db(DEFAULT_CLOSE_THRESH_DB),
      d_open_hangtime_ticks(0),     // computed below from ms
      d_close_hangtime_ticks(0),
      d_open_streak(0),
      d_close_streak(0),
      d_unmuted(false) {

  // Decimation factor — round to nearest int. For 16 kHz input -> 16x = 1 kHz.
  d_decim_factor = std::max(1, static_cast<int>(std::round(d_sample_rate / DECIM_TARGET_RATE)));
  const double decim_rate = d_sample_rate / d_decim_factor;

  // Find the configured bin BEFORE timing selection so that is_decode
  // derives from d_configured_bin >= 0 rather than configured_pl_freq > 0.
  // The two conditions diverge if a non-standard PL frequency is passed:
  // > 0 would select decode timing while the code falls through to the
  // search path — wrong window width. CTCSS_FREQS is a compile-time
  // constant so we can scan it here without building d_bins first.
  if (configured_pl_freq > 0.0f) {
    for (int i = 0; i < N_CTCSS; ++i) {
      if (std::abs(CTCSS_FREQS[i] - configured_pl_freq) < 1.0) {
        d_configured_bin = i;
        break;
      }
    }
  }

  // Select window/tick timing based on mode.
  const bool   is_decode = (d_configured_bin >= 0);
  const double window_s  = is_decode ? WINDOW_DURATION_DECODE_S : WINDOW_DURATION_SEARCH_S;
  const double tick_s    = is_decode ? EVAL_TICK_DECODE_S       : EVAL_TICK_SEARCH_S;

  // Goertzel window in decimated samples.
  d_window_samples = std::max(32, static_cast<int>(std::round(window_s * decim_rate)));

  // Eval tick in decimated samples.
  d_eval_tick_samples = std::max(1, static_cast<int>(std::round(tick_s * decim_rate)));

  // Squelch hangtimes from ms → eval ticks.
  d_open_hangtime_ticks  = std::max(1, static_cast<int>(std::round(DEFAULT_OPEN_HANG_MS  / 1000.0 * decim_rate / d_eval_tick_samples)));
  d_close_hangtime_ticks = std::max(1, static_cast<int>(std::round(DEFAULT_CLOSE_HANG_MS / 1000.0 * decim_rate / d_eval_tick_samples)));

  // Pre-design the anti-alias LPF for the decimator. ~300 Hz cutoff, ~32 taps.
  d_decim_taps = design_lpf(d_sample_rate, 300.0, 32);
  d_decim_state.assign(d_decim_taps.size(), 0.0f);
  d_decim_state_idx = 0;
  d_decim_phase     = 0;

  // Pre-build the 50 CTCSS bin Goertzel coefficients at the decimated rate.
  d_bins.resize(N_CTCSS);
  for (int i = 0; i < N_CTCSS; ++i) {
    d_bins[i].freq       = CTCSS_FREQS[i];
    d_bins[i].coeff      = 2.0 * std::cos(2.0 * M_PI * CTCSS_FREQS[i] / decim_rate);
    d_bins[i].cum_score  = 0.0;
    d_bins[i].win_ticks  = 0;
    d_bins[i].energy_sum = 0.0;
  }

  // Pre-allocate the sliding-window ring buffer (decimated samples).
  d_window.assign(d_window_samples, 0.0f);
  d_window_write_idx  = 0;
  d_window_filled     = false;
  d_samples_since_eval = 0;
  d_total_eval_ticks   = 0;

  BOOST_LOG_TRIVIAL(debug)
      << "ctcss_squelch_ff[" << (is_decode ? "decode" : "search") << "]"
      << " configured_pl=" << d_configured_pl_freq << " Hz"
      << " (bin=" << d_configured_bin << ")"
      << " gate_audio=" << (d_gate_audio ? "true" : "false")
      << " decim=" << d_decim_factor << "x → " << decim_rate << " Hz"
      << " window=" << d_window_samples << " samples (" << window_s * 1000.0 << " ms)"
      << " eval-every=" << d_eval_tick_samples << " samples (" << tick_s * 1000.0 << " ms)"
      << " open=" << d_open_threshold_db << " dB / " << DEFAULT_OPEN_HANG_MS << " ms"
      << " close=" << d_close_threshold_db << " dB / " << DEFAULT_CLOSE_HANG_MS << " ms";
}

ctcss_squelch_ff::~ctcss_squelch_ff() = default;

void ctcss_squelch_ff::reset() {
  boost::mutex::scoped_lock lock(d_mutex);
  for (auto &b : d_bins) {
    b.cum_score  = 0.0;
    b.win_ticks  = 0;
    b.energy_sum = 0.0;
  }
  std::fill(d_window.begin(), d_window.end(), 0.0f);
  std::fill(d_decim_state.begin(), d_decim_state.end(), 0.0f);
  d_window_write_idx   = 0;
  d_window_filled      = false;
  d_decim_state_idx    = 0;
  d_decim_phase        = 0;
  d_samples_since_eval = 0;
  d_total_eval_ticks   = 0;
  d_open_streak        = 0;
  d_close_streak       = 0;
  d_unmuted            = false;
}

void ctcss_squelch_ff::set_open_threshold_db(float db) {
  d_open_threshold_db = db;
}

void ctcss_squelch_ff::set_close_threshold_db(float db) {
  d_close_threshold_db = db;
}

void ctcss_squelch_ff::set_open_hangtime_ms(int ms) {
  const double decim_rate = d_sample_rate / d_decim_factor;
  d_open_hangtime_ticks = std::max(1, static_cast<int>(std::round(ms / 1000.0 * decim_rate / d_eval_tick_samples)));
}

void ctcss_squelch_ff::set_close_hangtime_ms(int ms) {
  const double decim_rate = d_sample_rate / d_decim_factor;
  d_close_hangtime_ticks = std::max(1, static_cast<int>(std::round(ms / 1000.0 * decim_rate / d_eval_tick_samples)));
}

bool ctcss_squelch_ff::is_unmuted() const {
  return d_unmuted.load(std::memory_order_acquire);
}

ctcss_squelch_verdict ctcss_squelch_ff::get_verdict() const {
  boost::mutex::scoped_lock lock(d_mutex);
  ctcss_squelch_verdict v;
  v.detected_hz     = 0.0f;
  v.confidence      = 0.0f;
  v.dominant_snr_db = 0.0f;

  if (d_total_eval_ticks <= 0) return v;

  // --- DECODE MODE: no 50-bin scan needed. The gate opening is already the
  // detection confirmation; win_ticks > 0 means the configured tone was
  // observed above SCORING_THRESHOLD_DB on at least one eval tick. ---
  if (d_configured_bin >= 0) {
    const auto &cb = d_bins[d_configured_bin];
    if (cb.win_ticks > 0) {
      v.detected_hz     = static_cast<float>(CTCSS_FREQS[d_configured_bin]);
      v.confidence      = static_cast<float>(cb.win_ticks) / static_cast<float>(d_total_eval_ticks);
      v.dominant_snr_db = static_cast<float>(cb.cum_score / cb.win_ticks);
    }
    BOOST_LOG_TRIVIAL(debug)
        << "ctcss_squelch_ff[decode] verdict@"
        << (v.detected_hz > 0.0f ? std::to_string(v.detected_hz) : std::string("none"))
        << " dom_snr=" << v.dominant_snr_db << "dB"
        << " | total_ticks=" << d_total_eval_ticks
        << " win_ticks=" << cb.win_ticks
        << " conf=" << v.confidence;
    return v;
  }

  // --- SEARCH MODE: full 50-bin scan + verdict guards ---
  // Build top-three (by win_ticks then cum_score) for diagnostics regardless
  // of whether the strict guards below pass — useful when a call doesn't
  // produce a confident verdict but the operator wants to see what the
  // detector saw.
  std::vector<int> idx(N_CTCSS);
  std::iota(idx.begin(), idx.end(), 0);
  std::partial_sort(idx.begin(), idx.begin() + std::min(3, N_CTCSS), idx.end(),
                    [this](int a, int b) {
                      if (d_bins[a].win_ticks != d_bins[b].win_ticks) {
                        return d_bins[a].win_ticks > d_bins[b].win_ticks;
                      }
                      return d_bins[a].cum_score > d_bins[b].cum_score;
                    });
  for (int i = 0; i < std::min(3, N_CTCSS); ++i) {
    if (d_bins[idx[i]].win_ticks <= 0) break;
    const float mean_snr = static_cast<float>(d_bins[idx[i]].cum_score / d_bins[idx[i]].win_ticks);
    v.top_three.emplace_back(static_cast<float>(d_bins[idx[i]].freq), mean_snr);
  }

  // Strict verdict guards — ported from the old tone_detector_block. A
  // real sustained CTCSS sine wave wins both metrics (consistency AND
  // loudness) AND wins them clearly. Voice fundamentals in the CTCSS
  // band (male speech ~85-180 Hz lands across bins 7-17) typically win
  // only by win_ticks OR by energy_sum, not both. Without these guards
  // the verdict reports whichever bin voice fundamental happened to
  // sit in for that speaker on that call.
  //
  // Four guards (ALL must pass):
  //   1. AGREEMENT: win-count winner == energy winner (a real CTCSS
  //      sine wins both; broadband voice typically wins one or neither)
  //   2. CONSISTENCY: win-count winner has ≥70 % of all eval ticks (the
  //      tone was dominant for the bulk of the call)
  //   3. WIN-COUNT DOMINANCE: winner has ≥1.5× the runner-up's wins
  //      (a single bin clearly dominates, not a tie)
  //   4. ENERGY DOMINANCE: winner energy_sum > 1.5× runner-up AND > 5×
  //      median (loudness sanity-check vs both runner-up and noise floor)
  int    top_wins_i = -1;
  long   top_wins    = 0;
  long   second_wins = 0;
  for (int i = 0; i < N_CTCSS; ++i) {
    const long w = d_bins[i].win_ticks;
    if (w > top_wins) {
      second_wins = top_wins;
      top_wins    = w;
      top_wins_i  = i;
    } else if (w > second_wins) {
      second_wins = w;
    }
  }

  int    top_energy_i = -1;
  double top_energy    = 0.0;
  double second_energy = 0.0;
  for (int i = 0; i < N_CTCSS; ++i) {
    const double e = d_bins[i].energy_sum;
    if (e > top_energy) {
      second_energy = top_energy;
      top_energy    = e;
      top_energy_i  = i;
    } else if (e > second_energy) {
      second_energy = e;
    }
  }

  // Median energy across all 50 bins for the noise-floor sanity check.
  std::vector<double> sorted_e(N_CTCSS);
  double total_energy = 0.0;
  for (int i = 0; i < N_CTCSS; ++i) {
    sorted_e[i] = d_bins[i].energy_sum;
    total_energy += d_bins[i].energy_sum;
  }
  std::sort(sorted_e.begin(), sorted_e.end());
  const double median_energy = sorted_e[N_CTCSS / 2];

  // Spectral concentration: what fraction of total sub-audible band energy
  // is at the winning bin. Used as a minimum-sanity floor (against pure
  // broadband noise) rather than a tight discriminator. The math is
  // top_bin_energy / (50_bins_worth_of_noise_plus_signal), so a clean
  // CTCSS tone at moderate SNR (~15-20 dB) naturally lands around 10-20%
  // (because the denominator includes 49 noise-floor bins regardless of
  // signal strength). The original 0.30 threshold I added rejected real
  // short keyups; pure broadband noise sits at ~2% (1/50), so 0.10 is a
  // safe minimum.
  //
  // CTCSS false-positives on DCS audio used to be caught HERE (DCS audio
  // can produce ~22% concentration when the bit-rate fundamental near
  // 134.4 Hz contaminates the 136.5 Hz Goertzel bin), but they're now
  // properly resolved at the tiebreak: with phase-diversity-recalibrated
  // DCS confidence, real DCS reaches conf=1.0 and beats any CTCSS verdict
  // in analog_recorder::stop()'s search-mode max-conf comparison.
  const double concentration = (total_energy > 0.0 && top_energy_i >= 0)
                                   ? d_bins[top_energy_i].energy_sum / total_energy
                                   : 0.0;
  static constexpr double MIN_CONCENTRATION_SEARCH = 0.10;
  const bool concentrated = (d_configured_bin >= 0)   // decode mode: skip the check
                                ? true
                                : (concentration >= MIN_CONCENTRATION_SEARCH);

  // Verdict guards. The per-tick adjacent-bin guards (and notch-ratio in
  // decode mode) already filter voice fundamentals out at the tick
  // level — only HIGH-CONFIDENCE ticks ever increment win_ticks. So the
  // verdict's job is no longer to second-guess questionable wins; it's
  // to confirm there are enough confirmed wins AND that they're not a
  // statistical fluke.
  //
  // Replaced the prior percentage-consistency rule with an absolute
  // minimum win count: ≥5 qualified ticks = ≥500 ms of confirmed CTCSS
  // presence. Bench evidence: under strict per-tick guards a real 5-12s
  // call yields 25-122 qualified wins; voice-only calls yield 0-3.
  // Setting the bar at 5 dispatches voice-only false positives while
  // accepting any call with a half-second of clean tone — which is
  // what TIA-603-E paging timing assumes anyway.
  static constexpr long MIN_QUALIFIED_WINS = 5;
  const bool agree           = (top_wins_i >= 0 && top_wins_i == top_energy_i);
  const bool enough_wins     = (top_wins >= MIN_QUALIFIED_WINS);              // ≥500ms qualified
  const bool win_dominant    = (top_wins >= second_wins * 3 / 2);             // ≥1.5×
  const bool energy_dominant = (top_energy_i >= 0 &&
                                top_energy > second_energy * 1.5 &&
                                top_energy > median_energy * 5.0);             // ≥5×
  if (agree && enough_wins && win_dominant && energy_dominant && concentrated) {
    v.detected_hz     = static_cast<float>(d_bins[top_wins_i].freq);
    v.confidence      = static_cast<float>(top_wins) / static_cast<float>(d_total_eval_ticks);
    v.dominant_snr_db = static_cast<float>(d_bins[top_wins_i].cum_score / d_bins[top_wins_i].win_ticks);
  }

  // Diagnostic — log the guard outcomes + the top wins/energy bins so we
  // can see WHY a call did or didn't produce a verdict. Logged
  // unconditionally for now; should be tunable / removed once tuning is
  // settled.
  const double w_hz = (top_wins_i   >= 0) ? d_bins[top_wins_i].freq   : 0.0;
  const double e_hz = (top_energy_i >= 0) ? d_bins[top_energy_i].freq : 0.0;
  BOOST_LOG_TRIVIAL(debug)
      << "ctcss_squelch_ff[search] verdict@"
      << (v.detected_hz > 0 ? std::to_string(v.detected_hz) : std::string("none"))
      << " dom_snr=" << v.dominant_snr_db << "dB"
      << " | total_ticks=" << d_total_eval_ticks
      << " win_winner=" << w_hz << "Hz (" << top_wins << " wins, 2nd=" << second_wins << ")"
      << " energy_winner=" << e_hz << "Hz"
      << " guards: agree=" << agree
      << " enough_wins(>=5)=" << enough_wins << " (" << top_wins << "/" << d_total_eval_ticks << ")"
      << " win_dom(>=1.5x)=" << win_dominant
      << " energy_dom(>=5x)=" << energy_dominant
      << " concentrated(>=0.10)=" << concentrated
      << " (e_top=" << top_energy << " e_2nd=" << second_energy
      << " e_med=" << median_energy << " concentration=" << concentration << ")"
      << " top3:["
      << [&]() {
           std::string s;
           for (size_t i = 0; i < v.top_three.size(); ++i) {
             if (i) s += ", ";
             s += std::to_string(v.top_three[i].first) + "Hz/"
                  + std::to_string(v.top_three[i].second) + "dB";
           }
           return s;
         }()
      << "]";
  return v;
}

void ctcss_squelch_ff::evaluate_goertzels() {
  // Don't evaluate until the window has been filled at least once.
  if (!d_window_filled) return;

  // --- VERIFY MODE: configured frequency known — 3-bin path ---
  // Only the configured bin and its two immediate CTCSS-table neighbours
  // are computed.
  if (d_configured_bin >= 0) {
    auto goertzel = [&](int i) -> double {
      const double C = d_bins[i].coeff;
      double s1 = 0.0, s2 = 0.0;
      int ri = d_window_write_idx;
      for (int n = 0; n < d_window_samples; ++n) {
        const double tmp = d_window[ri] + C * s1 - s2;
        s2 = s1;
        s1 = tmp;
        if (++ri >= d_window_samples) ri = 0;
      }
      const double e = s1 * s1 + s2 * s2 - C * s1 * s2;
      return std::max(e, 1e-12);
    };

    const double cfg_power = goertzel(d_configured_bin);

    double floor_sum = 0.0;
    int    floor_n   = 0;
    if (d_configured_bin > 0)         { floor_sum += goertzel(d_configured_bin - 1); ++floor_n; }
    if (d_configured_bin < N_CTCSS-1) { floor_sum += goertzel(d_configured_bin + 1); ++floor_n; }
    const double floor_lin = std::max(floor_n > 0 ? floor_sum / floor_n : cfg_power, 1e-12);

    // Silence gate: skip tick if total power is near-zero (upstream squelch ramp).
    if (cfg_power + floor_sum < 1e-4) return;

    const double snr_lin = cfg_power / floor_lin;
    const float  snr_db  = static_cast<float>(10.0 * std::log10(std::max(snr_lin, 1e-12)));

    if (snr_db >= SCORING_THRESHOLD_DB) {
      d_bins[d_configured_bin].cum_score += snr_db;
      d_bins[d_configured_bin].win_ticks += 1;
    }
    d_total_eval_ticks += 1;
    update_squelch_state(d_configured_bin, snr_db);
    return;
  }

  // --- SEARCH MODE: full 50-bin path ---
  std::vector<double> powers(N_CTCSS, 0.0);

  for (int i = 0; i < N_CTCSS; ++i) {
    const double C = d_bins[i].coeff;
    double s1 = 0.0, s2 = 0.0;
    int read_idx = d_window_write_idx; // oldest sample position in the ring
    for (int n = 0; n < d_window_samples; ++n) {
      const double s = d_window[read_idx] + C * s1 - s2;
      s2 = s1;
      s1 = s;
      if (++read_idx >= d_window_samples) read_idx = 0;
    }
    double e = s1 * s1 + s2 * s2 - C * s1 * s2;
    if (e < 1e-12) e = 1e-12;
    powers[i] = e;
  }

  // "Real audio present" gate. Skip this tick entirely if the total
  // Goertzel power across all 50 bins is below detection floor — this
  // catches the ramp-up / ramp-down transients from pwr_squelch_cc's
  // smooth fade (alpha=0.01 IIR + ramp=10), plus any other periods
  // where the upstream squelch is partially muting. Without this gate,
  // those ticks pick a bogus "winner" from near-zero noise and inflate
  // both win_ticks and energy_sum for bins that didn't actually see a
  // real tone. The threshold is well below any real voice or CTCSS
  // signal (a 0.1-amplitude CTCSS sine over a 200-sample window scores
  // ~100 per bin, so total across 50 bins is in the thousands; pure
  // zero input scores ~5e-11 total). 1e-3 is generous headroom for
  // weak signals while still rejecting silence.
  double total_power = 0.0;
  for (double p : powers) total_power += p;
  if (total_power < 1e-3) {
    // Don't increment d_total_eval_ticks either — the consistency /
    // confidence ratios should be over ACTIVE-AUDIO ticks only.
    return;
  }

  // Cumulative loudness (amplitude domain) for the verdict guards.
  // Updated only on active-audio ticks so silence doesn't pollute the
  // running totals.
  for (int i = 0; i < N_CTCSS; ++i) {
    d_bins[i].energy_sum += std::sqrt(powers[i]);
  }

  // Adaptive noise floor: median of all 50 bin powers.
  std::vector<double> sorted_powers = powers;
  std::sort(sorted_powers.begin(), sorted_powers.end());
  const double median = sorted_powers[N_CTCSS / 2];
  const double floor_lin = std::max(median, 1e-12);

  // Compute SNR per bin and find the winner.
  //
  // Adjacent-bin guard (from GNU Radio ctcss_squelch_ff_impl.cc style, and
  // the spectral-peak literature): a real CTCSS sinusoid is a knife-edge
  // peak — its bin towers above both immediate CTCSS neighbors. Voice
  // fundamentals smear across multiple adjacent bins (e.g. a male speaker
  // sitting at ~105 Hz raises 100.0/103.5/107.2 comparably). Requiring
  // power[i] > power[i-1] AND power[i] > power[i+1] disqualifies smeared
  // peaks at near-zero cost. This is the per-tick voice-fundamental
  // rejection that converts the median-floor metric from "report
  // whichever bin voice happens to peak in" into "report bins that look
  // like actual sinusoids."
  int    winner_idx = -1;
  double winner_snr_lin = 1.0; // any bin must beat unity (0 dB) to be "winner"
  for (int i = 0; i < N_CTCSS; ++i) {
    const bool left_ok  = (i == 0)              || powers[i] > powers[i - 1];
    const bool right_ok = (i == N_CTCSS - 1)    || powers[i] > powers[i + 1];
    if (!left_ok || !right_ok) continue;
    const double snr_lin = powers[i] / floor_lin;
    if (snr_lin > winner_snr_lin) {
      winner_snr_lin = snr_lin;
      winner_idx     = i;
    }
  }
  const float winner_snr_db = (winner_idx >= 0)
      ? static_cast<float>(10.0 * std::log10(winner_snr_lin))
      : 0.0f;

  // NOTE: A notched-reference path (OpenAudio analyze_CTCSS_F32 style) was
  // tried here and removed. In OpenAudio it works because the input is
  // the full 0-3000 Hz audio band where voice has rich harmonic content
  // BUT CTCSS sits in a relatively quiet sub-audible zone — the 2-Hz
  // notch then removes most of the "tone" energy while voice harmonics
  // (300-3000 Hz) dominate the reference. Our upstream chain LPFs to
  // 300 Hz before this block, so the notch's input is already
  // voice-fundamental-dominated (80-300 Hz). The notch removes ~0.1% of
  // the band energy, so target/notched ≈ CTCSS_pwr / voice_pwr ≈ 0.1
  // even for clean CTCSS — it rejects every legitimate detection that
  // happens during voice (validated by raw-wav ground-truth: 20/21
  // rejected calls had configured tone at rank-1 with 18-35 dB SNR).
  // Per-tick voice rejection is therefore left to the adjacent-bin
  // guards above; the verdict's MIN_QUALIFIED_WINS rule + agreement +
  // dominance guards catch the remaining false positives.

  // Cumulative scoring (identification path). Add the winner's SNR-dB to
  // its score whenever it beats SCORING_THRESHOLD_DB. The adjacent-bin
  // guards inside the winner loop already filter out voice-fundamental
  // smearing (a real CTCSS sine is a knife-edge peak; voice smears
  // across multiple bins and never wins the guards), so any winner we
  // see here is a high-confidence per-tick observation.
  if (winner_idx >= 0 && winner_snr_db >= SCORING_THRESHOLD_DB) {
    d_bins[winner_idx].cum_score += winner_snr_db;
    d_bins[winner_idx].win_ticks += 1;
  }
  d_total_eval_ticks += 1;

  // Squelch state update (gating path).
  update_squelch_state(winner_idx, winner_snr_db);
}

void ctcss_squelch_ff::update_squelch_state(int winner_idx, float winner_snr_db) {
  // For configured channels: gate opens when the configured bin is the
  // SNR-winner among the adjacent-bin-guarded set. Voice fundamentals
  // smear across multiple bins and never beat both ±1 neighbors, so
  // them being "the winner" is already strongly improbable.
  // For free-scan: gate on whatever winner survived the adjacent guards.
  float gating_snr_db = 0.0f;
  if (d_configured_bin >= 0) {
    if (winner_idx == d_configured_bin) {
      gating_snr_db = winner_snr_db;
    } else {
      gating_snr_db = 0.0f;  // configured tone isn't the winner
    }
  } else {
    // Free-scan: any winner above threshold opens.
    gating_snr_db = winner_snr_db;
  }

  if (gating_snr_db >= d_open_threshold_db) {
    d_open_streak  += 1;
    d_close_streak  = 0;
    if (!d_unmuted && d_open_streak >= d_open_hangtime_ticks) {
      d_unmuted = true;
    }
  } else if (gating_snr_db < d_close_threshold_db) {
    d_close_streak += 1;
    d_open_streak   = 0;
    if (d_unmuted && d_close_streak >= d_close_hangtime_ticks) {
      d_unmuted = false;
    }
  } else {
    // Between open and close thresholds → hysteresis dead-zone, keep current state.
    d_open_streak  = 0;
    d_close_streak = 0;
  }
}

int ctcss_squelch_ff::work(int noutput_items,
                            gr_vector_const_void_star &input_items,
                            gr_vector_void_star &output_items) {
  const float *in  = static_cast<const float *>(input_items[0]);
  float       *out = static_cast<float *>(output_items[0]);


  for (int i = 0; i < noutput_items; ++i) {
    // Step 1: feed the decimating LPF.
    d_decim_state[d_decim_state_idx] = in[i];
    if (++d_decim_state_idx >= static_cast<int>(d_decim_state.size())) {
      d_decim_state_idx = 0;
    }
    if (++d_decim_phase >= d_decim_factor) {
      d_decim_phase = 0;
      // Compute one decimated sample by convolving taps with delay line.
      float dec = 0.0f;
      int idx = d_decim_state_idx; // oldest sample position
      for (size_t t = 0; t < d_decim_taps.size(); ++t) {
        dec += d_decim_taps[t] * d_decim_state[idx];
        if (++idx >= static_cast<int>(d_decim_state.size())) idx = 0;
      }
      // Step 2: push into Goertzel sliding window.
      d_window[d_window_write_idx] = dec;
      if (++d_window_write_idx >= d_window_samples) {
        d_window_write_idx = 0;
        d_window_filled    = true;
      }
      // Step 3: every eval_tick_samples decimated samples, evaluate.
      if (++d_samples_since_eval >= d_eval_tick_samples) {
        d_samples_since_eval = 0;
        evaluate_goertzels();
      }
    }
    // Step 4: gate output. In gating mode pass input when unmuted, zero
    // otherwise. In detect-only mode (d_gate_audio==false) always pass
    // through — the squelch decision is still tracked internally for
    // is_unmuted() / get_verdict() callers, but the audio path is left
    // alone (used when this block is a side-chain detector or when
    // search mode wants audio passing without gating).
    if (d_gate_audio) {
      out[i] = d_unmuted ? in[i] : 0.0f;
    } else {
      out[i] = in[i];
    }
  }

  return noutput_items;
}

} // namespace trunkrecorder
} // namespace gr
