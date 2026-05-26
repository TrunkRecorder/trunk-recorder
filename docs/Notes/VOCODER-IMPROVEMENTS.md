# Vocoder Improvements

This branch (`dev/vocoder-improvements`) layers a series of decoder-side quality
and robustness fixes on top of the IMBE vocoder code vendored in
`lib/op25_repeater/`. Changes are listed individually below, each with the
problem it addresses, the technique used (with patent reference where
applicable; all referenced patents are expired and public-domain), the code
location, and the tuning knobs (if any).

For the best audio quality, set `"softVocoder": true` in `config.json` — the
float decoder (`software_imbe_decoder`) is where most of the quality work
lives. The fixed-point decoder (`imbe_vocoder`) gets the robustness/error-
concealment fixes but does not get the synthesis-quality improvements.

All audio-quality tuning knobs are consolidated in an `AUDIO TUNING
PARAMETERS` block at the top of
[software_imbe_decoder.cc](../../lib/op25_repeater/lib/software_imbe_decoder.cc).
Each knob has an inline comment describing its range and audible effect.
Defaults reflect what sounded best on the WMATA (P25 Phase 1) test recording.

---

## 1. Robust error concealment on the fixed-point IMBE path

**Problem.** The fixed-point `imbe_vocoder` had no internal mute/repeat policy
(only `software_imbe_decoder` did). Under LSM simulcast or any lossy RF, bit
errors caused screeching/loud-noise frames instead of clean silence.

A first attempt at gating used a PCM-level `memcpy` of the previous 20 ms
buffer to "repeat" frames. That stamps the same waveform every 20 ms — a 50 Hz
comb on top of the harmonics, audible as a kazoo / wax-paper-over-comb sound.

**Fix.** TIA-102.BABA-A §7.7–7.8 mute-and-repeat policy applied to the
fixed-point path:

- Smoothed ER = 0.95 × prev + 0.000365 × ET.
- If smoothed ER > 0.0875 → mute (zero PCM output).
- Else if `b0 > 207 || E0 ≥ 2 || ET ≥ 10 + 40·ER` → mark frame as repeat.
- After 4 consecutive repeats → mute.

On a repeat, **re-run the vocoder with the previous good `frame_vector`** so
synthesis re-derives output with naturally-advanced internal state (pitch,
spectral envelope, phase). Phase continuity is preserved — no comb.

Per-call state (`d_imbe_er`, `d_imbe_rpt_ctr`, `d_imbe_last_vec`) is reset in
`clear()` so a call ending in high ER doesn't carry that into the next call.

**Code.**
- [`p25p1_fdma.cc`](../../lib/op25_repeater/lib/p25p1_fdma.cc) (P25 trunked, primary path used by trunk-recorder)
- [`p25p1_voice_decode.cc`](../../lib/op25_repeater/lib/p25p1_voice_decode.cc) (legacy P25 voice decode)
- [`rx_sync.cc`](../../lib/op25_repeater/lib/rx_sync.cc) (YSF fullrate)

**Knobs.** None. TIA spec values used directly.

---

## 2. Crash prevention from bit errors

**Problem.** `mbelib.c` and `software_imbe_decoder::rearrange()` called
`exit()` when bit errors produced out-of-spec IMBE parameters (e.g., `L < 9`
or `L > 56`). A single bad frame could kill the trunk-recorder process.

**Fix.** Clamp out-of-range values to the valid range. The frame's audio will
be wrong but the caller's repeat/mute logic absorbs it.

**Code.**
- [`mbelib.c`](../../lib/op25_repeater/lib/mbelib.c)
- [`software_imbe_decoder.cc`](../../lib/op25_repeater/lib/software_imbe_decoder.cc) (`rearrange()`)

**Knobs.** None.

---

## 3. Long-period unvoiced excitation noise

**Problem.** The original noise generator was an LCG
`next_u(u) = (171·u + 11213) mod 53125`. Period exactly 53125 samples
(Hull-Dobell conditions all hold) ≈ 6.6 s of continuous unvoiced speech
before repetition. Audible as "repeating noise" on long sibilants /
fricatives.

**Fix.** Replaced with `xorshift32` maintaining a full 32-bit state
(`unvoiced_noise_state` member); the mod-53125 output is computed each
call without overwriting the state. Period is now genuinely 2³²−1.

An earlier version of this fix did `xorshift32(u) mod 53125` with the
*folded* output passed back as next state, which capped the effective
state space at 53125 again with no guarantee of full period. Worth
calling out so the same trap doesn't get reintroduced.

**Code.** [`software_imbe_decoder.cc`](../../lib/op25_repeater/lib/software_imbe_decoder.cc) (`next_u()`)

**Knobs.** None.

---

## 4. Voiced phase regeneration (US5701390 — expired Feb 2015)

**Problem.** The TIA spec voiced synthesis sets `phi[l] = psi1·l` for low
harmonics — linear/zero phase, all harmonics align at glottal-pulse instants,
sounds **buzzy**. Pre-branch master had this exclusively. An attempt to add
the TIA eq. 142 random-phase term used a static lookup table reused every
frame — same offsets at 50 Hz cadence = **kazoo / comb-filter sound**.

**Fix.** Implements DVSI's hardware approach from
[US5701390](https://patents.google.com/patent/US5701390). For each voiced
harmonic `l`:

```
phi_env(l) = c_env · Σ_{m odd, 1..D}  (2/(π·m)) · (B[l+m] − B[l−m])
B = log₂(M) − mean(log₂ M[1..L])   (DC removed)
```

This is a discrete Hilbert transform of log-magnitude across harmonics, using
the standard `2/(π·n)` kernel weights (odd n only — the zero-mean Type III
kernel). The Hilbert transform of log-magnitude *is* the phase of the
minimum-phase system with that magnitude — and the vocal tract is
approximately minimum-phase. So the resulting phase correlates with formant
shape: naturally non-aligning, no comb, no buzz.

Boundary handling per patent: `B[0]=0`, `B[−l]=B[l]` (reflection),
`B[L+k]=B[L]·γᵏ` (geometric decay outside valid harmonic range). DC is
subtracted from B before the convolution so the boundary extensions don't
leak DC into the phase output (the true Hilbert transform has no DC
response; this restores that property under truncation).

The phase computation runs in `compute_envelope_phases()`, called by
`decode_fullrate` **before** `apply_formant_postfilter`. That keeps the
two improvements orthogonal — phase regen reads the pristine spectral
envelope, the postfilter then reshapes M only for the synthesizer to use
in its amplitude lookup.

The linear-phase term `psi1·l` is retained at reduced weight on low harmonics
(`l ≤ L/4`) to preserve some glottal-pulse character. Optional residual TIA
random phase can be re-enabled via a knob.

**Code.** [`software_imbe_decoder.cc:compute_envelope_phases()`](../../lib/op25_repeater/lib/software_imbe_decoder.cc)

**Knobs.**

| Knob | Default | Effect |
|---|---|---|
| `PHASE_C_ENV` | 0.75 | Envelope-phase scaling. 0 disables → falls back to linear phase. ~2/π is true Hilbert. >1 sounds echoey. |
| `PHASE_W_RAND` | 0.0 | Residual TIA-eq.142 random-phase weight (multiplies Luv/L · z(l)). 0 = deterministic. >0 brings back some buzz. |
| `PHASE_LOW_BLEND` | 0.40 | Envelope-phase weight on low harmonics (`l ≤ L/4`). 0 = pure linear (glottal/buzzy). 1.0 = no glottal alignment (reedy). |
| `PHASE_KERNEL_D` | 19 | 1/m kernel half-length (full kernel = 2D+1 taps). Patent's preferred. |
| `PHASE_KERNEL_GAMMA` | 0.72 | Boundary decay for `B[l]` extrapolation. Patent value. Rarely needs change. |

---

## 5. Adaptive formant postfilter (US5241650 — expired ~2009)

**Problem.** Even with correct phase, IMBE-decoded audio sounds **synthetic /
hollow**. Commercial parametric vocoders run a postfilter at the decoder
output that emphasizes formant peaks and attenuates inter-formant valleys —
this is most of the perceptual difference between "synthetic" and "natural"
low-bitrate speech. OP25 had no postfilter.

**Fix.** Magnitude-domain spectral-contrast enhancement (functionally
similar to the patent's bandwidth-expanded LPC postfilter `H(z)=B(z)/A(z/ν)`
but a natural fit for MBE since harmonics already sample the envelope):

```
M'[l] = M[l] · 2^(α · (log₂ M[l] − log₂ M_smooth[l]))     iff vee[l] = 1
M'[l] = M[l]                                              if  vee[l] = 0
```

`M_smooth` is a (2W+1)-tap centered moving average over harmonics with
symmetric reflection at the band edges, so harmonics at `l=1` and `l=L`
see real neighbors rather than clamped copies of themselves (which used
to over-emphasize them). The contrast emphasis is gated on the per-band
voicing flag — unvoiced bands carry noise where there's no formant peak
to sharpen; the patent's LPC postfilter would be broadband but a
magnitude-contrast version should be voiced-only. Total energy is
renormalized across all bands (voiced modifications + unchanged unvoiced)
to preserve loudness — the patent's specific innovation, eliminating the
"time-varying brightness modulation" of naive postfilters.

**Code.** [`software_imbe_decoder.cc:apply_formant_postfilter()`](../../lib/op25_repeater/lib/software_imbe_decoder.cc)

**Knobs.**

| Knob | Default | Effect |
|---|---|---|
| `FMT_ALPHA` | 0.25 | Emphasis strength. 0 = off, 0.15 = very mild, 0.25 = mild (rec), 0.35 = noticeable, 0.5+ = tinny/sibilant. |
| `FMT_W` | 3 | Smoothing half-window (window = 2W+1 = 7-tap, ≈1 formant wide). Smaller → only narrow peaks boosted. Larger → less per-peak emphasis. |

---

## 6. Voicing-decision smoothing (US6912496 — expired Mar 2023)

**Problem.** IMBE voicing decisions `vee[l]` are per-band-per-frame. Under
marginal RF, individual bands can flip V/UV/V on successive frames. Audible as
clicks or warble at phoneme boundaries.

**Fix.** N-tap majority median filter on `vee[l][New]` using current +
(N−1) past frames, **gated on smoothed ER** so it only fires on noisy
audio. Patent specifies smoothing as a corrective step "around erroneously
coded frames" — applying it to clean audio costs latency on legitimate
fast V/UV transitions for no benefit. The ER gate matches that intent.
Original (pre-smoothing) values are stored in `vee_history[]` every frame
regardless, so when smoothing kicks in it has recent context.

**Code.** [`software_imbe_decoder.cc:smooth_voicing_decisions()`](../../lib/op25_repeater/lib/software_imbe_decoder.cc)

**Knobs.**

| Knob | Default | Effect |
|---|---|---|
| `voicing_smooth_taps` | 3 | Total filter length. 1 = off, 3 = drop single-frame outliers (rec), 5 = smoother but voicing-state changes lag 2 frames. |
| `voicing_smooth_er_threshold` | 0.01 | Minimum smoothed ER for smoothing to fire. 0.0 = always-on legacy behavior; raise to keep clean audio snappy. |

---

## 7. UV→V phase reset (US6963833 — expired Mar 2022)

**Problem.** When a voiced segment starts after silence or a fricative, the
running `psi1` phase accumulator carries whatever value it had when the
previous voiced segment ended (it was advancing through the unvoiced interval
too). That stale phase can land harmonics in alignment at the onset frame,
producing a click/pop.

**Fix.** Detect fully-unvoiced → voiced transitions (no `vee[l][Old]` set,
some `vee[l][New]` set). At the transition, reset `psi1` to 0 and
**pre-seed `phi[l][Old]` from a 56-entry table of deliberately misaligned
per-harmonic offsets** drawn from a fixed permutation of
`{0, π/3, 2π/3, π, 4π/3, 5π/3}`. That's the patent's "fixed set of values
for each harmonic" that guarantees harmonics don't align coherently at
sample 0 regardless of whether the envelope-phase term happens to be
small (smooth spectrum onset). Without the table, simply resetting
`psi1=0` left a hole where flat-spectrum onsets had all phases at zero —
exactly the alignment-and-saturation case the patent is defending
against.

**Code.** [`software_imbe_decoder.cc:compute_envelope_phases()`](../../lib/op25_repeater/lib/software_imbe_decoder.cc) (UV→V block + `uv_to_v_phase_table`)

**Knobs.**

| Knob | Default | Effect |
|---|---|---|
| `UV_TO_V_RESET` | true | Enable the reset. `false` keeps prior `psi1` across unvoiced intervals (legacy behavior). |

---

## 8. Widened amplitude/phase interpolation (US6131084 — expired ~2017)

**Problem.** `synth_voiced()` has two synthesis paths per voiced harmonic:

- **Fine transition.** Quadratic phase + linear amplitude interpolation
  across the 160-sample frame. Smoothest, sounds natural.
- **Coarse transition.** Windowed overlap-add of previous and current frame
  parameters. Sounds blockier on sustained vowels.

The TIA spec gates fine transition by `ell < 8 && |dw0|/w0 < 0.1`. The
patent describes interpolating amp/freq/phase to match adjacent segments for
more of the spectrum.

**Fix.** Loosen the gates — see knobs. Extends the smoother path higher in
the spectrum and through normal pitch wobble.

**Code.** [`software_imbe_decoder.cc:synth_voiced()`](../../lib/op25_repeater/lib/software_imbe_decoder.cc) (`ell < INTERP_MAX_L && ...` branch)

**Knobs.**

| Knob | Default | Effect |
|---|---|---|
| `INTERP_MAX_L` | 12 | Max harmonic for fine transition. 8 = TIA spec, 12 = recommended, 16 = smoothest (may smear consonants). |
| `INTERP_PITCH_TOL` | 0.15 | Max `|w0−Oldw0|/w0` for fine transition. 0.10 = TIA spec, 0.15 = catches normal pitch wobble, 0.20 = includes vibrato (risks smearing real jumps). |

---

## Combined synthesis pipeline (float decoder)

For reference, the order in which the above run for each 20 ms frame
(`software_imbe_decoder::decode_fullrate`):

1. `decode_spectral_amplitudes` — raw MBE parameters from bits (TIA spec).
2. `enhance_spectral_amplitudes` — TIA-spec spectral smoothing (existing).
3. `adaptive_smoothing` — TIA-spec error-rate-gated smoothing (existing).
4. **`smooth_voicing_decisions`** — Improvement 6 (gated on smoothed ER).
5. **`compute_envelope_phases`** — Improvement 4 (Hilbert kernel, DC-removed B,
   UV→V table-7-seed). Runs BEFORE the postfilter so the kernel sees the
   pristine spectral envelope, not a contrast-enhanced version.
6. **`apply_formant_postfilter`** — Improvement 5 (voiced-only, edge-reflected).
7. `synth_unvoiced` — Noise + per-band DFT scaling + WOLA (existing).
8. **`synth_voiced`** — Sinusoidal synth using the phi[][New] already
   populated by step 5, with the wider interpolation gating (8).

---

## Knob quick reference

All in [`software_imbe_decoder.cc`](../../lib/op25_repeater/lib/software_imbe_decoder.cc),
top of file:

All fields of `VocoderParams` (declared in [`software_imbe_decoder.h`](../../lib/op25_repeater/lib/software_imbe_decoder.h)). Override at runtime via `set_params()`.

| Field | Default | Improvement |
|---|---|---|
| `fmt_alpha` | 0.22 | 5 — formant emphasis strength |
| `fmt_w` | 5 | 5 — formant smoothing half-window (window = 2W+1) |
| `phase_c_env` | 0.90 | 4 — envelope phase scaling (re-tune after Hilbert kernel fix) |
| `phase_w_rand` | 0.15 | 4 — residual random phase weight |
| `phase_low_blend` | 0.85 | 4 — envelope phase weight on low harmonics |
| `phase_kernel_d` | 19 | 4 — Hilbert kernel half-length |
| `phase_kernel_gamma` | 0.72 | 4 — kernel boundary decay |
| `voicing_smooth_taps` | 3 | 6 — voicing median filter length |
| `voicing_smooth_er_threshold` | 0.01 | 6 — fire smoothing only when smoothed ER ≥ this |
| `uv_to_v_reset` | true | 7 — reset `psi1` + seed phi[Old] from phase table on voicing onset |
| `interp_max_l` | 12 | 8 — fine-transition max harmonic |
| `interp_pitch_tol` | 0.15 | 8 — fine-transition pitch tolerance |
| `repeat_amplitude_decay` | 0.85 | (repeat path) magnitude decay per consecutive repeat frame |

---

## Measuring / tuning

Two analysis tools are provided to make picking knob values empirical
rather than ear-only.

### Offline WAV analyzer

`utils/analyze_vocoder.py` reads decoded WAV files and prints objective
audio-quality metrics plus concrete tuning suggestions. No rebuild needed,
works on any existing recording.

```
python3 utils/analyze_vocoder.py call-*.wav
```

It computes crest factor, spectral flatness, formant/valley energy ratio,
spectral tilt, and HF-energy variance, grades each against target ranges
typical of natural P25-bandlimited speech, and recommends which knob to
nudge. With multiple files it also prints an aggregate row.

### Live telemetry

Setting the environment variable `OP25_DEBUG_VOCODER=1` before launching
`trunk-recorder` enables a static aggregator inside `software_imbe_decoder`
that collects per-frame stats across every P25 call handled by every
recorder. Every 60 seconds (`TELEMETRY_WINDOW_SEC`) it writes a summary to
stderr, like:

```
[VOCODER STATS] 60s window  audio=42s  frames=2103  uv->v=87
  frames    : muted=1.3%  repeated=3.2%
  voicing   : 6.4 band-flips/sec
  pitch     : 1.1% of frames with |dw0|/w0 > 0.25
  ER        : mean=0.0091  p95=0.0334   (mute threshold 0.0875)
  model     : L mean=22.4 (range 11-43)  Luv mean=7.2
  output    : crest=5.13  SFM=0.184
  hints     : (metrics in healthy ranges)
```

Use the two together: the WAV analyzer is great for A/B-comparing builds
on the same recording; the live telemetry tells you what your system is
actually decoding day-to-day. Both surface the same metrics so suggestions
map to the same knobs.

The telemetry is off by default and zero-cost when the env var isn't set
(one `getenv` on the first frame, then an early-return).

---

## Implementation gaps and assumptions

Patents typically describe *what* a technique does and *why* it works, but
leave a lot of "how exactly" unstated — the inventor presumes the implementer
will fill in the details based on the surrounding art. The implementations
above each made specific concrete choices where the patent was silent or
gestured at a class of solutions. This section catalogs those gaps so a
future maintainer (or future-me) knows where the implementation diverges
from a strict reading of the source patent, and which divergences are
likely to matter.

### #3 — `xorshift32` noise generator: period claim is overstated

**The gap.** I justified switching from LCG to xorshift32 by saying the
period went from "≤ 53125" to "2³²−1". That's wrong.

The original LCG `(171·u + 11213) mod 53125` is computed with state size
53125, and was designed so its full period IS 53125 — confirmed by checking
the Hull-Dobell conditions (gcd(11213, 53125)=1; 171−1=170=2·5·17 divisible
by all prime factors of 53125=5⁵·17).

My replacement is `xorshift32(u) mod 53125`, with the state ALSO
fed back through the modulus on each iteration. So the effective state space
is STILL 53125 values — same as the LCG. And there's no algebraic reason to
believe the cycle structure is as nice as the LCG's; it could split into
multiple short orbits.

**Likely impact.** Modest at worst. Even if the cycle is shorter (say a
few thousand samples instead of the LCG's full 53125), the *sequence* of
values is different from the LCG, so it won't sound identical to the old
"6.6 s of repeating noise" artifact. But the claim that period went from
6.6 s to ~9 days is not what the code actually does.

**Real fix.** Keep a full 32-bit state separately from the modulo output:

```cpp
uint32_t state_;  // member, initialized to 0xDEADBEEFu or similar
uint32_t next_u(uint32_t /*unused*/) {
    state_ ^= state_ << 13; state_ ^= state_ >> 17; state_ ^= state_ << 5;
    return state_ % 53125;  // full xorshift32 period of (2^32 - 1)
}
```

This requires teaching the caller (`synth_unvoiced`) not to pass the prior
output back in. Maintains TIA-correct output range and statistics, gives
the actual long period.

### #4 — Voiced phase regeneration: several specifics the patent doesn't fix

**Gap 4a — kernel form.** The patent says `h(m)` is "inversely
proportional to m" and antisymmetric, but doesn't pin the form. I chose
`h(m) = 1/m`. The standard discrete Hilbert transform kernel is actually
`h(n) = 2/(π·n)` for odd n, `h(n) = 0` for even n — *not* what I have.

Mathematically the proper Hilbert kernel only picks up odd-spaced harmonics
in the convolution; my `1/m` averages over all of them. Since the patent's
whole point is "this approximates a Hilbert transform of log magnitude,"
using the not-actual-Hilbert kernel produces a different envelope-phase
result than the patent author would.

**Likely impact.** Significant for purity of the math; the empirical
result (the audio) sounds reasonable, but I had to tune `PHASE_C_ENV` to
0.90 — far above the theoretical `2/π ≈ 0.637` that would apply to a true
Hilbert kernel. That's evidence my kernel under-emphasizes phase compared
to what a true Hilbert would give.

**Real fix.** Replace the `for (int m = 1; m <= D; m++)` loop with the
odd-only form, or precompute a proper Hilbert kernel as a constant array.

**Gap 4b — boundary handling normalization.** Patent specifies symmetric
reflection for `l ≤ 0` and geometric decay for `l > L`. I implement
both, but B = log₂(M) ranges from ~6 to ~13 — biased positive, never
near zero. The kernel's antisymmetric structure should make `Σ h(m)·B`
sum to ~0 for a constant B, but with positive-biased B and asymmetric
boundary treatment, that cancellation isn't perfect. The boundary terms
(especially `B[L+k] = γᵏ · B[L]`) decay slowly enough that they leak DC
into the phase output.

**Real fix.** Subtract `mean(B[1..L])` from all B values before the
convolution. The Hilbert transform has no DC response, so removing it
explicitly avoids the boundary-leak bias.

**Gap 4c — mixing with linear phase isn't in the patent.** US5701390
describes envelope-derived phase as *replacing* the random/zero phase
component. It doesn't say to keep a linear-phase `psi1·l` term too. I
mixed both because pure envelope phase made low harmonics sound "reedy"
without the glottal-pulse alignment from `psi1·l`. That's an empirical
hybrid I invented; the patent inventors might have considered this and
rejected it.

**Likely impact.** Subjectively the hybrid sounded better than pure
envelope phase on the test data. But:

1. The `PHASE_LOW_BLEND` knob exists *because* of this mixing — the
   patent wouldn't need it.
2. If 4a (proper Hilbert kernel) and 4b (DC removal) are fixed, the
   need for 4c's linear-phase mixing might disappear.

**Gap 4d — phase wrap.** `psi1` is wrapped to `(-π, π]`; `env_phase` is
not. For typical voiced frames `env_phase` stays in ~`(-3, 3)` range, but
during sharp formant transitions it could spike higher. Then `cos(phi)` is
mathematically fine, but at single-precision float, the precision of
`cos(large_value)` is degraded. The patent doesn't discuss this; in
practice it's a small effect.

### #5 — Adaptive formant postfilter: different math than the patent

**The gap.** US5241650 specifies an LPC-based postfilter
`H(z) = B(z)/A(z/ν)` where the numerator is a bandwidth-expanded copy of
the denominator. That requires fitting an LPC model to the spectrum,
applying bandwidth expansion via autocorrelation+Levinson, and filtering.

I implemented spectral-contrast enhancement in the magnitude domain:
`M'[l] = M[l] · 2^(α · (log₂ M[l] − log₂ M_smooth[l]))`. This is
**functionally similar** (emphasizes peaks, attenuates valleys) but
mathematically a different operation. They are NOT equivalent — the
patent's LPC postfilter has a specific time-domain effect that magnitude-
domain contrast doesn't reproduce.

**Likely impact.** Hard to say without direct comparison. The two
techniques target the same perceptual goal but achieve it through
different spectral shapes. The patent's specific innovation (bandwidth
expansion to avoid "time-varying brightness modulation") doesn't map
cleanly onto my magnitude-domain approach. I added an energy
re-normalization step that gets at the same goal, but it's not the same
mechanism.

**Gap 5a — edge harmonics get uneven treatment.** The smoothing window
clamps at the band edges: `int jc = (j < 1) ? 1 : (j > L ? L : j);`.
That means harmonic `l=1`'s "smoothed" value averages mostly itself plus
copies of itself — so it tends to be its own best peak by definition, and
gets less emphasis than mid-band harmonics. Same at `l=L`. The patent's
LPC formulation has no analogous edge artifact.

**Real fix.** Extend the smoothing window via reflection or extrapolation
like the phase-regen kernel does, OR skip postfilter on the bottom/top
two harmonics.

**Gap 5b — applied to unvoiced harmonics too.** The postfilter runs on
all `M[l]` indiscriminately. But unvoiced bands carry noise-like content
where there's no "formant peak" to emphasize — peaking those just makes
noise sharper. The patent's LPC postfilter is also broadband, but a
magnitude-domain version arguably should be voiced-only.

**Real fix.** Wrap the per-harmonic update in `if (vee[l][New]) { ... }`
or apply a reduced `α` to unvoiced bands.

### #6 — Voicing-decision smoothing: scope is broader than patent

**The gap.** US6912496 describes voicing smoothing as a corrective step
applied around frames the decoder has *already identified as erroneously
coded*. It's a targeted intervention.

My implementation smooths *every* frame's voicing decisions. That works
out OK most of the time because the median is identity for stable bands,
but it does pay a latency cost on legitimate fast V→UV transitions
(consonant onsets, plosives). The patent intended the smoothing to be
gated on the upstream error indicator (high ER or related).

**Real fix.** Gate `smooth_voicing_decisions()` on a sliding-window error
condition, e.g., only smooth when ER over recent frames exceeds some
threshold. That preserves snappy V/UV transitions in clean audio while
still de-chattering noisy segments.

**Gap 6a — centered vs past-only window.** Patent says "centered around
the erroneously coded frame." Centered means using past *and future*
frames. I use only current + past frames so there's no decoding latency.
That's a defensible trade — but means the smoothing decision is one-sided
and may under-smooth at the leading edge of a chatter burst.

### #7 — UV→V phase reset: my implementation is a simplification

**The gap.** US6963833 says "phases for each harmonic are initialized
with a fixed set of values for each transition from completely unvoiced
frames to voiced frames." That clearly implies a **per-harmonic table of
predetermined phase values** that was empirically chosen by the patent
inventors to produce "balanced output waveforms preventing saturation
distortions."

My implementation just sets `psi1 = 0` on the UV→V transition and lets
the envelope phase term provide whatever per-harmonic offsets it
provides. That's a simpler approach that hopes the envelope phase is
"good enough" — but if the envelope phase happens to be small (smooth
spectrum at onset), all harmonics land at `phi[l] = 0`, which is exactly
the alignment that causes saturation. The patent's specific defense
against this case is its specific fixed-value table, which I don't have.

**Likely impact.** I haven't observed the failure mode the patent is
defending against, but I also can't construct a counterexample to argue
my simplification is safe.

**Real fix.** At UV→V transition, set `phi[l][New]` to a precomputed
per-harmonic table of values that are deliberately *not* aligned (e.g.,
random values that were drawn once and frozen, like a permutation of
`{0, π/3, 2π/3, π, 4π/3, 5π/3}` cycled across l). Then let the envelope
phase add to those.

### #8 — Widened amp/phase interpolation: I just loosened the gate

**The gap.** US6131084 describes a fairly elaborate scheme: interpolate
amplitude, frequency, and phase across **subframe segments smaller than
the IMBE 20 ms frame** (the patent mentions 22.5 ms but really uses
sub-20-ms segments in practice). That allows smooth parameter
trajectories within a single frame.

My implementation just changed two constants in the existing TIA fine-
transition gate: `ell < 8` → `ell < 12` and `< 0.1` → `< 0.15`. That
broadens *which harmonics* and *which pitch changes* qualify for the
smoother synthesis path, but doesn't add subframe interpolation at all.
It's a fraction of what the patent describes.

**Likely impact.** Modest improvement, capped by the existing
synthesizer's resolution (still 20 ms frame granularity). A real
subframe implementation would be substantially smoother but require
restructuring `synth_voiced` to do multiple synthesis passes per frame.

**Real fix.** Genuinely beyond scope for this branch. If pursued, would
need to: (a) interpolate `L`, `w0`, `M[l]`, `vee[l]`, `phi[l]` to two or
three intermediate values per 20 ms frame, (b) call the synthesis loop
once per subframe summing into the output buffer with appropriate
windowing, (c) verify `psi1` accumulation still adds up correctly.

### Cross-cutting issues

**Improvements interact, but were designed orthogonally.** The biggest
example: `apply_formant_postfilter()` runs *before* `synth_voiced()`, so
the postfilter sharpens `M[l][New]`, then the phase regen takes
`log₂(M[l][New])` to compute its envelope phase. If the postfilter is
exaggerating peaks, the envelope phase sees sharper edges in B than the
raw spectrum had, and the Hilbert convolution outputs a larger phase
swing than it should. The two improvements are coupled in a way the
patents don't anticipate (they assume only one or the other is present).

**Likely impact.** The hand-tuned `PHASE_C_ENV = 0.90`, well above the
2/π theoretical value, may partly reflect this coupling. If the
postfilter were disabled or weaker, `PHASE_C_ENV` would probably want a
smaller value.

**Real fix.** Compute the envelope-phase term from the *pre-postfilter*
magnitudes (i.e., from `Mu[l]` instead of `M[l]`), so the two
improvements operate on independent inputs.

**The smoothed ER reused for multiple thresholds.** The same `ER`
variable drives:

- The mute threshold (`ER > 0.0875`)
- The repeat threshold (`ET >= 10 + 40·ER`)
- (Indirectly, via the gating decisions, the upstream voicing-smooth and
  postfilter activation when we gate on it)

TIA-102.BABA-A spec actually defines separate error metrics for different
purposes. I'm using one smoothing function for all of them. Probably
fine in practice but a subtle simplification.

**The `* 4` audio mix factor is in the path.** `samples[en] = suv[en] +
sv[en] * 4`. The factor of 4 was empirically chosen in the original
boatbod code to balance voiced/unvoiced loudness. With my postfilter
pushing `M[l]` values higher in formant bands, `sv[en]` can be louder
than it was before — which means the clipping check
(`if(abs(sample) > 32767)`) fires more often. Worth checking whether
peak-limiting is now degrading the audio.

### Status: closed gaps

The following have been fixed in code; the gap descriptions above are kept
for historical context.

| Gap | Status | Resolution |
|---|---|---|
| **3** xorshift32 period | ✅ closed | Full 32-bit state in `unvoiced_noise_state`; period now actually 2³²−1. |
| **4a** kernel form | ✅ closed | Proper `2/(πm)` weights, odd m only. |
| **4b** DC bias | ✅ closed | Subtract `mean(B[1..L])` before convolving; boundary extensions operate on zero-mean B. |
| **Cross-cutting** | ✅ closed | Phase regen extracted into `compute_envelope_phases()`, called *before* `apply_formant_postfilter()`. |
| **5a** edge harmonics | ✅ closed | Symmetric reflection at l=1 and l=L instead of clamp-to-edge. |
| **5b** postfilter on unvoiced | ✅ closed | Contrast emphasis gated on `vee[l][New]`; unvoiced bands unchanged. |
| **6** always-on smoothing | ✅ closed | New `voicing_smooth_er_threshold` knob (default 0.01); smoothing fires only when smoothed ER ≥ threshold. |
| **7** UV→V phase table | ✅ closed | 56-entry table of misaligned per-harmonic phase offsets pre-seeded into `phi[Old]` on UV→V transition. |

### Status: not addressed

| Gap | Status | Reason |
|---|---|---|
| **5 main** LPC vs spectral contrast | won't fix | Magnitude-contrast is functionally similar and fits MBE more naturally. The patent's LPC formulation would be a different implementation, not strictly better. |
| **4c** linear-phase mixing in patent | won't fix (knob retained) | Set `phase_low_blend=1.0` to disable the mixing; some users may prefer the glottal-pulse character. |
| **6a** centered window with future frames | offline-only | Live decoder remains past-only; the offline `imbe_tune` tool can use lookahead because it processes whole captured calls at once. |
| **8** true sub-frame interpolation | won't fix | Would require restructuring `synth_voiced` to do multiple synthesis passes per 20 ms frame for a marginal gain. Out of scope. |

### Multi-pass / offline mode

The live decoder is single-pass: each 20 ms IMBE frame is decoded
immediately to PCM. That precludes any improvement that needs *future*
frames as context.

The offline tool `imbe_tune` doesn't have that constraint - it sees the
whole captured `.imbe` file at once. With `--multipass`, it does two
decoder passes per parameter combination:

| Pass | What runs | Captured |
|---|---|---|
| 1 | Full decode with `voicing_smooth_taps = 1` (internal smoothing off) | Raw per-frame voicing via `get_decoded_voicing()` |
| -- | (in-process) Centered median smoothing over the full voicing sequence (window = `2·lookahead+1`, default 5 taps) | -- |
| 2 | Full decode; before each frame `set_voicing_override(smoothed[i])` is applied | Audio output written to WAV |

The override is applied AFTER the decoder's own (now-disabled) smoothing
step and BEFORE `compute_envelope_phases` / postfilter / synth - so the
synthesizer sees the centered-smoothed voicing without disturbing any
other per-frame state.

```bash
./build/imbe_tune --input call.imbe --sweep utils/sweep.json --output-dir /tmp/sw \
                  --multipass --lookahead 2
```

Cost: 2× decode time per combo. Negligible on the offline path.

### Possible follow-ups (not implemented)

Other multi-pass / quality-latency tradeoffs that could be added if
ear-truth says they're worth the complexity. None are in code today;
listed here so the next iteration knows where to look.

| Idea | Mechanism | Likely impact |
|---|---|---|
| Centered pitch (`w0`) median | Same 2-pass pattern: capture per-frame `w0`, median across window, override before pass 2. Requires also interpolating `M[l]` to the corrected harmonic grid (per US6912496); not trivial. | Catches occasional octave-error frames that produce audible glitches. |
| Adaptive mute threshold | Pass 1 collects the per-frame ER distribution; pass 2 uses a threshold set relative to that call's typical ER instead of the fixed 0.0875. | Frees normal-noise frames from being muted on quiet/distant calls; tightens threshold on clean calls. |
| Whole-call AGC | After pass 2, post-process the output WAV to normalize peak / RMS to a target. Pure Python, no decoder change. | Useful when systems have very mixed loudness across talkgroups. |
| Sub-frame interpolation (Gap 8) | Restructure `synth_voiced` to do M synthesis passes per 20 ms frame (M=2 or 3), interpolating `L`, `w0`, `M[l]`, `vee[l]` between frames. Patent US6131084 describes this. | Smoother sustained vowels, less frame-boundary artifact. |
| Frame-buffered live decoder | Buffer N IMBE frames inside `p25p1_fdma` before emitting audio; smoothing then uses past + future. Adds N·20 ms output delay. | Brings centered-voicing-smoothing benefit to live recordings (currently offline-only). |

If any of these turn out to be wanted, the API hooks added for the
voicing-multipass implementation (`get_decoded_voicing`,
`set_voicing_override`) are the template - just add the corresponding
accessor / override pair on the parameter you want to smooth across the
call.

### Re-tuning note

The empirical defaults were originally tuned against the gaps in place —
particularly `phase_c_env=0.90`, which was probably compensating for the
wrong-kernel + DC-bias issues (gaps 4a + 4b combined). With those fixed,
the proper Hilbert kernel produces ≈ 2.6× smaller phase swings on
typical voiced speech compared to the old `1/m` kernel. The current
default of 0.90 will therefore give *less* envelope phase than before;
expect to re-tune upward (possibly into the 1.5–2.5 range) for
equivalent perceptual phase variation. `phase_low_blend` might also no
longer be needed (was masking the reedy/over-flat low-harmonic phase);
set to 1.0 to test.

Use `utils/imbe_tune` + `utils/rank_sweep.py` to find the new sweet
spot quickly against captured `.imbe` files.

---

## References

- **US5241650** (Motorola, expired ~2009) — Digital speech decoder having a postfilter with reduced spectral distortion. <https://patents.google.com/patent/US5241650>
- **US5701390** (DVSI, expired Feb 2015) — Synthesis of MBE-based coded speech using regenerated phase information. <https://patents.google.com/patent/US5701390>
- **US6131084** (DVSI, expired ~2017) — Dual subframe quantization of spectral magnitudes (decoder describes subframe-interpolation synthesis). <https://patents.google.com/patent/US6131084>
- **US6912496** (DVSI, expired Mar 2023) — Preprocessing modules for quality enhancement of MBE coders and decoders. <https://patents.google.com/patent/US6912496>
- **US6963833** (DVSI, expired Mar 2022) — Modifications in the MBE model for generating high-quality speech at low bit rates. <https://patents.google.com/patent/US6963833>
- **TIA-102.BABA-A** — Project 25 IMBE Vocoder Description. The frame muting (§7.8) and frame repeat (§7.7) policies, equation 142 for the random voiced-phase term, and the baseline voiced-synthesis structure all come from this spec.
