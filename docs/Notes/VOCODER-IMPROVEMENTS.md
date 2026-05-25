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
`next_u(u) = (171·u + 11213) mod 53125`. Period ≤ 53125 samples ≈ 6.6 s of
continuous unvoiced speech before repetition. Audible as "repeating noise"
on long sibilants / fricatives.

**Fix.** Replaced with `xorshift32` folded to the same 0–53124 output range.
Period 2³²−1 — effectively non-repeating.

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
phi_env(l) = c_env · Σ (B[l+m] − B[l−m]) / m   for m = 1..D
B = log₂(M)  (enhanced spectral magnitudes)
```

This is a discrete Hilbert transform of log-magnitude across harmonics. The
Hilbert transform of log-magnitude *is* the phase of the minimum-phase system
with that magnitude — and the vocal tract is approximately minimum-phase. So
the resulting phase correlates with formant shape: naturally non-aligning,
no comb, no buzz.

Boundary handling per patent: `B[0]=0`, `B[−l]=B[l]` (reflection),
`B[L+k]=B[L]·γᵏ` (geometric decay outside valid harmonic range).

The linear-phase term `psi1·l` is retained at reduced weight on low harmonics
(`l ≤ L/4`) to preserve some glottal-pulse character. Optional residual TIA
random phase can be re-enabled via a knob.

**Code.** [`software_imbe_decoder.cc:synth_voiced()`](../../lib/op25_repeater/lib/software_imbe_decoder.cc)

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
equivalent to the patent's bandwidth-expanded LPC postfilter `H(z)=B(z)/A(z/ν)`
but a natural fit for MBE since harmonics already sample the envelope):

```
M'[l] = M[l] · 2^(α · (log₂ M[l] − log₂ M_smooth[l]))
```

`M_smooth` is a (2W+1)-tap centered moving average over harmonics. Above-
average harmonics (formant peaks) get boosted; below-average (valleys) get
attenuated. Total energy is renormalized to preserve loudness — this is the
patent's specific innovation, eliminating the "time-varying brightness
modulation" of naive postfilters.

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
(N−1) past frames. Original (pre-smoothing) values are stored in
`vee_history[]` so smoothing doesn't compound across time.

**Code.** [`software_imbe_decoder.cc:smooth_voicing_decisions()`](../../lib/op25_repeater/lib/software_imbe_decoder.cc)

**Knobs.**

| Knob | Default | Effect |
|---|---|---|
| `VOICING_SMOOTH_TAPS` | 3 | Total filter length. 1 = off, 3 = drop single-frame outliers (rec), 5 = smoother but voicing-state changes lag 2 frames. |

---

## 7. UV→V phase reset (US6963833 — expired Mar 2022)

**Problem.** When a voiced segment starts after silence or a fricative, the
running `psi1` phase accumulator carries whatever value it had when the
previous voiced segment ended (it was advancing through the unvoiced interval
too). That stale phase can land harmonics in alignment at the onset frame,
producing a click/pop.

**Fix.** Detect fully-unvoiced → voiced transitions (no `vee[l][Old]` set,
some `vee[l][New]` set) and reset `psi1` to 0. The envelope-derived phase
then provides the only per-harmonic offsets — patent's "balanced output
waveforms preventing saturation distortions."

**Code.** [`software_imbe_decoder.cc:synth_voiced()`](../../lib/op25_repeater/lib/software_imbe_decoder.cc) (UV→V detection block)

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
4. **`smooth_voicing_decisions`** — Improvement 6.
5. **`apply_formant_postfilter`** — Improvement 5.
6. `synth_unvoiced` — Noise + per-band DFT scaling + WOLA (existing).
7. **`synth_voiced`** — Sinusoidal synth with phase regen (4), UV→V reset (7), wider interpolation (8).

---

## Knob quick reference

All in [`software_imbe_decoder.cc`](../../lib/op25_repeater/lib/software_imbe_decoder.cc),
top of file:

| Knob | Default | Improvement |
|---|---|---|
| `FMT_ALPHA` | 0.25 | 5 — formant emphasis strength |
| `FMT_W` | 3 | 5 — formant smoothing half-window |
| `PHASE_C_ENV` | 0.75 | 4 — envelope phase scaling |
| `PHASE_W_RAND` | 0.0 | 4 — residual random phase weight |
| `PHASE_LOW_BLEND` | 0.40 | 4 — envelope phase weight on low harmonics |
| `PHASE_KERNEL_D` | 19 | 4 — Hilbert kernel half-length |
| `PHASE_KERNEL_GAMMA` | 0.72 | 4 — kernel boundary decay |
| `VOICING_SMOOTH_TAPS` | 3 | 6 — voicing median filter length |
| `UV_TO_V_RESET` | true | 7 — reset `psi1` on voicing onset |
| `INTERP_MAX_L` | 12 | 8 — fine-transition max harmonic |
| `INTERP_PITCH_TOL` | 0.15 | 8 — fine-transition pitch tolerance |

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

## References

- **US5241650** (Motorola, expired ~2009) — Digital speech decoder having a postfilter with reduced spectral distortion. <https://patents.google.com/patent/US5241650>
- **US5701390** (DVSI, expired Feb 2015) — Synthesis of MBE-based coded speech using regenerated phase information. <https://patents.google.com/patent/US5701390>
- **US6131084** (DVSI, expired ~2017) — Dual subframe quantization of spectral magnitudes (decoder describes subframe-interpolation synthesis). <https://patents.google.com/patent/US6131084>
- **US6912496** (DVSI, expired Mar 2023) — Preprocessing modules for quality enhancement of MBE coders and decoders. <https://patents.google.com/patent/US6912496>
- **US6963833** (DVSI, expired Mar 2022) — Modifications in the MBE model for generating high-quality speech at low bit rates. <https://patents.google.com/patent/US6963833>
- **TIA-102.BABA-A** — Project 25 IMBE Vocoder Description. The frame muting (§7.8) and frame repeat (§7.7) policies, equation 142 for the random voiced-phase term, and the baseline voiced-synthesis structure all come from this spec.
