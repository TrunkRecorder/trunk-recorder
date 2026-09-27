# Vocoder Changes from Stock OP25

Trunk Recorder vendors OP25's P25 voice decoders in `lib/op25_repeater/`.
This document describes how they differ from stock OP25: decoding fixes,
error concealment, and changes to the float decoder's synthesis. Phase 1
(full-rate IMBE) is covered first; [Phase 2](#phase-2-half-rate) shares the
synthesis and has its own fixes.

There are two decoders:

- **Float decoder** — `software_imbe_decoder`, used when
  `"softVocoder": true` (the default). This is where the synthesis changes
  are.
- **Fixed-point decoder** — `imbe_vocoder`, used when
  `"softVocoder": false`. It gets the error-concealment changes only.

The float decoder is the default because, with these changes, it matches
or beats the fixed-point decoder on clean audio and outperforms it under
channel errors on Phase 1, and matches it on Phase 2. Stock Trunk Recorder
defaults to the fixed-point decoder.

---

## Decoding fixes

### Float decoder read the unprotected bits one position off

Each IMBE frame carries 7 bits with no error protection (`u7`), holding the
two low bits of the pitch index, the low bit of the gain index and three
spectral bits. `imbe_header_decode()` returns them shifted left by one.
Stock OP25 shifts them back for the fixed-point decoder but passes them to
the float decoder unshifted, so most frames decode with a slightly wrong
pitch (up to ~5 %), gain and spectrum — heard as rough, warbly voice.
`decode_fullrate()` now does the shift itself, which also fixes `decode()`
(used for YSF and the legacy voice path). The layout was confirmed against
mbelib and JMBE.

### Decoder state is reset between calls

Stock OP25 never cleared vocoder state between calls in trunk-recorder
(`p25_frame_assembler_impl::clear()` had the call commented out), so the
smoothed error rate, repeat counter and synthesis history carried from one
call into the next; a call that ended noisy could start the next one muted.
The frame assembler now calls `p25p1_fdma::clear()`, which clears both
decoders, and `p25p1_voice_decode::clear()` and `rx_sync::call_end()` do the
same for their decoders.

### No `exit()` on bad frames

A corrupted frame could produce an out-of-range harmonic count, on which
`software_imbe_decoder::rearrange()` called `exit()`, and `mbelib.c` did the
same for an out-of-range `uvquality`. Both now clamp to the valid range.

### Persistent spectral energy estimate

TIA-102.BABA-A smooths the spectral energy `S_E = 0.95·S_E + 0.05·R_M0`
across frames (it sets the adaptive-smoothing threshold). Stock OP25 kept
`S_E` in a local variable reset every frame; it is now a member that
persists.

### Full-period unvoiced noise

The unvoiced excitation used a linear congruential generator with a period
of 53125 samples (~6.6 s), audible as repeating noise on long fricatives.
It is replaced by xorshift32 with full 32-bit state.

---

## Error concealment

TIA-102.BABA-A repeats the previous frame when its error counts are high
(§7.7) and mutes after too many repeats or when the smoothed error rate
gets too high (§7.8).

### Repeat threshold: E0 ≥ 3 instead of E0 ≥ 2

E0 is the number of bit errors the Golay(23,12) code corrected in the most
important 12 bits. Golay(23,12) is a perfect code that corrects up to three
errors, so a frame with E0 = 2 is almost always decoded correctly;
repeating it discards good speech. Both decoders now repeat at E0 ≥ 3,
which scored markedly better on simulated random and fading channels
(PESQ-NB +0.30 at 2 % bit error rate). The ET and mute rules are TIA's.

### Float decoder: mutes fade

Stock OP25 outputs hard silence for a muted frame and leaves stale
parameters in place, so the next good frame cross-fades from sound that
predates the mute. A muted frame is now synthesized as a zero-amplitude
frame holding the last good parameters: the previous frame fades out and
the next fades in.

### Fixed-point decoder: repeat and mute added

Stock OP25's fixed-point decoder had no concealment at all; corrupted
frames were synthesized as-is. `imbe_vocoder::imbe_decode_checked()` applies
the same rules as the float decoder. A repeat re-synthesizes the last good
frame's parameters (`imbe_repeat()`) rather than re-decoding the old bits,
which would apply its spectral prediction twice; a mute fades out
(`imbe_mute()`). `p25p1_fdma`, `p25p1_voice_decode` and `rx_sync` (YSF) all
use it.

---

## Float decoder synthesis

### Envelope phase for voiced harmonics

Stock OP25 gives every voiced harmonic a pure linear phase, so all
harmonics line up at each pitch pulse: a sharp, buzzy pulse train. Each
harmonic's phase now also gets a term computed from the spectral envelope —
a discrete Hilbert transform of the log amplitudes, i.e. the minimum-phase
response of the vocal tract (after US5701390) — plus TIA's random phase
term (eq. 142) on upper harmonics, weighted by the frame's unvoiced
fraction. Strength: `phase_c_env` 0.7.

### Every harmonic glides between frames

TIA's synthesis interpolates frequency, amplitude and phase smoothly from
one frame to the next only for harmonics below 8 and when the pitch changes
by less than 10 %; everything else is cross-faded over about 6 ms. Upper
harmonics therefore jump every 20 ms, which shows as broken, beaded lines on
a spectrogram. All harmonics now glide unless the pitch changes by more than
20 % (`interp_max_l` 57, `interp_pitch_tol` 0.2).

### Smooth unvoiced synthesis

TIA's noise synthesis changes its spectrum only inside a 49-sample
cross-fade at each frame boundary and holds it for the rest of the frame.
Each frame's noise is now generated with exact per-band energies and
cross-faded with power-complementary weights over the whole frame, so the
noise spectrum changes smoothly (`uv_synth_mode` 1). `decode_tap()` (the
Phase 2 path) uses the same synthesis.

### High-frequency presence lift

A +3 dB lift rising from 2.2 kHz to 3.7 kHz (`hf_lift_db`, `hf_lift_f1`)
gives a slightly crisper sound, similar to a DVSI-derived reference
decoder.

### Effect

On lab speech passed through simulated P25 channels and on live traffic
from two systems, the float decoder with these changes scores higher than
stock OP25's float decoder on PESQ-NB (3.10 vs 3.01 clean, 2.94 vs 2.76 on a
fading channel) and DNSMOS, and its output changes much more evenly within
each frame. The defaults were confirmed in listening tests on live traffic.

---

## Phase 2 (half-rate)

P25 Phase 2 carries half-rate AMBE+2 frames. `p25p2_tdma` unpacks them
with mbelib (`mbe_dequantizeAmbe2250Parms`), handles repeats and muting
itself, and synthesizes through `decode_tap()` on the same decoder objects.
The float decoder therefore gets all of the synthesis changes above on
Phase 2 as well. Phase 2 also has these changes of its own:

### Float decoder level corrected

mbelib's half-rate spectral amplitudes are on a larger scale than the
full-rate decoder's. Stock OP25 passes them to the float synthesizer as-is,
so Phase 2 speech came out about 8 dB hotter than Phase 1 and 2–2.5 % of
samples clipped, which put the float decoder well behind the fixed-point
one on Phase 2 (PESQ-NB 2.26 vs 2.99). `decode_tap()` now scales the
amplitudes by `tap_gain` (0.3), which matches the Phase 1 level and removes
the clipping; with the synthesis changes the float decoder then matches the
fixed-point decoder on Phase 2 (2.98 vs 2.99 clean, 2.87 vs 2.88 on a
fading channel). The same scale applies to the other half-rate paths that
use `decode_tap()` (DMR and D-STAR in `rx_sync`).

### Decoder state is reset between calls

`p25p2_tdma` initialized its error-rate tracker, repeat counter and
previous-frame parameters only when it was created. A call that ended on a
bad signal left the next call on that slot starting with a high error rate
and stale parameters: in testing, its first two frames were muted and the
next three came out about 20 dB too loud. `p25p2_tdma::clear()` now resets
all of it and both decoders, and the frame assembler calls it between
calls alongside the Phase 1 reset.

### Mutes fade

A muted Phase 2 frame used to be hard silence with no synthesis, so the
next frame started from stale state. It is now a zero-amplitude frame
through the synthesizer (`decode_tap_mute()` for the float decoder,
`imbe_mute()` for the fixed-point decoder), as for Phase 1.

### Persistent spectral energy estimate

`decode_tap()` used its own local `S_E`, reset every frame; it now uses the
persistent estimate, as the full-rate path does.

Phase 2's repeat rules and error thresholds are unchanged: they belong to
the half-rate codec and its own FEC, and the Phase 1 E0 ≥ 3 reasoning does
not carry over directly.

---

## Pipeline

Per 20 ms frame, `software_imbe_decoder::decode_fullrate`:

1. **Error gating** — update the smoothed error rate; mute, repeat, or
   decode.
2. **Decode** — `rearrange`, `decode_vuv`, `decode_spectral_amplitudes`,
   `enhance_spectral_amplitudes` (TIA spectral enhancement).
3. **Smoothing** — `adaptive_smoothing` (TIA), plus
   `smooth_voicing_decisions` and `smooth_amplitudes` (both off by default).
4. **Phase** — `compute_envelope_phases`.
5. **Postfilter** — `apply_formant_postfilter` (off by default).
6. **Synthesis** — `synth_unvoiced_smooth` (or TIA `synth_unvoiced`), then
   `synth_voiced`; output = unvoiced + 4 × voiced, clipped to 16 bits.

---

## Parameters

The float decoder's settings are the fields of `VocoderParams` in
[software_imbe_decoder.h](../../lib/op25_repeater/lib/software_imbe_decoder.h),
where each field is commented; `software_imbe_decoder::set_params()`
changes them at runtime. The fixed-point decoder has no settings.

| Field | Default | Stock OP25 equivalent | What it does |
|---|---|---|---|
| `phase_c_env` | 0.7 | 0 | Envelope-phase strength; 0 = linear phase only. |
| `phase_w_rand` | 0.25 | 0 | TIA eq. 142 random phase on upper harmonics, scaled by the unvoiced fraction. |
| `phase_low_blend` | 0.4 | — | Envelope-phase weight on harmonics ≤ L/4. |
| `phase_kernel` | 0 | — | 0 = odd-only Hilbert kernel 2/(πm) on mean-removed log amplitudes; 1 = US5701390's 1/m kernel. |
| `phase_kernel_d` | 19 | — | Kernel half-length. |
| `phase_kernel_gamma` | 0.6 | — | How the log envelope is extended past the last harmonic. |
| `phase_track` | 1.0 | 1.0 | Fraction of the way each gliding harmonic is steered to its new phase per frame. |
| `uv_to_v_reset` | true | — | Restart the pitch-phase accumulator at voicing onset. |
| `interp_max_l` | 57 | 8 | Highest harmonic that glides between frames. |
| `interp_pitch_tol` | 0.2 | 0.1 | Largest relative pitch change that still glides. |
| `uv_synth_mode` | 1 | 0 | 1 = smooth unvoiced synthesis; 0 = TIA. |
| `uv_smooth_gain` | 1.0 | — | Level of the smooth unvoiced path relative to TIA's. |
| `uv_xfade` | 160 | — | Cross-fade length (samples) of the smooth unvoiced path. |
| `hf_lift_db` | 3.0 | 0 | Presence lift at 3.7 kHz; 0 = off. |
| `hf_lift_f1` | 2200 | — | Frequency (Hz) where the lift starts. |
| `tap_gain` | 0.3 | 1.0 | Scale on the half-rate amplitudes passed to `decode_tap()` (Phase 2, DMR). |
| `mute_er` | 0.0875 | 0.0875 | Mute when the smoothed error rate exceeds this (TIA §7.8). |
| `repeat_e0` | 3 | 2 | Repeat when E0 ≥ this. |
| `repeat_et_base`, `repeat_et_slope` | 10, 40 | 10, 40 | Repeat when ET ≥ base + slope × error rate (TIA §7.7). |
| `max_repeats` | 4 | 4 | Consecutive repeats before muting. |
| `repeat_amplitude_decay` | 1.0 | 1.0 | Amplitude factor per consecutive repeated frame. |

Optional processing, off by default. These were measured and did not
improve the result on the tested systems; they remain for experiments.

| Field | Default | What it does |
|---|---|---|
| `fmt_alpha`, `fmt_w` | 0, 7 | Formant postfilter (peak/valley contrast on voiced harmonics). The TIA spectral enhancement already does this; adding it lowered quality. |
| `voicing_smooth_taps`, `voicing_smooth_er_threshold` | 1, 0 | Majority filter on voicing decisions over recent frames, when the error rate exceeds the threshold. Lowered quality under errors. |
| `amp_smooth` | 0 | Pull each harmonic's level toward the previous frame. Less flutter, but blurs fast changes. |
| `aper_max`, `aper_f1`, `aper_f2` | 0, 2000, 3500 | Render part of each voiced harmonic above `aper_f1` as noise. |
| `onset_ramp_mode` | 0 | Fade harmonics that start or stop over the whole frame instead of TIA's short ramp. |

---

## Code

| File | Change |
|---|---|
| `software_imbe_decoder.h/.cc` | Bit fix, state reset, S_E, noise generator, error concealment, synthesis changes, `VocoderParams`; `decode_tap()` level and `decode_tap_mute()` for half-rate. |
| `imbe_vocoder/imbe_vocoder.h`, `imbe_vocoder/decode.cc` | `imbe_decode_checked()`, `imbe_repeat()`, `imbe_mute()`. |
| `p25p1_fdma.cc` | Clears both decoders on `clear()`; fixed-point path uses `imbe_decode_checked()`. |
| `p25_frame_assembler_impl.cc` | Calls `p1fdma.clear()` and `p2tdma.clear()` between calls. |
| `p25p2_tdma.h/.cc` | `clear()` resets half-rate decoding state; muted frames fade. |
| `p25p1_voice_decode.cc`, `rx_sync.cc` | Clear their decoders between calls; fixed-point paths use `imbe_decode_checked()`. |
| `mbelib.c` | Clamp `uvquality` instead of `exit()`. |

All paths are under `lib/op25_repeater/lib/`.

---

## References

- **TIA-102.BABA-A** — Project 25 IMBE vocoder description: frame repeat
  (§7.7), muting (§7.8), spectral enhancement, adaptive smoothing, and the
  baseline synthesis.
- **US5701390** (DVSI, expired 2015) — Synthesis of MBE-based coded speech
  using regenerated phase information (envelope phase).
  <https://patents.google.com/patent/US5701390>
- **US6131084** (DVSI, expired ~2017) — Dual subframe quantization of
  spectral magnitudes (interpolated synthesis).
  <https://patents.google.com/patent/US6131084>
- **US6912496** (DVSI, expired 2023) — Preprocessing modules for quality
  enhancement of MBE coders and decoders (voicing smoothing).
  <https://patents.google.com/patent/US6912496>
- **US6963833** (DVSI, expired 2022) — Modifications in the MBE model for
  generating high-quality speech at low bit rates (voicing-onset phase
  reset).
  <https://patents.google.com/patent/US6963833>
- **US5241650** (Motorola, expired ~2009) — Digital speech decoder having a
  postfilter with reduced spectral distortion.
  <https://patents.google.com/patent/US5241650>
