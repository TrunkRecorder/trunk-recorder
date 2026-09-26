# Vocoder Improvements

This branch changes the P25 Phase 1 (IMBE) voice decoders vendored in
`lib/op25_repeater/`: the float decoder `software_imbe_decoder` (used when
`"softVocoder": true`) and the fixed-point `imbe_vocoder` (the default). It
fixes several decoding bugs, replaces the error concealment, and reworks the
float decoder's synthesis. Every change was measured objectively, and the
final settings were chosen in listening tests on live traffic from two
systems: dcfd (P25) and wmata (SmartNet with P25 voice).

**Recommended setting:** `"softVocoder": true`. With these changes the float
decoder is the best or tied-best option in every condition tested,
including channel errors. The fixed-point decoder gets the correctness and
error-concealment fixes but not the synthesis work.

All float-decoder settings live in `VocoderParams`
([software_imbe_decoder.h](../../lib/op25_repeater/lib/software_imbe_decoder.h)),
with defaults in the struct; `software_imbe_decoder::set_params()` overrides
them at runtime.

---

## What changed

### Decoding fixes (both decoders unless noted)

1. **Float decoder read the 7 unprotected bits one position off.**
   `imbe_header_decode()` returns them shifted left by one. `p25p1_fdma`
   shifted them back for the fixed-point decoder but passed them unshifted to
   `software_imbe_decoder`, whose `rearrange()` expects the same layout as the
   fixed-point `ch_decode` (confirmed against mbelib and JMBE). Most frames
   decoded with wrong pitch LSBs (77 % of frames, mean f0 error 1.2 %, up to
   5 %), a wrong gain LSB and three wrong spectral bits, heard as rough,
   warbly pitch. The same bug is in upstream OP25. `decode_fullrate()` now
   does the shift itself, which also fixes `decode()` (YSF, legacy voice
   path). +0.09 PESQ-NB on its own.
2. **Decoder state is reset between calls.** `p25_frame_assembler_impl`
   now calls `p1fdma.clear()`, which clears both vocoders. Previously the
   smoothed error rate, repeat counter and synthesis state carried into the
   next call; a call that ended noisy could start the next one muted.
3. **No more `exit()` on bad frames.** `rearrange()` clamps an out-of-range
   harmonic count and `mbelib.c` clamps `uvquality` instead of terminating
   the process.
4. **Persistent S_E.** The TIA recursion `S_E = 0.95·S_E + 0.05·R_M0` used a
   local variable reset to 0 each frame, so it never smoothed anything.
5. **Full-period unvoiced noise.** The LCG (period 53125 samples, ~6.6 s)
   is replaced by xorshift32 with a full 32-bit state.

### Error concealment

6. **Repeat only when E0 ≥ 3** (TIA-102.BABA-A says E0 ≥ 2). Golay(23,12) is
   a perfect code, so E0 = 2 means two errors were corrected, almost always
   correctly; repeating that frame throws away good data. +0.30 PESQ at 2 %
   BER and +0.19 on a fading channel. The ET and mute rules are TIA's.
7. **Float decoder: repeats keep advancing phase, mutes fade.** The repeat
   path used to freeze each harmonic's phase, which detuned it or cancelled
   it in the overlap-add. A mute now synthesizes a zero-amplitude frame, so
   the previous frame's tail fades out and the next frame fades in.
8. **Fixed-point decoder: proper repeat and mute.** It had no concealment.
   `imbe_vocoder::imbe_decode_checked()` applies the same rules; a repeat
   re-synthesizes the last good frame's parameters (re-decoding the old bit
   vector would apply its spectral prediction twice) and a mute fades out.
   `p25p1_fdma`, `p25p1_voice_decode` and `rx_sync` all call it.

### Float decoder synthesis

9. **Envelope phase** (after US5701390). Each voiced harmonic's phase is the
   linear pitch-pulse phase plus a term from a discrete Hilbert transform of
   the log spectral envelope (the minimum-phase response of the envelope),
   instead of pure linear phase. `phase_c_env` 0.7 matches the pulse
   sharpness of natural speech and was preferred in listening tests.
10. **Every harmonic glides between frames.** The TIA path interpolates
    frequency, amplitude and phase only for harmonics below 8 and cross-fades
    the rest over ~6 ms, which breaks upper harmonics into beads on a
    spectrogram. All harmonics are now interpolated unless the pitch changes
    by more than 20 %.
11. **Smooth unvoiced synthesis.** The TIA noise changes spectrum only inside
    a 49-sample cross-fade. Each frame's noise is now generated with exact
    band energies and cross-faded with power-complementary weights across
    the whole frame.
12. **High-frequency presence lift.** +3 dB rising from 2.2 kHz to 3.7 kHz,
    chosen in listening tests; a DVSI-derived reference decoder carries a
    similar lift and sounded "crisper".
13. **Smaller fixes:** the random-phase weight uses the current frame's
    unvoiced count (it was a frame stale), and `decode_tap()` (Phase 2 path)
    uses the same unvoiced synthesis as `decode_fullrate()`.

---

## Synthesis pipeline (float decoder, per 20 ms frame)

`software_imbe_decoder::decode_fullrate`:

1. Error gating: smoothed error rate, then mute / repeat / decode (items
   6–7).
2. Decode: `rearrange` → `decode_vuv` → `decode_spectral_amplitudes` →
   `enhance_spectral_amplitudes` (TIA spectral enhancement).
3. `adaptive_smoothing` (TIA, error-rate gated), `smooth_voicing_decisions`
   and `smooth_amplitudes` (both off by default).
4. `compute_envelope_phases` — linear phase plus envelope phase.
5. `apply_formant_postfilter` (off by default).
6. `synth_unvoiced_smooth` (or TIA `synth_unvoiced`), then `synth_voiced`;
   output = unvoiced + 4 × voiced, clipped to 16 bits.

---

## Parameters (`VocoderParams`)

Defaults are what runs live. Options marked *off* were measured and did not
help; they remain for experiments.

| Field | Default | What it does |
|---|---|---|
| `phase_c_env` | 0.7 | Envelope-phase strength. 0 = pure linear phase (buzzy). 0.7 measured closest to natural pulse sharpness; 2.0 is worse. |
| `phase_w_rand` | 0.25 | Random phase on upper harmonics, scaled by the frame's unvoiced fraction (TIA eq. 142). 0–0.6 made no measurable difference. |
| `phase_low_blend` | 0.4 | Envelope-phase weight on harmonics ≤ L/4. 1.0 measured worse. |
| `phase_kernel` | 0 | 0 = odd-only Hilbert kernel 2/(πm) on mean-removed log-magnitudes; 1 = US5701390's 1/m kernel. Indistinguishable. |
| `phase_kernel_d` | 19 | Kernel half-length. |
| `phase_kernel_gamma` | 0.6 | Extension of the log envelope beyond the last harmonic. |
| `phase_track` | 1.0 | How far each harmonic is steered to its new target phase per frame. Lower values reduce decoder-added frequency jitter but made no audible difference. |
| `uv_to_v_reset` | true | Restart the pitch-phase accumulator at voicing onset. No measurable effect. |
| `interp_max_l` | 57 | Highest harmonic interpolated between frames (TIA: 8). |
| `interp_pitch_tol` | 0.2 | Largest relative pitch change still interpolated (TIA: 0.1); 0.15–0.3 measured the same. |
| `uv_synth_mode` | 1 | 1 = smooth unvoiced synthesis; 0 = TIA. |
| `uv_smooth_gain` | 1.0 | Level of the smooth unvoiced path relative to TIA's. |
| `uv_xfade` | 160 | Cross-fade length (samples) of the smooth unvoiced path. |
| `hf_lift_db` | 3.0 | Presence lift at 3.7 kHz; 0 = off. |
| `hf_lift_f1` | 2200 | Frequency (Hz) where the lift starts. |
| `fmt_alpha` | 0 (*off*) | Formant postfilter strength. Every setting tried lowered PESQ and DNSMOS; the TIA enhancement already acts as a postfilter. |
| `fmt_w` | 7 | Postfilter smoothing half-window. |
| `voicing_smooth_taps` | 1 (*off*) | Voicing median filter length. 3 taps cost ~0.3 PESQ at 2 % BER. |
| `voicing_smooth_er_threshold` | 0 | Error rate above which voicing smoothing applies. |
| `amp_smooth` | 0 (*off*) | Pull each harmonic's level toward the previous frame. Raised DNSMOS but lowered PESQ; not preferred by ear. |
| `aper_max`, `aper_f1`, `aper_f2` | 0 (*off*), 2000, 3500 | Share of voiced power above `aper_f1` rendered as noise. Reduced measured high-band periodicity; no audible difference. |
| `onset_ramp_mode` | 0 (*off*) | Full-frame ramps for harmonics that start or stop. Worse on every metric. |
| `repeat_amplitude_decay` | 1.0 | Amplitude decay per consecutive repeated frame. 0.85 made no measurable difference. |
| `mute_er` | 0.0875 | Mute when the smoothed error rate exceeds this (TIA §7.8). |
| `repeat_e0` | 3 | Repeat when E0 ≥ this (TIA: 2). |
| `repeat_et_base`, `repeat_et_slope` | 10, 40 | Repeat when ET ≥ base + slope × error rate (TIA §7.7). |
| `max_repeats` | 4 | Consecutive repeats before muting (TIA). |

The fixed-point decoder has no parameters; `imbe_decode_checked()` uses
E0 ≥ 3 and TIA's other thresholds.

---

## Measurements

### Method

- **Lab corpus:** 11 Open Speech Repository Harvard-sentence recordings
  (8 kHz, male and female, ~8 min), encoded with the OP25 fixed-point IMBE
  encoder, passed through the real P25 voice FEC with bit errors, and
  decoded exactly as `p25p1_fdma` does. Channels: random errors at 0–4 %
  BER, and two fading channels with per-frame good/bad states — A: 0.2 % /
  8 % (≈0.7 % average), B: 0.5 % / 15 % (≈3.9 % average). For scale, dcfd
  averages about 0.2–0.4 corrected errors per frame and wmata about 0.8.
- **Live traffic:** 80 transmissions (40 dcfd, 40 wmata) captured as IMBE
  frames from the running recorders and decoded directly, so every decoder
  saw exactly the frames the radios sent.
- **Metrics:** PESQ-NB (P.862) and STOI against the clean original;
  DNSMOS P.835 (no reference needed, used for live traffic); plus custom
  measures of frame-boundary spectral flux, pitch-pulse jitter, high-band
  periodicity and per-harmonic frequency jitter.
- **Listening tests** on live traffic decided the final settings. The
  metrics missed some of what was audible, in both directions.

The evaluation tools were built for this work and are not kept in the tree.

### Results

PESQ-NB on the lab corpus, mean of 11 files:

| Decoder | clean | 1 % | 2 % | 4 % | fading A | fading B |
|---|---|---|---|---|---|---|
| master fixed-point (`softVocoder: false`) | 3.135 | 3.115 | 3.033 | 2.483 | 2.766 | 1.614 |
| master float (`softVocoder: true`) | 3.010 | 2.909 | 2.635 | 2.111 | 2.762 | 1.787 |
| first version of this branch, float | 2.605 | 2.563 | 2.325 | 1.916 | 2.421 | 1.701 |
| this branch, fixed-point | 3.135 | 3.115 | 3.016 | 2.666 | 2.905 | 1.922 |
| this branch, float, fixes only (items 1–8) | 3.132 | 3.093 | 3.021 | 2.721 | 2.954 | 1.949 |
| + smooth synthesis (items 10–11) | 3.089 | 3.049 | 2.989 | 2.714 | 2.925 | 1.961 |

The first version of this branch had a formant postfilter on by default,
which was its largest regression (−0.44 PESQ).

Float decoder synthesis stages, lab corpus and 80 live transmissions:

| | PESQ clean | PESQ fading A | DNSMOS live | DNSMOS SIG live | frame-boundary flux, live (dB) |
|---|---|---|---|---|---|
| master float | 3.010 | 2.762 | 2.497 | 2.859 | 0.74 |
| fixes, TIA synthesis | 3.132 | 2.954 | 2.488 | 2.873 | 0.66 |
| + all harmonics interpolated | 3.135 | 2.971 | 2.557 | 2.944 | 0.42 |
| + smooth unvoiced synthesis | 3.089 | 2.925 | 2.560 | 2.956 | 0.34 |
| + `phase_c_env` 0.7 | 3.101 | 2.937 | 2.565 | 2.959 | 0.48 |
| + 3 dB presence lift (**current**) | 3.100 | 2.936 | 2.572 | 2.964 | 0.47 |

Frame-boundary flux is how unevenly the spectrum changes within each 20 ms
frame (natural speech ≈ 0.1 dB). The smooth unvoiced path costs about 0.04
PESQ on the lab corpus but scored better on live traffic and removes the
blotchy, stepped look of the TIA noise; set `uv_synth_mode = 0` to revert
it. `phase_c_env` 0.7 also cut pitch-pulse timing jitter on live traffic
from 5.6 % to 3.8 %.

### Other IMBE decoders

The same live frames were decoded with other open-source IMBE decoders,
loudness-matched, and compared by measurement and by ear:

| Decoder | PESQ clean | PESQ fading A | DNSMOS live | pulse jitter | freq. jitter < / > 1.5 kHz |
|---|---|---|---|---|---|
| this branch (`phase_c_env` 0.7) | 3.101 | 2.937 | 2.565 | 3.9 % | 3.4 / 5.7 Hz |
| [JMBE](https://github.com/DSheirer/jmbe) (SDRTrunk) | 3.118 | 2.753 | 2.494 | 2.2 % | 1.2 / 3.4 Hz |
| [blip25-vocoder](https://github.com/OpenBLIP25/blip25-vocoder) (DVSI-derived) | 3.112 | 2.842 | 2.446 | 5.0 % | 4.3 / 6.4 Hz |
| [mbelib-neo](https://github.com/arancormonk/mbelib-neo) (DSD-FME) | 2.477 | 2.408 | 2.668 | 5.3 % | 5.0 / 6.6 Hz |
| [mbelib](https://github.com/szechyjs/mbelib) (DSD) | 2.880 | 2.468 | 2.512 | 7.1 % | 1.6 / 3.4 Hz |
| [GopherTrunk](https://github.com/MattCheramie/GopherTrunk) | 2.063 | 1.952 | 1.962 | — | — |

JMBE only receives re-encoded clean frames here, so it cannot use the
channel's error counts (hence its lower fading score). By ear, this
branch was among the best, blip25 sounded slightly crisper (hence the
presence lift), mbelib was consistently worst, and GopherTrunk's current
decoder clicked and dropped out. blip25-vocoder is reverse-engineered from
a DVSI image and licensed for research and interoperability study only; it
was used as a listening reference, not as a source for this code.

---

## Findings from live traffic

- **wmata sounds "buzzy" because of how its radios encode.** Its radios
  send brighter audio (spectral tilt −6.7 vs −12.7 dB/kHz on dcfd) and mark
  56 % of loud vowels fully voiced up to 3.7 kHz (dcfd 36 %), so the top
  octave decodes as a pure pulse train. The DVSI-derived decoder is just as
  periodic there on the same frames.
- **The fast "warble" is in the transmitted parameters.** Every decoder
  above warbled about equally on the same wmata frames. Smoothing the pitch
  or amplitude tracks, or removing this decoder's per-frame phase steering,
  did not help audibly.
- **Channel errors are a minor factor.** wmata has about 4× dcfd's
  corrected-bit rate, but only ~2 % of frames are repeated or muted on
  either system. Most repeats and mutes fall in the last few frames of a
  transmission, where the radio has unkeyed mid-superframe and the
  remaining frames are noise.

---

## Tried and not adopted

- Formant postfilter (`fmt_alpha` 0.1–0.45, `fmt_w` 3–7): worse on every
  metric.
- Voicing median smoothing (`voicing_smooth_taps` 3): −0.3 PESQ at 2 % BER.
- A noise floor inside voiced bands (0.05–0.2 of band amplitude): worse on
  every metric.
- Full-frame onset/offset ramps (`onset_ramp_mode`): worse on every metric.
- Cross-frame amplitude smoothing (`amp_smooth`), high-band aperiodicity
  (`aper_max`), partial phase steering (`phase_track`), and centred pitch or
  amplitude smoothing with a frame of lookahead: measurable changes, no
  audible improvement in listening tests.
- Raising the ET repeat threshold (6 or 14 instead of 10) or decaying
  repeated frames (`repeat_amplitude_decay` 0.85): no consistent
  difference.

## Possible follow-ups

| Idea | Mechanism | Likely impact |
|---|---|---|
| Mute noise frames at transmission end | Frames with ET ≥ 12 are random bits; muting them at once instead of repeating up to 3 times avoids a held syllable at unkey. | Cleaner transmission endings. |
| Centred pitch median | One frame of lookahead (20 ms delay) to catch octave errors; needs amplitudes re-sampled to the corrected harmonic grid. | Fewer isolated pitch glitches. |
| Sub-frame interpolation | Synthesize 2–3 sub-frames per 20 ms frame (US6131084). | Smoother sustained vowels. |
| Soft-decision FEC | Use symbol reliabilities in the Golay/Hamming decoders. | Fewer bad frames on marginal signals. |

---

## References

- **TIA-102.BABA-A** — Project 25 IMBE vocoder description: frame repeat
  (§7.7), muting (§7.8), spectral enhancement and adaptive smoothing, and
  the baseline synthesis.
- **US5701390** (DVSI, expired 2015) — Synthesis of MBE-based coded speech
  using regenerated phase information.
  <https://patents.google.com/patent/US5701390>
- **US6131084** (DVSI, expired ~2017) — Dual subframe quantization of
  spectral magnitudes (sub-frame interpolation synthesis).
  <https://patents.google.com/patent/US6131084>
- **US6912496** (DVSI, expired 2023) — Preprocessing modules for quality
  enhancement of MBE coders and decoders (voicing smoothing).
  <https://patents.google.com/patent/US6912496>
- **US6963833** (DVSI, expired 2022) — Modifications in the MBE model for
  generating high-quality speech at low bit rates (UV→V phase reset).
  <https://patents.google.com/patent/US6963833>
- **US5241650** (Motorola, expired ~2009) — Digital speech decoder having a
  postfilter with reduced spectral distortion.
  <https://patents.google.com/patent/US5241650>
