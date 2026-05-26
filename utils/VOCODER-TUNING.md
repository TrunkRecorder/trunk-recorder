# Vocoder Tuning Toolchain

A workflow for tuning the IMBE vocoder synthesis parameters
(`VocoderParams` in `lib/op25_repeater/lib/software_imbe_decoder.h`) without
relying on live RF — capture once, decode many times offline.

The motivation: when you change a tuning knob and listen to the next live
call, the audio is *also* different because it's a different speaker, words,
RF conditions, and talkgroup. That makes A/B comparisons meaningless. This
toolchain captures the raw IMBE frames once, then re-decodes the same bits
through arbitrary parameter combinations so the only thing that varies is
the synthesis.

For the patent-derived rationale behind each knob, see
[`docs/Notes/VOCODER-IMPROVEMENTS.md`](../docs/Notes/VOCODER-IMPROVEMENTS.md).

---

## Quick start

```bash
# 1. Capture (set env var, run trunk-recorder, let it pick up a few real calls)
OP25_IMBE_CAPTURE_DIR=/tmp/imbe ./build/trunk-recorder -c your-config.json

# 2. Pick 2-3 representative .imbe files and sweep each into its own dir
./build/imbe_tune --input /tmp/imbe/p25imbe_tg744_<ts>.imbe \
                  --sweep utils/sweep.json --output-dir /tmp/sw1
./build/imbe_tune --input /tmp/imbe/p25imbe_tg100_<ts>.imbe \
                  --sweep utils/sweep.json --output-dir /tmp/sw2
./build/imbe_tune --input /tmp/imbe/p25imbe_tg704_<ts>.imbe \
                  --sweep utils/sweep.json --output-dir /tmp/sw3

# 3. Objective ranking - which combo's metrics look most like natural speech
python3 utils/rank_sweep.py --detail /tmp/sw1 /tmp/sw2 /tmp/sw3

# 4. Subjective ranking - blind A/B with Elo scoring (this is the ear truth)
python3 utils/blind_compare.py --rounds 60 /tmp/sw1 /tmp/sw2 /tmp/sw3
```

The top combo from steps 3 and 4 should agree. Apply the winning params
either by editing `VocoderParams` defaults in
`lib/op25_repeater/lib/software_imbe_decoder.h` or at runtime via
`software_imbe_decoder::set_params()`.

---

## The tools, in pipeline order

### 1. IMBE capture (built into trunk-recorder)

Set `OP25_IMBE_CAPTURE_DIR` before launching trunk-recorder. While set,
every P25 Phase 1 call writes a binary `.imbe` file alongside its `.wav`
into the named directory. Capture has zero impact when the env var isn't
set (one `getenv` at startup).

```bash
mkdir -p /tmp/imbe
OP25_IMBE_CAPTURE_DIR=/tmp/imbe ./build/trunk-recorder -c config.json 2>&1 \
    | grep "IMBE capture"
```

You should see `[IMBE capture] enabled, dir=/tmp/imbe` at startup, then one
`[IMBE capture] -> /tmp/imbe/p25imbe_tg<tgid>_<epoch_ms>.imbe` per call. If
fopen fails (missing dir, perms), you'll see a one-shot error and the
capture self-disables until process restart.

Only works on P25 **Phase 1** (FDMA). Phase 2 (TDMA), DMR, NXDN and YSF go
through different code paths.

#### `.imbe` binary file format

| Offset | Size | Field |
|--------|------|-------|
| 0      | 8    | magic `"P25IMBE\0"` |
| 8      | 4    | uint32 version (=1) |
| 12     | 4    | uint32 reserved |
| 16+40·n | 40  | record n: `u[0..7]` (8 × uint32) + `E0` (uint32) + `ET` (uint32) |

50 frames per second of decoded audio. So a 10-second call is ≈ 20 KB.

### 2. `imbe_tune` (built by CMake as part of the normal trunk-recorder build)

Sweeps `VocoderParams` combinations against one captured `.imbe` file,
producing one `.wav` and one `.json` sidecar per combination plus a
`summary.csv`.

```bash
./build/imbe_tune \
    --input  /tmp/imbe/p25imbe_tg744_<ts>.imbe \
    --sweep  utils/sweep.json \
    --output-dir /tmp/sw1
```

Decodes are fast — a 10 s call swept across 24 combos finishes in well
under a minute. Output structure:

```
/tmp/sw1/
├── combo_0000.wav   # decoded with combo 0's params
├── combo_0000.json  # exact param values used (the "identity" for ranking)
├── combo_0001.wav
├── combo_0001.json
├── ...
└── summary.csv      # one row per combo with params + quick in-tool metrics
```

The sidecar JSON is the canonical record of which parameter set produced
that WAV — `rank_sweep.py` and `blind_compare.py` both key off it.

### 3. `utils/sweep.json`

The sweep specification. Each top-level key is a `VocoderParams` field
name; its array is the values to try. The tool runs the cartesian product
of every listed dimension. Knobs not listed keep their `VocoderParams`
defaults.

```json
{
  "_about": "Lines starting with _ are ignored - use them for comments.",
  "fmt_alpha":              [0.18, 0.22, 0.28],
  "fmt_w":                  [3, 5],
  "phase_low_blend":        [0.70, 0.85],
  "repeat_amplitude_decay": [0.85, 0.95]
}
```

All `VocoderParams` fields are sweepable: `fmt_alpha`, `fmt_w`,
`phase_c_env`, `phase_w_rand`, `phase_low_blend`, `phase_kernel_d`,
`phase_kernel_gamma`, `voicing_smooth_taps`, `uv_to_v_reset`,
`interp_max_l`, `interp_pitch_tol`, `repeat_amplitude_decay`.

Sizing: pick total combo count to suit downstream use. ~20–30 combos is a
good sweet spot — fast to decode, enough variety for blind A/B but not so
many that no combo gets enough Elo rounds to stabilize.

### 4. `analyze_vocoder.py`

Objective audio-quality measurement on one or many WAV files. Aggregates
across **all** files into a single overall reading. Useful for "is the
system as a whole producing healthy-sounding audio."

```bash
python3 utils/analyze_vocoder.py /tmp/wavs/*.wav
```

Reports five metrics each graded against natural-speech target ranges:

| Metric             | Target      | What too-high / too-low means |
|--------------------|-------------|-------------------------------|
| crest factor       | 4.0 – 6.5   | high = buzzy / glottal-aligned; low = flat / over-randomized |
| spectral flatness  | 0.10 – 0.30 | high = no formant structure; low = heavily peaked |
| formant/valley     | 3.0 – 6.0   | high = over-sharp peaks (tinny); low = weak formants |
| spectral tilt      | -12 to -6   | flatter than -6 = too bright; steeper = too dull |
| HF energy variance | ≤ 0.6       | high = voicing chatter or fricative-heavy content |

If metrics drift, the tool suggests which `VocoderParams` knob to nudge.

### 5. `rank_sweep.py`

Objective audio-quality measurement on a sweep, **grouped by parameter
combination**. Ranks combos by an objective "distance from healthy range"
score. This is the script for "which combo is best."

```bash
python3 utils/rank_sweep.py --detail /tmp/sw1 /tmp/sw2 /tmp/sw3
```

Pass one or more `imbe_tune` output directories. The same combo present
in multiple directories (from different source `.imbe` files) has its
metrics averaged, so cross-source robustness is rewarded. Output ends
with a copy-pasteable C++ snippet for the winning combo:

```
=== Best combo (3 wavs, 3 source(s), score=0.124) ===

   params.fmt_alpha = 0.22f;
   params.fmt_w = 5;
   params.phase_low_blend = 0.85f;
   params.repeat_amplitude_decay = 0.85f;
```

The composite scoring weights are at the top of `rank_sweep.py` in
`METRIC_WEIGHTS`. Edit if you want to chase a different objective.

### 6. `blind_compare.py`

Subjective A/B comparison with hidden labels and Elo ratings. Plays a
random pair of WAVs (always from the same source `.imbe` so you're not
comparing different speech), takes a vote, updates per-combo Elo. After
N rounds prints a ranking by Elo.

```bash
python3 utils/blind_compare.py --rounds 60 /tmp/sw1 /tmp/sw2 /tmp/sw3
```

Controls during a round:

| Key | Action |
|-----|--------|
| `<enter>` | replay current / advance |
| `a`, `b`  | vote |
| `t`       | tie / can't tell |
| `r`       | replay both A and B |
| `s`       | skip (no vote) |
| `q`       | quit and print ranking |

State persists to `./blind_compare_state.json` after every round.
`--resume` to continue, `--rank-only` to inspect without playing more.

Requires an audio player on PATH: `afplay` (macOS), `paplay`
(PulseAudio), or `aplay` (ALSA).

### 7. `fetch_recent_wavs.py`

Helper to pull recent recordings off a remote machine running
trunk-recorder. Not strictly part of the tuning pipeline but useful when
the recorder runs on a separate box.

```bash
./utils/fetch_recent_wavs.py --today -n 25 --local-dir /tmp/wavs
# then:
python3 utils/analyze_vocoder.py /tmp/wavs/*.wav
```

Defaults to today's date directory under the configured base path. Uses
ssh + scp.

---

## Interpreting results

### When objective and subjective agree

You have a clear winner. Apply the params, move on.

### When they disagree

The objective score chases natural-speech *statistics*. The ear catches
artifacts the statistics miss (e.g., a sustained tone that sounds bad but
doesn't move the average crest much). Trust the ear-truth for borderline
cases. The objective ranking is best used to **narrow down** to a top
3-5, then blind-A/B the finalists.

### When objective is good but the audio still sounds off

Two common scenarios:

1. **Different artifact than the metrics measure.** HF chatter, comb
   filtering, voicing chatter at phoneme edges, transient clicks at
   call boundaries. Some of these are visible in a spectrogram (audacity,
   sox spectrogram) but not in the 5 aggregate metrics.

2. **You're at the IMBE model's perceptual floor.** The codec is a
   1980s vocoder — there's a quality ceiling. Past a certain point,
   tweaking parameters just trades one mild artifact for another.

### When you've nailed it

If `analyze_vocoder.py` shows all 5 metrics in their target ranges *and*
`blind_compare.py` ranks the top combo at >50 Elo above #2 after 60
rounds, you're at a good operating point. Commit the values to
`VocoderParams` defaults.

---

## Tips for capturing good sources

- **Variety matters more than length.** 3 × 10 s calls with different
  speakers beats 1 × 60 s call from one talkgroup. Different pitch
  ranges and voices stress different knobs.
- **Pick clean RF.** A call captured with high ER frames will mostly
  measure the gating logic, not the synthesis you're trying to tune.
  Look at the `summary.csv` from `imbe_tune` — combos with high mute
  rate aren't useful comparisons.
- **Pick mid-call clips.** The first 100 ms of a call has voicing-onset
  transients that vary independently of the steady-state quality.
  Within reason, this works itself out across multiple calls.

---

## Related files

- [`docs/Notes/VOCODER-IMPROVEMENTS.md`](../docs/Notes/VOCODER-IMPROVEMENTS.md)
  — patent-derived rationale and tuning knob reference
- [`lib/op25_repeater/lib/software_imbe_decoder.h`](../lib/op25_repeater/lib/software_imbe_decoder.h)
  — `VocoderParams` struct definition + per-field comments
- [`lib/op25_repeater/lib/software_imbe_decoder.cc`](../lib/op25_repeater/lib/software_imbe_decoder.cc)
  — the long-form "AUDIO TUNING PARAMETERS" comment block at the top
- [`lib/op25_repeater/lib/p25p1_fdma.cc`](../lib/op25_repeater/lib/p25p1_fdma.cc)
  — IMBE capture hook (search for `capture_dir_`)
