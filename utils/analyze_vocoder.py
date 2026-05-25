#!/usr/bin/env python3
"""
analyze_vocoder.py - Measure decoded-audio quality and suggest vocoder tuning.

Reads WAV files produced by trunk-recorder's P25 IMBE decoder (8 kHz, 16-bit
mono) and computes objective metrics that map back to the tuning knobs in
lib/op25_repeater/lib/software_imbe_decoder.cc. Prints per-file metrics, an
aggregate row when given multiple files, and concrete knob adjustments to try.

Metrics
-------
  Crest factor (peak / RMS)
        Natural speech: 4-6. >8 = buzzy / glottal-aligned (more phase regen
        needed). <3 = flat / over-randomized (less random phase).
  Spectral flatness measure (geometric mean / arithmetic mean of PSD)
        Natural speech: 0.10-0.30. High = no formant structure (need more
        postfilter). Low = heavily peaked.
  Formant / valley energy ratio
        Power in F1/F2 bands vs power in inter-formant valleys. Natural
        speech: 3-6. Low = weak formants. High = over-emphasized.
  Spectral tilt (dB / octave, 200-3500 Hz)
        Natural speech: -6 to -12 dB/oct (low end dominant). Flatter = too
        bright. Steeper = too dull.
  HF energy stability
        Frame-to-frame variance of 2-3.5 kHz band energy. High = voicing
        chatter or noisy frames.

Usage
-----
    ./analyze_vocoder.py recording.wav
    ./analyze_vocoder.py *.wav                # batch
    ./analyze_vocoder.py --quiet *.wav         # only aggregate + suggestions

Requires: numpy, scipy.  Install:  pip3 install numpy scipy
"""

import argparse
import os
import sys

try:
    import numpy as np
    from scipy import signal
    from scipy.io import wavfile
except ImportError as e:
    print(f"error: {e}", file=sys.stderr)
    print("       install with: pip3 install numpy scipy", file=sys.stderr)
    sys.exit(1)


# Target ranges for natural P25-bandlimited speech at 8 kHz.
TARGETS = {
    'crest':     (4.0, 6.5),
    'sfm':       (0.10, 0.30),
    'fmt_ratio': (3.0, 6.0),
    'tilt':      (-12.0, -6.0),  # dB / octave
    'hf_var':    (0.0, 0.6),
}


def load_audio(path):
    """Load WAV file, mix to mono, trim leading/trailing silence.

    Returns an empty array for files that are fully silent or essentially
    silent (peak below 0.05 % of full scale, ~ -66 dBFS). Caller should
    treat empty audio as 'skip' so an all-zero leftover file doesn't
    pollute the aggregate with crest=inf etc.
    """
    fs, audio = wavfile.read(path)
    if audio.ndim > 1:
        audio = audio.mean(axis=1)
    audio = audio.astype(np.float64)
    if len(audio) == 0:
        return fs, audio
    peak = float(np.max(np.abs(audio)))
    if peak < 16.0:                       # ~ -66 dBFS - file is silent
        return fs, np.array([], dtype=np.float64)
    mask = np.abs(audio) > peak * 0.005
    if mask.any():
        first = int(mask.argmax())
        last = len(audio) - int(mask[::-1].argmax())
        audio = audio[first:last]
    return fs, audio


def crest_factor(audio):
    if len(audio) == 0:
        return float('nan')
    rms = float(np.sqrt(np.mean(audio ** 2)))
    peak = float(np.max(np.abs(audio)))
    return peak / rms if rms > 0 else float('inf')


def spectral_flatness(audio, fs):
    if len(audio) < 256:
        return float('nan')
    f, pxx = signal.welch(audio, fs=fs, nperseg=256)
    band = (f >= 200) & (f <= 3500)
    p = pxx[band] + 1e-12
    return float(np.exp(np.mean(np.log(p))) / np.mean(p))


def spectral_tilt(audio, fs):
    """dB / octave slope of log-PSD vs log-freq, 200-3500 Hz."""
    if len(audio) < 512:
        return float('nan')
    f, pxx = signal.welch(audio, fs=fs, nperseg=512)
    band = (f >= 200) & (f <= 3500)
    log_f = np.log2(f[band])
    log_p = 10.0 * np.log10(pxx[band] + 1e-12)
    slope, _ = np.polyfit(log_f, log_p, 1)
    return float(slope)


def formant_ratio(audio, fs):
    """Mean PSD in F1/F2 bands vs inter-formant valleys."""
    if len(audio) < 256:
        return float('nan')
    f, pxx = signal.welch(audio, fs=fs, nperseg=256)
    f1 = (f > 300) & (f < 900)
    f2 = (f > 1200) & (f < 2400)
    v1 = (f > 900) & (f < 1200)
    v2 = (f > 2400) & (f < 3500)
    if not (f1.any() and f2.any() and v1.any() and v2.any()):
        return float('nan')
    formant = (float(np.mean(pxx[f1])) + float(np.mean(pxx[f2]))) / 2
    valley = (float(np.mean(pxx[v1])) + float(np.mean(pxx[v2]))) / 2
    return formant / (valley + 1e-12)


def hf_variance(audio, fs):
    """Std-dev of log10(HF band energy) across 20 ms frames."""
    frame = int(fs * 0.020)
    n = len(audio) // frame
    if n < 10:
        return float('nan')
    energies = []
    for i in range(n):
        seg = audio[i * frame:(i + 1) * frame]
        fft_mag = np.abs(np.fft.rfft(seg))
        freqs = np.fft.rfftfreq(len(seg), 1.0 / fs)
        band = (freqs > 2000) & (freqs < 3500)
        e = float(np.sum(fft_mag[band] ** 2))
        energies.append(np.log10(e + 1.0))
    return float(np.std(np.array(energies)))


def compute(audio, fs):
    return {
        'crest':     crest_factor(audio),
        'sfm':       spectral_flatness(audio, fs),
        'tilt':      spectral_tilt(audio, fs),
        'fmt_ratio': formant_ratio(audio, fs),
        'hf_var':    hf_variance(audio, fs),
        'duration':  len(audio) / fs if fs else 0.0,
    }


def grade(val, lo, hi):
    if val != val:
        return '?'
    if val < lo:
        return f'LO  (target {lo:.2f}-{hi:.2f})'
    if val > hi:
        return f'HI  (target {lo:.2f}-{hi:.2f})'
    return f'OK  (target {lo:.2f}-{hi:.2f})'


def suggest(m):
    """Return list of (knob, direction, reason) tuples."""
    out = []
    c, s, t, fr, hv = m['crest'], m['sfm'], m['tilt'], m['fmt_ratio'], m['hf_var']

    if c == c:
        if c > 8.5:
            out.append(('PHASE_C_ENV', 'up (+0.10)',
                        f"crest {c:.1f} - buzzy / glottal-aligned. Stronger envelope phase spreads alignment."))
            out.append(('PHASE_LOW_BLEND', 'up (+0.10)',
                        "Most buzz lives in low harmonics; raise envelope-phase weight there."))
        elif c < 3.0:
            out.append(('PHASE_C_ENV', 'down (-0.10)',
                        f"crest {c:.1f} - flat / over-randomized. Reduce envelope phase scaling."))
            if m.get('phase_w_rand_likely_high'):
                out.append(('PHASE_W_RAND', 'down (-0.10)',
                            "Also reduce residual random-phase weight."))

    if s == s:
        if s > 0.35:
            out.append(('FMT_ALPHA', 'up (+0.05)',
                        f"SFM {s:.2f} - little formant structure. Stronger postfilter sharpens peaks."))
        elif s < 0.08:
            out.append(('FMT_ALPHA', 'down (-0.05)',
                        f"SFM {s:.2f} - heavily peaked spectrum (likely tinny). Reduce postfilter."))

    if fr == fr:
        if fr < 2.0:
            out.append(('FMT_ALPHA', 'up (+0.05)',
                        f"formant/valley {fr:.1f} - weak; postfilter has room."))
        elif fr > 8.0:
            out.append(('FMT_ALPHA', 'down (-0.05)',
                        f"formant/valley {fr:.1f} - over-aggressive; likely sharp/sibilant."))

    if t == t:
        if t > -5.0:
            out.append(('FMT_ALPHA / FMT_W', 'review',
                        f"tilt {t:+.1f} dB/oct - spectrum too flat (no LF emphasis)."))
        elif t < -15.0:
            out.append(('FMT_ALPHA', 'down',
                        f"tilt {t:+.1f} dB/oct - too steep; HF attenuated."))

    if hv == hv and hv > 0.8:
        out.append(('VOICING_SMOOTH_TAPS', 'up (try 5)',
                    f"HF variance {hv:.2f} - chatter or noisy frames; longer voicing-smoothing window."))

    # Deduplicate same knob mentioned twice with same direction
    seen = set()
    deduped = []
    for k, d, r in out:
        key = (k, d)
        if key in seen:
            continue
        seen.add(key)
        deduped.append((k, d, r))
    return deduped


def print_metrics(label, m):
    print(f"\n=== {label}  ({m['duration']:.1f} s) ===")
    print(f"  crest factor       : {m['crest']:6.2f}    {grade(m['crest'],     *TARGETS['crest'])}")
    print(f"  spectral flatness  : {m['sfm']:6.3f}   {grade(m['sfm'],       *TARGETS['sfm'])}")
    print(f"  formant / valley   : {m['fmt_ratio']:6.2f}    {grade(m['fmt_ratio'], *TARGETS['fmt_ratio'])}")
    print(f"  spectral tilt      : {m['tilt']:+6.2f} dB/oct  {grade(m['tilt'], *TARGETS['tilt'])}")
    print(f"  HF energy variance : {m['hf_var']:6.2f}    {grade(m['hf_var'],    *TARGETS['hf_var'])}")


def print_suggestions(sugg):
    if not sugg:
        print("  -> no tuning changes suggested (metrics in healthy range).")
        return
    print("  -> tuning suggestions:")
    for knob, direction, reason in sugg:
        print(f"     - {knob:20s}  {direction}")
        print(f"         {reason}")


def aggregate(metrics_list):
    keys = ['crest', 'sfm', 'tilt', 'fmt_ratio', 'hf_var']
    agg = {}
    for k in keys:
        vals = [m[k] for m in metrics_list if m[k] == m[k]]
        agg[k] = float(np.mean(vals)) if vals else float('nan')
    agg['duration'] = sum(m['duration'] for m in metrics_list)
    return agg


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('files', nargs='+', help='WAV file(s)')
    parser.add_argument('--quiet', action='store_true',
                        help='skip per-file output; just print aggregate + suggestions')
    args = parser.parse_args()

    metrics_list = []
    for path in args.files:
        if not os.path.isfile(path):
            print(f"skip {path}: not a file", file=sys.stderr)
            continue
        try:
            fs, audio = load_audio(path)
        except Exception as e:
            print(f"skip {path}: {e}", file=sys.stderr)
            continue
        if len(audio) < fs:
            print(f"skip {path}: under 1 s of audio", file=sys.stderr)
            continue
        m = compute(audio, fs)
        metrics_list.append(m)
        if not args.quiet:
            print_metrics(os.path.basename(path), m)
            print_suggestions(suggest(m))

    if not metrics_list:
        print("no usable files", file=sys.stderr)
        sys.exit(1)

    if len(metrics_list) > 1:
        agg = aggregate(metrics_list)
        print_metrics(f"AGGREGATE ({len(metrics_list)} files)", agg)
        print_suggestions(suggest(agg))


if __name__ == '__main__':
    main()
