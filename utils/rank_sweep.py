#!/usr/bin/env python3
"""
rank_sweep.py - Rank vocoder parameter combinations by objective audio metrics.

Reads one or more imbe_tune output directories. Groups WAVs by the
VocoderParams they were decoded with (from the .json sidecars), averages the
analyze_vocoder.py metrics across every instance of each combo (e.g., when
the same combo was run against multiple source .imbe files), and ranks the
combos by an objective "distance from healthy range" score.

This complements (does not replace) analyze_vocoder.py:
  - analyze_vocoder.py - aggregates ALL files into a single overall reading
  - rank_sweep.py      - groups by parameter set and finds the best combo

Usage:
    rank_sweep.py DIR [DIR ...]              # rank everything in those dirs
    rank_sweep.py --top 10 DIR1 DIR2         # show top 10 combos only
    rank_sweep.py --csv out.csv DIR          # also dump full ranking to CSV
    rank_sweep.py --detail DIR               # show per-metric grades, not just score

Requires: numpy, scipy.
"""

import argparse
import csv
import json
import os
import sys
from glob import glob

# Pull in the metric functions and target ranges from the sibling script.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyze_vocoder import (
    TARGETS,
    compute,
    load_audio,
)

try:
    import numpy as np
except ImportError as e:
    print(f"error: {e}", file=sys.stderr)
    print("       install with: pip3 install numpy scipy", file=sys.stderr)
    sys.exit(1)


# ---------------------------------------------------------------------------
# Combo identity

def combo_key(params):
    """Stable, sorted-JSON identity for a parameter combination."""
    return json.dumps(params, sort_keys=True, separators=(",", ":"))


def fmt_params(params):
    return "  ".join(f"{k}={v}" for k, v in sorted(params.items()))


# ---------------------------------------------------------------------------
# Scoring: distance from target range, normalized to range width

# Per-metric weight in the composite objective. Tuned so the metrics that have
# the strongest perceptual impact dominate; tweak here if you want a different
# objective. All metrics still contribute, but their pulls aren't equal.
METRIC_WEIGHTS = {
    "crest":     1.0,   # perceptual buzziness
    "sfm":       1.0,   # postfilter shape
    "fmt_ratio": 0.8,   # formant peakiness
    "tilt":      0.5,   # spectral balance
    "hf_var":    0.3,   # content-driven for radio dispatch, weight low
}


def distance_from_range(value, lo, hi):
    """0 if value is inside [lo, hi]; otherwise fraction of (hi-lo) outside.
    Returns NaN if value is NaN."""
    if value != value:
        return float("nan")
    width = hi - lo
    if width <= 0:
        return 0.0
    if value < lo:
        return (lo - value) / width
    if value > hi:
        return (value - hi) / width
    return 0.0


def composite_score(metrics):
    """Weighted sum of per-metric distances. Lower is better.
    NaN metrics are skipped (don't penalize or reward)."""
    total = 0.0
    used = 0.0
    for k, w in METRIC_WEIGHTS.items():
        if k not in metrics:
            continue
        d = distance_from_range(metrics[k], *TARGETS[k])
        if d != d:
            continue
        total += w * d
        used += w
    if used <= 0:
        return float("nan")
    return total / used


# ---------------------------------------------------------------------------
# Loading

def scan_one_dir(d):
    """Yield (wav_path, params_dict, source_dir) for each combo file in d."""
    for js in sorted(glob(os.path.join(d, "combo_*.json"))):
        wav = js[:-5] + ".wav"
        if not os.path.isfile(wav):
            continue
        try:
            with open(js) as f:
                side = json.load(f)
        except (OSError, json.JSONDecodeError):
            continue
        params = {k: v for k, v in side.items() if not k.startswith("_")}
        if not params:
            continue
        yield wav, params, d


def measure_one(wav_path):
    """Load + compute metrics for one WAV. Returns metrics dict (may have NaN)."""
    fs, audio = load_audio(wav_path)
    if len(audio) < fs:
        return None
    return compute(audio, fs)


# ---------------------------------------------------------------------------
# Aggregation per combo

def mean_skip_nan(values):
    vs = [v for v in values if v == v]
    return float(np.mean(vs)) if vs else float("nan")


def aggregate(per_wav):
    """Given list of (wav_path, params, source, metrics), group by combo and
    return list of dicts with averaged metrics + score."""
    by_combo = {}
    for wav, params, source, metrics in per_wav:
        k = combo_key(params)
        rec = by_combo.setdefault(k, {
            "key": k,
            "params": params,
            "wavs": [],
            "sources": set(),
            "metrics_list": [],
        })
        rec["wavs"].append(wav)
        rec["sources"].add(source)
        rec["metrics_list"].append(metrics)

    out = []
    for k, rec in by_combo.items():
        keys = ["crest", "sfm", "fmt_ratio", "tilt", "hf_var", "duration"]
        avg = {kk: mean_skip_nan([m[kk] for m in rec["metrics_list"] if kk in m])
               for kk in keys}
        rec["metrics_avg"] = avg
        rec["score"] = composite_score(avg)
        rec["n"] = len(rec["wavs"])
        rec["sources_n"] = len(rec["sources"])
        out.append(rec)

    out.sort(key=lambda r: (r["score"] if r["score"] == r["score"] else float("inf")))
    return out


# ---------------------------------------------------------------------------
# Formatting

def grade_short(value, lo, hi):
    """Short status for a single metric: 'ok', 'lo', 'hi', or '?'."""
    if value != value:
        return "?"
    if value < lo:
        return "lo"
    if value > hi:
        return "hi"
    return "ok"


def print_summary(ranked, top, detail):
    print(f"\n=== Ranked combos ({len(ranked)} total, showing top {min(top, len(ranked))}) ===")
    print("score = weighted distance from target ranges (lower = better)")
    print()
    header = f"{'#':>3} {'score':>6} {'wavs':>5} {'src':>4}   {'crest':>5} {'sfm':>5} {'fmt':>4} {'tilt':>6} {'hfv':>4}   parameters"
    print(header)
    print("-" * len(header))
    for i, r in enumerate(ranked[:top], 1):
        m = r["metrics_avg"]
        mark = ""
        if detail:
            # Show grades next to numbers
            crest_s = f"{m['crest']:5.2f}" if m['crest'] == m['crest'] else "  nan"
            sfm_s   = f"{m['sfm']:5.3f}" if m['sfm'] == m['sfm'] else "  nan"
            fmt_s   = f"{m['fmt_ratio']:4.2f}" if m['fmt_ratio'] == m['fmt_ratio'] else " nan"
            tilt_s  = f"{m['tilt']:+6.2f}" if m['tilt'] == m['tilt'] else "   nan"
            hfv_s   = f"{m['hf_var']:4.2f}" if m['hf_var'] == m['hf_var'] else " nan"
        else:
            crest_s = f"{m['crest']:5.2f}"
            sfm_s   = f"{m['sfm']:5.3f}"
            fmt_s   = f"{m['fmt_ratio']:4.2f}"
            tilt_s  = f"{m['tilt']:+6.2f}"
            hfv_s   = f"{m['hf_var']:4.2f}"

        s = f"{i:3d} {r['score']:6.3f} {r['n']:5d} {r['sources_n']:4d}   {crest_s} {sfm_s} {fmt_s} {tilt_s} {hfv_s}   {fmt_params(r['params'])}"
        print(s)

    if detail and ranked:
        print()
        print("per-metric grades for best combo:")
        m = ranked[0]["metrics_avg"]
        for k in ("crest", "sfm", "fmt_ratio", "tilt", "hf_var"):
            tgt = TARGETS[k]
            g = grade_short(m[k], *tgt)
            print(f"  {k:10s} {m[k]:8.3f}  [{g}]  target {tgt[0]} .. {tgt[1]}")


def print_best(ranked):
    if not ranked:
        return
    best = ranked[0]
    print(f"\n=== Best combo ({best['n']} wavs, {best['sources_n']} source(s), score={best['score']:.3f}) ===")
    print()
    print("Apply by editing VocoderParams defaults in software_imbe_decoder.h or")
    print("at runtime via software_imbe_decoder::set_params():")
    print()
    for k, v in sorted(best["params"].items()):
        if isinstance(v, bool):
            vstr = "true" if v else "false"
        elif isinstance(v, float):
            vstr = f"{v}f"
        else:
            vstr = str(v)
        print(f"   params.{k} = {vstr};")


# ---------------------------------------------------------------------------

def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("dirs", nargs="+", help="imbe_tune output directories")
    p.add_argument("--top", type=int, default=10,
                   help="show this many combos (default: 10)")
    p.add_argument("--csv", default=None,
                   help="also write the full ranking to this CSV path")
    p.add_argument("--detail", action="store_true",
                   help="show per-metric grades for the best combo")
    args = p.parse_args()

    # Collect every (wav, params, source) tuple
    items = []
    for d in args.dirs:
        if not os.path.isdir(d):
            print(f"warning: skipping non-dir: {d}", file=sys.stderr)
            continue
        n = 0
        for wav, params, source in scan_one_dir(d):
            items.append((wav, params, source))
            n += 1
        if n == 0:
            print(f"warning: no combo_*.{{wav,json}} in {d}", file=sys.stderr)

    if not items:
        print("error: no usable wavs found", file=sys.stderr)
        sys.exit(1)

    # Measure each WAV
    per_wav = []
    skipped = 0
    print(f"measuring {len(items)} wav(s)...", file=sys.stderr)
    for wav, params, source in items:
        m = measure_one(wav)
        if m is None:
            skipped += 1
            continue
        per_wav.append((wav, params, source, m))
    if skipped:
        print(f"  (skipped {skipped} short/silent file(s))", file=sys.stderr)

    if not per_wav:
        print("error: no analyzable audio", file=sys.stderr)
        sys.exit(1)

    ranked = aggregate(per_wav)

    print_summary(ranked, args.top, args.detail)
    print_best(ranked)

    if args.csv:
        with open(args.csv, "w", newline="") as f:
            w = csv.writer(f)
            # Header: meta + metrics + every parameter we saw
            all_param_keys = sorted({k for r in ranked for k in r["params"]})
            w.writerow(["rank", "score", "n_wavs", "n_sources",
                        "crest", "sfm", "fmt_ratio", "tilt", "hf_var",
                        *all_param_keys])
            for rank, r in enumerate(ranked, 1):
                m = r["metrics_avg"]
                row = [rank, f"{r['score']:.4f}", r["n"], r["sources_n"],
                       f"{m['crest']:.3f}", f"{m['sfm']:.4f}",
                       f"{m['fmt_ratio']:.3f}", f"{m['tilt']:.3f}",
                       f"{m['hf_var']:.3f}"]
                for k in all_param_keys:
                    row.append(r["params"].get(k, ""))
                w.writerow(row)
        print(f"\nfull ranking written to {args.csv}")


if __name__ == "__main__":
    main()
