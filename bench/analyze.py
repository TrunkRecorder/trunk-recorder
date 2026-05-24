#!/usr/bin/env python3
"""Analyze an overnight bench session.

Walks bench/runs/<session>/window-*/ directories, joins each window's bench
CSV with its powermetrics plist by timestamp, and reports for each branch:

  power(t) ~= idle_W + per_recorder_W * n_recording(t)

with bootstrap confidence intervals. Also prints per-window medians as a
sanity check against the pooled fit.

Usage: python3 analyze.py <session-dir>
"""
import argparse
import os
import plistlib
import re
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

SETTLE_SEC_DEFAULT = 10


def parse_powermetrics(plist_path: Path):
    """powermetrics -f plist writes a stream of plist documents back-to-back.

    Yields (epoch_ms, package_W, per_pid_W_dict) for each sample.
    package_W is the global package power; per_pid_W_dict maps pid -> watts
    estimated from cputime_ms_per_s of that task (a rough but consistent proxy).
    """
    raw = plist_path.read_bytes()
    # Split on the </plist> closing tag, keep the tag with each chunk.
    chunks = re.split(rb"(?<=</plist>)\s*", raw)
    for chunk in chunks:
        chunk = chunk.strip()
        if not chunk:
            continue
        # powermetrics sometimes prefixes a NUL byte between samples.
        chunk = chunk.lstrip(b"\x00")
        if not chunk.startswith(b"<?xml"):
            continue
        try:
            doc = plistlib.loads(chunk)
        except Exception:
            continue

        ts = doc.get("timestamp")
        epoch_ms = None
        if isinstance(ts, (int, float)):
            epoch_ms = int(ts * 1000)
        elif "hw_model" in doc and "elapsed_ns" in doc:
            # Fallback: use 'timestamp' field, ms since epoch as int.
            epoch_ms = int(doc.get("timestamp", 0))

        processor = doc.get("processor", {})
        # On Apple Silicon, prefer combined_power; fall back to package_joules
        # divided by interval if needed.
        pkg_mw = processor.get("combined_power") \
            or processor.get("package_watts", 0) * 1000 \
            or processor.get("cpu_power", 0)
        # combined_power is reported in mW on AS.
        pkg_W = float(pkg_mw) / 1000.0 if pkg_mw else 0.0

        per_pid = {}
        # tasks section has per-process info when --show-process-energy is set.
        for task in doc.get("tasks", []):
            pid = task.get("pid")
            if pid is None:
                continue
            # cputime_ms_per_s is the share of a single CPU-second this task
            # used. Multiply by package_W / total_cpu_busy to get attributed W.
            # Without knowing total busy, use cputime_ms_per_s directly as a
            # proxy (consistent across branches).
            cput_ms_per_s = task.get("cputime_ms_per_s", 0.0)
            per_pid[int(pid)] = float(cput_ms_per_s)

        if epoch_ms:
            yield epoch_ms, pkg_W, per_pid


def load_window(window_dir: Path, settle_sec: int):
    """Return aligned arrays (n_recording, package_W) for one window."""
    csv_path = window_dir / "bench_active.csv"
    pm_path = window_dir / "powermetrics.plist"
    if not csv_path.exists() or not pm_path.exists():
        return None

    start_epoch = int((window_dir / "start_epoch").read_text().strip())
    settle_until = (start_epoch + settle_sec) * 1000

    # Load bench CSV: epoch_ms,pid,n_active,n_recording,branch_tag
    bench = []
    branch_tag = None
    tr_pid = None
    with csv_path.open() as f:
        next(f, None)  # header
        for line in f:
            parts = line.strip().split(",")
            if len(parts) < 5:
                continue
            ts_ms = int(parts[0])
            pid = int(parts[1])
            n_recording = int(parts[3])
            tr_pid = pid
            branch_tag = parts[4]
            if ts_ms >= settle_until:
                bench.append((ts_ms, n_recording))

    if not bench:
        return None
    bench_ts = np.array([b[0] for b in bench])
    bench_nr = np.array([b[1] for b in bench])

    # Load powermetrics; align each PM sample to the nearest bench sample.
    aligned_nr = []
    aligned_pw = []
    for epoch_ms, pkg_W, _per_pid in parse_powermetrics(pm_path):
        if epoch_ms < settle_until:
            continue
        idx = np.searchsorted(bench_ts, epoch_ms)
        if idx >= len(bench_ts):
            idx = len(bench_ts) - 1
        elif idx > 0 and abs(bench_ts[idx - 1] - epoch_ms) < abs(bench_ts[idx] - epoch_ms):
            idx -= 1
        aligned_nr.append(bench_nr[idx])
        aligned_pw.append(pkg_W)

    if not aligned_pw:
        return None
    return {
        "branch": branch_tag,
        "n_recording": np.array(aligned_nr, dtype=float),
        "power_W": np.array(aligned_pw, dtype=float),
        "n_samples": len(aligned_pw),
    }


def linear_fit(x, y):
    """Return (intercept, slope) of y = intercept + slope * x via least squares."""
    if len(x) < 2 or np.allclose(x, x[0]):
        return float(np.mean(y)), 0.0
    slope, intercept = np.polyfit(x, y, 1)
    return float(intercept), float(slope)


def bootstrap_ci(x, y, n_iter=1000, alpha=0.05, rng=None):
    rng = rng or np.random.default_rng(42)
    n = len(x)
    intercepts = np.empty(n_iter)
    slopes = np.empty(n_iter)
    for i in range(n_iter):
        idx = rng.integers(0, n, n)
        intercepts[i], slopes[i] = linear_fit(x[idx], y[idx])
    lo, hi = 100 * alpha / 2, 100 * (1 - alpha / 2)
    return {
        "intercept": (np.percentile(intercepts, lo), np.percentile(intercepts, hi)),
        "slope":     (np.percentile(slopes, lo),     np.percentile(slopes, hi)),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("session_dir", type=Path)
    ap.add_argument("--settle-sec", type=int, default=SETTLE_SEC_DEFAULT)
    args = ap.parse_args()

    windows = sorted(args.session_dir.glob("window-*"))
    if not windows:
        print(f"No windows found under {args.session_dir}", file=sys.stderr)
        return 1

    by_branch = defaultdict(list)
    per_window = []
    for wd in windows:
        data = load_window(wd, args.settle_sec)
        if data is None:
            print(f"  skipped (no data): {wd.name}")
            continue
        by_branch[data["branch"]].append(data)
        idle, slope = linear_fit(data["n_recording"], data["power_W"])
        per_window.append((wd.name, data["branch"], data["n_samples"],
                           float(np.median(data["n_recording"])),
                           float(np.median(data["power_W"])), idle, slope))

    print(f"\nPer-window summary ({len(per_window)} windows):")
    print(f"{'window':<28} {'branch':<10} {'N':>5} {'med_nrec':>9} {'med_W':>7} {'idle_W':>8} {'per_rec':>9}")
    for row in per_window:
        print(f"{row[0]:<28} {row[1]:<10} {row[2]:>5} {row[3]:>9.2f} "
              f"{row[4]:>7.2f} {row[5]:>8.2f} {row[6]:>9.3f}")

    print("\nPooled fit per branch:")
    print(f"  power(t) = idle_W + per_recorder_W * n_recording(t)\n")
    for branch in sorted(by_branch):
        xs = np.concatenate([d["n_recording"] for d in by_branch[branch]])
        ys = np.concatenate([d["power_W"]     for d in by_branch[branch]])
        idle, slope = linear_fit(xs, ys)
        ci = bootstrap_ci(xs, ys)
        print(f"  {branch:>10s}: N={len(xs):>6d}  "
              f"idle = {idle:6.2f} W  [{ci['intercept'][0]:5.2f}, {ci['intercept'][1]:5.2f}]  "
              f"per_rec = {slope:6.3f} W  [{ci['slope'][0]:6.3f}, {ci['slope'][1]:6.3f}]")

    branches = sorted(by_branch)
    if len(branches) == 2:
        a, b = branches
        xa = np.concatenate([d["n_recording"] for d in by_branch[a]])
        ya = np.concatenate([d["power_W"]     for d in by_branch[a]])
        xb = np.concatenate([d["n_recording"] for d in by_branch[b]])
        yb = np.concatenate([d["power_W"]     for d in by_branch[b]])
        ia, sa = linear_fit(xa, ya)
        ib, sb = linear_fit(xb, yb)
        print(f"\nDelta ({b} - {a}):")
        print(f"  idle      : {ib - ia:+.2f} W  ({100 * (ib - ia) / ia:+.1f}%)")
        if abs(sa) > 1e-6:
            print(f"  per_rec   : {sb - sa:+.3f} W  ({100 * (sb - sa) / sa:+.1f}%)")
        else:
            print(f"  per_rec   : {sb - sa:+.3f} W")

    return 0


if __name__ == "__main__":
    sys.exit(main())
