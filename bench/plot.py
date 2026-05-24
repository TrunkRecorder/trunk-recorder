#!/usr/bin/env python3
"""Generate plots from an overnight bench session.

Produces three PNGs in the session directory:
  - power_vs_recorders.png   scatter + regression lines, the headline plot
  - predicted_power.png      side-by-side bars at N=0,1,2,4,8 active recorders
  - power_timeline.png       per-window median power across the night

Usage:
  pip install matplotlib numpy
  python3 bench/plot.py bench/runs/<session-dir>
"""
import argparse
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")  # No display needed; we just write PNGs.
import matplotlib.pyplot as plt

# Re-use the load logic from analyze.py rather than duplicate it.
sys.path.insert(0, str(Path(__file__).parent))
from analyze import load_window, linear_fit, SETTLE_SEC_DEFAULT

COLOR_BASELINE = "#d62728"  # red
COLOR_TUNED = "#2ca02c"     # green


def collect(session_dir: Path, settle_sec: int):
    by_branch = defaultdict(list)
    per_window = []
    for wd in sorted(session_dir.glob("window-*")):
        data = load_window(wd, settle_sec)
        if data is None:
            continue
        by_branch[data["branch"]].append(data)
        per_window.append({
            "name": wd.name,
            "branch": data["branch"],
            "med_W": float(np.median(data["power_W"])),
            "med_nrec": float(np.median(data["n_recording"])),
        })
    return by_branch, per_window


def plot_scatter(by_branch, out_path: Path):
    fig, ax = plt.subplots(figsize=(9, 6))

    fits = {}
    for branch, color in [("baseline", COLOR_BASELINE), ("tuned", COLOR_TUNED)]:
        if branch not in by_branch:
            continue
        xs = np.concatenate([d["n_recording"] for d in by_branch[branch]])
        ys = np.concatenate([d["power_W"]     for d in by_branch[branch]])

        # Jitter x so overlapping integer values are visible.
        x_jit = xs + np.random.default_rng(42 if branch == "baseline" else 17) \
                       .uniform(-0.12, 0.12, size=len(xs))
        ax.scatter(x_jit, ys, s=6, alpha=0.15, color=color, label=None)

        idle, slope = linear_fit(xs, ys)
        fits[branch] = (idle, slope)

        x_line = np.linspace(xs.min(), xs.max(), 100)
        ax.plot(x_line, idle + slope * x_line, color=color, linewidth=2.5,
                label=f"{branch}: {idle:.2f} + {slope:.3f}·N")

    ax.set_xlabel("Active recorders (N)")
    ax.set_ylabel("Package power (W)")
    ax.set_title("Power vs active recorder count\n(scatter = all samples, line = linear fit)")
    ax.legend(loc="upper left", framealpha=0.9)
    ax.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return fits


def plot_predicted(fits, out_path: Path):
    n_values = [0, 1, 2, 4, 8]
    branches = list(fits.keys())
    colors = {"baseline": COLOR_BASELINE, "tuned": COLOR_TUNED}

    fig, ax = plt.subplots(figsize=(10, 6))
    width = 0.35
    x = np.arange(len(n_values))

    for i, branch in enumerate(branches):
        idle, slope = fits[branch]
        ys = [idle + slope * n for n in n_values]
        bars = ax.bar(x + (i - 0.5) * width, ys, width,
                      label=branch, color=colors.get(branch, "#888"))
        for bar, y in zip(bars, ys):
            ax.text(bar.get_x() + bar.get_width() / 2, y + 0.3,
                    f"{y:.1f}", ha="center", fontsize=9)

    # Annotate percentage savings at each N (assumes exactly two branches).
    if len(branches) == 2 and "baseline" in fits and "tuned" in fits:
        ib, sb = fits["baseline"]
        it, st = fits["tuned"]
        for i, n in enumerate(n_values):
            y_base = ib + sb * n
            y_tuned = it + st * n
            pct = 100 * (y_tuned - y_base) / y_base if y_base else 0
            ax.text(x[i], max(y_base, y_tuned) + 1.5, f"{pct:+.1f}%",
                    ha="center", fontsize=11, fontweight="bold",
                    color="#2ca02c" if pct < 0 else "#d62728")

    ax.set_xticks(x)
    ax.set_xticklabels([str(n) for n in n_values])
    ax.set_xlabel("Active recorders (N)")
    ax.set_ylabel("Predicted package power (W)")
    ax.set_title("Predicted power at various recorder counts\n(from pooled linear fit)")
    ax.legend()
    ax.grid(alpha=0.3, axis="y")

    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_timeline(per_window, out_path: Path):
    fig, ax = plt.subplots(figsize=(12, 5))

    idx_baseline = [i for i, w in enumerate(per_window) if w["branch"] == "baseline"]
    idx_tuned    = [i for i, w in enumerate(per_window) if w["branch"] == "tuned"]
    y_baseline = [per_window[i]["med_W"] for i in idx_baseline]
    y_tuned    = [per_window[i]["med_W"] for i in idx_tuned]

    ax.bar([i + 1 for i in idx_baseline], y_baseline, color=COLOR_BASELINE,
           label="baseline", alpha=0.9)
    ax.bar([i + 1 for i in idx_tuned], y_tuned, color=COLOR_TUNED,
           label="tuned", alpha=0.9)

    # Annotate windows where any recording happened.
    for i, w in enumerate(per_window):
        if w["med_nrec"] > 0:
            ax.annotate(f"N={int(w['med_nrec'])}",
                        xy=(i + 1, w["med_W"]),
                        xytext=(0, 5), textcoords="offset points",
                        ha="center", fontsize=8)

    ax.set_xlabel("Window number (chronological, A/B/A/B...)")
    ax.set_ylabel("Median package power (W)")
    ax.set_title("Per-window median power\n(label = median active recorders in that window)")
    ax.legend()
    ax.grid(alpha=0.3, axis="y")

    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("session_dir", type=Path)
    ap.add_argument("--settle-sec", type=int, default=SETTLE_SEC_DEFAULT)
    args = ap.parse_args()

    if not args.session_dir.exists():
        print(f"No such session dir: {args.session_dir}", file=sys.stderr)
        return 1

    print(f"Loading windows from {args.session_dir}…")
    by_branch, per_window = collect(args.session_dir, args.settle_sec)
    if not by_branch:
        print("No windows loaded.", file=sys.stderr)
        return 1

    out1 = args.session_dir / "power_vs_recorders.png"
    out2 = args.session_dir / "predicted_power.png"
    out3 = args.session_dir / "power_timeline.png"

    fits = plot_scatter(by_branch, out1)
    plot_predicted(fits, out2)
    plot_timeline(per_window, out3)

    print(f"\nWrote:\n  {out1}\n  {out2}\n  {out3}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
