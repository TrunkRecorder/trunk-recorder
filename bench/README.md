# Bench: A/B compute comparison

Compare the power/compute cost of two trunk-recorder branches (`master` vs
`dev/fft-tuning`) against live RF, while normalizing for variable workload
(number of actively-recording recorders).

## What it measures

For each branch, fits

```
power(t) ~= idle_W + per_recorder_W * n_recording(t)
```

so we report two numbers per branch:

- `idle_W` — always-on cost (channelizer, control-channel decode, idle loops).
  The FFT/channelizer changes in `dev/fft-tuning` should mostly move this.
- `per_recorder_W` — marginal cost of one additional active recorder.

Reporting them separately is more informative than a single ratio: a single
"watts per recorder-second" number conflates fixed and variable cost.

## How variance is controlled

- **Deterministic-ish workload**: live RF, but interleaved 15-min windows
  (A/B/A/B…) so adjacent windows see similar traffic.
- **P-core pinning** via `taskpolicy -c user-interactive` (Apple Silicon
  schedules user-interactive QoS on P-cores), so we don't conflate
  heterogeneous cores.
- **Frequency-immune metric**: package power in watts, not CPU%.
- **Many windows**: ~32 windows over 8 hours = 16 per branch, plenty for
  bootstrap CIs.

## Files

- `plugins/bench_logger/` — minimal plugin that hooks `calls_active(...)` and
  writes `(epoch_ms, pid, n_active, n_recording, branch_tag)` to a CSV.
- `bench/setup_worktrees.sh` — one-time setup: creates two git worktrees
  (`../tr-baseline`, `../tr-fft-tuning`), copies the bench plugin into the
  baseline worktree, builds both.
- `bench/run_overnight.sh` — the overnight harness. Alternates the two binaries
  in 15-min windows, capturing bench CSV and `powermetrics` plist per window.
- `bench/analyze.py` — pools all windows per branch, fits the linear model,
  reports per-branch coefficients with bootstrap CIs and a delta summary.

## Running it

### One-time setup

From the repo root, on `dev/fft-tuning` (where the plugin lives):

```bash
bench/setup_worktrees.sh
```

This creates `../tr-baseline` (on `master`) and `../tr-fft-tuning` worktrees,
injects the bench plugin into the baseline, and builds both. Expect 10–30
minutes depending on CPU.

### Overnight run

`powermetrics` requires root. Either run with `sudo` or add a sudoers entry.

```bash
sudo bench/run_overnight.sh
```

Defaults: 15-min windows × 32 windows ≈ 8 hours. Override via env vars:

```bash
sudo WINDOW_SEC=600 TOTAL_WINDOWS=48 bench/run_overnight.sh
```

Output goes to `bench/runs/<YYYYMMDD_HHMMSS>/window-NNN_<branch>/`.

### Analysis

```bash
python3 bench/analyze.py bench/runs/<session-dir>
```

Requires only `numpy`. Prints per-window medians (sanity check) and pooled
per-branch fits with bootstrap CIs.

## Caveats

- **Background noise**: the harness assumes the rest of your machine is
  reasonably quiet. Heavy Xcode builds or Time Machine runs in the middle of
  the session will bias the affected windows. Median across many windows
  cancels most of this; egregious outliers will show up in the per-window
  table.
- **Traffic drift**: if RF activity changes substantially across the night
  (e.g., the system is much busier in the evening than at 3 am), the linear
  fit absorbs this evenly across branches because of the interleaving — but
  the *variance* of the fit will be higher than for a deterministic workload.
- **Per-PID power**: powermetrics' per-task energy_impact is a unitless score
  on Apple Silicon, not real joules. We use global package power instead and
  rely on the rest of the machine being quiet for the comparison to hold.
- **Pinning is soft**: `taskpolicy -c user-interactive` biases the scheduler,
  it doesn't force P-core-only execution. Threads with lower QoS classes set
  internally (e.g., by GNU Radio) may still land on E-cores. Both branches
  are subject to the same softness so it's fair, just noisier than a hard
  pin would be.

## Cleanup

Worktrees can be removed when done:

```bash
git worktree remove ../tr-baseline
git worktree remove ../tr-fft-tuning
```

Session output under `bench/runs/` is just files — delete what you don't
need.
