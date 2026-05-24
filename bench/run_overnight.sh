#!/usr/bin/env bash
# Overnight A/B benchmark harness.
#
# Alternates between the baseline and tuned trunk-recorder binaries built by
# setup_worktrees.sh, running each for WINDOW_SEC and capturing:
#   - bench_active.csv from the bench_logger plugin (active recorder count)
#   - powermetrics plist (per-PID energy + per-core power)
#
# Output: bench/runs/<session-timestamp>/<window-NNN>_<branch>/
#
# REQUIRES sudo for powermetrics. Either:
#   sudo bench/run_overnight.sh
# or add a sudoers entry letting your user run /usr/bin/powermetrics without a
# password.
set -euo pipefail

REPO_ROOT="$(git -C "$(dirname "$0")/.." rev-parse --show-toplevel)"
BENCH_DIR="$REPO_ROOT/bench"
PARENT_DIR="$(dirname "$REPO_ROOT")"

# Pick up binary paths written by setup_worktrees.sh, if available. Allows
# the tuned worktree to be the user's current source dir rather than a
# sibling tr-fft-tuning directory.
if [ -f "$BENCH_DIR/.paths" ]; then
  . "$BENCH_DIR/.paths"
fi

BASELINE_BIN="${BASELINE_BIN:-$PARENT_DIR/tr-baseline/build/trunk-recorder}"
TUNED_BIN="${TUNED_BIN:-$PARENT_DIR/tr-fft-tuning/build/trunk-recorder}"
CONFIG_FILE="${CONFIG_FILE:-$REPO_ROOT/config.json}"

WINDOW_SEC="${WINDOW_SEC:-900}"        # 15 minutes of *measurement* per window
STARTUP_SEC="${STARTUP_SEC:-30}"       # Wait after launch for trunk-recorder
                                       # to lock onto the control channel
                                       # before measurement begins
INTER_WINDOW_SEC="${INTER_WINDOW_SEC:-15}"  # Gap between windows for the SDR
                                            # USB handle to be released
TOTAL_WINDOWS="${TOTAL_WINDOWS:-32}"   # ~8.5 hours total at default settings
SETTLE_SEC="${SETTLE_SEC:-10}"         # Belt-and-suspenders: analyzer trims
                                       # first N sec of each window's CSV
POWERMETRICS_INTERVAL_MS="${POWERMETRICS_INTERVAL_MS:-1000}"

for f in "$BASELINE_BIN" "$TUNED_BIN" "$CONFIG_FILE"; do
  if [ ! -e "$f" ]; then
    echo "ERROR: missing $f" >&2
    echo "Run bench/setup_worktrees.sh first, or set BASELINE_BIN/TUNED_BIN/CONFIG_FILE." >&2
    exit 1
  fi
done

if [ "$(id -u)" -ne 0 ]; then
  echo "ERROR: powermetrics requires root. Re-run with sudo." >&2
  exit 1
fi

SESSION="$(date +%Y%m%d_%H%M%S)"
RUN_ROOT="$BENCH_DIR/runs/$SESSION"
mkdir -p "$RUN_ROOT"
echo "Session: $RUN_ROOT"
echo "Window: ${WINDOW_SEC}s   Total: $TOTAL_WINDOWS windows   Settle: ${SETTLE_SEC}s"

# Build a temporary config per branch that enables the bench_logger plugin
# pointing at the window-specific CSV path. We do this by injecting a "plugins"
# entry into a copy of the user's config.json.
make_window_config() {
  local out_config="$1"
  local csv_path="$2"
  local branch_tag="$3"
  python3 - "$CONFIG_FILE" "$out_config" "$csv_path" "$branch_tag" <<'PY'
import json, sys
src, dst, csv_path, branch_tag = sys.argv[1:5]
with open(src) as f:
    cfg = json.load(f)
plugins = cfg.get("plugins", [])
# Drop any existing bench_logger entry, then add ours.
plugins = [p for p in plugins if p.get("name") != "bench_logger"
                                and "bench_logger" not in p.get("library", "")]
plugins.append({
    "name": "bench_logger",
    "library": "libbench_logger.so",
    "bench_csv": csv_path,
    "branch_tag": branch_tag,
    "enabled": True,
})
cfg["plugins"] = plugins
with open(dst, "w") as f:
    json.dump(cfg, f, indent=2)
PY
}

run_window() {
  local idx="$1"
  local branch="$2"
  local bin="$3"

  local pad="$(printf '%03d' "$idx")"
  local window_dir="$RUN_ROOT/window-${pad}_${branch}"
  mkdir -p "$window_dir"

  local csv_path="$window_dir/bench_active.csv"
  local config_path="$window_dir/config.json"
  local pm_plist="$window_dir/powermetrics.plist"
  local stdout_log="$window_dir/trunk-recorder.log"

  make_window_config "$config_path" "$csv_path" "$branch"

  local build_dir
  build_dir="$(dirname "$bin")"

  echo
  echo "[$(date '+%H:%M:%S')] window $pad / $TOTAL_WINDOWS  branch=$branch"

  # Launch trunk-recorder pinned to P-cores via QoS (user-interactive on
  # Apple Silicon biases scheduling to P-cores).
  (
    cd "$build_dir"
    taskpolicy -c user-interactive ./trunk-recorder -c "$config_path" \
      >"$stdout_log" 2>&1 &
    echo $! > "$window_dir/tr.pid"
  )

  # Wait for trunk-recorder to come up and lock onto the control channel.
  # Without this grace period, the first chunk of each window captures SDR
  # warmup / control-channel-acquisition cost rather than steady-state.
  echo "  waiting ${STARTUP_SEC}s for trunk-recorder to settle…"
  sleep "$STARTUP_SEC"

  local tr_pid
  tr_pid="$(cat "$window_dir/tr.pid")"

  if ! kill -0 "$tr_pid" 2>/dev/null; then
    echo "  ERROR: trunk-recorder failed to start, see $stdout_log" >&2
    return 1
  fi

  # Start powermetrics, filtered to nothing (we want global power) but with
  # per-process samplers so we can attribute by PID in analysis.
  powermetrics --samplers cpu_power,tasks --show-process-energy \
    -i "$POWERMETRICS_INTERVAL_MS" -f plist -o "$pm_plist" \
    --hide-cpu-duty-cycle >/dev/null 2>&1 &
  local pm_pid=$!

  # Note settle window — analysis trims first SETTLE_SEC seconds.
  echo "$(date +%s)" > "$window_dir/start_epoch"
  sleep "$WINDOW_SEC"
  echo "$(date +%s)" > "$window_dir/end_epoch"

  # Tear down powermetrics first so we don't capture trunk-recorder shutting down.
  kill -INT "$pm_pid" 2>/dev/null || true
  wait "$pm_pid" 2>/dev/null || true

  # Clean shutdown of trunk-recorder.
  kill -TERM "$tr_pid" 2>/dev/null || true
  for _ in $(seq 1 30); do
    if ! kill -0 "$tr_pid" 2>/dev/null; then break; fi
    sleep 1
  done
  if kill -0 "$tr_pid" 2>/dev/null; then
    echo "  trunk-recorder did not exit on TERM, sending KILL" >&2
    kill -KILL "$tr_pid" 2>/dev/null || true
  fi

  echo "  done. tr.pid=$tr_pid  csv=$(wc -l <"$csv_path" 2>/dev/null || echo 0) lines"

  # Inter-window gap so the SDR USB handle is fully released before the next
  # trunk-recorder tries to claim it. Single-SDR setups need this.
  if [ "$INTER_WINDOW_SEC" -gt 0 ]; then
    sleep "$INTER_WINDOW_SEC"
  fi
}

# Interleave: alternate starting branch so we don't always start cold on the
# same branch (interleave bias mitigation).
START_BRANCH="${START_BRANCH:-baseline}"

for i in $(seq 1 "$TOTAL_WINDOWS"); do
  if [ $((i % 2)) -eq 1 ]; then
    [ "$START_BRANCH" = "baseline" ] && b=baseline || b=tuned
  else
    [ "$START_BRANCH" = "baseline" ] && b=tuned || b=baseline
  fi
  if [ "$b" = "baseline" ]; then
    run_window "$i" baseline "$BASELINE_BIN"
  else
    run_window "$i" tuned "$TUNED_BIN"
  fi
done

echo
echo "All windows complete."
echo "Analyze with: python3 $BENCH_DIR/analyze.py $RUN_ROOT"
