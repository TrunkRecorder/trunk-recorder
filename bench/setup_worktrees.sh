#!/usr/bin/env bash
# One-time setup: create two git worktrees (baseline + fft-tuning), each with
# its own build directory, so the overnight harness can flip between them
# without rebuilding. Run this from inside the trunk-recorder repo.
set -euo pipefail

REPO_ROOT="$(git -C "$(dirname "$0")/.." rev-parse --show-toplevel)"
cd "$REPO_ROOT"

BASELINE_BRANCH="${BASELINE_BRANCH:-master}"
TUNED_BRANCH="${TUNED_BRANCH:-dev/fft-tuning}"

# Worktrees live as sibling directories so they don't clutter the main checkout.
PARENT_DIR="$(dirname "$REPO_ROOT")"
BASELINE_WT="$PARENT_DIR/tr-baseline"
TUNED_WT="$PARENT_DIR/tr-fft-tuning"

echo "Creating worktrees:"
echo "  baseline: $BASELINE_WT  ($BASELINE_BRANCH)"
echo "  tuned:    $TUNED_WT     ($TUNED_BRANCH)"

if [ ! -d "$BASELINE_WT" ]; then
  git worktree add "$BASELINE_WT" "$BASELINE_BRANCH"
else
  echo "  baseline worktree already exists, skipping"
fi

if [ ! -d "$TUNED_WT" ]; then
  git worktree add "$TUNED_WT" "$TUNED_BRANCH"
else
  echo "  tuned worktree already exists, skipping"
fi

# The bench_logger plugin only exists on dev/fft-tuning right now (where we
# wrote it). Copy it into the baseline worktree and register it in the
# baseline CMakeLists so both binaries can log active-recorder counts.
echo
echo "Copying bench_logger plugin into baseline worktree…"
mkdir -p "$BASELINE_WT/plugins/bench_logger"
cp "$REPO_ROOT/plugins/bench_logger/bench_logger.cc" "$BASELINE_WT/plugins/bench_logger/"
cp "$REPO_ROOT/plugins/bench_logger/CMakeLists.txt"  "$BASELINE_WT/plugins/bench_logger/"

if ! grep -q "add_subdirectory(plugins/bench_logger)" "$BASELINE_WT/CMakeLists.txt"; then
  # Insert after the simplestream registration to match the pattern.
  awk '
    { print }
    /add_subdirectory\(plugins\/simplestream\)/ && !done {
      print "add_subdirectory(plugins/bench_logger)"; done=1
    }
  ' "$BASELINE_WT/CMakeLists.txt" > "$BASELINE_WT/CMakeLists.txt.tmp"
  mv "$BASELINE_WT/CMakeLists.txt.tmp" "$BASELINE_WT/CMakeLists.txt"
  echo "  registered bench_logger in baseline CMakeLists.txt"
else
  echo "  bench_logger already registered in baseline"
fi

echo
echo "Building baseline (this may take a while)…"
mkdir -p "$BASELINE_WT/build"
( cd "$BASELINE_WT/build" && cmake ../ && make -j"$(sysctl -n hw.ncpu)" )

echo
echo "Building tuned…"
mkdir -p "$TUNED_WT/build"
( cd "$TUNED_WT/build" && cmake ../ && make -j"$(sysctl -n hw.ncpu)" )

echo
echo "Done. Binaries:"
echo "  $BASELINE_WT/build/trunk-recorder"
echo "  $TUNED_WT/build/trunk-recorder"
echo
echo "Next: edit bench/run_overnight.sh to point at your config.json, then run it."
