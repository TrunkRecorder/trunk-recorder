#!/usr/bin/env python3
"""
blind_compare.py - Blind A/B comparison of vocoder tuning combinations.

Plays randomized pairs of WAV files produced by imbe_tune, hides the
parameter labels, takes A/B/tie judgments, and maintains per-combo Elo
ratings so the best-sounding parameter set comes to the top after enough
rounds.

Variety
-------
Pass multiple output directories (one per source .imbe file). Each round
picks a random source first, then a random pair *within* that source - so
you never compare clips of different speech against each other. The Elo
key is the parameter tuple, so judgments accumulate across sources for
the same combo.

Workflow
--------
    # capture 3 different real calls, then sweep each into its own dir:
    ./build/imbe_tune --input call1.imbe --sweep utils/sweep.json --output-dir /tmp/sw1
    ./build/imbe_tune --input call2.imbe --sweep utils/sweep.json --output-dir /tmp/sw2
    ./build/imbe_tune --input call3.imbe --sweep utils/sweep.json --output-dir /tmp/sw3

    # 50 blind rounds (each round = listen to A, listen to B, vote):
    python3 utils/blind_compare.py --rounds 50 /tmp/sw1 /tmp/sw2 /tmp/sw3

Controls during a round
-----------------------
    <enter>     replay the current clip / play next clip
    a           A sounds better
    b           B sounds better
    t           tie / cannot tell
    r           replay both A and B
    s           skip (no judgment)
    q           quit and print ranking

State is persisted to ./blind_compare_state.json after every round, so you
can quit any time and resume with --resume.

Requires: afplay (macOS), paplay (PulseAudio), or aplay (ALSA) on PATH.
"""

import argparse
import json
import os
import random
import shutil
import subprocess
import sys
from glob import glob


# ---------------------------------------------------------------------------
# Audio playback - auto-detect a system player

def find_player():
    for name, args in [("afplay", []), ("paplay", []), ("aplay", ["-q"])]:
        path = shutil.which(name)
        if path:
            return path, args
    print("error: need afplay, paplay, or aplay on PATH", file=sys.stderr)
    sys.exit(1)


def play_wav(player, args, path):
    try:
        subprocess.run([player, *args, path], check=False)
    except KeyboardInterrupt:
        pass


# ---------------------------------------------------------------------------
# Scanning imbe_tune output dirs

def scan_dir(d):
    """Return [{wav, params, source}, ...] for one imbe_tune output dir."""
    clips = []
    for json_path in sorted(glob(os.path.join(d, "combo_*.json"))):
        wav_path = json_path[:-5] + ".wav"
        if not os.path.isfile(wav_path):
            continue
        with open(json_path) as f:
            side = json.load(f)
        # strip private fields (e.g. _metrics) from the param identity
        params = {k: v for k, v in side.items() if not k.startswith("_")}
        if not params:
            continue
        clips.append({"wav": wav_path, "params": params, "source": d})
    return clips


def combo_key(params):
    """Stable identity for a parameter combination."""
    return json.dumps(params, sort_keys=True, separators=(",", ":"))


def fmt_params(params):
    """One-line readable summary."""
    return "  ".join(f"{k}={v}" for k, v in sorted(params.items()))


# ---------------------------------------------------------------------------
# Elo

ELO_K = 32
ELO_INITIAL = 1500.0


def expected(ra, rb):
    return 1.0 / (1.0 + 10.0 ** ((rb - ra) / 400.0))


def elo_update(ra, rb, score_a):
    """score_a: 1.0 (A wins), 0.0 (B wins), 0.5 (tie). Returns (new_ra, new_rb)."""
    ea = expected(ra, rb)
    return ra + ELO_K * (score_a - ea), \
           rb + ELO_K * ((1.0 - score_a) - (1.0 - ea))


# ---------------------------------------------------------------------------
# Persistent state

def load_state(path):
    if not os.path.isfile(path):
        return {"ratings": {}, "judgments": [], "rounds": 0}
    with open(path) as f:
        return json.load(f)


def save_state(path, state):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(state, f, indent=2)
    os.replace(tmp, path)


# ---------------------------------------------------------------------------
# Round logic

def prompt(msg, allowed, default=None):
    """Read a single-letter answer; <enter> returns default if given."""
    while True:
        try:
            ans = input(msg).strip().lower()
        except EOFError:
            return "q"
        if ans == "" and default is not None:
            return default
        if ans in allowed:
            return ans
        print(f"  (valid: {', '.join(repr(x) for x in allowed)})")


def play_pair(player, pargs, a, b):
    print("\n  >>> A")
    play_wav(player, pargs, a["wav"])
    if prompt("      [enter] play B, q quit > ", ["", "q"], default="") == "q":
        return None
    print("  >>> B")
    play_wav(player, pargs, b["wav"])
    return True


def run_round(player, pargs, source_clips, state, round_idx, target):
    # source must contain at least 2 distinct combos
    eligible = [
        s for s, clips in source_clips.items()
        if len({combo_key(c["params"]) for c in clips}) >= 2
    ]
    if not eligible:
        print("error: no source has at least 2 distinct combos", file=sys.stderr)
        return False

    src = random.choice(eligible)
    clips = source_clips[src]
    a, b = random.sample(clips, 2)
    while combo_key(a["params"]) == combo_key(b["params"]):
        b = random.choice(clips)

    # randomize A/B side
    if random.random() < 0.5:
        a, b = b, a

    print(f"\n=== Round {round_idx + 1}/{target}  (source: {os.path.basename(src.rstrip('/'))}) ===")

    while True:
        if play_pair(player, pargs, a, b) is None:
            return False
        ans = prompt(
            "      a / b / t (tie) / r (replay) / s (skip) / q (quit) > ",
            ["a", "b", "t", "r", "s", "q"])
        if ans == "r":
            continue
        if ans == "q":
            return False
        if ans == "s":
            return True
        if ans == "a":
            score_a = 1.0
            verdict = "A"
        elif ans == "b":
            score_a = 0.0
            verdict = "B"
        else:
            score_a = 0.5
            verdict = "tie"
        break

    ka, kb = combo_key(a["params"]), combo_key(b["params"])
    ratings = state["ratings"]
    ra = ratings.get(ka, ELO_INITIAL)
    rb = ratings.get(kb, ELO_INITIAL)
    new_ra, new_rb = elo_update(ra, rb, score_a)
    ratings[ka] = new_ra
    ratings[kb] = new_rb

    state["judgments"].append({
        "round": state["rounds"] + 1,
        "source": src,
        "a": a["params"], "b": b["params"],
        "verdict": verdict,
        "ra_before": ra, "rb_before": rb,
        "ra_after": new_ra, "rb_after": new_rb,
    })
    state["rounds"] += 1

    print(f"      noted: {verdict}.  A {ra:.0f} -> {new_ra:.0f}   B {rb:.0f} -> {new_rb:.0f}")
    return True


# ---------------------------------------------------------------------------
# Ranking report

def print_ranking(state, params_by_key):
    if not state["ratings"]:
        print("no judgments yet.")
        return

    # Count games per combo from judgment history
    games = {}
    for j in state["judgments"]:
        for side in ("a", "b"):
            k = combo_key(j[side])
            games[k] = games.get(k, 0) + 1

    rows = sorted(state["ratings"].items(), key=lambda kv: -kv[1])
    print(f"\n=== Ranking after {state['rounds']} rounds ({len(rows)} combos) ===")
    print(f"  {'Elo':>5}  {'n':>4}   parameters")
    for k, r in rows:
        n = games.get(k, 0)
        label = params_by_key.get(k, k)
        marker = "<-- top" if k == rows[0][0] and n > 0 else ""
        print(f"  {r:5.0f}  {n:4d}   {label}  {marker}")


# ---------------------------------------------------------------------------
# Main

def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("dirs", nargs="*", help="imbe_tune output directories")
    p.add_argument("--rounds", type=int, default=30,
                   help="how many judgments to do this run (default: 30)")
    p.add_argument("--state", default="./blind_compare_state.json",
                   help="persistence file (default: ./blind_compare_state.json)")
    p.add_argument("--resume", action="store_true",
                   help="resume; dirs are inferred from state if not given")
    p.add_argument("--rank-only", action="store_true",
                   help="print current ranking from --state and exit")
    args = p.parse_args()

    state = load_state(args.state)

    if args.rank_only:
        params_by_key = {}
        for j in state["judgments"]:
            params_by_key[combo_key(j["a"])] = fmt_params(j["a"])
            params_by_key[combo_key(j["b"])] = fmt_params(j["b"])
        print_ranking(state, params_by_key)
        return

    player, pargs = find_player()

    if not args.dirs:
        if not args.resume:
            print("error: pass one or more dirs, or --resume", file=sys.stderr)
            sys.exit(1)
        args.dirs = sorted({j["source"] for j in state["judgments"]})
        if not args.dirs:
            print("error: --resume but state has no source dirs recorded", file=sys.stderr)
            sys.exit(1)

    source_clips = {}
    params_by_key = {}
    for d in args.dirs:
        if not os.path.isdir(d):
            print(f"warning: not a directory: {d}", file=sys.stderr)
            continue
        clips = scan_dir(d)
        if len(clips) < 2:
            print(f"warning: <2 clips in {d}, skipping", file=sys.stderr)
            continue
        source_clips[d] = clips
        for c in clips:
            params_by_key[combo_key(c["params"])] = fmt_params(c["params"])

    if not source_clips:
        print("error: no usable directories", file=sys.stderr)
        sys.exit(1)

    n_clips = sum(len(c) for c in source_clips.values())
    print(f"loaded {n_clips} clips across {len(source_clips)} source(s); "
          f"{len(params_by_key)} unique combos")
    print(f"player: {player}")
    print(f"state:  {args.state}  ({state['rounds']} prior rounds)")
    target = state["rounds"] + args.rounds
    print(f"target: {target} total rounds  (this run: {args.rounds})")
    print("\ncontrols: enter (next/replay), a/b (vote), t (tie), s (skip), q (quit)")

    try:
        while state["rounds"] < target:
            keep_going = run_round(player, pargs, source_clips,
                                   state, state["rounds"], target)
            save_state(args.state, state)
            if not keep_going:
                break
    except KeyboardInterrupt:
        print("\n(interrupted)")

    print_ranking(state, params_by_key)
    print(f"\nsaved state -> {args.state}")


if __name__ == "__main__":
    main()
