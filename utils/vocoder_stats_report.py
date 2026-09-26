#!/usr/bin/env python3
"""
vocoder_stats_report.py - summarize OP25_VOCODER_STATS output (one JSON line
per P25 Phase 1 transmission, written by vocoder_monitor).

    utils/vocoder_stats_report.py vocoder-data/dcfd/stats.jsonl
    utils/vocoder_stats_report.py --bucket 15 --since 2 vocoder-data/*/stats.jsonl

Sections:
  over time     per-bucket reception and concealment rates
  channel       ET histogram, error-burst lengths
  concealment   why frames were repeated or muted
  vocoder       pitch jumps, voicing, level, clipping
  talkgroups    busiest talkgroups
  noise frames  where pure-noise frames (ET >= 12) fall within transmissions,
                from the .imbe captures next to the stats file
  worst         transmissions with the most concealment, with the .imbe
                capture that replays each one through utils/imbe_eval
"""

import argparse
import json
import os
import sys
import time
from collections import defaultdict


def load(paths, since_h):
    rows = []
    cutoff = time.time() - since_h * 3600 if since_h else 0
    for p in paths:
        name = os.path.basename(os.path.dirname(os.path.abspath(p)))
        with open(p) as f:
            for line in f:
                try:
                    r = json.loads(line)
                except ValueError:
                    continue
                if r['t'] >= cutoff and r['frames'] - r['enc_frames'] > 0:
                    r['sys'] = name
                    rows.append(r)
    rows.sort(key=lambda r: r['t'])
    return rows


def agg(rows):
    a = defaultdict(float)
    for r in rows:
        n = r['frames'] - r['enc_frames']
        a['tx'] += 1
        a['frames'] += n
        a['et'] += r['et_sum']
        a['e0ge2'] += r['e0'][2] + r['e0'][3]
        a['et_ge6'] += sum(r['et'][6:])
        a['rep'] += r['repeated']
        a['mute'] += r['muted']
        a['mute_runs'] += r['mute_runs']
        a['clip'] += r['clip_samples']
        a['pj'] += r['pitch_jumps']
        a['oj'] += r['octave_jumps']
        a['pf'] += r['pitch_frames']
        a['bursts_long'] += sum(r['bursts'][3:])
        if r['active_frames']:
            a['rms_w'] += r['rms_dbfs'] * r['active_frames']
            a['active'] += r['active_frames']
        a['tx_with_mute'] += 1 if r['muted'] else 0
    return a


def pct(x, n):
    return 100.0 * x / n if n else 0.0


def over_time(rows, bucket_min):
    print(f'\n== Over time ({bucket_min} min buckets) ==')
    print(f"{'time':16s}{'sys':>7s}{'tx':>6s}{'voice m':>8s}{'ET/fr':>7s}{'E0>=2%':>7s}{'rep%':>7s}{'mute%':>7s}"
          f"{'tx mute%':>9s}{'burst4+':>8s}{'pjump%':>7s}{'rms dB':>7s}{'clip/m':>7s}")
    buckets = defaultdict(list)
    for r in rows:
        buckets[(int(r['t'] // (bucket_min * 60)), r['sys'])].append(r)
    for (b, s) in sorted(buckets):
        a = agg(buckets[(b, s)])
        vm = a['frames'] * 0.02 / 60
        ts = time.strftime('%m-%d %H:%M', time.localtime(b * bucket_min * 60))
        print(f"{ts:16s}{s:>7s}{int(a['tx']):6d}{vm:8.1f}{a['et'] / a['frames']:7.3f}{pct(a['e0ge2'], a['frames']):7.2f}"
              f"{pct(a['rep'], a['frames']):7.2f}{pct(a['mute'], a['frames']):7.2f}{pct(a['tx_with_mute'], a['tx']):9.1f}"
              f"{int(a['bursts_long']):8d}{pct(a['pj'], a['pf']):7.2f}{(a['rms_w'] / a['active'] if a['active'] else 0):7.1f}"
              f"{(a['clip'] / vm if vm else 0):7.1f}")


def channel(rows):
    print('\n== Channel (FEC-corrected errors) ==')
    for s in sorted({r['sys'] for r in rows}):
        rs = [r for r in rows if r['sys'] == s]
        et = [sum(r['et'][i] for r in rs) for i in range(17)]
        e0 = [sum(r['e0'][i] for r in rs) for i in range(4)]
        b = [sum(r['bursts'][i] for r in rs) for i in range(6)]
        n = sum(et)
        print(f'{s}: {n} frames')
        print('  ET/frame   ' + '  '.join(f'{i if i < 16 else "16+"}:{pct(v, n):.2f}%' for i, v in enumerate(et) if v))
        print('  E0 (u0)    ' + '  '.join(f'{i}:{pct(v, n):.2f}%' for i, v in enumerate(e0)))
        print('  bad-frame bursts (E0>=2 or ET>=6), by length: ' +
              '  '.join(f'{lab}:{v}' for lab, v in zip(['1', '2', '3', '4-7', '8-15', '16+'], b)))


def concealment(rows):
    print('\n== Concealment ==')
    for s in sorted({r['sys'] for r in rows}):
        rs = [r for r in rows if r['sys'] == s]
        n = sum(r['frames'] - r['enc_frames'] for r in rs)
        c = {k: sum(r['cause'][k] for r in rs) for k in ('b0', 'e0', 'et', 'er', 'maxrep')}
        rep = sum(r['repeated'] for r in rs)
        mute = sum(r['muted'] for r in rs)
        print(f"{s}: decoded {pct(n - rep - mute, n):.2f}%  repeated {pct(rep, n):.2f}%  muted {pct(mute, n):.2f}%   "
              f"longest repeat run {max(r['max_rep_run'] for r in rs)}  longest mute run {max(r['max_mute_run'] for r in rs)} frames")
        print(f"  frames triggering: b0>207 {c['b0']}  E0>=3 {c['e0']}  ET limit {c['et']}  ER mute {c['er']}  "
              f"repeat limit {c['maxrep']}   (b0 silence frames {sum(r['b0_silence'] for r in rs)}, "
              f"other invalid b0 {sum(r['b0_invalid'] for r in rs)})")


def vocoder(rows):
    print('\n== Vocoder ==')
    for s in sorted({r['sys'] for r in rows}):
        rs = [r for r in rows if r['sys'] == s]
        a = agg(rs)
        vm = a['frames'] * 0.02 / 60
        f0 = [r['f0_mean'] for r in rs if r['pitch_frames'] >= 25]
        f0.sort()
        print(f"{s}: pitch jumps >25% {pct(a['pj'], a['pf']):.2f}% of voiced frames, octave jumps {pct(a['oj'], a['pf']):.2f}%;"
              f" median talker f0 {f0[len(f0) // 2] if f0 else 0:.0f} Hz")
        vf = sum(r['voiced_frac'] * r['frames'] for r in rs) / max(1, sum(r['frames'] for r in rs))
        vc = sum(r['vuv_change_per_s'] * r['frames'] for r in rs) / max(1, sum(r['frames'] for r in rs))
        print(f"  voiced fraction {vf:.2f}, voicing change {vc:.2f}/s;  active level {a['rms_w'] / max(1, a['active']):.1f} dBFS,"
              f" peak>=32000 in {sum(1 for r in rs if r['peak'] >= 32000)} tx, clipped samples {int(a['clip'])} ({a['clip'] / vm if vm else 0:.1f}/voice-min)")


def talkgroups(rows, top):
    print(f'\n== Busiest talkgroups (top {top}) ==')
    by = defaultdict(list)
    for r in rows:
        by[(r['sys'], r['tg'])].append(r)
    ranked = sorted(by.items(), key=lambda kv: -sum(r['frames'] for r in kv[1]))[:top]
    print(f"{'sys':>7s}{'tg':>7s}{'tx':>6s}{'voice m':>8s}{'ET/fr':>7s}{'rep%':>7s}{'mute%':>7s}{'rms dB':>7s}")
    for (s, tg), rs in ranked:
        a = agg(rs)
        print(f"{s:>7s}{tg:7d}{int(a['tx']):6d}{a['frames'] * 0.02 / 60:8.1f}{a['et'] / a['frames']:7.3f}"
              f"{pct(a['rep'], a['frames']):7.2f}{pct(a['mute'], a['frames']):7.2f}{a['rms_w'] / max(1, a['active']):7.1f}")


def noise_positions(rows, stats_paths):
    print('\n== Noise frames (ET >= 12, i.e. random bits) by position ==')
    import struct
    cap_dirs = {os.path.basename(os.path.dirname(os.path.abspath(p))): os.path.join(os.path.dirname(os.path.abspath(p)), 'capture')
                for p in stats_paths}
    for s in sorted({r['sys'] for r in rows}):
        head = tail = body = ntx = tx_tail = 0
        for r in rows:
            if r['sys'] != s or not r.get('capture'):
                continue
            path = os.path.join(cap_dirs.get(s, ''), r['capture'])
            try:
                data = open(path, 'rb').read()[16:]
            except OSError:
                continue
            n = len(data) // 40
            if n == 0:
                continue
            et = [struct.unpack_from('<I', data, i * 40 + 36)[0] for i in range(n)]
            ntx += 1
            for i, e in enumerate(et):
                if e >= 12:
                    if i >= n - 9:
                        tail += 1
                    elif i < 9:
                        head += 1
                    else:
                        body += 1
            if et[-1] >= 12:
                tx_tail += 1
        tot = head + tail + body
        print(f"{s}: {tot} noise frames in {ntx} captured tx - first 9 frames {pct(head, tot):.0f}%, last 9 {pct(tail, tot):.0f}%,"
              f" elsewhere {pct(body, tot):.0f}%;  {pct(tx_tail, ntx):.1f}% of transmissions end on a noise frame")


def worst(rows, top):
    print(f'\n== Most concealment (top {top}, >= 2 s) ==')
    cand = [r for r in rows if r['frames'] >= 100]
    cand.sort(key=lambda r: -(r['repeated'] + 2 * r['muted']) / r['frames'])
    for r in cand[:top]:
        ts = time.strftime('%m-%d %H:%M:%S', time.localtime(r['t']))
        print(f"{ts} {r['sys']:>6s} tg {r['tg']:<6d} {r['frames'] * 0.02:5.1f}s  ET/fr {r['et_sum'] / r['frames']:.2f}  "
              f"rep {pct(r['repeated'], r['frames']):4.1f}%  mute {pct(r['muted'], r['frames']):4.1f}%  {r['capture']}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('files', nargs='+')
    ap.add_argument('--bucket', type=int, default=60, help='minutes per time bucket (default 60)')
    ap.add_argument('--since', type=float, default=0, help='only the last N hours')
    ap.add_argument('--top', type=int, default=10)
    args = ap.parse_args()
    rows = load(args.files, args.since)
    if not rows:
        sys.exit('no transmissions')
    span = (rows[-1]['t'] - rows[0]['t']) / 3600
    print(f"{len(rows)} transmissions over {span:.1f} h "
          f"({time.strftime('%m-%d %H:%M', time.localtime(rows[0]['t']))} - {time.strftime('%m-%d %H:%M', time.localtime(rows[-1]['t']))})")
    over_time(rows, args.bucket)
    channel(rows)
    concealment(rows)
    vocoder(rows)
    talkgroups(rows, args.top)
    noise_positions(rows, args.files)
    worst(rows, args.top)


if __name__ == '__main__':
    main()
