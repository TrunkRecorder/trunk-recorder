#!/usr/bin/env python3
"""
fetch_recent_wavs.py - Download the N most recent trunk-recorder call recordings
                      from a remote host, sorted by the timestamp embedded in
                      the filename.

Filename format expected:
    <talkgroup>-<unix_timestamp>_<frequency>-call_<callid>.wav
    e.g. 744-1779733362.624_859037500.0-call_4882.wav

Uses ssh + scp under the hood, so SSH key-based auth to the remote host
should already be set up (otherwise scp will prompt for a password each
download).

Usage:
    fetch_recent_wavs.py                                  # all defaults
    fetch_recent_wavs.py -n 25 --local-dir ./inbox
    fetch_recent_wavs.py --remote-dir '/path/to/other/dir'
    fetch_recent_wavs.py --host user@host  --remote-dir '...'
"""

import argparse
import os
import re
import shlex
import subprocess
import sys
from datetime import datetime


# <tg>-<unix_ts>_<freq>-call_<callid>.wav
FILENAME_RE = re.compile(r'^(\d+)-(\d+(?:\.\d+)?)_[\d.]+-call_\d+\.wav$')


def run(cmd, **kw):
    """Wrapper around subprocess.run that returns (rc, stdout, stderr)."""
    r = subprocess.run(cmd, capture_output=True, text=True, **kw)
    return r.returncode, r.stdout, r.stderr


def list_remote_wavs(host, remote_dir):
    """Return list of (timestamp, talkgroup, filename) sorted newest first."""
    cmd = ['ssh', host, f'ls -1 {shlex.quote(remote_dir)}']
    rc, out, err = run(cmd)
    if rc != 0:
        raise RuntimeError(f"ssh ls failed (rc={rc}): {err.strip()}")
    files = []
    for line in out.splitlines():
        name = line.strip()
        m = FILENAME_RE.match(name)
        if not m:
            continue
        tg = int(m.group(1))
        ts = float(m.group(2))
        files.append((ts, tg, name))
    files.sort(key=lambda x: -x[0])
    return files


def download_one(host, remote_dir, name, local_dir):
    """scp a single file. Returns (ok, local_path_or_error)."""
    remote_full = os.path.join(remote_dir, name)
    # Pass the raw path (no shell-quoting). Modern scp (OpenSSH 9.0+) uses the
    # SFTP protocol which talks to the remote sftp-server directly without a
    # shell - quoting would end up inside the filename. The whole "host:path"
    # is one subprocess arg so spaces survive without quoting.
    src = f"{host}:{remote_full}"
    dst = os.path.join(local_dir, name)
    cmd = ['scp', '-q', '-p', src, dst]
    rc, _out, err = run(cmd)
    if rc != 0:
        return False, err.strip() or f"scp returned {rc}"
    if not os.path.isfile(dst) or os.path.getsize(dst) == 0:
        return False, "downloaded file is empty or missing"
    return True, dst


def human_size(n):
    for u in ('B', 'KB', 'MB', 'GB'):
        if n < 1024:
            return f"{n:.1f} {u}"
        n /= 1024
    return f"{n:.1f} TB"


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--host', default='radiobox@192.168.1.100',
                        help='SSH target (default: %(default)s)')
    parser.add_argument(
        '--base-dir',
        default='/Users/radiobox/Projects/Trunk Recorder/trunk-build/dcfd',
        help='Remote base directory; trunk-recorder writes <base>/<YYYY>/<M>/<D> '
             '(default: %(default)s)')
    parser.add_argument(
        '--today', action='store_true',
        help="Use today's date (YYYY/M/D, unpadded) appended to --base-dir. "
             "This is the default behavior if --remote-dir is not given.")
    parser.add_argument(
        '--remote-dir', default=None,
        help='Explicit remote path; overrides --base-dir and --today')
    parser.add_argument('--local-dir', default='./recent_wavs',
                        help='Local directory to save into (default: %(default)s)')
    parser.add_argument('-n', '--count', type=int, default=10,
                        help='Number of most-recent files to download (default: %(default)s)')
    parser.add_argument('--skip-existing', action='store_true',
                        help='Skip files already present in local-dir')
    parser.add_argument('--dry-run', action='store_true',
                        help='List the files that would be downloaded; do not transfer')
    args = parser.parse_args()

    # Resolve the remote directory.
    if args.remote_dir:
        remote_dir = args.remote_dir
    else:
        # Default to today; --today flag is explicit form of the same thing.
        now = datetime.now()
        remote_dir = os.path.join(args.base_dir, str(now.year), str(now.month), str(now.day))

    os.makedirs(args.local_dir, exist_ok=True)

    try:
        files = list_remote_wavs(args.host, remote_dir)
    except RuntimeError as e:
        print(f"error: {e}", file=sys.stderr)
        sys.exit(1)

    if not files:
        print(f"no WAV files matched in {remote_dir}", file=sys.stderr)
        sys.exit(1)

    selected = files[:args.count]

    print(f"{args.host}:{remote_dir}")
    print(f"  -> {os.path.abspath(args.local_dir)}")
    print(f"  {len(files)} matching file(s) on remote; selecting {len(selected)} newest")
    print()

    total_bytes = 0
    ok = skipped = failed = 0
    for ts, tg, name in selected:
        dt = datetime.fromtimestamp(ts).isoformat(timespec='seconds')
        local_path = os.path.join(args.local_dir, name)
        prefix = f"  [{dt}] tg={tg:<6} {name}"
        if args.dry_run:
            print(f"{prefix}  (dry-run)")
            continue
        if args.skip_existing and os.path.isfile(local_path) and os.path.getsize(local_path) > 0:
            print(f"{prefix}  skipped (already exists)")
            skipped += 1
            continue
        sys.stdout.write(prefix + " ... ")
        sys.stdout.flush()
        success, info = download_one(args.host, remote_dir, name, args.local_dir)
        if success:
            sz = os.path.getsize(info)
            total_bytes += sz
            print(f"ok ({human_size(sz)})")
            ok += 1
        else:
            print(f"FAILED: {info}")
            failed += 1

    if args.dry_run:
        print(f"\nwould download {len(selected)} files")
    else:
        print(f"\ndownloaded {ok}, skipped {skipped}, failed {failed}  total={human_size(total_bytes)}")
        if failed:
            sys.exit(2)


if __name__ == '__main__':
    main()
