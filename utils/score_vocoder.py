#!/usr/bin/env python3
"""
score_vocoder.py - objective quality scores for decoded IMBE audio.

For each decoded WAV, reports:
  PESQ-NB  ITU-T P.862 narrowband, against the clean reference (intrusive)
  STOI     short-time objective intelligibility, against the reference
  DNSMOS   Microsoft DNSMOS P.835 SIG / BAK / OVRL (non-intrusive, no reference
           needed - use it for live recordings)
  FLUX     peak-to-mean (dB) of spectral flux folded onto the 160-sample
           synthesis frame; natural speech ~0.1 dB, a decoder whose frame
           transitions are abrupt shows more (non-intrusive)

Decoded files are matched to references by basename prefix: with --ref-dir,
"<name>.<tag>.wav" is scored against "<ref-dir>/<name>.wav". Without
--ref-dir only DNSMOS is computed.

    pip install numpy scipy soundfile pesq pystoi onnxruntime
    utils/score_vocoder.py --ref-dir corpus/ out/*.float.wav
    utils/score_vocoder.py live/*.wav                   # DNSMOS only

Typical loop (see docs/Notes/VOCODER-IMPROVEMENTS.md):
    build/imbe_eval enc corpus/a.wav out/a.imbe 0.02
    build/imbe_eval dec out/a.imbe out/a.float.wav float
    utils/score_vocoder.py --ref-dir corpus out/*.float.wav
"""

import argparse
import atexit
import os
import sys
import urllib.request
from concurrent.futures import ProcessPoolExecutor

import numpy as np
import soundfile as sf
from scipy.signal import resample_poly

DNSMOS_URL = 'https://github.com/microsoft/DNS-Challenge/raw/master/DNSMOS/DNSMOS/sig_bak_ovr.onnx'
DNSMOS_LEN = 144160  # 9.01 s at 16 kHz
# Non-personalized calibration polynomials from DNS-Challenge dnsmos_local.py
P_SIG = np.poly1d([-0.08397278, 1.22083953, 0.0052439])
P_BAK = np.poly1d([-0.13166888, 1.60915514, -0.39604546])
P_OVR = np.poly1d([-0.06766283, 1.11546468, 0.04602535])

_session = None


def model_path():
    d = os.path.join(os.path.expanduser('~'), '.cache', 'trunk-recorder')
    p = os.path.join(d, 'dnsmos_sig_bak_ovr.onnx')
    if not os.path.exists(p):
        os.makedirs(d, exist_ok=True)
        urllib.request.urlretrieve(DNSMOS_URL, p)
    return p


def load(path, rate):
    x, sr = sf.read(path, dtype='float32')
    if x.ndim > 1:
        x = x.mean(axis=1)
    if sr != rate:
        x = resample_poly(x, rate, sr).astype('float32')
    return x


def dnsmos(path):
    global _session
    if _session is None:
        import onnxruntime as ort
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = 1
        _session = ort.InferenceSession(model_path(), opts)
    x = load(path, 16000)
    if len(x) == 0:
        return (float('nan'),) * 3
    while len(x) < DNSMOS_LEN:
        x = np.concatenate([x, x])
    scores = []
    for start in range(0, len(x) - DNSMOS_LEN + 1, 16000):
        sig, bak, ovr = _session.run(None, {'input_1': x[None, start:start + DNSMOS_LEN]})[0][0]
        scores.append((P_SIG(sig), P_BAK(bak), P_OVR(ovr)))
    return tuple(np.mean(scores, axis=0))


def frame_flux(path):
    """Spectral flux vs. position within the 20 ms frame; peak/mean in dB."""
    from scipy.signal import stft
    x = load(path, 8000).astype('float64')
    hop, win = 4, 64
    f, _, Z = stft(x, 8000, nperseg=win, noverlap=win - hop, nfft=128, boundary=None, padded=False)
    band = (f > 200) & (f < 3600)
    S = np.log(np.abs(Z[band]) + 1e-6)
    energy = (np.abs(Z[band]) ** 2).sum(0)
    act = energy > np.percentile(energy, 50)
    flux = np.abs(np.diff(S, axis=1)).mean(0)
    a = act[1:] & act[:-1]
    pos = ((np.arange(1, S.shape[1]) * hop + win // 2) % 160) // hop
    prof = np.array([flux[a & (pos == k)].mean() for k in range(160 // hop)])
    return float(20 * np.log10(prof.max() / prof.mean()))


def align(ref, deg, max_lag=400):
    """Remove the decoder's fixed delay by cross-correlation."""
    n = min(len(ref), len(deg))
    ref, deg = ref[:n], deg[:n]
    lag = max(range(max_lag), key=lambda k: float(np.dot(ref[:n - k], deg[k:n])))
    return ref[:n - lag], deg[lag:n]


def score(job):
    deg_path, ref_path = job
    out = {'file': deg_path}
    if ref_path:
        from pesq import pesq
        from pystoi import stoi
        ref, deg = align(load(ref_path, 8000).astype('float64'), load(deg_path, 8000).astype('float64'))
        out['pesq'] = pesq(8000, ref, deg, 'nb')
        out['stoi'] = stoi(ref, deg, 8000)
    out['sig'], out['bak'], out['ovrl'] = dnsmos(deg_path)
    out['flux'] = frame_flux(deg_path)
    return out


def _worker_init():
    # onnxruntime can abort in its static destructors when a pool worker
    # exits on macOS; skip them - workers hold no state worth flushing.
    atexit.register(os._exit, 0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('files', nargs='+', help='decoded WAV files')
    ap.add_argument('--ref-dir', help='directory of clean reference WAVs (enables PESQ/STOI)')
    ap.add_argument('--jobs', type=int, default=os.cpu_count())
    args = ap.parse_args()

    jobs = []
    for f in args.files:
        ref = None
        if args.ref_dir:
            ref = os.path.join(args.ref_dir, os.path.basename(f).split('.')[0] + '.wav')
            if not os.path.exists(ref):
                print(f'no reference for {f}', file=sys.stderr)
                continue
        jobs.append((f, ref))

    with ProcessPoolExecutor(args.jobs, initializer=_worker_init) as ex:
        results = list(ex.map(score, jobs))

    cols = (['pesq', 'stoi'] if args.ref_dir else []) + ['sig', 'bak', 'ovrl', 'flux']
    print(f"{'file':50s}" + ''.join(f'{c:>8s}' for c in cols))
    for r in results:
        print(f"{os.path.basename(r['file'])[:50]:50s}" + ''.join(f'{r[c]:8.3f}' for c in cols))
    print(f"{'MEAN (' + str(len(results)) + ' files)':50s}" + ''.join(f'{np.nanmean([r[c] for r in results]):8.3f}' for c in cols))


if __name__ == '__main__':
    main()
