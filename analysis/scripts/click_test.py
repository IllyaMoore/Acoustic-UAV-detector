"""The sample-alignment gate: prove the SD_A -> SD_B offset is a constant.

Setup (README, "Status"): a click source directly above the
centroid, so the true TDOA is zero for every pair. Record with the firmware
(a dozen reboots -> a dozen rec_*.wav), then:

    uv run python scripts/click_test.py /sd/rec_*.wav

For every click in every file this measures the M1-M2, M1-M3 and M2-M3 lags
with GCC-PHAT, and checks the three things the design relies on:

  (a) the lags are integers           - it is a FIFO offset, not acoustics
  (b) the same in every file/reboot   - one calibration constant suffices
  (c) no drift within a file          - one clock domain really is one

and prints the value for CONFIG_UAV_M3_LAG_SAMPLES. Exit code 0 = PASS.
Try it on synthetic data first:

    uv run python scripts/click_test.py --demo
"""

import argparse
import sys

import numpy as np
from scipy import signal

from uavdoa import PAIRS, gcc_phat
from uavdoa.simulate import click_train, fractional_delay
from uavdoa.wavio import read_array_wav

MAX_LAG_S = 64 / 48_000  # generous: the offset can exceed the acoustic max


def find_clicks(x: np.ndarray, fs: int, min_gap_s: float = 0.2) -> np.ndarray:
    """Onsets of impulsive events: energy envelope peaks well above the median."""
    env = signal.lfilter(np.ones(64) / 64, 1, np.sum(x**2, axis=0))
    thr = np.median(env) * 30 + 1e-12
    peaks, _ = signal.find_peaks(env, height=thr, distance=int(min_gap_s * fs))
    return peaks


def measure_file(x: np.ndarray, fs: int, win: int = 2048) -> np.ndarray:
    """Per click: lags for the three pairs, (n_clicks, 3)."""
    out = []
    for p in find_clicks(x, fs):
        a, b = p - win // 2, p + win // 2
        if a < 0 or b > x.shape[1]:
            continue
        seg = x[:, a:b] * np.hanning(win)
        out.append([gcc_phat(seg[i], seg[j], fs, MAX_LAG_S, f_lo=500, f_hi=8000).lag for i, j in PAIRS])
    return np.array(out).reshape(-1, 3)


def demo_file(offset: int, seed: int, fs: int = 48_000) -> np.ndarray:
    rng = np.random.default_rng(seed)
    c = click_train(4.0, fs, 0.4, rng)
    x = np.stack([c, c, c]) + rng.normal(0, 0.002, (3, len(c)))
    x[2] = np.roll(x[2], offset)
    x[1] = fractional_delay(x[1], 0.02)  # a hair of real geometric error
    return x


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("wavs", nargs="*")
    ap.add_argument("--demo", action="store_true", help="run on 12 synthetic reboots with offset 3")
    ap.add_argument("--tol", type=float, default=0.25, help="max deviation from an integer / from the mean")
    a = ap.parse_args()

    files = []
    if a.demo:
        files = [(f"demo-{k:02d}", demo_file(3, k), 48_000) for k in range(12)]
    for path in a.wavs:
        x, fs = read_array_wav(path)
        files.append((path, x, fs))
    if not files:
        ap.error("give WAV files or --demo")

    per_file = []
    print(f"{'file':<28} clicks   M1-M2    M1-M3    M2-M3   drift(M1-M3)")
    for name, x, fs in files:
        lags = measure_file(x, fs)
        if len(lags) < 3:
            print(f"{name:<28} {len(lags):>4}   too few clicks, skipped")
            continue
        med = np.median(lags, axis=0)
        drift = np.ptp(lags[:, 1])
        per_file.append((med, drift))
        print(f"{name:<28} {len(lags):>4}  " + "  ".join(f"{m:+7.2f}" for m in med) + f"   {drift:6.2f}")

    if not per_file:
        print("FAIL: no usable recordings")
        return 1
    meds = np.array([m for m, _ in per_file])
    drifts = np.array([d for _, d in per_file])
    m3 = meds[:, 1]
    checks = {
        "(a) integer offsets": np.all(np.abs(meds - np.round(meds)) < a.tol),
        "(b) same across files/reboots": np.ptp(np.round(m3)) == 0 and np.ptp(meds[:, 0]) < a.tol,
        "(c) no drift within a file": np.all(drifts < 2 * a.tol),
    }
    print()
    for k, ok in checks.items():
        print(f"  {'PASS' if ok else 'FAIL'}  {k}")
    lag = int(np.round(-np.median(m3)))
    print(f"\n  CONFIG_UAV_M3_LAG_SAMPLES = {lag}   (M1-M3 lag reads {-lag:+d}: M3 is {lag} samples late)")
    return 0 if all(checks.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
