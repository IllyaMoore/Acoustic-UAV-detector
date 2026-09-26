"""Run the pipeline over a recording and plot what the device would report.

    uv run python scripts/localize.py rec_00012.wav --m3-lag 3 --plot out.png

Prints one line per 256 ms block; with --plot draws azimuth, SNR and
coherence over time with detections highlighted.
"""

import argparse

import numpy as np

from uavdoa.pipeline import Pipeline, PipelineConfig
from uavdoa.wavio import read_array_wav


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("wav")
    ap.add_argument("--m3-lag", type=int, default=0, help="CONFIG_UAV_M3_LAG_SAMPLES from the click test")
    ap.add_argument("--band", type=float, nargs=2, default=(200, 4000), metavar=("LO", "HI"))
    ap.add_argument("--plot", help="save a PNG here")
    a = ap.parse_args()

    x, fs = read_array_wav(a.wav)
    if a.m3_lag > 0:
        x = np.stack([np.roll(x[0], a.m3_lag), np.roll(x[1], a.m3_lag), x[2]])
    elif a.m3_lag < 0:
        x = np.stack([x[0], x[1], np.roll(x[2], -a.m3_lag)])
    cfg = PipelineConfig(fs=fs, f_lo=a.band[0], f_hi=a.band[1])
    blocks = Pipeline(cfg).run(x)
    dt = cfg.frame_len * cfg.avg_frames / fs

    print(" t[s]    az    el   snr   coh  tonal  det  track")
    for b in blocks:
        el = f"{b.el_deg:5.1f}" if np.isfinite(b.el_deg) else "  -  "
        trk = f"{b.track_az_deg:6.1f}" if b.tracking else "     -"
        print(f"{(b.index + 1) * dt:5.2f} {b.az_deg:6.1f} {el} {b.snr_db:5.1f} {b.coherence:5.2f} "
              f"{b.tonality:5.2f}   {'*' if b.detected else ' '}  {trk}")

    if a.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        t = (np.array([b.index for b in blocks]) + 1) * dt
        det = np.array([b.detected for b in blocks])
        fig, ax = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
        az = np.array([b.az_deg for b in blocks])
        ax[0].scatter(t[~det], az[~det], s=8, c="0.7", label="no detection")
        ax[0].scatter(t[det], az[det], s=12, c="C0", label="detected")
        ax[0].set_ylim(0, 360)
        ax[0].set_ylabel("azimuth, deg")
        ax[0].legend(loc="upper right")
        ax[1].plot(t, [b.snr_db for b in blocks])
        ax[1].axhline(cfg.min_snr_db, ls="--", c="0.5")
        ax[1].set_ylabel("band SNR, dB")
        ax[2].plot(t, [b.coherence for b in blocks], label="coherence")
        ax[2].plot(t, [b.tonality for b in blocks], label="tonality")
        ax[2].set_ylabel("0..1")
        ax[2].set_xlabel("time, s")
        ax[2].legend(loc="upper right")
        fig.tight_layout()
        fig.savefig(a.plot, dpi=120)
        print(f"plot -> {a.plot}")


if __name__ == "__main__":
    main()
