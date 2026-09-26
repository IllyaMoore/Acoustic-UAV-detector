"""Accuracy vs SNR: Monte Carlo through the real pipeline, against the CRLB.

    uv run python scripts/error_budget.py --out ../docs/img/error_budget.png

Also prints the error budget table used in the README: how each physical
error source maps to degrees of azimuth.
"""

import argparse

import numpy as np

from uavdoa import angle_diff_deg, crlb
from uavdoa.pipeline import Pipeline, PipelineConfig
from uavdoa.simulate import Scene, simulate

FS = 48_000
BAND = (200.0, 4000.0)


def in_band_snr_db(broadband_snr_db: float) -> float:
    # The simulator sets SNR against white noise spread over 0..fs/2; nearly
    # all drone power sits inside the analysis band, only a slice of the noise.
    return broadband_snr_db + 10 * np.log10((FS / 2) / (BAND[1] - BAND[0]))


def monte_carlo(snrs, trials, wind_db=-100.0):
    cfg = PipelineConfig()
    block_s = cfg.frame_len * cfg.avg_frames / FS
    rms = []
    for snr in snrs:
        errs = []
        for k in range(trials):
            az = (k * 37.0) % 360
            x = simulate(Scene(az_deg=az, el_deg=15, snr_db=snr, wind_db=wind_db),
                         seconds=block_s + 0.01, seed=1000 + k)
            errs.append(angle_diff_deg(Pipeline(cfg).run(x)[0].az_deg, az))
        errs = np.array(errs)
        # Robust RMS: gross outliers (a pair locking onto noise) are reported
        # separately by the detector's closure check, not averaged in here.
        good = np.abs(errs) < 30
        rms.append((np.sqrt(np.mean(errs[good] ** 2)) if good.any() else np.nan, 1 - good.mean()))
    return np.array(rms), block_s


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="error_budget.png")
    ap.add_argument("--trials", type=int, default=40)
    a = ap.parse_args()

    snrs = np.arange(-20, 21, 5)
    calm, block_s = monte_carlo(snrs, a.trials)
    windy, _ = monte_carlo(snrs, a.trials, wind_db=0.0)
    ib = np.array([in_band_snr_db(s) for s in snrs])
    bound = [crlb.azimuth_crlb_deg(s, 30, 15, *BAND, block_s) for s in ib]

    print("in-band SNR [dB]   RMS az calm   RMS az wind=0dB   CRLB    outliers calm/wind")
    for s, c, w, b in zip(ib, calm, windy, bound):
        print(f"  {s:6.1f}          {c[0]:7.2f}        {w[0]:7.2f}        {b:6.3f}     {c[1]:.0%} / {w[1]:.0%}")

    print("\nError budget (azimuth, 1 sigma, 150 mm baseline):")
    print(f"  1 mm mic position error ............. {crlb.geometry_error_deg(1e-3):.2f} deg")
    print("  1 sample of timing (7.1 mm) ......... %.2f deg" % crlb.geometry_error_deg(343 / FS))
    print("  0.1 sample (sub-sample interp.) ..... %.2f deg" % crlb.geometry_error_deg(0.1 * 343 / FS))
    print("  speed of sound +-5 % (+-15 degC) .... ~0 deg azimuth (common scale), biases elevation")
    print("  unknown M1 heading ................... 1:1 - measure it")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(7.5, 4.5))
    ax.semilogy(ib, calm[:, 0], "o-", label="pipeline, calm")
    ax.semilogy(ib, windy[:, 0], "s-", label="pipeline, wind = drone level")
    ax.semilogy(ib, bound, "k--", label="CRLB (flat-spectrum approx.)")
    ax.axhline(crlb.geometry_error_deg(1e-3), c="C3", ls=":", label="1 mm geometry error")
    ax.set_xlabel("in-band SNR per channel, dB (200-4000 Hz)")
    ax.set_ylabel("azimuth RMS error, deg")
    ax.set_title(f"3-mic, 150 mm, one {block_s * 1000:.0f} ms block, {a.trials} trials/point")
    ax.grid(True, which="both", alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(a.out, dpi=130)
    print(f"\nplot -> {a.out}")


if __name__ == "__main__":
    main()
