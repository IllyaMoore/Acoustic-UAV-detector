"""Write a synthetic 3-channel recording in the firmware's SD-card format.

    uv run python scripts/simulate_wav.py out.wav --az 60 --el 20 --snr 10 --wind 0 --offset 3 --onset 2

Useful for exercising doa_cli / localize.py / click_test.py before any
hardware exists, and as a regression input.
"""

import argparse

from uavdoa.simulate import DroneSource, Scene, simulate
from uavdoa.wavio import write_array_wav


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out")
    ap.add_argument("--az", type=float, default=60.0, help="azimuth, deg clockwise from M1")
    ap.add_argument("--el", type=float, default=20.0)
    ap.add_argument("--snr", type=float, default=10.0, help="drone vs sensor noise, dB")
    ap.add_argument("--wind", type=float, default=-100.0, help="wind vs drone, dB (-100 = calm)")
    ap.add_argument("--offset", type=int, default=0, help="FIFO offset: samples M3 lags")
    ap.add_argument("--bpf", type=float, default=180.0, help="blade-pass frequency, Hz")
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--onset", type=float, default=2.0,
                    help="seconds of background before the drone appears (the noise floor needs them)")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    x = simulate(Scene(az_deg=a.az, el_deg=a.el, snr_db=a.snr, wind_db=a.wind, fifo_offset=a.offset,
                       onset_s=a.onset),
                 seconds=a.seconds, src=DroneSource(bpf_hz=a.bpf), seed=a.seed)
    write_array_wav(a.out, x)
    print(f"wrote {a.out}: {a.seconds:.1f} s, az {a.az} el {a.el}, snr {a.snr} dB, wind {a.wind} dB")


if __name__ == "__main__":
    main()
