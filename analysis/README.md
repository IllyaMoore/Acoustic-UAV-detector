# Analysis

The Python side has three jobs:

- It is the **reference implementation** the firmware is checked against.
- It is the **workbench** for recordings pulled off the SD card.
- It holds the **host software** that turns node output into a map picture.

```sh
cd analysis
uv run pytest -q                 # reference tests + C++/Python parity (needs g++)
```

(`uv` creates the environment on first run. If the checkout sits on a
filesystem without symlinks (exFAT/NTFS), point it elsewhere with
`UV_PROJECT_ENVIRONMENT=/some/where/venv`.)

## Package `uavdoa`

| Module | What |
| --- | --- |
| `geometry` | mic coordinates, azimuth convention, far-field delays |
| `gccphat` | textbook GCC-PHAT on one frame (IFFT + parabolic fit) |
| `doa` | least-squares bearing from three pair delays, closure check |
| `pipeline` | line-for-line mirror of `firmware/components/dsp` |
| `simulate` | drone harmonics + broadband noise, exact fractional delays, wind that is uncorrelated between capsules, FIFO offset |
| `crlb` | Cramér-Rao bounds for TDOA and azimuth |
| `fusion` | multi-node bearing-only triangulation with covariance / CEP |
| `nmea`, `cot` | parse `$UAVDOA`, build Cursor-on-Target XML |
| `wavio` | the SD card's WAV layout |

## Scripts

| Script | Use |
| --- | --- |
| `simulate_wav.py` | make a synthetic recording (az, el, SNR, wind, FIFO offset) |
| `localize.py` | run the pipeline over a WAV, print and plot bearings |
| `click_test.py` | **the gate**: prove the SD_A/SD_B offset is constant, print `UAV_M3_LAG_SAMPLES` |
| `error_budget.py` | Monte Carlo accuracy vs SNR against the CRLB → `docs/img/error_budget.png` |
| `triangulate.py` | 3-node fusion demo → `docs/img/triangulation.png` |
| `cot_bridge.py` | serial `$UAVDOA` → fused CoT → ATAK / WinTAK over UDP multicast |

A full offline loop with no hardware:

```sh
uv run python scripts/simulate_wav.py /tmp/s.wav --az 250 --wind 0 --offset 3 --seconds 6
uv run python scripts/click_test.py --demo
uv run python scripts/localize.py /tmp/s.wav --m3-lag 3 --plot /tmp/s.png
../firmware/test/host/build/doa_cli /tmp/s.wav --m3-lag 3     # same, with the firmware's C++
uv run python scripts/cot_bridge.py --demo --dry-run           # what ATAK would receive
```
