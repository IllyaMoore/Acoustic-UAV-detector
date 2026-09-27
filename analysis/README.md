# Analysis

Python reference for the firmware DSP, tools for SD-card recordings, and the
host side (fusion, ATAK bridge).

```sh
cd analysis
uv run pytest -q     # includes the C++ vs Python check, needs g++
```

On a filesystem without symlinks (exFAT/NTFS) set
`UV_PROJECT_ENVIRONMENT=/somewhere/else/venv`.

| Module | |
|---|---|
| `geometry` | mic positions, azimuth convention |
| `gccphat` | single-frame GCC-PHAT |
| `doa` | bearing from three pair delays |
| `pipeline` | mirror of `firmware/components/dsp` |
| `simulate` | drone harmonics, uncorrelated wind, exact fractional delays, FIFO offset |
| `crlb` | Cramér–Rao bounds |
| `fusion` | multi-node triangulation with covariance |
| `nmea`, `cot` | parse `$UAVDOA`, build CoT XML |
| `wavio` | the SD card WAV format |

| Script | |
|---|---|
| `simulate_wav.py` | synthetic recording |
| `localize.py` | bearings over time from a WAV, optional plot |
| `click_test.py` | checks the SD_A/SD_B offset is constant, prints `UAV_M3_LAG_SAMPLES` |
| `error_budget.py` | accuracy vs SNR against the bound → `docs/img/error_budget.png` |
| `triangulate.py` | 3-node demo → `docs/img/triangulation.png` |
| `cot_bridge.py` | serial `$UAVDOA` → CoT over UDP multicast |

Without hardware:

```sh
uv run python scripts/simulate_wav.py /tmp/s.wav --az 250 --wind 0 --offset 3 --seconds 6
uv run python scripts/click_test.py --demo
uv run python scripts/localize.py /tmp/s.wav --m3-lag 3 --plot /tmp/s.png
../firmware/test/host/build/doa_cli /tmp/s.wav --m3-lag 3
uv run python scripts/cot_bridge.py --demo --dry-run
```
