# Firmware

ESP-IDF (C++17, FreeRTOS), built with PlatformIO or plain `idf.py`. Targets:
the ESP-WROOM-32 devkit on hand (`esp32dev`, the default) and the ESP32-S3
DevKitC (`esp32s3`). Why ESP-IDF and not Arduino or Rust: see
[docs/RESEARCH.md §5](../docs/RESEARCH.md#5-firmware-stack-the-decision).

## Layout

```
firmware/
├── components/dsp/        hardware-free signal chain (also built on the host)
│   ├── fft                radix-2 complex FFT
│   ├── gcc_phat           PHAT peak by direct lag evaluation, sub-sample fit
│   ├── doa                3 pair delays -> azimuth (+|elevation|), closure check
│   ├── pipeline           framing, cross-spectrum averaging, detector, tracker
│   └── aligner            removes the constant SD_A/SD_B FIFO offset
├── main/                  everything that touches hardware
│   ├── capture            I2S0 master + I2S1 slave, one clock domain
│   ├── sd_logger          WAV + detections.csv on microSD, own task
│   ├── report             $UAVDOA / $UAVCAL sentences
│   ├── board_pins.hpp     WROOM-32 and S3 pin maps
│   ├── Kconfig.projbuild  node id, heading, FIFO offset, modes
│   └── app_main           tasks and wiring
└── test/host/             unit tests + doa_cli (runs the pipeline on a WAV)
```

The split is the point. Everything in `components/dsp` compiles and is
tested on a laptop in about a second, and `doa_cli` replays SD-card recordings
through exactly the code that runs on the chip. Only `main/` needs hardware.

## Build, flash, watch

```sh
cd firmware
pio run                        # build for esp32dev
pio run -t upload              # flash
pio device monitor             # console: logs + $UAVDOA sentences
pio run -t menuconfig          # "Acoustic UAV detector" menu
pio run -e esp32s3             # the S3 variant
make -C test/host test         # host unit tests, no board needed
```

## Tasks

```
core 0  capture    prio 10  I2S0+I2S1 -> aligner -> float frame ──queue──► dsp
                                        └─► int16 chunk ──ring buffer──► sd_logger
core 0  sd_logger  prio 3   ring -> rec_NNNNN.wav, lines -> detections.csv
core 1  dsp        prio 8   uav::Pipeline -> $UAVDOA on UART, LED, CSV line
```

- **Capture never blocks.** If DSP falls behind, a frame is dropped and
  counted. If the card stalls (cards do, for 100+ ms), an audio chunk is
  dropped and counted. Both counters appear in the 10-second status log with
  the measured DSP load and free heap.
- **Fixed memory.** There is a 4-frame pool and a 64 kB ring, with no malloc
  in the steady state. Static RAM is about 82 kB on the WROOM-32.
- **The raw WAV keeps the FIFO offset**, so it can be re-measured offline;
  only the DSP path is aligned.

## Configuration (menuconfig → "Acoustic UAV detector")

| Option | Default | Notes |
| --- | --- | --- |
| `UAV_NODE_ID` | 1 | carried in every sentence |
| `UAV_ARRAY_HEADING_DEG` | 0 | true heading of the M1 arm, measured at siting |
| `UAV_M3_LAG_SAMPLES` | 0 | from the click test |
| `UAV_CLOCK_LOOPBACK_*` | external | the jumper (visible on a scope) or the GPIO matrix |
| `UAV_SWAP_SD_A_SLOTS` | n | if tapping M1 shows on channel 2 |
| `UAV_RECORD_WAV` | y | 3 ch × 16 bit × 48 kHz = 288 kB/s |
| `UAV_WAV_FILE_SECONDS` | 60 | header rewritten every 2 s, so it survives power loss |
| `UAV_WAV_GAIN_BITS` | 2 | shift before truncating 24 → 16 bits |
| `UAV_CALIBRATION_MODE` | n | click-test mode: 500-8000 Hz, no alignment, $UAVCAL |
| `UAV_STATUS_LED_GPIO` | 2 / -1 | the onboard LED on the WROOM devkit |

Pins follow CLAUDE.md. In external-loopback mode, add two jumpers:

| | WROOM-32 | ESP32-S3 |
| --- | --- | --- |
| SCK → I2S1 BCK in | GPIO26 → GPIO27 | GPIO16 → GPIO4 |
| WS → I2S1 WS in | GPIO25 → GPIO14 | GPIO15 → GPIO5 |

## Output

```
$UAVDOA,<node>,<uptime_ms>,<det>,<az_rel>,<az_true>,<el>,<conf>,<snr_db>,<coh>,<trk>,<trk_az_true>*HH
$UAVDOA,1,48213,1,62.4,152.4,18.7,0.83,11.2,0.71,1,151.9*38
```

One sentence every 256 ms. `el` is `-1` when undefined. The XOR checksum
follows NMEA 0183. `analysis/scripts/cot_bridge.py` turns these into
Cursor-on-Target for ATAK.

## Bring-up checklist

These steps are ordered: each one is the prerequisite for the next.

1. **Power and clocks.** Flash, open the monitor, check that the status line
   shows 0 dropped frames. Scope SCK (3.072 MHz) and WS (48 kHz) at the far
   mic.
2. **Channel map.** Tap each mic in turn with a pencil and run
   `analysis/scripts/localize.py` on the recording (or look at it in
   Audacity). M1 must land on channel 1, M2 on 2, M3 on 3. If M1 and M2 are
   swapped, set `UAV_SWAP_SD_A_SLOTS`.
3. **Click test (the gate).** Set `UAV_CALIBRATION_MODE`. Put a click source
   above the centroid, record across a dozen reboots, and run
   `analysis/scripts/click_test.py rec_*.wav`. All three checks must PASS.
   Copy the printed value into `UAV_M3_LAG_SAMPLES` and clear calibration
   mode.
4. **Known bearing.** Play drone audio from a speaker at a measured bearing
   and elevation 3-5 m away. Compare `$UAVDOA` with the truth at 8 bearings.
   The residual error should look like `docs/img/error_budget.png`, not
   like a constant offset (a constant offset means the heading, the mic
   coordinates or the channel map is wrong).
5. **Outdoors.** With windscreens fitted, record, replay through `doa_cli`,
   and re-tune the detector thresholds in `PipelineConfig`.

## Honest status

The firmware builds for both targets, and its DSP core is covered by host
tests and cross-checked against the Python reference. **It has not yet run on
the assembled array**, because that hardware is not currently accessible. Expect the bring-up above to find issues in `capture.cpp` (slot order,
slave start-up) that no simulator can.
