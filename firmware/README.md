# Firmware

ESP-IDF, C++17, built with PlatformIO. Two targets: `esp32dev` (the WROOM-32
DevKit V1, default) and `esp32s3`. Why not Arduino or Rust:
[docs/RESEARCH.md](../docs/RESEARCH.md#firmware-stack).

```sh
pio run                   # build esp32dev
pio run -t upload         # flash
pio device monitor        # logs + $UAVDOA lines
pio run -t menuconfig     # "Acoustic UAV detector" menu
pio run -e esp32s3
make -C test/host test    # DSP unit tests on the PC, no board needed
```

## Layout

```
components/dsp/   no hardware dependencies, also built on the host
  fft             radix-2 complex FFT
  gcc_phat        PHAT peak by evaluating only the physical lags
  doa             pair delays -> azimuth, |elevation|, closure check
  pipeline        framing, cross-spectrum averaging, detector, tracker
  aligner         removes the fixed SD_A/SD_B offset
main/             hardware
  capture         I2S0 master + I2S1 slave
  sd_logger       WAV + detections.csv, own task
  report          $UAVDOA / $UAVCAL lines
  board_pins.hpp  WROOM-32 and S3 pin maps
test/host/        unit tests, and doa_cli which runs the pipeline on a WAV
```

`doa_cli` runs the exact on-chip DSP over a recording pulled off the SD card,
so thresholds can be tuned without reflashing.

## Tasks

| Task | Core | Prio | Does |
|---|---|---|---|
| capture | 0 | 10 | reads both I2S, aligns, sends frames to dsp, audio to the SD ring |
| sd_logger | 0 | 3 | writes `rec_NNNNN.wav` and `detections.csv` |
| dsp | 1 | 8 | pipeline, prints `$UAVDOA`, drives the LED |

Capture never waits. If DSP falls behind it drops a frame, if the card stalls it
drops an audio chunk, and both counts show up in a status log every 10 s along
with the measured DSP load. Memory is a fixed 4-frame pool and a 64 kB ring
buffer, about 82 kB of static RAM on the WROOM-32. The WAV keeps the raw offset
so it can be re-measured later; only the DSP path is aligned.

## Config (menuconfig)

| Option | Default | |
|---|---|---|
| `UAV_NODE_ID` | 1 | |
| `UAV_ARRAY_HEADING_DEG` | 0 | true heading of the M1 arm |
| `UAV_M3_LAG_SAMPLES` | 0 | from the click test |
| `UAV_CLOCK_LOOPBACK_*` | external | jumper wires, or internal via the GPIO matrix |
| `UAV_SWAP_SD_A_SLOTS` | n | if M1 shows up on channel 2 |
| `UAV_RECORD_WAV` | y | 3 ch, 16 bit, 48 kHz = 288 kB/s |
| `UAV_WAV_FILE_SECONDS` | 60 | header rewritten every 2 s, survives power loss |
| `UAV_WAV_GAIN_BITS` | 2 | shift before cutting 24 bits to 16 |
| `UAV_CALIBRATION_MODE` | n | click test: 500–8000 Hz, no alignment, prints `$UAVCAL` |
| `UAV_STATUS_LED_GPIO` | 2 / -1 | onboard LED on the WROOM devkit |

Clock return jumpers for the external mode:

| | WROOM-32 | ESP32-S3 |
|---|---|---|
| SCK to I2S1 | GPIO26 → GPIO14 | GPIO16 → GPIO4 |
| WS to I2S1 | GPIO25 → GPIO27 | GPIO15 → GPIO5 |

On the bench GPIO14/27 used to drive M3's clock from a second master. Here they
are inputs, so M3's SCK/WS have to move to GPIO26/25 first or M3 gets no clock.

## Output

```
$UAVDOA,<node>,<uptime_ms>,<det>,<az_rel>,<az_true>,<el>,<conf>,<snr_db>,<coh>,<trk>,<trk_az_true>*HH
$UAVDOA,1,48213,1,62.4,152.4,18.7,0.83,11.2,0.71,1,151.9*38
```

One line per 256 ms, NMEA-style XOR checksum, `el` is -1 when undefined.
`analysis/scripts/cot_bridge.py` turns these into CoT for ATAK.

## Bring-up

1. Flash, open the monitor, check 0 dropped frames. Scope SCK (3.072 MHz) and
   WS (48 kHz) at the far mic.
2. Tap each mic with a pencil and look at the WAV (Audacity is fine): M1 on
   channel 1, M2 on 2, M3 on 3. If M1 and M2 are swapped, set
   `UAV_SWAP_SD_A_SLOTS`.
3. Click test: calibration mode on, clicks above the centroid, a dozen reboots,
   then `analysis/scripts/click_test.py rec_*.wav`. Copy the value into
   `UAV_M3_LAG_SAMPLES`, calibration mode off.
4. Play drone audio from a speaker at known bearings 3–5 m away. A constant
   error means the heading, mic coordinates or channel map is wrong.
5. Outdoors with windscreens: record, replay through `doa_cli`, retune the
   thresholds in `PipelineConfig`.

None of this has been done with this firmware yet. Expect step 2 and 3 to turn
up problems in `capture.cpp` (slot order, slave start-up).
