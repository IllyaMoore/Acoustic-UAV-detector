# Research notes

Reading done before writing the firmware, September 2026. Unverified items are
marked.

## Who else does this

Acoustic detection got serious in Ukraine because it's cheap, passive and
ignores jamming.

- **Sky Fortress.** A microphone and a phone on a pole, roughly 9,500–14,000
  nodes at $400–1,000 each, fused centrally and pushed to mobile fire teams.
  [Defense One](https://www.defenseone.com/defense-systems/2024/07/ukraines-cheap-sensors-are-helping-troops-fight-waves-russian-drones/398204/),
  [United24](https://united24media.com/war-in-ukraine/sky-fortress-ukraines-acoustic-detection-system-that-tracks-drones-cheap-and-fast-9451)
- **Zvook.** Acoustic mirrors on 10–12 m masts along the border, feeding the
  Delta system in about 12 s; newer FPV and vehicle-mounted sensors.
  [United24](https://united24media.com/defense-tech/ukraines-zvook-acoustic-system-tests-mobile-detection-of-low-flying-missiles-and-drones-21587),
  [Unmanned Airspace](https://www.unmannedairspace.info/counter-uas-systems-and-policies/ukrainian-start-up-zvook-launches-new-low-cost-c-uas-acoustic-sensor/)
- **Europe.** Squarehead Discovair (128 mics plus camera,
  [sqhead.com](https://www.sqhead.com/drone-detection)), Monava
  ([tech.eu](https://tech.eu/2026/08/04/monava-closes-funding-round-as-demand-for-passive-drone-detection-grows/)),
  Neuron Soundware Sound Shield at €100–150 per sensor
  ([TNW](https://thenextweb.com/news/neuron-soundware-sound-shield-acoustic-drone-detection-czech-ai)).

The pattern is many cheap nodes, each reporting a bearing with a confidence,
fused centrally and delivered into an existing C2 tool (Delta, TAK). The
software here follows that: the node sends a bearing line, the host fuses and
publishes CoT.

## Signatures

- Electric multirotors and FPV: blade-pass harmonics from low hundreds of Hz to
  a few kHz, shifting with throttle. Short range (one vendor claims 200–300 m
  for FPV, [Defense Mirror](https://defensemirror.com/news/40656/), unverified).
- Shahed-136: piston two-stroke (MD-550, a Limbach L550E derivative), strong
  harmonics around 200–2000 Hz, audible for kilometres.
  [IntechOpen](https://www.intechopen.com/chapters/1203123)
- No public Shahed or FPV recordings exist
  ([OSINTO](https://www.osinto.com/the-osborne-report/acoustic-drone-detection/)).

Hence the default 200–4000 Hz band, which also keeps most wind out.

## Processing

Usual pipeline: band-limit, detect (harmonic features or a CNN on
mel/MFCC; 98–99 % in controlled tests, much worse with distance,
[review](https://pubs.aip.org/aip/adv/article/15/12/120701/3373725/)), estimate
direction with GCC-PHAT, SRP-PHAT or MUSIC, then fuse across nodes. This repo
uses a rule-based detector (band SNR, tonality, coherence), GCC-PHAT with
cross-spectra averaged over 256 ms, and weighted least-squares fusion. The
closest paper I found is
[Acta Acustica 2026](https://acta-acustica.edpsciences.org/articles/aacus/full_html/2026/01/aacus250134/aacus250134.html)
(MEMS mics, TDOA, random-forest classifier; read the abstract only).

## Open source

| Project | Licence | Use |
|---|---|---|
| [ESP-IDF](https://github.com/espressif/esp-idf) `i2s_std` | Apache-2.0 | capture, I2S1 as slave |
| [esp-dsp](https://github.com/espressif/esp-dsp) | Apache-2.0 | possible faster FFT later ([benchmarks](https://docs.espressif.com/projects/esp-dsp/en/latest/esp32/esp-dsp-benchmarks.html)) |
| [pyroomacoustics](https://github.com/LCAV/pyroomacoustics) | MIT | room/reflection simulation, SRP/MUSIC comparison |
| [ODAS](https://github.com/introlab/odas) | MIT | reference embedded SRP-PHAT and tracker |
| [PyTAK](https://github.com/snstac/pytak) | Apache-2.0 | if TLS to a TAK Server is needed |
| TFLite Micro, ESP-NN, ESP-DL | Apache-2.0 (ESP-DL unverified) | future on-device classifier |

Datasets: [DroneAudioSet](https://huggingface.co/datasets/ahlab-drone-project/DroneAudioSet/)
(MIT, 23.5 h, recorded on the drone),
[Al-Emadi](https://github.com/saraalemadi/DroneAudioDataset) (no licence,
cite only), [DREGON](https://dregon.inria.fr/) (academic use),
[DDL](https://zenodo.org/records/6459183) (licence not checked).

## Firmware stack

| Option | Verdict |
|---|---|
| Arduino-ESP32 | No: its I2S library has no slave mode ([#9945](https://github.com/espressif/arduino-esp32/issues/9945)) |
| Rust `esp-hal` | Not yet: I2S is `unstable` and master-only as of 1.2 ([docs](https://docs.rs/esp-hal/latest/esp_hal/i2s/index.html)) |
| Rust `esp-idf-hal` | Wraps the IDF driver; slave support not checked |
| MicroPython | Too slow for the DSP, and its I2S is master-only (confirmed on the bench) |
| **ESP-IDF C++** | Chosen: documented slave role, pinned tasks per core, esp-dsp/ESP-NN available |

IDF has had slave-mode bugs ([#15497](https://github.com/espressif/esp-idf/issues/15497),
[#6784](https://github.com/espressif/esp-idf/issues/6784)), so the firmware
starts the slave before the master and treats the offset as something to
measure.

## Compute (estimate)

Per 21 ms frame: three 2048-point FFTs and cross-spectra over ~160 bins. Every
256 ms: GCC-PHAT evaluated at 45 lags × ~160 bins per pair (about 7k complex
MACs), instead of three inverse FFTs. My guess is 10–15 % of one core on the
ESP32; the firmware logs the real figure every 10 s. Band-pass is done by
dropping FFT bins, since a 256-tap FIR on three channels would eat most of a
core.
