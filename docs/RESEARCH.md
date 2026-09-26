# Research notes: where this project sits

Background reading done before writing the firmware (September 2026): what
already exists, what the field does, and why the software is built the way
it is. Claims I could not verify are marked as such. Links were live when
checked.

## 1. The problem, and who else is solving it

Acoustic detection became a serious counter-UAS layer in Ukraine because it
is cheap, passive and does not care about RF jamming or GNSS denial.

- **Sky Fortress (Небесна фортеця).** Started as a microphone and a phone on
  a 2 m pole. Estimates range from ~9,500 to ~14,000 nodes at $400-1,000
  each. A central server fuses the nodes and pushes tracks to mobile fire
  teams on tablets. It is reported to have covered 80 of 84 UAVs in one
  attack, and further nodes have been reported as NATO-funded.
  [Defense One](https://www.defenseone.com/defense-systems/2024/07/ukraines-cheap-sensors-are-helping-troops-fight-waves-russian-drones/398204/),
  [United24](https://united24media.com/war-in-ukraine/sky-fortress-ukraines-acoustic-detection-system-that-tracks-drones-cheap-and-fast-9451)
- **Zvook.** Uses 0.5 m acoustic mirrors on 10-12 m masts near the border.
  Detections reach the Delta situational-awareness system in about 12 s. It
  has since added a low-cost Ethernet FPV sensor and a vehicle-mounted variant.
  [United24](https://united24media.com/defense-tech/ukraines-zvook-acoustic-system-tests-mobile-detection-of-low-flying-missiles-and-drones-21587),
  [Unmanned Airspace](https://www.unmannedairspace.info/counter-uas-systems-and-policies/ukrainian-start-up-zvook-launches-new-low-cost-c-uas-acoustic-sensor/)
- **Europe:**
  - Squarehead Discovair G2+ (Norway): a 128-microphone array plus a camera,
    at the expensive end. [sqhead.com](https://www.sqhead.com/drone-detection)
  - Monava (Nordics): passive acoustic sensing plus AI, deployed in Ukraine.
    [tech.eu](https://tech.eu/2026/08/04/monava-closes-funding-round-as-demand-for-passive-drone-detection-grows/)
  - Neuron Soundware Sound Shield (Czech): around €100-150 per sensor.
    [TNW](https://thenextweb.com/news/neuron-soundware-sound-shield-acoustic-drone-detection-czech-ai)

**What that means for this build.** The value is not in one clever node. It
comes from many cheap nodes, each doing honest edge DSP and saying clearly how
sure it is, fused centrally and delivered into the C2 system people already use
(Delta in Ukraine, TAK for NATO-aligned forces). The software here is shaped
around exactly that: a node emits a bearing plus confidence in a simple
self-checking wire format, and a host bridge fuses bearings from several nodes
and publishes CoT.

## 2. Signatures

- **Small electric multirotors and FPV drones.** Blade-pass harmonics sit
  roughly in the low hundreds of Hz to a few kHz and shift with throttle.
  Range is short: a vendor claims FPV detection at 200-300 m
  ([Defense Mirror](https://defensemirror.com/news/40656/), unverified).
- **Shahed-136 / Geran-2.** A piston two-stroke engine (MD-550, derived from
  the Limbach L550E) produces strong engine harmonics at roughly 200-2000 Hz
  over a broadband floor. It is audible at kilometres.
  [IntechOpen](https://www.intechopen.com/chapters/1203123)
- **Gap.** There is no public Shahed or FPV audio dataset
  ([OSINTO review](https://www.osinto.com/the-osborne-report/acoustic-drone-detection/)).
  Characterising a real target remains a field task (see README, next steps).

The default analysis band, **200-4000 Hz**, covers both classes and keeps
wind (mostly below 200 Hz) out of the correlation.

## 3. State of the art in processing

| Stage | Common choices | What this repo uses |
| --- | --- | --- |
| Detection | Harmonic/BPF features; CNN or CRNN on mel/MFCC (98-99 % in controlled tests, dropping sharply with distance: [review](https://pubs.aip.org/aip/adv/article/15/12/120701/3373725/)) | Rule-based: band SNR plus tonality plus inter-mic coherence; a classifier slot is left for TinyML |
| Direction | GCC-PHAT TDOA; SRP-PHAT; MUSIC | GCC-PHAT on cross-spectra averaged over 256 ms, least squares over 3 pairs |
| Sanity | Rarely documented | Closure check tau01 + tau12 - tau02 ≈ 0 |
| Network | Central fusion, triangulation, tracking | Bearing-only weighted least squares with a covariance / CEP |
| Output | Proprietary, Delta, CoT | NMEA-style sentences on the node, CoT from the host bridge |

The closest academic match: [Acta Acustica 2026](https://acta-acustica.edpsciences.org/articles/aacus/full_html/2026/01/aacus250134/aacus250134.html)
(distributed MEMS microphones, TDOA azimuth, Random-Forest classifier). I
could only read the abstract.

## 4. Reusable open source

| Project | Licence | Used for |
| --- | --- | --- |
| [ESP-IDF](https://github.com/espressif/esp-idf) `i2s_std` driver | Apache-2.0 | capture: I2S0 master, I2S1 slave |
| [esp-dsp](https://github.com/espressif/esp-dsp) | Apache-2.0 | not yet; the drop-in FFT once profiled (1024-pt complex radix-2: ~113k cycles on ESP32, [benchmarks](https://docs.espressif.com/projects/esp-dsp/en/latest/esp32/esp-dsp-benchmarks.html)) |
| [pyroomacoustics](https://github.com/LCAV/pyroomacoustics) | MIT | future: room/reflection simulation, SRP-PHAT/MUSIC comparisons |
| [ODAS](https://github.com/introlab/odas) | MIT | reference for an embedded SRP-PHAT and tracker |
| [PyTAK](https://github.com/snstac/pytak) | Apache-2.0 | alternative to the stdlib CoT code here if TLS to a TAK Server is needed |
| TFLite Micro / ESP-NN / ESP-DL | Apache-2.0 (ESP-DL licence not verified) | future on-node classifier |

Datasets for a future classifier:

- [DroneAudioSet](https://huggingface.co/datasets/ahlab-drone-project/DroneAudioSet/)
  (MIT, 23.5 h; recorded on board, so it suits ego-noise work)
- [Al-Emadi DroneAudioDataset](https://github.com/saraalemadi/DroneAudioDataset)
  (**no licence stated**, so cite it and do not redistribute)
- [DREGON](https://dregon.inria.fr/) (academic use only)
- [DDL on Zenodo](https://zenodo.org/records/6459183) (licence not checked)

## 5. Firmware stack: the decision

| Option | Verdict |
| --- | --- |
| Arduino-ESP32 | **Rejected.** The current `ESP_I2S` library does not support slave mode ([issue #9945](https://github.com/espressif/arduino-esp32/issues/9945)), and the whole capture design depends on I2S1 as a slave. |
| Rust, `esp-hal` | **Rejected for now.** As of esp-hal 1.2 (Sept 2026), I2S sits behind `unstable` and offers master mode only, no slave ([docs](https://docs.rs/esp-hal/latest/esp_hal/i2s/index.html)). Worth revisiting. |
| Rust, `esp-idf-hal` | Possible (it wraps the IDF driver), but slave support was not verified. It adds a layer without removing the C dependency. |
| MicroPython / Zephyr | MicroPython is too slow for the DSP. Zephyr offers no advantage here. |
| **ESP-IDF, C++17, FreeRTOS** | **Chosen.** It has a documented slave role, cores can be pinned (capture and DSP on separate cores), it gives direct access to esp-dsp / ESP-NN later, and PlatformIO wraps it for a one-command build. |

**Known risk:** I2S slave-mode bugs in IDF are on record
([#15497](https://github.com/espressif/esp-idf/issues/15497),
[#6784](https://github.com/espressif/esp-idf/issues/6784)). The firmware
therefore enables the slave before the master starts the clock, and it treats
the FIFO offset as something to **measure** (the click test), never assume.

## 6. Compute budget (estimate, to be measured)

Per 21.3 ms frame the node does three 2048-point complex FFTs, cross-spectrum
products over ~160 in-band bins, and every 12 frames a direct 45-lag PHAT
evaluation. That is about 5k complex MACs per pair, far cheaper than the
three 2048-point inverse FFTs it replaces.

The portable radix-2 FFT should cost roughly 0.25M cycles per transform on
the ESP32. That puts the pipeline at roughly **10-15 % of one 240 MHz core**
(my estimate, not a measurement). The firmware logs its real DSP load every
10 s, and that number replaces this paragraph once the board runs.

The one trap the research flagged is band-pass filtering in the time domain:
a 256-tap FIR on three channels would take most of a core. It is avoided
entirely by masking bins in the frequency domain.
