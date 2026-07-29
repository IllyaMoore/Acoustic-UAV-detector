# Acoustic UAV Detector

A passive acoustic drone detector. Three digital MEMS microphones, placed at the
vertices of an equilateral triangle, listen for propeller noise; the
microcontroller measures how much earlier the sound reached each microphone
(TDOA, via GCC-PHAT) and turns those differences into an azimuth — the bearing
to the source over a full 360°.

The device emits nothing: it only listens. That means it works without GNSS and
without radio — including where the RF spectrum is jammed — and needs no
transmit permit.

This is a learning prototype. First a breadboard build that records 3-channel
audio to microSD for offline analysis, then real-time azimuth estimation on the
device, and only after that a weatherized enclosure for outdoor battery
operation.

What it cannot do: range is modest (hundreds of meters, depending on background
noise and drone type), wind and urban noise degrade accuracy, and a flat
three-microphone array yields azimuth only — no elevation.

Technical details (pin map, array geometry, signal-processing pipeline) live in
[CLAUDE.md](CLAUDE.md).

## Components

### Microcontroller

![ESP32 board](imgs/controller.jpg)

A devkit board built around the ESP-WROOM-32 module: 30 pins, USB-C for power
and flashing. The brain of the device — it reads the microphones, runs the
correlation math, and writes results to the card.

The critical property is that the chip has **two** independent I2S controllers.
One controller serves only two microphones (the left and right channels of a
single stereo stream), so the third microphone goes on the second controller.
Both must run off synchronized clocking; otherwise the streams drift relative to
one another and all of the direction math becomes invalid.

Quantity: 1.

### Microphones

![INMP441 microphone module](imgs/microphone.jpg)

INMP441 — a digital MEMS microphone with an I2S output, on a round breakout
board (headers included, not yet soldered). A digital output means there is no
analog signal on the run from microphone to board that stray pickup could
corrupt — that is the main reason to choose this part over an analog capsule
plus preamp.

Supply is strictly 3.3 V; 5 V will destroy the module. The L/R pin selects which
slot of the stereo stream the microphone drives its data into: tied to GND it is
the left channel, tied to 3V3 the right. That is exactly how two microphones
share one I2S bus.

Quantity: 3 (one per triangle vertex, baseline about 15 cm).

### microSD module

![microSD module](imgs/stcard-slot.jpg)

A microSD card holder with an SPI interface (GND, MISO, SCK, MOSI, CS, power).
It stores raw 3-channel recordings (WAV) plus a log of detections and azimuths.
The recordings matter mostly for debugging the algorithm offline on a computer:
tuning filter and correlation parameters in Python against a saved file is far
easier than reflashing the board on every iteration.

Check the supply voltage this particular module expects before wiring it up:
compact boards are usually fed 3.3 V directly with no onboard regulator, while
larger ones carry an LDO and expect 5 V.

Quantity: 1 (plus the card itself).

### Rest of the build

- A flat, rigid base for the array (plywood, FR4, or a printed frame) — the
  triangle vertices must be fixed to within a few millimeters, because geometric
  error maps directly onto azimuth error.
- Foam windscreens for every microphone: wind is the main TDOA killer outdoors.
- Protoboard, wire, passives. Keep I2S runs short (10–15 cm); for spatially
  separated microphones use twisted pairs or shielded cable.
- Power: USB-C 5 V on the bench; for the field, an 18650 with a protection board
  and 3.3 V / 5 V regulation.
- Later: a weatherized (IP-rated) enclosure and a mount that raises the array
  off the ground.

## Project status

Components are in hand; assembly has not started. Next step is bringing up
capture on both I2S controllers and **proving** that the three streams are
sample-aligned. This gates everything else: without confirmed synchronization,
direction estimation is meaningless.
