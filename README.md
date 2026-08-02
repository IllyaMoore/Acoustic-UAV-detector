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

## The whole thing on one page

![System schematic: wiring, board sizes, head assembly, array geometry](imgs/schema.jpg)

Everything the build needs is on this one sheet: the signal chain down the left,
the board footprints and the head assembly across the top right, and the array
geometry — triangle and edge-on view — along the bottom right. The sections below
just read it out in words.

## How it is wired

An I2S bus carries two channels, so three microphones need both of the chip's I2S
controllers. The important part of the design is that only **one** of them
generates the clock. The clock pair — SCK (bit clock) and WS (which channel is on
the wire) — leaves the first controller, runs as a shared bus to all three
microphones, and also feeds back into the second controller, which is configured
as a slave and generates nothing.

Only the data lines are separate: `SD_A` brings back microphones M1 and M2 as one
stereo stream, `SD_B` brings back M3 on its own. The card sits on SPI at 3.3 V.

The point of the single clock is that every microphone samples on the same clock
edge, so the three streams cannot drift apart — which is exactly what would
happen with two independent clocks. What can still differ is the instant each
receive buffer starts filling, and that appears as a fixed offset of a whole
number of samples between the two data lines. A fixed offset is harmless: measure
it once, subtract it forever. Proving that it really is fixed — the same after
every reboot, stable over a long recording — is the first milestone of the build.

## The array

Three microphones on an equilateral triangle with 150 mm sides: M1 at the apex,
M2 and M3 at the base, and the centroid marked in the middle. Each microphone is
86.6 mm from the centre. Bearings are reported relative to M1, so whichever frame
is built, the M1 corner has to be marked physically and its real-world heading
noted when the array is set up — otherwise an azimuth means nothing.

There are two ways to hold the microphones in that shape. One is a solid flat
plate with the microphones at its corners: simple and rigid, but the plate is a
reflecting surface directly under the capsules and it catches wind. The other —
the one drawn as the *head* on the schematic — is a three-armed frame with the
electronics box slung underneath the hub.

The edge-on view at the bottom of the sheet shows why that arrangement is worth
the trouble: all three capsules sit on one horizontal plane, and everything
solid hangs below them, out of the acoustic path. It also offers the wind far
less to push against. The cost is stiffness — thin arms flex, and flex is
geometric error, which turns straight into bearing error. Both options put the
microphones in identical positions, so this is a construction choice, not a
change to the algorithm.

For sizing the head: the ESP32 board is about 55 mm long and the microSD module
about 30 mm, which is what the box under the hub has to swallow.

## Components

### Microcontroller

A devkit board built around the ESP-WROOM-32 module: 30 pins, USB-C for power
and flashing. The brain of the device — it reads the microphones, runs the
correlation math, and writes results to the card.

The critical property is that the chip has **two** independent I2S controllers,
which is what makes three microphones possible at all — see the wiring section
above for how they share a clock.

Quantity: 1.

### Microphones

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

A microSD card holder with an SPI interface (GND, MISO, SCK, MOSI, CS, power).
It stores raw 3-channel recordings (WAV) plus a log of detections and azimuths.
The recordings matter mostly for debugging the algorithm offline on a computer:
tuning filter and correlation parameters in Python against a saved file is far
easier than reflashing the board on every iteration.

This is the compact variant, with neither an onboard regulator nor a level
shifter, so it runs directly on 3.3 V — the same rail as the microphones. (The
larger modules, the ones carrying an LDO, are what expect 5 V; this is not one of
those.)

Quantity: 1 (plus the card itself).

### Rest of the build

- A rigid frame for the array — plate or three arms, as above. Either way the
  vertices must be fixed to within a few millimeters, because geometric error maps
  directly onto azimuth error.
- Foam windscreens for every microphone: wind is the main TDOA killer outdoors.
- Protoboard, wire, passives. Keep I2S runs short (10–15 cm); for spatially
  separated microphones use twisted pairs or shielded cable.
- Power: USB-C 5 V on the bench; for the field, an 18650 with a protection board
  and 3.3 V / 5 V regulation.
- Later: a weatherized (IP-rated) enclosure and a mount that raises the array
  off the ground.

## Project status

Components are in hand and the wiring and array layouts are drawn; assembly has
not started. Next step is bringing up capture on both I2S controllers and
**proving** that the offset between the two data lines is a constant. This gates
everything else: without confirmed synchronization, direction estimation is
meaningless.

The test for it is simple. Put an impulse source — a clap, or a click from a small
speaker — directly above the centroid, equally distant from all three microphones.
The true time difference is then zero for every pair, so whatever delay the
correlation reports is the buffer offset itself, and it can be measured, repeated
across reboots, and watched for drift.
