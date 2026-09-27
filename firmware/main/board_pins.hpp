// Pin map - one block per chip, selected at compile time by the IDF target.
// The WROOM-32 map matches the wiring table in hardware/README.md.
#pragma once

#include "sdkconfig.h"

namespace pins {

#if CONFIG_IDF_TARGET_ESP32
// ESP-WROOM-32, 30-pin devkit (the board on hand).
// Avoid GPIO6-11 (flash), strapping 0/2/5/12/15 as inputs, 34-39 are input-only.
constexpr int kWs = 25;
constexpr int kSck = 26;
constexpr int kSdA = 33;  // M1 (left) + M2 (right)
constexpr int kSdB = 32;  // M3 (left), right slot empty
// I2S1 slave clock inputs, jumpered from the shared bus. Matches the bench
// wiring recorded in the README ("SCK return" / "WS return").
constexpr int kSckIn = 14;  // external loopback: jumper from kSck
constexpr int kWsIn = 27;   // external loopback: jumper from kWs
constexpr int kSdSck = 18;
constexpr int kSdMosi = 23;
constexpr int kSdMiso = 19;
constexpr int kSdCs = 5;
#elif CONFIG_IDF_TARGET_ESP32S3
// ESP32-S3 DevKitC. Avoid strapping 0/3/45/46, USB 19/20, flash/PSRAM 26-32.
constexpr int kWs = 15;
constexpr int kSck = 16;
constexpr int kSdA = 17;
constexpr int kSdB = 18;
constexpr int kSckIn = 4;
constexpr int kWsIn = 5;
constexpr int kSdSck = 12;
constexpr int kSdMosi = 11;
constexpr int kSdMiso = 13;
constexpr int kSdCs = 10;
#else
#error "No pin map for this target - add one to board_pins.hpp"
#endif

}  // namespace pins
