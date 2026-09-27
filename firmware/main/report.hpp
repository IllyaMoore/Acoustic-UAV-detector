// Wire format for results: NMEA-0183-style sentences on the console UART.
//
//   $UAVDOA,<node>,<uptime_ms>,<det>,<az_rel>,<az_true>,<el>,<conf>,<snr_db>,<coh>,<trk>,<trk_az_true>*HH
//   $UAVCAL,<node>,<uptime_ms>,<lag01>,<lag02>,<lag12>,<peak01>,<peak02>,<peak12>*HH
//
// NMEA-style because it's line-based, has a checksum, and is easy to parse
// anywhere. Log lines share the UART; only sentences start with '$'.
// analysis/scripts/cot_bridge.py turns $UAVDOA into CoT for ATAK.
#pragma once

#include <cstddef>
#include <cstdint>

#include "dsp/pipeline.hpp"

namespace report {

// Formats into buf (>= 128 bytes), returns length. Adds "*HH\r\n".
std::size_t format_doa(char *buf, std::size_t cap, int node, uint32_t uptime_ms,
                       const uav::BlockResult &r, float heading_deg);
std::size_t format_cal(char *buf, std::size_t cap, int node, uint32_t uptime_ms,
                       const uav::BlockResult &r);

// XOR of every character between '$' and '*'.
uint8_t nmea_checksum(const char *body, std::size_t len);

}  // namespace report
