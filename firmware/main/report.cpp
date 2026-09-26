#include "report.hpp"

#include <cmath>
#include <cstdio>

namespace report {

uint8_t nmea_checksum(const char *body, std::size_t len) {
    uint8_t cs = 0;
    for (std::size_t i = 0; i < len; ++i) cs ^= static_cast<uint8_t>(body[i]);
    return cs;
}

namespace {
std::size_t finish(char *buf, std::size_t cap, int n) {
    if (n <= 0 || static_cast<std::size_t>(n) + 6 > cap) return 0;
    const uint8_t cs = nmea_checksum(buf + 1, static_cast<std::size_t>(n) - 1);
    return static_cast<std::size_t>(n) + std::snprintf(buf + n, cap - n, "*%02X\r\n", cs);
}
}  // namespace

std::size_t format_doa(char *buf, std::size_t cap, int node, uint32_t uptime_ms,
                       const uav::BlockResult &r, float heading_deg) {
    const float az_true = uav::wrap_deg(r.bearing.az_deg + heading_deg);
    const float trk_true = r.tracking ? uav::wrap_deg(r.track_az_deg + heading_deg) : 0.0f;
    const float el = std::isnan(r.bearing.el_deg) ? -1.0f : r.bearing.el_deg;  // -1: undefined
    const int n = std::snprintf(buf, cap, "$UAVDOA,%d,%lu,%d,%.1f,%.1f,%.1f,%.2f,%.1f,%.2f,%d,%.1f", node,
                                static_cast<unsigned long>(uptime_ms), r.detected ? 1 : 0,
                                r.bearing.az_deg, az_true, el, r.confidence, r.snr_db, r.coherence,
                                r.tracking ? 1 : 0, trk_true);
    return finish(buf, cap, n);
}

std::size_t format_cal(char *buf, std::size_t cap, int node, uint32_t uptime_ms,
                       const uav::BlockResult &r) {
    const int n = std::snprintf(buf, cap, "$UAVCAL,%d,%lu,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f", node,
                                static_cast<unsigned long>(uptime_ms), r.pairs[0].lag, r.pairs[1].lag,
                                r.pairs[2].lag, r.pairs[0].peak, r.pairs[1].peak, r.pairs[2].peak);
    return finish(buf, cap, n);
}

}  // namespace report
