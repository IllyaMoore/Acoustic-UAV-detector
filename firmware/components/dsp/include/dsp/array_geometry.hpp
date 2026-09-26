// Array geometry and azimuth convention. Mirrors analysis/uavdoa/geometry.py;
// change one, change both (tests/test_firmware_parity.py will notice).
//
// Equilateral triangle, side 150 mm, origin at the centroid, +y toward M1.
// Azimuth 0 deg = toward M1, increasing CLOCKWISE seen from above
// (M3 at 120 deg, M2 at 240 deg): az = atan2(x, y).
#pragma once

#include <array>
#include <cmath>

namespace uav {

constexpr float kSpeedOfSound = 343.0f;  // m/s at ~20 degC
constexpr int kNumMics = 3;
constexpr int kNumPairs = 3;

struct Vec2 {
    float x, y;
};

// Vertex positions in metres. Measure the real frame and replace these -
// 1 mm of error is ~0.38 deg of azimuth at this baseline.
constexpr std::array<Vec2, kNumMics> kMics = {{
    {0.0f, 0.0866025f},     // M1 apex
    {-0.075f, -0.0433013f}, // M2 bottom-left
    {+0.075f, -0.0433013f}, // M3 bottom-right
}};

// Pair (i, j), i < j. Reported delay tau_ij = t_i - t_j, positive when mic i
// hears the sound later.
struct Pair {
    int i, j;
};
constexpr std::array<Pair, kNumPairs> kPairs = {{{0, 1}, {0, 2}, {1, 2}}};

inline float pair_baseline_m(const Pair &p) {
    const float dx = kMics[p.j].x - kMics[p.i].x;
    const float dy = kMics[p.j].y - kMics[p.i].y;
    return std::sqrt(dx * dx + dy * dy);
}

}  // namespace uav
