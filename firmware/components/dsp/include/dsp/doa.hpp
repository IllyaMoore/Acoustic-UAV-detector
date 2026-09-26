// Pair delays -> azimuth (+ elevation magnitude). Mirrors analysis/uavdoa/doa.py.
//
// tau_ij = (r_j - r_i) . u / c for a plane wave from direction u. Three pairs,
// two unknowns (the array is flat, u_z drops out) -> 2x2 normal equations.
// The pair matrix depends only on geometry, so its pseudo-inverse is computed
// once at construction.
#pragma once

#include <array>

#include "dsp/array_geometry.hpp"

namespace uav {

struct Bearing {
    float az_deg = 0;     // [0, 360), clockwise from M1
    float el_deg = 0;     // |elevation|; NaN if the solved vector is >10 % longer than 1
    float closure_s = 0;  // tau01 + tau12 - tau02; ~0 unless a pair locked onto garbage
    float residual_s = 0; // RMS least-squares residual
};

class DoaSolver {
public:
    explicit DoaSolver(float c = kSpeedOfSound);
    Bearing solve(const std::array<float, kNumPairs> &tau_s) const;

private:
    float a_[kNumPairs][2];     // tau = A u
    float pinv_[2][kNumPairs];  // (A^T A)^-1 A^T
};

float wrap_deg(float deg);           // -> [0, 360)
float angle_diff_deg(float a, float b);  // a - b, in (-180, 180]

}  // namespace uav
