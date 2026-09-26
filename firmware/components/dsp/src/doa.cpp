#include "dsp/doa.hpp"

#include <cmath>

namespace uav {

namespace {
constexpr float kRadToDeg = 57.29577951308232f;
constexpr float kHorizonTolerance = 1.1f;
}

DoaSolver::DoaSolver(float c) {
    for (int p = 0; p < kNumPairs; ++p) {
        const Pair &pr = kPairs[p];
        a_[p][0] = (kMics[pr.j].x - kMics[pr.i].x) / c;
        a_[p][1] = (kMics[pr.j].y - kMics[pr.i].y) / c;
    }
    // Normal matrix N = A^T A (2x2), then pinv = N^-1 A^T.
    float n00 = 0, n01 = 0, n11 = 0;
    for (int p = 0; p < kNumPairs; ++p) {
        n00 += a_[p][0] * a_[p][0];
        n01 += a_[p][0] * a_[p][1];
        n11 += a_[p][1] * a_[p][1];
    }
    const float det = n00 * n11 - n01 * n01;
    const float i00 = n11 / det, i01 = -n01 / det, i11 = n00 / det;
    for (int p = 0; p < kNumPairs; ++p) {
        pinv_[0][p] = i00 * a_[p][0] + i01 * a_[p][1];
        pinv_[1][p] = i01 * a_[p][0] + i11 * a_[p][1];
    }
}

Bearing DoaSolver::solve(const std::array<float, kNumPairs> &tau) const {
    float ux = 0, uy = 0;
    for (int p = 0; p < kNumPairs; ++p) {
        ux += pinv_[0][p] * tau[p];
        uy += pinv_[1][p] * tau[p];
    }
    Bearing b;
    b.az_deg = wrap_deg(std::atan2(ux, uy) * kRadToDeg);
    const float r = std::sqrt(ux * ux + uy * uy);
    // |u_xy| = cos(el). Noise pushes it slightly past 1 near the horizon;
    // up to 10 % over is read as "on the horizon", beyond that it is garbage.
    b.el_deg = r <= 1.0f ? std::acos(r) * kRadToDeg : (r <= kHorizonTolerance ? 0.0f : NAN);
    b.closure_s = tau[0] + tau[2] - tau[1];
    float ss = 0;
    for (int p = 0; p < kNumPairs; ++p) {
        const float e = tau[p] - (a_[p][0] * ux + a_[p][1] * uy);
        ss += e * e;
    }
    b.residual_s = std::sqrt(ss / kNumPairs);
    return b;
}

float wrap_deg(float deg) {
    float d = std::fmod(deg, 360.0f);
    if (d < 0) d += 360.0f;
    return d >= 360.0f ? 0.0f : d;
}

float angle_diff_deg(float a, float b) {
    float d = std::fmod(a - b + 180.0f, 360.0f);
    if (d < 0) d += 360.0f;
    return d - 180.0f;
}

}  // namespace uav
