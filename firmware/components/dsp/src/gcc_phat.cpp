#include "dsp/gcc_phat.hpp"

#include <cmath>
#include <vector>

namespace uav {

PairDelay gcc_phat_peak(const cf32 *cross, std::size_t bin_lo, std::size_t n_bins,
                        std::size_t nfft, int max_lag, float fs) {
    PairDelay out;
    if (n_bins == 0 || max_lag < 1) return out;

    // PHAT weighting: keep only the phase of every bin.
    std::vector<cf32> r(n_bins);
    std::size_t active = 0;
    for (std::size_t b = 0; b < n_bins; ++b) {
        const float m = std::abs(cross[b]);
        if (m > 1e-20f) {
            r[b] = cross[b] / m;
            ++active;
        }
    }
    if (active == 0) return out;

    // cc(l) = (1/active) * sum_b Re{ r_b * exp(+j 2 pi k_b l / nfft) }.
    // The phasor for consecutive k comes from a recurrence (one complex
    // multiply per bin) instead of sin/cos per bin; drift over a few hundred
    // steps is ~1e-5.
    const int n_lags = 2 * max_lag + 1;
    std::vector<float> cc(n_lags);
    const double two_pi = 6.283185307179586;
    for (int li = 0; li < n_lags; ++li) {
        const int lag = li - max_lag;
        const double a0 = two_pi * static_cast<double>(bin_lo) * lag / static_cast<double>(nfft);
        const double da = two_pi * lag / static_cast<double>(nfft);
        cf32 w(static_cast<float>(std::cos(a0)), static_cast<float>(std::sin(a0)));
        const cf32 dw(static_cast<float>(std::cos(da)), static_cast<float>(std::sin(da)));
        float acc = 0;
        for (std::size_t b = 0; b < n_bins; ++b) {
            acc += r[b].real() * w.real() - r[b].imag() * w.imag();
            w *= dw;
        }
        cc[li] = acc / static_cast<float>(active);
    }

    int k = 0;
    for (int li = 1; li < n_lags; ++li)
        if (cc[li] > cc[k]) k = li;

    // Parabolic interpolation through the peak and its neighbours.
    float shift = 0;
    if (k > 0 && k < n_lags - 1) {
        const float a = cc[k - 1], b = cc[k], c = cc[k + 1];
        const float denom = a - 2 * b + c;
        if (denom != 0) shift = 0.5f * (a - c) / denom;
    }
    out.lag = static_cast<float>(k - max_lag) + shift;
    out.tau_s = out.lag / fs;
    out.peak = cc[k];
    return out;
}

}  // namespace uav
