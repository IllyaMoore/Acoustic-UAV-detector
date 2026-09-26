#include "dsp/fft.hpp"

#include <cassert>
#include <cmath>
#include <utility>

namespace uav {

Fft::Fft(std::size_t n) : n_(n), twiddle_(n / 2), bitrev_(n) {
    assert(n >= 2 && (n & (n - 1)) == 0);
    const double pi = 3.14159265358979323846;
    for (std::size_t k = 0; k < n / 2; ++k) {
        // Computed in double once at start-up so the table itself adds no
        // rounding error beyond float storage.
        const double a = -2.0 * pi * static_cast<double>(k) / static_cast<double>(n);
        twiddle_[k] = cf32(static_cast<float>(std::cos(a)), static_cast<float>(std::sin(a)));
    }
    std::size_t bits = 0;
    while ((std::size_t{1} << bits) < n) ++bits;
    for (std::size_t i = 0; i < n; ++i) {
        std::size_t r = 0;
        for (std::size_t b = 0; b < bits; ++b)
            if (i & (std::size_t{1} << b)) r |= std::size_t{1} << (bits - 1 - b);
        bitrev_[i] = r;
    }
}

void Fft::forward(cf32 *x) const {
    for (std::size_t i = 0; i < n_; ++i)
        if (i < bitrev_[i]) std::swap(x[i], x[bitrev_[i]]);

    for (std::size_t len = 2; len <= n_; len <<= 1) {
        const std::size_t half = len / 2;
        const std::size_t step = n_ / len;  // twiddle stride for this stage
        for (std::size_t start = 0; start < n_; start += len) {
            for (std::size_t k = 0; k < half; ++k) {
                const cf32 w = twiddle_[k * step];
                const cf32 a = x[start + k];
                const cf32 b = x[start + k + half] * w;
                x[start + k] = a + b;
                x[start + k + half] = a - b;
            }
        }
    }
}

}  // namespace uav
