// Minimal in-place radix-2 complex FFT, float.
//
// Portable so the host tests exercise the same code as the chip. esp-dsp's
// dsps_fft2r_fc32 can replace it if profiling shows the FFT matters.
#pragma once

#include <complex>
#include <cstddef>
#include <vector>

namespace uav {

using cf32 = std::complex<float>;

class Fft {
public:
    explicit Fft(std::size_t n);  // n must be a power of two

    std::size_t size() const { return n_; }

    // Forward transform, X[k] = sum x[n] exp(-j 2 pi k n / N). In place.
    void forward(cf32 *data) const;

private:
    std::size_t n_;
    std::vector<cf32> twiddle_;          // exp(-j 2 pi k / N), k < N/2
    std::vector<std::size_t> bitrev_;    // bit-reversal permutation
};

}  // namespace uav
