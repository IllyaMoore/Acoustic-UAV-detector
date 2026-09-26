// Minimal in-place radix-2 complex FFT, float.
//
// Portable on purpose: the same code runs on the ESP32 and in the host unit
// tests, so a test that passes on the laptop is a test of the firmware maths.
// esp-dsp's dsps_fft2r_fc32 is the obvious drop-in once the board is profiled
// (it is ~1.5x faster with the Xtensa-optimised butterflies), but at the
// frame rate this pipeline runs it is not the bottleneck - see firmware/README.
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
