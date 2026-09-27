// GCC-PHAT peak search on an already-accumulated cross-spectrum.
//
// Instead of an inverse FFT, the correlation is evaluated directly at the
// physically possible lags (150 mm at 48 kHz is 21 samples; the pipeline
// searches +-22, i.e. 45 lags) over the in-band bins only. Cheaper than a
// 2048-point IFFT, and out-of-band bins are simply never summed.
//
// Reference: C. Knapp, G. Carter, "The generalized correlation method for
// estimation of time delay", IEEE Trans. ASSP 24(4), 1976.
#pragma once

#include <cstddef>

#include "dsp/fft.hpp"

namespace uav {

struct PairDelay {
    float lag = 0;    // fractional samples, positive: first mic of the pair is later
    float tau_s = 0;  // same, seconds
    float peak = 0;   // PHAT peak height, 0..1 (1 = fully coherent across the band)
};

// cross[b] is the cross-spectrum X_i * conj(X_j) at FFT bin (bin_lo + b),
// b < n_bins, for an FFT of size nfft. Lags searched: -max_lag..+max_lag.
PairDelay gcc_phat_peak(const cf32 *cross, std::size_t bin_lo, std::size_t n_bins,
                        std::size_t nfft, int max_lag, float fs);

}  // namespace uav
