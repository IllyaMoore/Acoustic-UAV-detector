// GCC-PHAT peak search on an already-accumulated cross-spectrum.
//
// The textbook recipe is "whiten, inverse FFT, find the max". Here the
// inverse FFT is skipped: the physically possible lags are only +-21 samples
// (150 mm at 48 kHz), so the correlation is evaluated directly at those 43
// lags over the in-band bins only. That is ~7k complex MACs per pair instead
// of a 2048-point IFFT, and the band-pass comes for free - out-of-band bins
// are simply never summed.
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
