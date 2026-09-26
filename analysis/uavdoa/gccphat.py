"""GCC-PHAT time-delay estimation.

Reference implementation. The firmware port (firmware/components/dsp) follows
it step by step, and tests/test_firmware_parity.py checks the two agree.

GCC-PHAT whitens the cross-spectrum - keeps only its phase - before going back
to the time domain. For a broadband source like a propeller that turns the
cross-correlation into a sharp spike at the true delay instead of a broad hump
shaped by the source's own spectrum, and it makes the estimate far less
sensitive to how loud any single harmonic is.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class DelayEstimate:
    tau_s: float  # delay of x relative to y (positive: x is LATER), seconds
    lag: float  # same thing in (fractional) samples
    peak: float  # height of the PHAT peak, 0..1; a crude confidence figure


def band_mask(n_fft: int, fs: float, f_lo: float, f_hi: float) -> np.ndarray:
    """Boolean mask over the rfft bins inside [f_lo, f_hi]."""
    f = np.fft.rfftfreq(n_fft, 1.0 / fs)
    return (f >= f_lo) & (f <= f_hi)


def gcc_phat(x: np.ndarray, y: np.ndarray, fs: float, max_tau_s: float | None = None,
             f_lo: float = 0.0, f_hi: float | None = None, interp: bool = True,
             eps: float = 1e-12) -> DelayEstimate:
    """Estimate how much later x arrives than y.

    x, y        equal-length frames (already windowed if you want a window)
    max_tau_s   search only |tau| <= this; set it to baseline / c so the
                peak picker cannot latch onto an unphysical lag
    f_lo, f_hi  band limits applied in the whitened spectrum - this is the
                cheap band-pass: out-of-band bins are simply zeroed
    interp      parabolic sub-sample interpolation around the peak
    """
    n = len(x)
    n_fft = 2 * n  # zero-pad to 2N so the circular correlation equals the linear one
    X = np.fft.rfft(x, n_fft)
    Y = np.fft.rfft(y, n_fft)
    R = X * np.conj(Y)
    R /= np.abs(R) + eps
    if f_hi is None:
        f_hi = fs / 2
    R[~band_mask(n_fft, fs, f_lo, f_hi)] = 0.0

    cc = np.fft.irfft(R, n_fft)
    max_lag = n - 1
    if max_tau_s is not None:
        max_lag = min(max_lag, int(np.ceil(max_tau_s * fs)))
    # Rearrange to lags -max_lag..+max_lag.
    cc = np.concatenate((cc[-max_lag:], cc[: max_lag + 1]))

    k = int(np.argmax(cc))
    shift = 0.0
    if interp and 0 < k < len(cc) - 1:
        a, b, c = cc[k - 1], cc[k], cc[k + 1]
        denom = a - 2 * b + c
        if denom != 0:
            shift = 0.5 * (a - c) / denom
    lag = k - max_lag + shift

    # Normalise the peak by the number of active bins: 1.0 means perfectly
    # coherent across the band, ~0 means no common signal. Each rfft bin
    # (except DC and Nyquist) stands for two bins of the full spectrum.
    active = 2 * np.count_nonzero(R)
    peak = float(cc[k] * n_fft / active) if active else 0.0
    return DelayEstimate(tau_s=lag / fs, lag=lag, peak=peak)
