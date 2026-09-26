"""Cramér-Rao lower bounds: how good can this array possibly be?

TDOA bound for two sensors seeing a common Gaussian source in independent
noise (Knapp & Carter 1976):

    var(tau) >= 1 / ( 2T * integral (2 pi f)^2 * gamma(f) / (1 - gamma(f)) df )

gamma = magnitude-squared coherence. With equal per-channel SNR S (source
over noise power spectral density, flat across the band):

    gamma / (1 - gamma) = S^2 / (1 + 2 S)

The azimuth bound then follows by pushing the pair covariance through the
least-squares solver: cov(u) = P cov(tau) P^T with P = pinv(A). The three
pair delays are not independent (they share mics); the full cov(tau) built
here accounts for that.
"""

from __future__ import annotations

import numpy as np

from .doa import pair_matrix
from .geometry import MICS_M, PAIRS, SPEED_OF_SOUND, unit_vector


def tdoa_crlb_s(snr_db: float, f_lo: float, f_hi: float, T: float, n_grid: int = 2000) -> float:
    """Std-dev bound (s) of one pair delay, flat SNR over [f_lo, f_hi]."""
    s = 10 ** (snr_db / 10)
    f = np.linspace(f_lo, f_hi, n_grid)
    integrand = (2 * np.pi * f) ** 2 * s**2 / (1 + 2 * s)
    fisher = 2 * T * np.trapezoid(integrand, f)
    return float(1 / np.sqrt(fisher))


def pair_covariance(sigma_tau: float) -> np.ndarray:
    """Covariance of (tau01, tau02, tau12) when each mic's timing noise is
    independent. tau_ij = t_i - t_j, var(t_k) = sigma_tau^2 / 2 so that each
    pair has variance sigma_tau^2; pairs sharing a mic are correlated."""
    v = sigma_tau**2 / 2
    C = np.zeros((3, 3))
    for a, (i, j) in enumerate(PAIRS):
        for b, (k, l) in enumerate(PAIRS):
            C[a, b] = v * ((i == k) - (i == l) - (j == k) + (j == l))
    return C


def azimuth_crlb_deg(snr_db: float, az_deg: float, el_deg: float, f_lo: float, f_hi: float,
                     T: float, mics: np.ndarray = MICS_M, c: float = SPEED_OF_SOUND) -> float:
    sigma = tdoa_crlb_s(snr_db, f_lo, f_hi, T)
    A = pair_matrix(mics, c)
    P = np.linalg.pinv(A)
    cov_u = P @ pair_covariance(sigma) @ P.T
    u = unit_vector(az_deg, el_deg)[:2]
    r = np.linalg.norm(u)
    t = np.array([u[1], -u[0]]) / r  # tangential direction (d az)
    return float(np.degrees(np.sqrt(t @ cov_u @ t) / r))


def geometry_error_deg(delta_m: float, baseline_m: float = 0.15) -> float:
    """Rule of thumb: an in-plane position error delta on one mic costs
    about delta / baseline radians of azimuth."""
    return float(np.degrees(delta_m / baseline_m))
