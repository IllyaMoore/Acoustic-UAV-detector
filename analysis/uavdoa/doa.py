"""Pair delays -> azimuth (and an elevation magnitude).

For a plane wave from unit direction u, the pair delay is

    tau_ij = t_i - t_j = (r_j - r_i) . u / c

Three pairs, two unknowns (u_x, u_y - the array is flat, so u_z drops out),
solved by least squares. Only two of the three pairs are independent
(tau_02 = tau_01 + tau_12 in the noise-free case), so the third pair is
redundant - and that redundancy is useful: the "closure" residual
tau_01 + tau_12 - tau_02 should be ~0, and if it is not, at least one pair
locked onto a reflection or a second source. That is the firmware's cheapest
sanity check.

The length of the solved in-plane vector is cos(elevation). A flat array
cannot tell up from down, so the elevation comes out as a magnitude only;
for drones we assume "above the horizon".
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .geometry import MICS_M, PAIRS, SPEED_OF_SOUND, azimuth_deg

HORIZON_TOLERANCE = 1.1


@dataclass
class Bearing:
    az_deg: float
    el_deg: float  # |elevation|; NaN if the solved vector is >10 % longer than 1
    closure_s: float  # tau_01 + tau_12 - tau_02, should be ~0
    residual_s: float  # RMS least-squares residual over the three pairs


def pair_matrix(mics: np.ndarray = MICS_M, c: float = SPEED_OF_SOUND) -> np.ndarray:
    """3x2 matrix A with tau = A @ u_xy."""
    return np.array([(mics[j] - mics[i]) / c for i, j in PAIRS])


def solve_bearing(tau_s: np.ndarray, mics: np.ndarray = MICS_M,
                  c: float = SPEED_OF_SOUND) -> Bearing:
    tau_s = np.asarray(tau_s, dtype=float)
    A = pair_matrix(mics, c)
    u, *_ = np.linalg.lstsq(A, tau_s, rcond=None)
    r = np.linalg.norm(u)
    # |u_xy| = cos(el). Noise pushes it slightly past 1 near the horizon; up to
    # 10 % over reads as "on the horizon", beyond that it is garbage.
    if r <= 1.0:
        el = float(np.degrees(np.arccos(r)))
    else:
        el = 0.0 if r <= HORIZON_TOLERANCE else float("nan")
    res = tau_s - A @ u
    return Bearing(
        az_deg=azimuth_deg(u[0], u[1]),
        el_deg=el,
        closure_s=float(tau_s[0] + tau_s[2] - tau_s[1]),
        residual_s=float(np.sqrt(np.mean(res**2))),
    )


def angle_diff_deg(a: float, b: float) -> float:
    """Signed smallest difference a - b on the circle, in (-180, 180]."""
    return float((a - b + 180.0) % 360.0 - 180.0)
