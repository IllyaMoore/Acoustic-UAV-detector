"""Bearing-only fusion across nodes.

One node gives a line, not a point. Two or more nodes at known positions give
a fix where their bearing lines cross - that is how networked acoustic
systems (Sky Fortress, Zvook) turn cheap single-node bearings into tracks.

Method: weighted least squares on the perpendicular distance from the target
to each bearing line (a.k.a. the Stansfield / "pseudo-linear" estimator).
Closed form, no iterations, and its covariance says how good the geometry is:
two nodes looking at the target from nearly the same direction give a long
thin error ellipse no matter how accurate each bearing is.

Coordinates are local ENU metres (x east, y north). Bearings are TRUE, i.e.
already corrected for each node's M1 heading.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

EARTH_R = 6_371_000.0


@dataclass
class Fix:
    xy: np.ndarray  # (2,) east, north, metres
    cov: np.ndarray  # (2, 2)
    rms_miss_m: float  # RMS perpendicular distance from the fix to the lines

    @property
    def cep50_m(self) -> float:
        """Circular error probable (50 %) from the covariance, approx."""
        s = np.sqrt(np.clip(np.linalg.eigvalsh(self.cov), 0, None))
        return float(0.59 * (s[0] + s[1]))


def triangulate(nodes_xy: np.ndarray, bearings_deg: np.ndarray,
                sigma_deg: np.ndarray | float = 2.0) -> Fix | None:
    """nodes_xy (n, 2), bearings_deg (n,), per-bearing 1-sigma in degrees."""
    nodes_xy = np.asarray(nodes_xy, float)
    b = np.radians(np.asarray(bearings_deg, float))
    n = len(b)
    if n < 2:
        return None
    sig = np.broadcast_to(np.radians(np.asarray(sigma_deg, float)), (n,))
    # Line through node p with direction d = (sin b, cos b): normal
    # nrm = (cos b, -sin b); target x satisfies nrm . x = nrm . p.
    nrm = np.stack([np.cos(b), -np.sin(b)], axis=1)
    rhs = np.einsum("ij,ij->i", nrm, nodes_xy)
    # First pass unweighted, to get ranges for the weights: a bearing error
    # of sigma at range R is a cross-track error of R * sigma.
    x0, *_ = np.linalg.lstsq(nrm, rhs, rcond=None)
    rng = np.maximum(np.linalg.norm(x0 - nodes_xy, axis=1), 1.0)
    w = 1.0 / (rng * sig) ** 2
    W = np.diag(w)
    N = nrm.T @ W @ nrm
    if np.linalg.cond(N) > 1e10:
        return None  # parallel bearings: no crossing
    cov = np.linalg.inv(N)
    x = cov @ nrm.T @ W @ rhs
    # Reject a "fix" behind a node (lines crossing on the wrong side).
    ahead = np.einsum("ij,ij->i", x - nodes_xy, np.stack([np.sin(b), np.cos(b)], axis=1))
    if np.any(ahead < 0):
        return None
    miss = nrm @ x - rhs
    return Fix(xy=x, cov=cov, rms_miss_m=float(np.sqrt(np.mean(miss**2))))


def enu_from_latlon(lat: float, lon: float, lat0: float, lon0: float) -> np.ndarray:
    """Equirectangular approximation - fine over the few km a node network spans."""
    x = np.radians(lon - lon0) * EARTH_R * np.cos(np.radians(lat0))
    y = np.radians(lat - lat0) * EARTH_R
    return np.array([x, y])


def latlon_from_enu(xy: np.ndarray, lat0: float, lon0: float) -> tuple[float, float]:
    lat = lat0 + np.degrees(xy[1] / EARTH_R)
    lon = lon0 + np.degrees(xy[0] / (EARTH_R * np.cos(np.radians(lat0))))
    return float(lat), float(lon)
