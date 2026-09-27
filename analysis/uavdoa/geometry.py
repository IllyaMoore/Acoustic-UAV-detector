"""Array geometry and the azimuth convention.

Everything here mirrors the README and must stay in step with
firmware/components/dsp/include/array_geometry.hpp:

* Equilateral triangle, side 150 mm, all capsules coplanar, facing up.
* M1 at the apex, M2 bottom-left, M3 bottom-right.
* Origin at the centroid, +y from the centroid toward M1, +x to the right.
* Azimuth 0 deg points at M1 and increases CLOCKWISE seen from above
  (M3 at 120 deg, M2 at 240 deg). That is a compass-style bearing, so
  az = atan2(x, y), not the maths-textbook atan2(y, x).
"""

from __future__ import annotations

import numpy as np

SPEED_OF_SOUND = 343.0  # m/s at ~20 degC; see speed_of_sound() for temperature
SIDE_M = 0.150

# Vertex coordinates relative to the centroid, metres. Rows: M1, M2, M3.
MICS_M = np.array(
    [
        [0.0, SIDE_M / np.sqrt(3)],  # M1 (0, +86.6 mm)
        [-SIDE_M / 2, -SIDE_M / (2 * np.sqrt(3))],  # M2 (-75, -43.3 mm)
        [+SIDE_M / 2, -SIDE_M / (2 * np.sqrt(3))],  # M3 (+75, -43.3 mm)
    ]
)

# The three pairs, always ordered (i, j) with i < j. The delay reported for a
# pair is tau_ij = t_i - t_j: POSITIVE when mic i hears the sound LATER.
PAIRS = ((0, 1), (0, 2), (1, 2))


def speed_of_sound(temp_c: float) -> float:
    """Speed of sound in dry air. 331.3 * sqrt(1 + T/273.15).

    A 30 degC swing changes c by ~5 %, which scales every delay by the same
    factor. For the azimuth that is (almost) harmless - it cancels in the
    atan2 - but it biases the elevation estimate, so feed a real temperature
    when you have one.
    """
    return 331.3 * np.sqrt(1.0 + temp_c / 273.15)


def unit_vector(az_deg: float, el_deg: float = 0.0) -> np.ndarray:
    """3-D unit vector pointing FROM the array TOWARD the source."""
    az, el = np.radians(az_deg), np.radians(el_deg)
    return np.array([np.cos(el) * np.sin(az), np.cos(el) * np.cos(az), np.sin(el)])


def azimuth_deg(x: float, y: float) -> float:
    """Compass bearing of the in-plane vector (x, y), in [0, 360)."""
    return float(np.degrees(np.arctan2(x, y)) % 360.0)


def far_field_delays(az_deg: float, el_deg: float = 0.0, c: float = SPEED_OF_SOUND,
                     mics: np.ndarray = MICS_M) -> np.ndarray:
    """Ideal pair delays tau_ij = t_i - t_j (seconds) for a plane wave.

    Arrival time at mic k is t_k = -(r_k . u) / c: a mic further along the
    source direction hears it earlier.
    """
    u = unit_vector(az_deg, el_deg)[:2]
    t = -(mics @ u) / c
    return np.array([t[i] - t[j] for i, j in PAIRS])


def max_delay_s(mics: np.ndarray = MICS_M, c: float = SPEED_OF_SOUND) -> float:
    """Largest physically possible |tau| over all pairs (longest baseline / c)."""
    return max(np.linalg.norm(mics[i] - mics[j]) for i, j in PAIRS) / c
