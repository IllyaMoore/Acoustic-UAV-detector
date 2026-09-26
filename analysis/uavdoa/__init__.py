"""Reference DSP for the acoustic UAV detector."""

from .doa import Bearing, angle_diff_deg, solve_bearing
from .gccphat import DelayEstimate, gcc_phat
from .geometry import MICS_M, PAIRS, SPEED_OF_SOUND, far_field_delays, max_delay_s

__all__ = [
    "Bearing", "DelayEstimate", "MICS_M", "PAIRS", "SPEED_OF_SOUND",
    "angle_diff_deg", "far_field_delays", "gcc_phat", "max_delay_s", "solve_bearing",
]
