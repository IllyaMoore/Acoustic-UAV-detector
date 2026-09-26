"""WAV in/out in the layout the firmware writes to microSD.

The firmware writes 3-channel, 16-bit PCM, 48 kHz: channel order M1, M2, M3,
with the FIFO offset NOT yet removed (it is applied by the pipeline, from
CONFIG_UAV_M3_LAG_SAMPLES), so the raw file can still be used to re-measure it.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from scipy.io import wavfile


def write_array_wav(path: str | Path, x: np.ndarray, fs: int = 48_000, peak: float = 0.5) -> None:
    """x: (3, n) float. Normalised so the loudest sample sits at `peak` FS."""
    y = x / (np.max(np.abs(x)) + 1e-12) * peak
    wavfile.write(str(path), fs, np.round(y.T * 32767).astype(np.int16))


def read_array_wav(path: str | Path) -> tuple[np.ndarray, int]:
    """Returns ((3, n) float in [-1, 1), fs). Extra channels are dropped."""
    fs, d = wavfile.read(str(path))
    if d.ndim != 2 or d.shape[1] < 3:
        raise ValueError(f"{path}: need >= 3 channels, got shape {d.shape}")
    d = d[:, :3]
    if d.dtype == np.int16:
        x = d.astype(np.float64) / 32768
    elif d.dtype == np.int32:
        x = d.astype(np.float64) / 2**31
    else:
        x = d.astype(np.float64)
    return x.T, int(fs)
