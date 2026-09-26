"""Python mirror of firmware/components/dsp (Pipeline).

Same framing, window, band, cross-spectrum averaging, direct-lag GCC-PHAT,
detector and tracker, written in numpy. It exists for two reasons:

1. It is the executable specification the C++ is checked against
   (tests/test_firmware_parity.py runs both on the same WAV).
2. Tuning: thresholds and band limits are explored here, on recordings,
   with plots, then copied into PipelineConfig.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .doa import solve_bearing
from .geometry import MICS_M, PAIRS, SPEED_OF_SOUND


@dataclass
class PipelineConfig:
    fs: float = 48_000.0
    frame_len: int = 1024
    avg_frames: int = 12
    f_lo: float = 200.0
    f_hi: float = 4000.0
    c: float = SPEED_OF_SOUND
    min_snr_db: float = 6.0
    min_coherence: float = 0.25
    min_tonality: float = 0.30
    max_closure_samples: float = 1.5
    floor_rise_db_per_s: float = 1.0
    track_alpha: float = 0.35
    track_hold_blocks: int = 4


@dataclass
class BlockResult:
    index: int
    lags: np.ndarray  # (3,) fractional samples
    peaks: np.ndarray  # (3,)
    az_deg: float
    el_deg: float
    closure_samples: float
    band_db: float
    snr_db: float
    tonality: float
    coherence: float
    confidence: float
    detected: bool
    tracking: bool
    track_az_deg: float = field(default=0.0)


def phat_peak(cross: np.ndarray, bin_lo: int, nfft: int, max_lag: int) -> tuple[float, float]:
    """Direct evaluation of the PHAT-weighted correlation at integer lags."""
    mag = np.abs(cross)
    ok = mag > 1e-20
    if not ok.any():
        return 0.0, 0.0
    r = np.where(ok, cross / np.where(ok, mag, 1), 0)
    k = bin_lo + np.arange(len(cross))
    lags = np.arange(-max_lag, max_lag + 1)
    cc = np.real(r[None, :] * np.exp(2j * np.pi * np.outer(lags, k) / nfft)).sum(axis=1) / ok.sum()
    i = int(np.argmax(cc))
    shift = 0.0
    if 0 < i < len(cc) - 1:
        a, b, c = cc[i - 1], cc[i], cc[i + 1]
        d = a - 2 * b + c
        if d != 0:
            shift = 0.5 * (a - c) / d
    return float(i - max_lag + shift), float(cc[i])


class Pipeline:
    def __init__(self, cfg: PipelineConfig | None = None):
        self.cfg = cfg = cfg or PipelineConfig()
        self.nfft = 2 * cfg.frame_len
        df = cfg.fs / self.nfft
        lo = max(int(np.ceil(cfg.f_lo / df)), 1)
        hi = min(int(np.floor(cfg.f_hi / df)), self.nfft // 2 - 1)
        self.bin_lo, self.n_bins = lo, max(hi - lo + 1, 0)
        base = max(np.linalg.norm(MICS_M[i] - MICS_M[j]) for i, j in PAIRS)
        self.max_lag = int(np.ceil(base / cfg.c * cfg.fs)) + 1
        n = np.arange(cfg.frame_len)
        self.window = 0.5 - 0.5 * np.cos(2 * np.pi * n / cfg.frame_len)
        self._reset_block()
        self.block_index = 0
        self.floor_db: float | None = None
        self.tracking = False
        self.miss = 0
        self.track = np.zeros(2)

    def _reset_block(self):
        self.cross = np.zeros((len(PAIRS), self.n_bins), complex)
        self.power = np.zeros(self.n_bins)
        self.frames = 0

    def push_frame(self, frame: np.ndarray) -> BlockResult | None:
        """frame: (3, frame_len). Returns a BlockResult every avg_frames frames."""
        X = np.fft.fft(frame * self.window, self.nfft, axis=1)[:, self.bin_lo:self.bin_lo + self.n_bins]
        self.power += (np.abs(X) ** 2).sum(axis=0)
        for p, (i, j) in enumerate(PAIRS):
            self.cross[p] += X[i] * np.conj(X[j])
        self.frames += 1
        if self.frames < self.cfg.avg_frames:
            return None
        return self._finish_block()

    def run(self, x: np.ndarray) -> list[BlockResult]:
        """x: (3, n) - process a whole recording."""
        out = []
        L = self.cfg.frame_len
        for off in range(0, x.shape[1] - L + 1, L):
            r = self.push_frame(x[:, off:off + L])
            if r is not None:
                out.append(r)
        return out

    def _finish_block(self) -> BlockResult:
        cfg = self.cfg
        lp = [phat_peak(self.cross[p], self.bin_lo, self.nfft, self.max_lag) for p in range(len(PAIRS))]
        lags = np.array([a for a, _ in lp])
        peaks = np.array([b for _, b in lp])
        b = solve_bearing(lags / cfg.fs, c=cfg.c)
        coherence = float(peaks.mean())

        pw = self.power + 1e-30
        mean = pw.mean()
        tonality = float(np.clip(1 - np.exp(np.log(pw).mean()) / mean, 0, 1))
        band_db = float(10 * np.log10(mean / (cfg.avg_frames * 3)))

        block_s = cfg.avg_frames * cfg.frame_len / cfg.fs
        if self.floor_db is None:
            self.floor_db = band_db
        snr_db = band_db - self.floor_db
        closure = abs(b.closure_s) * cfg.fs
        detected = (snr_db >= cfg.min_snr_db and coherence >= cfg.min_coherence
                    and tonality >= cfg.min_tonality and closure <= cfg.max_closure_samples)
        if band_db < self.floor_db:
            self.floor_db += 0.5 * (band_db - self.floor_db)
        else:
            rise = cfg.floor_rise_db_per_s * block_s * (0.1 if detected else 1.0)
            self.floor_db = min(band_db, self.floor_db + rise)

        c_snr = np.clip(snr_db / (2 * cfg.min_snr_db), 0, 1)
        c_coh = np.clip(coherence / (2 * cfg.min_coherence), 0, 1)
        c_ton = np.clip(tonality / (2 * cfg.min_tonality), 0, 1)
        confidence = float(np.cbrt(c_snr * c_coh * c_ton))

        if detected:
            u = np.array([np.sin(np.radians(b.az_deg)), np.cos(np.radians(b.az_deg))])
            if not self.tracking:
                self.track, self.tracking = u, True
            else:
                t = (1 - cfg.track_alpha) * self.track + cfg.track_alpha * u
                n = np.linalg.norm(t)
                self.track = t / n if n > 1e-6 else t
            self.miss = 0
        elif self.tracking:
            self.miss += 1
            if self.miss > cfg.track_hold_blocks:
                self.tracking = False
        track_az = float(np.degrees(np.arctan2(*self.track)) % 360) if self.tracking else 0.0

        res = BlockResult(
            index=self.block_index, lags=lags, peaks=peaks, az_deg=b.az_deg, el_deg=b.el_deg,
            closure_samples=b.closure_s * cfg.fs, band_db=band_db, snr_db=snr_db,
            tonality=tonality, coherence=coherence, confidence=confidence,
            detected=bool(detected), tracking=self.tracking, track_az_deg=track_az,
        )
        self.block_index += 1
        self._reset_block()
        return res
