"""Synthetic array recordings.

There is no hardware to record from right now, so every stage downstream is
developed and tested against this simulator first. It is deliberately simple
but models the things that actually break TDOA outdoors:

* a drone-like source: a blade-pass harmonic series with slow RPM wander,
  plus broadband rotor/airflow noise;
* exact fractional delays per microphone (applied in the frequency domain);
* wind: strong, low-frequency, and UNCORRELATED between capsules - the
  reason wind is the main TDOA killer is that it is local turbulence at each
  port, not a propagating wave;
* sensor self-noise (INMP441: 61 dB(A) SNR);
* the constant integer FIFO offset between the SD_A (M1+M2) and SD_B (M3)
  streams that the click test is supposed to measure.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy import signal

from .geometry import MICS_M, SPEED_OF_SOUND, unit_vector


@dataclass
class DroneSource:
    bpf_hz: float = 180.0  # blade-pass frequency: RPM/60 * blades
    n_harmonics: int = 12
    rpm_wander: float = 0.03  # relative slow wander of the fundamental
    broadband_db: float = -12.0  # rotor noise level relative to the harmonics


def drone_signal(n: int, fs: float, src: DroneSource, rng: np.random.Generator) -> np.ndarray:
    # Slow random walk on the instantaneous frequency, then integrate to phase.
    wander = signal.lfilter([1e-3], [1, -0.999], rng.standard_normal(n))
    wander *= src.rpm_wander / (np.std(wander) + 1e-12)
    f_inst = src.bpf_hz * (1.0 + wander)
    phase = 2 * np.pi * np.cumsum(f_inst) / fs
    x = np.zeros(n)
    for h in range(1, src.n_harmonics + 1):
        if h * src.bpf_hz >= fs / 2:
            break
        amp = 1.0 / h**0.7  # propeller harmonics roll off slowly
        x += amp * np.sin(h * phase + rng.uniform(0, 2 * np.pi))
    bb = signal.lfilter(*signal.butter(2, [300, 6000], "bandpass", fs=fs), rng.standard_normal(n))
    x += bb / np.std(bb) * np.std(x) * 10 ** (src.broadband_db / 20)
    return x / np.max(np.abs(x))


def fractional_delay(x: np.ndarray, delay_samples: float) -> np.ndarray:
    """Delay x by a (possibly fractional) number of samples, circularly."""
    n = len(x)
    X = np.fft.rfft(x)
    k = np.arange(len(X))
    return np.fft.irfft(X * np.exp(-2j * np.pi * k * delay_samples / n), n)


def wind_noise(n: int, fs: float, rng: np.random.Generator) -> np.ndarray:
    """Pink-ish rumble concentrated below ~200 Hz, gusty envelope."""
    w = signal.lfilter(*signal.butter(2, 150, "low", fs=fs), rng.standard_normal(n))
    env = signal.lfilter([1.0], [1, -0.9995], rng.standard_normal(n))  # ~0.1 s gusts
    w *= np.clip(1.0 + 0.8 * env / (np.std(env) + 1e-12), 0.1, None)
    return w / (np.std(w) + 1e-12)


@dataclass
class Scene:
    az_deg: float = 60.0
    el_deg: float = 20.0
    snr_db: float = 10.0  # drone vs sensor self-noise, per channel
    wind_db: float = -100.0  # wind vs drone; -100 = calm
    fifo_offset: int = 0  # integer samples SD_B (M3) lags SD_A (M1, M2)
    onset_s: float = 0.0  # drone silent before this (noise/wind only), 0.5 s fade-in
    c: float = SPEED_OF_SOUND


def simulate(scene: Scene, seconds: float = 1.0, fs: float = 48_000,
             src: DroneSource | None = None, seed: int = 0,
             mics: np.ndarray = MICS_M) -> np.ndarray:
    """Return a (3, n) float array: M1, M2, M3 as the firmware would see them."""
    rng = np.random.default_rng(seed)
    src = src or DroneSource()
    pad = 4096  # guard so the circular delay never wraps real content in
    n = int(seconds * fs)
    s = drone_signal(n + 2 * pad, fs, src, rng)
    u = unit_vector(scene.az_deg, scene.el_deg)[:2]
    t = np.arange(n) / fs
    gate = np.clip((t - scene.onset_s) / 0.5, 0, 1) if scene.onset_s > 0 else 1.0
    out = np.empty((len(mics), n))
    for k, r in enumerate(mics):
        d = -(r @ u) / scene.c * fs  # arrival time in samples, relative to centroid
        ch = fractional_delay(s, d)[pad:pad + n]
        p = np.std(ch)  # levels are set relative to the drone at full strength
        ch = ch * gate + rng.standard_normal(n) * p * 10 ** (-scene.snr_db / 20)
        if scene.wind_db > -90:
            ch = ch + wind_noise(n, fs, rng) * p * 10 ** (scene.wind_db / 20)
        out[k] = ch
    if scene.fifo_offset:
        out[2] = np.roll(out[2], scene.fifo_offset)
    return out


def click_train(seconds: float, fs: float, period_s: float = 0.5,
                rng: np.random.Generator | None = None) -> np.ndarray:
    """Impulses for the sample-alignment test: short band-limited clicks."""
    rng = rng or np.random.default_rng(0)
    n = int(seconds * fs)
    x = np.zeros(n)
    x[:: int(period_s * fs)] = 1.0
    b, a = signal.butter(4, [500, 8000], "bandpass", fs=fs)
    return signal.lfilter(b, a, x)
