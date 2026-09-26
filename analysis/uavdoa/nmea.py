"""Parse the firmware's $UAVDOA / $UAVCAL sentences (see firmware/main/report.hpp)."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class DoaReport:
    node: int
    uptime_ms: int
    detected: bool
    az_rel_deg: float
    az_true_deg: float
    el_deg: float | None  # None when the firmware sent -1 (undefined)
    confidence: float
    snr_db: float
    coherence: float
    tracking: bool
    track_az_true_deg: float


@dataclass
class CalReport:
    node: int
    uptime_ms: int
    lags: tuple[float, float, float]
    peaks: tuple[float, float, float]


def checksum(body: str) -> int:
    cs = 0
    for ch in body:
        cs ^= ord(ch)
    return cs


def parse(line: str) -> DoaReport | CalReport | None:
    """Returns None for anything that is not a valid sentence (log lines,
    corrupted lines, bad checksum)."""
    line = line.strip()
    if not line.startswith("$") or "*" not in line:
        return None
    body, _, cs = line[1:].rpartition("*")
    try:
        if int(cs, 16) != checksum(body):
            return None
    except ValueError:
        return None
    f = body.split(",")
    try:
        if f[0] == "UAVDOA" and len(f) == 12:
            el = float(f[6])
            return DoaReport(
                node=int(f[1]), uptime_ms=int(f[2]), detected=f[3] == "1",
                az_rel_deg=float(f[4]), az_true_deg=float(f[5]), el_deg=None if el < 0 else el,
                confidence=float(f[7]), snr_db=float(f[8]), coherence=float(f[9]),
                tracking=f[10] == "1", track_az_true_deg=float(f[11]),
            )
        if f[0] == "UAVCAL" and len(f) == 9:
            return CalReport(node=int(f[1]), uptime_ms=int(f[2]),
                             lags=(float(f[3]), float(f[4]), float(f[5])),
                             peaks=(float(f[6]), float(f[7]), float(f[8])))
    except ValueError:
        return None
    return None
