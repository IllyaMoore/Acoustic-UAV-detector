"""Cursor-on-Target (CoT) events for ATAK / WinTAK / TAK Server.

CoT is the XML event format the TAK ecosystem uses for situational awareness;
putting a sensor's output on a TAK map is the usual first integration step
for anything that wants to be used by NATO-aligned forces. Two event kinds:

* sensor_event: the node itself, as a friendly ground sensor, with a
  <sensor> detail carrying the current bearing as a field-of-view wedge.
  One node cannot range, so the wedge IS the information: "something is
  out there, that way".
* track_event: a fused position from two or more nodes (uavdoa.fusion),
  as an unknown air track with its error circle (ce).

Standard library only (xml.etree) so the bridge runs on anything with Python.
Field names follow the public CoT base schema (event/point/detail). The
<sensor> detail is what ATAK uses for sensor FOV display - check the
rendering against your ATAK version before relying on it.
"""

from __future__ import annotations

import datetime as dt
import xml.etree.ElementTree as ET

# MIL-STD-2525 derived CoT types.
TYPE_SENSOR = "a-f-G-E-S"      # friendly / ground / equipment / sensor
TYPE_UNKNOWN_UAV = "a-u-A-M-F-Q"  # unknown / air / military / fixed wing / UAV
# "unknown", not "hostile": an acoustic signature alone does not establish
# affiliation, and the operator makes that call, not the sensor.


def _iso(t: dt.datetime) -> str:
    return t.astimezone(dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")[:-4] + "Z"


def _event(uid: str, cot_type: str, lat: float, lon: float, ce: float, stale_s: float,
           now: dt.datetime | None, how: str) -> tuple[ET.Element, ET.Element]:
    now = now or dt.datetime.now(dt.timezone.utc)
    ev = ET.Element("event", {
        "version": "2.0", "uid": uid, "type": cot_type, "how": how,
        "time": _iso(now), "start": _iso(now), "stale": _iso(now + dt.timedelta(seconds=stale_s)),
    })
    ET.SubElement(ev, "point", {"lat": f"{lat:.7f}", "lon": f"{lon:.7f}", "hae": "9999999.0",
                                "ce": f"{ce:.1f}", "le": "9999999.0"})
    return ev, ET.SubElement(ev, "detail")


def sensor_event(node: int, lat: float, lon: float, bearing_true_deg: float | None,
                 fov_deg: float = 10.0, range_m: float = 800.0, confidence: float = 0.0,
                 now: dt.datetime | None = None, stale_s: float = 10.0) -> bytes:
    """The node, plus its current bearing as a wedge (omit the wedge if None)."""
    ev, det = _event(f"acoustic-uav-node-{node}", TYPE_SENSOR, lat, lon, 5.0, stale_s, now, "h-g-i-g-o")
    ET.SubElement(det, "contact", {"callsign": f"ACOUSTIC-{node}"})
    if bearing_true_deg is not None:
        ET.SubElement(det, "sensor", {
            "azimuth": f"{bearing_true_deg:.1f}", "fov": f"{fov_deg:.1f}", "range": f"{range_m:.0f}",
            "vfov": "90", "elevation": "45", "model": "3-mic TDOA", "type": "r-e",
        })
        ET.SubElement(det, "remarks").text = (
            f"Acoustic bearing {bearing_true_deg:.1f} deg true, confidence {confidence:.2f}")
    else:
        ET.SubElement(det, "remarks").text = "Acoustic sensor, no detection"
    return ET.tostring(ev, encoding="utf-8", xml_declaration=True)


def track_event(track_id: str, lat: float, lon: float, cep_m: float, n_nodes: int,
                now: dt.datetime | None = None, stale_s: float = 5.0) -> bytes:
    """A fused fix. 'how' = m-f: machine-generated, fused."""
    ev, det = _event(f"acoustic-uav-track-{track_id}", TYPE_UNKNOWN_UAV, lat, lon, cep_m, stale_s, now, "m-f")
    ET.SubElement(det, "contact", {"callsign": f"AC-TRK-{track_id}"})
    ET.SubElement(det, "remarks").text = f"Acoustic fix from {n_nodes} nodes, CEP50 ~{cep_m:.0f} m"
    return ET.tostring(ev, encoding="utf-8", xml_declaration=True)
