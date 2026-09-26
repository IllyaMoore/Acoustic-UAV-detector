"""Host-side network pieces: sentence parsing, fusion, CoT, error bounds."""

import xml.etree.ElementTree as ET

import numpy as np
import pytest

from uavdoa import cot, crlb, fusion, nmea
from uavdoa.pipeline import Pipeline
from uavdoa.simulate import Scene, simulate


def sentence(body: str) -> str:
    return f"${body}*{nmea.checksum(body):02X}"


def test_parse_doa_roundtrip():
    r = nmea.parse(sentence("UAVDOA,3,123456,1,45.0,135.5,20.1,0.81,14.2,0.55,1,134.9"))
    assert isinstance(r, nmea.DoaReport)
    assert r.node == 3 and r.detected and r.tracking
    assert r.az_true_deg == 135.5 and r.el_deg == pytest.approx(20.1)


def test_parse_rejects_bad_checksum_and_log_lines():
    good = sentence("UAVCAL,1,10,0.00,-3.02,-2.98,0.9,0.8,0.8")
    assert isinstance(nmea.parse(good), nmea.CalReport)
    assert nmea.parse(good[:-2] + "00") is None
    assert nmea.parse("I (1234) main: dsp load 12.3 %") is None


def test_parse_undefined_elevation():
    r = nmea.parse(sentence("UAVDOA,1,1,0,10.0,10.0,-1.0,0.10,1.0,0.10,0,0.0"))
    assert r.el_deg is None


def test_triangulation_three_nodes():
    rng = np.random.default_rng(0)
    nodes = np.array([[0, 0], [600, 0], [300, 500]], float)
    target = np.array([450, 1200.0])
    true_b = np.degrees(np.arctan2(target[0] - nodes[:, 0], target[1] - nodes[:, 1])) % 360
    fix = fusion.triangulate(nodes, true_b + rng.normal(0, 1.0, 3), sigma_deg=1.0)
    assert fix is not None
    assert np.linalg.norm(fix.xy - target) < 60
    assert fix.cep50_m > 0


def test_triangulation_rejects_parallel_and_behind():
    nodes = np.array([[0, 0], [100, 0]], float)
    assert fusion.triangulate(nodes, [0.0, 0.0]) is None     # parallel
    assert fusion.triangulate(nodes, [135.0, 225.0]) is not None  # cross ahead, to the south
    assert fusion.triangulate(nodes, [315.0, 45.0]) is None  # diverging: lines meet behind


def test_latlon_enu_roundtrip():
    lat0, lon0 = 50.45, 30.52
    xy = fusion.enu_from_latlon(50.46, 30.54, lat0, lon0)
    lat, lon = fusion.latlon_from_enu(xy, lat0, lon0)
    assert lat == pytest.approx(50.46, abs=1e-9) and lon == pytest.approx(30.54, abs=1e-9)


def test_cot_events_are_well_formed():
    ev = ET.fromstring(cot.sensor_event(2, 50.45, 30.52, 123.4, confidence=0.7))
    assert ev.tag == "event" and ev.get("type") == cot.TYPE_SENSOR
    assert ev.find("detail/sensor").get("azimuth") == "123.4"
    tr = ET.fromstring(cot.track_event("7", 50.46, 30.53, 42.0, 3))
    assert tr.get("type") == cot.TYPE_UNKNOWN_UAV and tr.find("point").get("ce") == "42.0"


def test_crlb_scales_as_expected():
    a = crlb.tdoa_crlb_s(10, 200, 4000, 0.25)
    b = crlb.tdoa_crlb_s(20, 200, 4000, 0.25)
    assert b < a / 2.5  # 10 dB more SNR -> ~sqrt(10) better at high SNR
    assert crlb.azimuth_crlb_deg(10, 30, 20, 200, 4000, 0.25) > 0


def test_pipeline_does_not_beat_the_bound():
    # Sanity: measured spread must not be below the CRLB (that would mean
    # the bound or the simulator is wrong).
    errs = []
    for seed in range(12):
        x = simulate(Scene(az_deg=30, el_deg=0, snr_db=0), seconds=0.3, seed=seed)
        errs.append(Pipeline().run(x)[0].az_deg - 30)
    assert np.std(errs) >= 0.5 * crlb.azimuth_crlb_deg(0, 30, 0, 200, 4000, 0.256)
