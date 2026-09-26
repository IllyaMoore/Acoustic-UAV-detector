"""The C++ firmware pipeline and the Python mirror must agree.

Builds firmware/test/host/doa_cli (needs make + a C++17 compiler; skipped
otherwise), feeds it and uavdoa.pipeline the same 16-bit WAV, and compares
block by block.
"""

import csv
import io
import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest

from uavdoa import angle_diff_deg
from uavdoa.pipeline import Pipeline, PipelineConfig
from uavdoa.simulate import Scene, simulate
from uavdoa.wavio import read_array_wav, write_array_wav

HOST = Path(__file__).resolve().parents[2] / "firmware" / "test" / "host"


@pytest.fixture(scope="module")
def doa_cli():
    if not shutil.which("make") or not (shutil.which("g++") or shutil.which("c++")):
        pytest.skip("no C++ toolchain")
    subprocess.run(["make", "-C", str(HOST), "doa_cli"], check=True, capture_output=True)
    return HOST / "build" / "doa_cli"


def run_cli(cli, wav, *args):
    out = subprocess.run([str(cli), str(wav), *map(str, args)], check=True,
                         capture_output=True, text=True).stdout
    return list(csv.DictReader(io.StringIO(out)))


@pytest.mark.parametrize("az,wind", [(40, -100), (200, 0), (310, 6)])
def test_cpp_matches_python(doa_cli, tmp_path, az, wind):
    wav = tmp_path / "scene.wav"
    write_array_wav(wav, simulate(Scene(az_deg=az, el_deg=15, snr_db=12, wind_db=wind),
                                  seconds=2.0, seed=az))
    x, fs = read_array_wav(wav)
    py = Pipeline(PipelineConfig(fs=fs)).run(x)
    cpp = run_cli(doa_cli, wav)
    assert len(py) == len(cpp) > 0
    for p, c in zip(py, cpp):
        np.testing.assert_allclose(p.lags, [float(c[k]) for k in ("lag01", "lag02", "lag12")], atol=0.02)
        assert abs(angle_diff_deg(p.az_deg, float(c["az_deg"]))) < 0.2
        assert p.snr_db == pytest.approx(float(c["snr_db"]), abs=0.05)
        assert p.tonality == pytest.approx(float(c["tonality"]), abs=1e-3)
        assert p.detected == bool(int(c["detected"]))


def test_cli_applies_m3_lag(doa_cli, tmp_path):
    wav = tmp_path / "offset.wav"
    write_array_wav(wav, simulate(Scene(az_deg=0, el_deg=90, snr_db=25, fifo_offset=4), seconds=0.6))
    raw = run_cli(doa_cli, wav)[0]
    fixed = run_cli(doa_cli, wav, "--m3-lag", 4)[0]
    assert float(raw["lag02"]) == pytest.approx(-4, abs=0.2)
    assert float(fixed["lag02"]) == pytest.approx(0, abs=0.2)
    assert float(fixed["lag12"]) == pytest.approx(0, abs=0.2)
