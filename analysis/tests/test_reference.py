"""Tests of the Python reference DSP against the simulator."""

import numpy as np
import pytest

from uavdoa import PAIRS, angle_diff_deg, far_field_delays, gcc_phat, max_delay_s, solve_bearing
from uavdoa.pipeline import Pipeline
from uavdoa.simulate import Scene, simulate

FS = 48_000


@pytest.mark.parametrize("az", [0, 17, 90, 133, 210, 275, 359])
@pytest.mark.parametrize("el", [0, 35])
def test_solver_inverts_ideal_delays(az, el):
    b = solve_bearing(far_field_delays(az, el))
    assert abs(angle_diff_deg(b.az_deg, az)) < 1e-6
    assert b.el_deg == pytest.approx(el, abs=1e-3)
    assert abs(b.closure_s) < 1e-12


def test_gcc_phat_single_frame():
    x = simulate(Scene(az_deg=80, el_deg=10, snr_db=20), seconds=0.1)
    truth = far_field_delays(80, 10) * FS
    w = np.hanning(x.shape[1])
    for p, (i, j) in enumerate(PAIRS):
        est = gcc_phat(x[i] * w, x[j] * w, FS, max_delay_s(), f_lo=200, f_hi=4000)
        assert est.lag == pytest.approx(truth[p], abs=0.3)


@pytest.mark.parametrize("az", [5, 120, 250])
def test_pipeline_in_wind(az):
    # Wind 6 dB louder than the drone, uncorrelated between capsules: the
    # band-limit plus cross-spectrum averaging should still hold the bearing.
    x = simulate(Scene(az_deg=az, el_deg=20, snr_db=15, wind_db=6), seconds=1.1, seed=az)
    blocks = Pipeline().run(x)
    assert blocks
    errs = [abs(angle_diff_deg(b.az_deg, az)) for b in blocks]
    assert np.median(errs) < 2.0


def test_fifo_offset_is_visible_then_removable():
    # With a 3-sample FIFO offset on M3 and a source at the zenith (true TDOA 0),
    # the pairs involving M3 read the offset directly - that is the click test.
    x = simulate(Scene(az_deg=0, el_deg=90, snr_db=25, fifo_offset=3), seconds=0.6)
    b = Pipeline().run(x)[0]
    assert b.lags[0] == pytest.approx(0, abs=0.2)   # M1-M2: same stream
    assert b.lags[1] == pytest.approx(-3, abs=0.2)  # M1-M3
    assert b.lags[2] == pytest.approx(-3, abs=0.2)  # M2-M3
