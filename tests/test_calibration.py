import numpy as np
import pytest

from umc_twin import Kinematics, load_machine
from umc_twin.calibration import CalibrationError, calibrate, compute_overlay, save_overlay, summary


def test_uncalibrated_summary_matches_cad():
    s = summary(load_machine())
    assert s["nose_to_platter_at_home_mm"] == pytest.approx(261.28)
    np.testing.assert_allclose(s["mrzp_mm"], [-252.5, -204.7, -210.48])
    assert not s["calibrated"]


def test_nose_measurement_moves_the_model(tmp_path):
    path = tmp_path / "cal.yaml"
    result = calibrate(path=path, nose_to_platter=250.0)
    assert result["summary"]["nose_to_platter_at_home_mm"] == pytest.approx(250.0)
    m = load_machine(calibration=path)
    tip = Kinematics(m).tool_tip_world(np.zeros(5))
    assert tip[2] - m.raw["table"]["top_z"] == pytest.approx(250.0)


def test_mrzp_round_trips_and_wins_over_tape(tmp_path):
    path = tmp_path / "cal.yaml"
    mrzp_in = [-9.95, -8.05, -8.30]
    result = calibrate(path=path, units="in", mrzp=mrzp_in, nose_to_platter=10.0)
    np.testing.assert_allclose(result["summary"]["mrzp_in"], mrzp_in, atol=1e-4)
    assert any("overrides" in n for n in result["notes"])
    # at the MRZP position a zero-length tool sits exactly on the pivot
    kin = Kinematics(load_machine(calibration=path))
    np.testing.assert_allclose(kin.tool_tip_world(np.array([*np.array(mrzp_in) * 25.4, 0, 0])), 0, atol=1e-9)


def test_suspicious_values_are_flagged():
    _, notes = compute_overlay(load_machine(calibration=None), mrzp=[9.94, -8.06, -8.29], units="in")
    assert any("WARNING" in n and "MRZP X" in n for n in notes)


def test_directions_options_and_merge(tmp_path):
    path = tmp_path / "cal.yaml"
    save_overlay({"joints": {"B": {"max_velocity": 5000.0}}}, path)
    calibrate(path=path, b_direction="left", c_direction="ccw", options={"spindle": "hsk"})
    m = load_machine(calibration=path)
    np.testing.assert_allclose(m.joints["B"].axis, [0, -1, 0])
    np.testing.assert_allclose(m.joints["C"].axis, [0, 0, 1])
    assert m.joints["B"].max_velocity == 5000.0  # earlier calibration kept
    assert m.options["spindle"] == "hsk"
    with pytest.raises(CalibrationError):
        calibrate(path=path, b_direction="up")


def test_dry_run_does_not_write(tmp_path):
    path = tmp_path / "cal.yaml"
    calibrate(path=path, save=False, nose_to_platter=250.0)
    assert not path.exists()
