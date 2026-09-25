from pathlib import Path

import pytest

from umc_twin import Kinematics, load_machine
from umc_twin.compare import compare, synthetic_recording
from umc_twin.gcode import simulate
from umc_twin.job import load_setup
from umc_twin.material import simulate_material
from umc_twin.physics import spindle_load

EXAMPLES = Path(__file__).resolve().parent.parent / "examples"


@pytest.fixture(scope="module")
def sim():
    m = load_machine()
    kin = Kinematics(m)
    setup = load_setup(EXAMPLES / "setup_demo.yaml", kin)
    traj = simulate((EXAMPLES / "demo_5axis.nc").read_text(), kin, setup)
    load = spindle_load(traj, simulate_material(traj, kin, setup), setup, m.raw["spindle"])
    return traj, load


def test_identical_run_matches(sim):
    traj, load = sim
    r = compare(traj, synthetic_recording(traj, time_scale=1.0, load=load), load)
    assert r["cycle_time"]["ratio"] == pytest.approx(1.0, abs=0.01)
    assert r["position"]["max_dev_mm"] < 1e-3
    assert r["lines_matched"] == r["lines_in_recording"]


def test_slower_shifted_heavier_machine(sim):
    traj, load = sim
    rec = synthetic_recording(traj, time_scale=1.2, offset=(0, 0.1, 0, 0, 0), load=load, load_scale=1.5)
    r = compare(traj, rec, load)
    assert r["cycle_time"]["ratio"] == pytest.approx(1.2, abs=0.01)
    assert r["cycle_time"]["cutting_ratio"] == pytest.approx(1.2, abs=0.03)
    assert r["position"]["max_dev_mm"] == pytest.approx(0.1, abs=0.005)
    assert r["spindle_load"]["scale_factor"] == pytest.approx(1.5, rel=0.15)


def test_unmatched_line_numbers_are_reported(sim):
    traj, _ = sim
    rec = synthetic_recording(traj)
    for s in rec:
        s["line"] = s["line"] + 10000
    assert "N-numbers" in compare(traj, rec)["error"]
