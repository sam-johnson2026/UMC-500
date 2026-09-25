from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("fcl")

from umc_twin import Kinematics, load_machine  # noqa: E402
from umc_twin.collision import CollisionChecker  # noqa: E402
from umc_twin.job import Tool, default_setup, load_setup  # noqa: E402
from umc_twin.sim import run_job  # noqa: E402

EXAMPLES = Path(__file__).resolve().parent.parent / "examples"


@pytest.fixture(scope="module")
def ctx():
    m = load_machine()
    kin = Kinematics(m)
    setup = default_setup(kin)
    setup.tools[1] = Tool(1, length=120.0, diameter=12.0)
    return m, kin, setup, CollisionChecker(m, kin, setup)


def test_home_is_clear_through_full_b_range(ctx):
    *_, cc = ctx
    for b in range(-35, 111, 5):
        assert cc.check_pose(np.array([0, 0, 0, float(b), 0]), tool=1) == [], b


def test_spindle_driven_into_platter(ctx):
    *_, cc = ctx
    over_table = np.array([-252.5, -204.7, 0.0, 0, 0])
    assert cc.check_pose(over_table, tool=1) == []
    over_table[2] = -300.0
    hit = {frozenset(p[:2]) for p in cc.check_pose(over_table, tool=1)}
    assert frozenset({"spindle_cat40", "platter_tslot"}) in hit


def test_demo_program_is_clean():
    m = load_machine()
    kin = Kinematics(m)
    setup = load_setup(EXAMPLES / "setup_demo.yaml", kin)
    r = run_job((EXAMPLES / "demo_5axis.nc").read_text(), m, setup)
    assert r["limits"] == [] and r["collisions"] == [] and r["warnings"] == []
    assert r["material"]["issues"] == []
    assert r["material"]["removed_volume_mm3"] == pytest.approx(14000, rel=0.05)


def test_without_cutting_sim_rapid_into_stock_is_a_collision():
    m = load_machine()
    kin = Kinematics(m)
    setup = load_setup(EXAMPLES / "setup_demo.yaml", kin)
    r = run_job((EXAMPLES / "crash_demo.nc").read_text(), m, setup, material=False)
    assert r["material"] is None
    assert (9, "rapid_into_stock") in {(c["line"], c["kind"]) for c in r["collisions"]}


def test_crash_demo_catches_each_mistake():
    m = load_machine()
    kin = Kinematics(m)
    setup = load_setup(EXAMPLES / "setup_demo.yaml", kin)
    r = run_job((EXAMPLES / "crash_demo.nc").read_text(), m, setup)
    cutting = {(i["line"], i["kind"]) for i in r["material"]["issues"]}
    assert (9, "rapid_into_material") in cutting                  # 1: G00 plunge (cutting sim)
    assert any(c["b"] == "platter_tslot" for c in r["collisions"])  # 2: head into tilted platter
    assert any(lim["message"].startswith("X=") for lim in r["limits"])  # 3: over-travel
