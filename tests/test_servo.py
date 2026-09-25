import pytest

from umc_twin import Kinematics, load_machine
from umc_twin.gcode import simulate
from umc_twin.job import Solid, Tool, default_setup
from umc_twin.servo import contour_error
from umc_twin.sim import run_job


@pytest.fixture(scope="module")
def ctx():
    m = load_machine()
    k = Kinematics(m)
    s = default_setup(k)
    s.tools[1] = Tool(1, length=100.0, diameter=10.0)
    return m, k, s


def circles(R, F):
    return (f"G21 G90 G17 G94\nT1 M6\nG54 G43 H1\nG0 X{R}. Y0 Z5.\nG1 Z-1. F500.\n"
            f"G3 X{R}. Y0 I-{R}. J0 F{F}.\nG3 X{R}. Y0 I-{R}. J0\nG3 X{R}. Y0 I-{R}. J0")


@pytest.mark.parametrize("R,F", [(10, 3000), (25, 3000), (10, 1200)])
def test_steady_circle_matches_first_order_theory(ctx, R, F):
    m, k, s = ctx
    r = contour_error(simulate(circles(R, F), k, s), k, {1: 100.0})
    kv, ff = 40.0, 0.8
    x = (F / 60) / (R * kv)
    theory = (1 - ff) * R * x * x - ((1 - ff) * R * x) ** 2 / (2 * R)
    steady = {d["line"]: d["max_contour_mm"] for d in r["per_line"]}[8]    # third full circle
    assert steady == pytest.approx(theory, abs=0.01)                       # arcs are 0.01 mm chord polylines


def test_gain_and_feedforward_scale_the_error(ctx):
    m, k, s = ctx
    def steady():
        r = contour_error(simulate(circles(25, 3000), k, s), k, {1: 100.0})
        return {d["line"]: d["max_contour_mm"] for d in r["per_line"]}[8]

    saved = m.raw["servo"]
    base = steady()
    m.raw["servo"] = {"kv": 80.0, "feedforward": 0.8}
    try:
        stiffer = steady()
    finally:
        m.raw["servo"] = saved
    assert stiffer < base / 2.5                                             # ~1/Kv^2


def test_tolerance_flags_lines(ctx):
    m, k, s = ctx
    s.stock = Solid(type="box", size=[80, 80, 20], position=[0, 0, -70.8])
    s.material = {"tolerance": 0.005, "resolution": 1.0}
    try:
        r = run_job(circles(10, 3000), m, s, check_collisions=False)
    finally:
        s.stock, s.material = None, {}
    assert r["servo"]["in_material_only"] and r["servo"]["issues"]
    assert all(i["kind"] == "contour_error" for i in r["servo"]["issues"])
