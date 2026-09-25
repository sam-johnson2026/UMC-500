import math

import numpy as np
import pytest

from umc_twin import Kinematics, load_machine
from umc_twin.gcode import simulate
from umc_twin.job import Tool, default_setup


@pytest.fixture(scope="module")
def ctx():
    m = load_machine()
    kin = Kinematics(m)
    setup = default_setup(kin)
    setup.tools[1] = Tool(1, length=100.0)
    return m, kin, setup


PREAMBLE = "G21 G90 G94\nT1 M6\nG54 G43 H1\nG0 X0 Y0 Z50.\nG4 P0.1\n"  # G4: start from rest


def move_time(ctx, body, n):
    """Duration of the n-th (1-based) line of `body`."""
    m, kin, setup = ctx
    line = PREAMBLE.count("\n") + n
    tr = simulate(PREAMBLE + body, kin, setup)
    t = np.array(tr.t)
    ks = [k for k, ln in enumerate(tr.line) if ln == line]
    return t[ks[-1]] - t[ks[0] - 1], tr


def test_long_rapid_is_trapezoid(ctx):
    m, *_ = ctx
    dt, _ = move_time(ctx, "G0 X-200.", 1)
    v = m.joints["X"].max_velocity / 60.0
    a = m.joints["X"].max_accel
    assert dt == pytest.approx(200.0 / v + v / a, rel=1e-6)


def test_short_move_never_reaches_speed(ctx):
    m, *_ = ctx
    dt, _ = move_time(ctx, "G0 X-2.", 1)
    a = m.joints["X"].max_accel
    assert dt == pytest.approx(2 * math.sqrt(2.0 / a), rel=1e-6)


def test_sharp_corner_is_slower_than_straight(ctx):
    def total(body):
        return sum(move_time(ctx, body, line)[0] for line in (1, 2))
    straight = total("G1 X-50. F15000.\nX-100.")
    corner = total("G1 X-50. F15000.\nY-50.")
    gentle = total("G1 X-50. F15000.\nX-100. Y-2.")
    assert corner > straight + 0.05
    assert gentle < straight + 0.01


def test_small_arc_limited_by_centripetal_accel(ctx):
    m, *_ = ctx
    r, F = 3.0, 6000.0
    dt, _ = move_time(ctx, f"G1 X{r:.1f} Y0 F{F:.0f}\nG3 X{r:.1f} Y0 I-{r:.1f} J0", 2)
    ideal = 2 * math.pi * r / (F / 60.0)
    v_limit = math.sqrt(m.joints["X"].max_accel * r)  # ~95 mm/s < 100 mm/s programmed
    assert dt > ideal
    assert dt >= 2 * math.pi * r / v_limit * 0.95


def test_events_and_ideal_times_kept(ctx):
    _, tr = move_time(ctx, "G0 X-200.\nM5", 1)
    assert tr.t_ideal is not None and tr.t[-1] > tr.t_ideal[-1]
    assert tr.events[-1]["t"] == pytest.approx(tr.t[-1])
