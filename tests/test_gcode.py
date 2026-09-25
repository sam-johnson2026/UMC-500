import math

import numpy as np
import pytest

from umc_twin import Kinematics, load_machine
from umc_twin.gcode import MOTION, GCodeError, simulate
from umc_twin.job import Tool, default_setup


@pytest.fixture(scope="module")
def kin():
    return Kinematics(load_machine())


@pytest.fixture
def setup(kin):
    s = default_setup(kin)  # G54 = platter centre, top face
    s.tools[1] = Tool(1, length=100.0, diameter=10.0)
    return s


def run(kin, setup, body: str):
    # accel=False: these tests check the interpreter's feed-rate timing; test_timing covers the planner
    return simulate("G21 G90 G17 G94\nT1 M6\nG54 G43 H1\n" + body, kin, setup, accel=False)


def last_tip(tr):
    return np.array(tr.tip[-1])


def test_absolute_incremental_and_units(kin, setup):
    tr = run(kin, setup, "G0 X10. Y20. Z30.\nG91 G0 X5.\nG90 G20 G0 X1.")
    np.testing.assert_allclose(last_tip(tr), [25.4, 20, -50.8 + 30], atol=1e-9)


def test_rapid_time_uses_slowest_axis(kin, setup):
    tr = run(kin, setup, "G0 X0 Y0 Z50.\nG0 X254.")
    dt = tr.t[-1] - tr.t[-2]
    assert dt == pytest.approx(254.0 / 25400.0 * 60.0)


def test_feed_time(kin, setup):
    tr = run(kin, setup, "G0 X0 Y0 Z5.\nG1 X100. F1000.")
    assert tr.t[-1] - tr.t[-2] == pytest.approx(6.0)


def test_inverse_time_feed(kin, setup):
    tr = run(kin, setup, "G0 X0 Y0 Z5.\nG93 G1 X10. B10. F30.")
    assert tr.t[-1] - tr.t[-2] == pytest.approx(2.0)


@pytest.mark.parametrize("arc,mid_y", [("G2 X20. Y0 I10. J0", 10.0), ("G3 X20. Y0 I10. J0", -10.0),
                                       ("G2 X20. Y0 R10.", 10.0), ("G3 X20. Y0 R10.", -10.0)])
def test_arcs_direction_and_radius(kin, setup, arc, mid_y):
    tr = run(kin, setup, f"G0 X0 Y0 Z5.\nG1 F500.\n{arc}")
    arc_tips = np.array([tip for tip, m in zip(tr.tip, tr.motion) if m == MOTION["arc"]])
    np.testing.assert_allclose(np.hypot(arc_tips[:, 0] - 10.0, arc_tips[:, 1]), 10.0, atol=1e-6)
    top = arc_tips[np.argmax(np.abs(arc_tips[:, 1]))]
    assert top[1] == pytest.approx(mid_y, abs=0.05)
    np.testing.assert_allclose(arc_tips[-1][:2], [20, 0], atol=1e-9)


def test_full_circle_and_helix(kin, setup):
    tr = run(kin, setup, "G0 X10. Y0 Z5.\nG1 Z0 F500.\nG3 X10. Y0 I-10. J0 Z-2.")
    arc_time = sum(b - a for a, b, m in zip(tr.t, tr.t[1:], tr.motion[1:]) if m == MOTION["arc"])
    assert arc_time == pytest.approx(2 * math.pi * 10 / 500 * 60, rel=1e-3)
    np.testing.assert_allclose(last_tip(tr), [10, 0, -52.8], atol=1e-9)


def test_tcpc_keeps_tip_on_the_part(kin, setup):
    tr = run(kin, setup, "G0 X30. Y10. Z5.\nG234 H1\nG1 B60. C135. F2000.")
    tips = np.array(tr.tip)
    tcpc = [k for k, line in enumerate(tr.line) if line == 6]
    assert len(tcpc) > 10  # sampled, because the machine path is curved
    np.testing.assert_allclose(tips[tcpc], np.tile([30, 10, -45.8], (len(tcpc), 1)), atol=1e-6)


def test_without_tcpc_rotary_moves_the_part_away(kin, setup):
    tr = run(kin, setup, "G0 X30. Y10. Z5.\nG0 B60.")
    assert np.linalg.norm(last_tip(tr) - [30, 10, -45.8]) > 10


def test_g28_and_g53(kin, setup):
    tr = run(kin, setup, "G0 X10. Y10. Z10.\nG91 G28 Z0.\nG90 G53 X-100.")
    np.testing.assert_allclose(tr.q[-1][:3], [-100.0, tr.q[-2][1], 0.0])


def test_drill_cycles(kin, setup):
    tr = run(kin, setup, "G0 X0 Y0 Z20.\nG98 G83 X5. Y5. Z-12. R2. Q5. F300.\nX15.\nG80")
    zs = [tip[2] + 50.8 for tip, line in zip(tr.tip, tr.line) if line == 5]
    assert min(zs) == pytest.approx(-12.0)
    assert zs[-1] == pytest.approx(20.0)  # G98: back to the initial level
    feeds = [m for m, line in zip(tr.motion, tr.line) if line == 5 and m == MOTION["feed"]]
    assert len(feeds) == 3  # 5 + 5 + 2 mm pecks
    np.testing.assert_allclose(last_tip(tr)[:2], [15, 5])


def test_dwell_seconds_vs_milliseconds(kin, setup):
    a = run(kin, setup, "G4 P1.5")
    b = run(kin, setup, "G4 P1500")
    assert a.duration == pytest.approx(b.duration)


def test_tool_change_time_and_unknown_codes(kin, setup):
    tr = run(kin, setup, "G0 X0 Y0 Z10.\nG12.1\nM123\nT7 M6")
    assert sum(e["type"] == "toolchange" for e in tr.events) == 2
    msgs = " ".join(w["message"] for w in tr.warnings)
    assert "G12.1" in msgs and "M123" in msgs and "T7" in msgs


def test_feed_without_f_is_an_error(kin, setup):
    with pytest.raises(GCodeError):
        simulate("G1 X10.", kin, setup)
