"""Macros, subprograms, cutter compensation, G68 and G10."""
import numpy as np
import pytest

import umc_twin.gcode as gcode
from umc_twin import Kinematics, load_machine
from umc_twin.gcode import MOTION, GCodeError, simulate
from umc_twin.job import Tool, default_setup

TOP = -50.8


@pytest.fixture(scope="module")
def kin():
    return Kinematics(load_machine())


@pytest.fixture
def setup(kin):
    s = default_setup(kin)
    s.tools[1] = Tool(1, length=100.0, diameter=10.0)
    return s


def run(kin, setup, body, **kw):
    return simulate("G21 G90 G17 G94\nT1 M6\nG54 G43 H1\n" + body, kin, setup, accel=False, **kw)


def xy_at(tr, line):
    return [np.array(t[:2]) for t, ln in zip(tr.tip, tr.line) if ln == line]


def feed_tips(tr):
    return np.array([t for t, m in zip(tr.tip, tr.motion) if m in (MOTION["feed"], MOTION["arc"])])


# ------------------------------------------------------------------ macros
def test_while_loop_and_expressions(kin, setup):
    prog = """#100 = 0
#101 = 2.5 * [1 + 1]
WHILE [#100 LT 3] DO1
G0 X[#100 * #101] Y-#101 Z5.
#100 = #100 + 1
END1
G0 X[SQRT[16] + ABS[-1]] Y[ROUND[2.6]]"""
    tr = run(kin, setup, prog)
    xs = [t[0] for t, ln in zip(tr.tip, tr.line) if ln == 7]
    assert xs == pytest.approx([0.0, 5.0, 10.0])
    np.testing.assert_allclose(tr.tip[-1][:2], [5.0, 3.0])


def test_if_goto_and_if_then(kin, setup):
    prog = """#1 = 7
IF [#1 GT 5] GOTO 100
G0 X99.
N100 IF [#1 EQ 7] THEN #2 = 3
G0 X#2 Y0 Z5."""
    tr = run(kin, setup, prog)
    assert not any(ln == 6 for ln in tr.line)             # skipped by the GOTO
    np.testing.assert_allclose(tr.tip[-1][:2], [3.0, 0.0])


def test_vacant_variable_drops_the_word(kin, setup):
    tr = run(kin, setup, "G0 X10. Y10. Z5.\nG0 X#150 Y20.")
    np.testing.assert_allclose(tr.tip[-1][:2], [10.0, 20.0])


def test_system_variables(kin, setup):
    prog = """G0 X10. Y20. Z5.
#5221 = [#5221 + 1.0]
#110 = #5041
#111 = #2001
G0 X#110 Y0."""
    tr = run(kin, setup, prog)
    assert setup.work_offsets["G54"][0] == pytest.approx(-252.5 + 1.0)
    # the work position read back after the offset change (X10 - 1) is where it goes
    np.testing.assert_allclose(tr.tip[-1][0], 10.0, atol=1e-9)


def test_alarm_stops_the_program(kin, setup):
    tr = run(kin, setup, "#3000 = 1 (PROBE FAILED)\nG0 X50. Z5.")
    assert any(e["type"] == "alarm" and "PROBE FAILED" in e["text"] for e in tr.events)
    assert not any(ln == 5 for ln in tr.line)


def test_runaway_loop_is_stopped(kin, setup, monkeypatch):
    monkeypatch.setattr(gcode, "MAX_BLOCKS", 500)
    with pytest.raises(GCodeError, match="endless loop"):
        run(kin, setup, "WHILE [1 EQ 1] DO1\n#100 = 1\nEND1")


# ------------------------------------------------------------------ subprograms
def test_m97_local_subprogram_with_repeats(kin, setup):
    prog = """G0 X0 Y0 Z5.
G91
M97 P100 L3
G90
M30
N100 G0 X10.
M99"""
    tr = run(kin, setup, prog)
    np.testing.assert_allclose(tr.tip[-1][0], 30.0)


def test_m98_in_same_file_and_external_and_m99_p(kin, setup, tmp_path):
    (tmp_path / "O02000.nc").write_text("%\nO02000\nG0 Y25.\nM99\n%\n")
    prog = """G0 X0 Y0 Z5.
M98 P1000
M98 P2000
M99 P300
N300 G0 X99.
M30
O01000
G0 X15.
M99"""
    with pytest.raises(GCodeError):
        run(kin, setup, prog)                              # O2000 not found without search paths
    tr = run(kin, setup, prog, search_paths=[tmp_path])
    np.testing.assert_allclose(tr.tip[-1][:2], [15.0, 25.0])  # M99 P300 in main -> ends
    assert any("M99 in the main program" in w["message"] for w in tr.warnings)


def test_g65_macro_call_with_arguments(kin, setup):
    prog = """G0 X0 Y0 Z5. S3000 M3 F500.
#1 = 111
G65 P9000 X12. Y4. R3. S6. F7.
G0 Z[#1]
M30
O09000
G0 X[#24 + #18] Y#25
M99"""
    tr = run(kin, setup, prog)
    xy = [t for t, ln in zip(tr.tip, tr.line) if ln == 7][-1]
    np.testing.assert_allclose(xy[:2], [15.0, 4.0])
    assert tr.tip[-1][2] == pytest.approx(TOP + 111)       # caller's #1 untouched by the call
    assert tr.spindle[-1] == 3000.0                        # S6. was an argument, not a spindle speed


# ------------------------------------------------------------------ G68 / G10
def test_g68_rotation(kin, setup):
    tr = run(kin, setup, "G0 Z5.\nG68 X0 Y0 R90.\nG0 X10. Y0\nG69\nG0 X10. Y0")
    rotated = xy_at(tr, 6)[-1]
    np.testing.assert_allclose(rotated, [0.0, 10.0], atol=1e-9)
    np.testing.assert_allclose(tr.tip[-1][:2], [10.0, 0.0], atol=1e-9)


def test_g10_offsets(kin, setup):
    tr = run(kin, setup, "G10 L2 P2 X-200. Y-150. Z-250.\nG10 L10 P1 R80.\nG55 G43 H1\nG0 X0 Y0 Z0")
    np.testing.assert_allclose(setup.work_offsets["G55"][:3], [-200, -150, -250])
    assert setup.tools[1].length == 80.0
    np.testing.assert_allclose(tr.q[-1][:3], [-200, -150, -250 + 80])


# ------------------------------------------------------------------ cutter compensation
SQUARE_CW = """G0 X-40. Y-20. Z5.
G1 Z-2. F500.
G41 D1 G1 X-20. Y-20.
Y20.
X20.
Y-20.
X-20.
G40 G1 X-40. Y-20."""


def square_distance(p, h=20.0):
    """Distance from p to the boundary of the square |x|,|y| <= h (negative inside)."""
    d = np.abs(p[:, :2]) - h
    outside = np.linalg.norm(np.maximum(d, 0), axis=1)
    inside = np.minimum(np.max(d, axis=1), 0)
    return outside + inside


def test_g41_outside_contour_offsets_by_the_radius(kin, setup):
    tr = run(kin, setup, SQUARE_CW)
    lines = [ln for ln in tr.line]
    tips = np.array([t for t, ln in zip(tr.tip, lines) if 7 <= ln <= 10])   # the square, after start-up
    d = square_distance(tips)
    assert d.min() == pytest.approx(5.0, abs=1e-6) and d.max() == pytest.approx(5.0, abs=1e-6)
    # outside corners are arcs of the tool radius around the corner
    corner = tips[(tips[:, 0] < -20) & (tips[:, 1] > 20)]
    assert len(corner) > 3
    np.testing.assert_allclose(np.linalg.norm(corner[:, :2] - [-20, 20], axis=1), 5.0, atol=1e-6)


def test_g42_inside_pocket_trims_corners(kin, setup):
    prog = SQUARE_CW.replace("G41", "G42").replace("X-40. Y-20.", "X0 Y0")
    tr = run(kin, setup, prog)
    # sides 1-3 (the last side ends at the G40 cancel, perpendicular to its own end, as on the control)
    tips = np.array([t for t, ln in zip(tr.tip, tr.line) if 7 <= ln <= 9])
    d = square_distance(tips)
    assert d.max() == pytest.approx(-5.0, abs=1e-6)                  # always 5 inside the wall
    corners = {tuple(np.round(t[:2], 6)) for t in tips}
    assert (-15.0, 15.0) in corners and (15.0, 15.0) in corners       # sharp, trimmed corners


def test_d_offset_zero_means_no_offset(kin, setup):
    setup.tools[1].d_offset = 0.0
    tr = run(kin, setup, SQUARE_CW)
    tips = np.array([t for t, ln in zip(tr.tip, tr.line) if 7 <= ln <= 10])
    assert np.abs(square_distance(tips)).max() < 1e-6


def test_comp_with_arcs_and_z_moves(kin, setup):
    prog = """G0 X-30. Y0 Z5.
G1 Z-1. F500.
G41 D1 G1 X-20. Y0
G2 X20. Y0 I20. J0
G1 Z-2.
G2 X-20. Y0 I-20. J0
G40 G1 X-30. Y0"""
    tr = run(kin, setup, prog)
    tips = feed_tips(tr)
    on_circle = [t for t, ln in zip(tr.tip, tr.line) if ln in (7, 9)]
    radii = np.linalg.norm(np.array(on_circle)[:, :2], axis=1)
    assert radii.min() == pytest.approx(25.0, abs=1e-6) and radii.max() == pytest.approx(25.0, abs=1e-6)
    assert tips[:, 2].min() == pytest.approx(TOP - 2)                  # the Z move ran, in order
    assert not tr.warnings
