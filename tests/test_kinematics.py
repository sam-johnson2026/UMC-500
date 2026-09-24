import numpy as np
import pytest

from umc_twin import Kinematics, load_machine


@pytest.fixture(scope="module")
def kin():
    return Kinematics(load_machine())


def test_home_tip_is_gauge_point(kin):
    np.testing.assert_allclose(kin.tool_tip_world(np.zeros(5)), kin.gauge)
    np.testing.assert_allclose(kin.tool_tip_world(np.zeros(5), 100.0), kin.gauge + [0, 0, -100])


def test_linear_axes_move_spindle_in_machine_directions(kin):
    tip0 = kin.tool_tip_world(np.zeros(5))
    tip = kin.tool_tip_world(np.array([-10.0, -20.0, -30.0, 0, 0]))
    np.testing.assert_allclose(tip - tip0, [-10, -20, -30])


def test_pivot_is_fixed_under_rotation(kin):
    T = kin.table_transform(np.array([0, 0, 0, 37.0, 123.0]))
    np.testing.assert_allclose(T @ [0, 0, 0, 1], [0, 0, 0, 1], atol=1e-9)


def test_b_positive_tilts_platter_face_toward_plus_x(kin):
    R = kin.table_transform(np.array([0, 0, 0, 90.0, 0]))[:3, :3]
    np.testing.assert_allclose(R @ [0, 0, 1], [1, 0, 0], atol=1e-9)


def test_c_is_about_platter_normal(kin):
    R = kin.table_transform(np.array([0, 0, 0, 30.0, 77.0]))[:3, :3]
    Rb = kin.table_transform(np.array([0, 0, 0, 30.0, 0]))[:3, :3]
    np.testing.assert_allclose(R @ [0, 0, 1], Rb @ [0, 0, 1], atol=1e-9)


@pytest.mark.parametrize("b,c", [(0, 0), (45, 0), (-30, 200), (110, -45)])
def test_inverse_tcp_round_trip(kin, b, c):
    p = np.array([12.5, -40.0, 20.0])
    q = kin.inverse_tcp(p, b, c, tool_length=95.0)
    assert q[3] == b and q[4] == c
    np.testing.assert_allclose(kin.tool_tip_in_table(q, 95.0), p, atol=1e-9)


def test_work_offset_round_trip(kin):
    point = np.array([10.0, -5.0, -0.8])
    wo = kin.work_offset_from_table_point(point)
    np.testing.assert_allclose(kin.table_point_from_work_offset(wo), point)
    # platter centre, top face, with a zero-length tool
    np.testing.assert_allclose(kin.work_offset_from_table_point([0, 0, -50.8]), [-252.5, -204.7, -261.28])


def test_limits(kin):
    assert kin.limit_violations(np.zeros(5)) == []
    assert kin.limit_violations(np.array([-600.0, 0, 0, 0, 0]))[0].startswith("X=")
    assert kin.limit_violations(np.array([0, 0, 0, -40.0, 9999.0]))[0].startswith("B=")  # C is continuous


def test_options_select_parts():
    ids = {p.id for p in load_machine(options={"spindle": "hsk"}).active_parts()}
    assert "spindle_hsk" in ids and "spindle_cat40" not in ids
    with pytest.raises(ValueError):
        load_machine(options={"spindle": "bt30"})
