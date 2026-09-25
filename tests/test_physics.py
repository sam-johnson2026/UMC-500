import pytest

from umc_twin import Kinematics, load_machine
from umc_twin.gcode import simulate
from umc_twin.job import Solid, Tool, default_setup
from umc_twin.material import simulate_material
from umc_twin.physics import spindle_load


@pytest.fixture(scope="module")
def ctx():
    m = load_machine()
    kin = Kinematics(m)
    s = default_setup(kin)
    s.work_offsets["G54"][:3] = kin.work_offset_from_table_point([0, 0, -50.8 + 20])
    s.tools[1] = Tool(1, length=100.0, diameter=10.0, flute_length=25.0)
    s.stock = Solid(type="box", size=[200, 40, 20], position=[0, 0, -50.8])
    s.material = {"name": "steel_1018", "resolution": 0.5}
    return m, kin, s


def run(ctx, body):
    m, kin, s = ctx
    tr = simulate("G21 G90 G94\nT1 M6\nG54 G43 H1 S3000 M3\nG0 X-120. Y0 Z5.\nG1 Z-4. F500.\n" + body, kin, s)
    mat = simulate_material(tr, kin, s)
    return spindle_load(tr, mat, s, m.raw["spindle"])


def test_full_slot_power_matches_hand_calculation(ctx):
    m, *_ = ctx
    sl = run(ctx, "G1 X100. F600.")
    mrr = 10 * 4 * 600 / 60.0                       # mm^3/s: width x depth x feed
    power = mrr * 2.5 / m.raw["spindle"]["efficiency"]  # steel_1018: 2.5 J/mm^3
    assert sl["peak"]["mrr_cm3_min"] == pytest.approx(mrr * 60 / 1000, rel=0.08)
    assert sl["peak"]["power_kw"] == pytest.approx(power / 1000, rel=0.08)
    omega = 3000 * 2 * 3.14159265 / 60
    assert sl["peak"]["torque_nm"] == pytest.approx(power / omega, rel=0.08)
    assert sl["issues"] == []


def test_overload_and_rpm_flags(ctx):
    sl = run(ctx, "S20000\nG1 X100. F30000.")      # absurd feed and a speed above the spindle max
    kinds = {i["kind"] for i in sl["issues"]}
    assert "rpm_over_max" in kinds


def test_overload_at_low_rpm(ctx):
    m, kin, s = ctx
    tr = simulate("G21 G90 G94\nT1 M6\nG54 G43 H1 S200 M3\nG0 X-120. Y0 Z5.\nG1 Z-10. F500.\nG1 X100. F3000.", kin, s)
    sl = spindle_load(tr, simulate_material(tr, kin, s).removed_per_sample, s, m.raw["spindle"])
    assert any(i["kind"] == "spindle_overload" for i in sl["issues"])


def test_unknown_material(ctx):
    m, kin, s = ctx
    from umc_twin.physics import specific_energy
    s2 = default_setup(kin)
    s2.material = {"name": "unobtainium"}
    with pytest.raises(ValueError):
        specific_energy(s2)
    s2.material = {"name": "unobtainium", "specific_energy": 9.0}
    assert specific_energy(s2)[1] == 9.0
