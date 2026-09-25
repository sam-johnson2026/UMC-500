import numpy as np
import pytest

from umc_twin import Kinematics, load_machine
from umc_twin.job import Solid, Tool, default_setup, setup_from_dict
from umc_twin.sim import run_job

TOP = -50.8


@pytest.fixture(scope="module")
def kin():
    return Kinematics(load_machine())


def test_vise_geometry_and_rotation(kin):
    s = setup_from_dict({"fixtures": [{"name": "vise", "type": "vise", "model": "5axis", "opening": 60,
                                       "position": [0, 0, TOP], "rotation": [0, 0, 90]}]}, kin)
    b = s.fixtures[0].mesh.bounds
    # 280 long body turned 90 deg -> along Y; jaws 30 high on a 110 body
    np.testing.assert_allclose(b, [[-62.5, -140, TOP], [62.5, 140, TOP + 140]], atol=1e-6)
    with pytest.raises(ValueError):
        setup_from_dict({"fixtures": [{"type": "vise", "opening": 500}]}, kin)


def test_tool_into_vise_jaw_is_a_collision(kin):
    m = load_machine()
    s = setup_from_dict({"fixtures": [{"name": "vise", "type": "vise", "opening": 60, "position": [0, 0, TOP]}]}, kin)
    s.tools[1] = Tool(1, length=100, diameter=10)
    s.work_offsets["G54"][:3] = kin.work_offset_from_table_point([0, 0, TOP + 110])   # jaw floor
    r = run_job("T1 M6\nG54 G43 H1 G0 X45. Y0 Z50.\nG1 Z10. F500.", m, s, material=False)
    assert any(c["b"] == "fixture:vise" and c["line"] == 3 for c in r["collisions"])   # jaw top is 30 up


def test_second_operation_starts_from_first_ops_stock(kin, tmp_path):
    m = load_machine()
    face = "T1 M6\nG54 G43 H1 S6000 M3\nG0 X-50. Y-30. Z5.\nG1 Z{z} F800.\n" + \
           "".join(f"G1 X{50 if i % 2 == 0 else -50}. Y{-30 + i * 8}. F2000.\nG1 Y{-30 + (i + 1) * 8}.\n" for i in range(8))
    # op 1: face 2 mm off the top of a 60 x 40 x 20 block
    op1 = default_setup(kin)
    op1.tools[1] = Tool(1, length=100, diameter=12)
    op1.stock = Solid(type="box", size=[60, 40, 20], position=[0, 0, TOP])
    op1.material = {"resolution": 0.5}
    op1.work_offsets["G54"][:3] = kin.work_offset_from_table_point([0, 0, TOP + 20])
    stl = tmp_path / "op1.stl"
    r1 = run_job(face.format(z="-2."), m, op1, check_collisions=False, stock_out=stl)
    assert r1["material"]["removed_volume_mm3"] == pytest.approx(60 * 40 * 2, rel=0.05)
    # op 2: flip it (180 about X) so the old bottom is on top, face 1 mm
    op2 = setup_from_dict({"tools": {1: {"length": 100, "diameter": 12}},
                           "stock": {"file": str(stl), "rotation": [180, 0, 0], "position": [0, 0, 2 * TOP + 18]},
                           "work_offsets": {"G54": {"table_point": [0, 0, TOP + 18]}},
                           "material": {"resolution": 0.5}}, kin)
    b = op2.stock.mesh.bounds
    np.testing.assert_allclose(b[:, 2], [TOP, TOP + 18], atol=1e-6)
    r2 = run_job(face.format(z="-1."), m, op2, check_collisions=False)
    assert r2["material"]["stock_volume_mm3"] == pytest.approx(60 * 40 * 18, rel=0.03)
    assert r2["material"]["removed_volume_mm3"] == pytest.approx(60 * 40 * 1, rel=0.08)
