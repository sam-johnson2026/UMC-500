import numpy as np
import pytest
import trimesh

from umc_twin import Kinematics, load_machine
from umc_twin.gcode import simulate
from umc_twin.job import JobSetup, Solid, Tool, default_setup
from umc_twin.material import VoxelGrid, occupancy, simulate_material, surface_mesh, voxelize_mesh

TOP = -50.8 + 20.0  # 20 mm tall stock on the platter; G54 below = its top centre


@pytest.fixture(scope="module")
def kin():
    return Kinematics(load_machine())


def job(kin, part=None, flute_length=None, res=0.5) -> JobSetup:
    s = default_setup(kin)
    s.work_offsets["G54"][:3] = kin.work_offset_from_table_point([0, 0, TOP])
    s.tools[1] = Tool(1, type="flat", length=100.0, diameter=10.0, flute_length=flute_length or 25.0,
                      holder=[[40, 30, 30], [20, 50, 50]])
    s.stock = Solid(type="box", size=[80, 40, 20], position=[0, 0, -50.8])
    s.part = part
    s.material = {"resolution": res}
    return s


def run(kin, setup, body):
    prog = "G21 G90 G94\nT1 M6\nG54 G43 H1 S8000 M3\nG0 X-60. Y0 Z10.\n" + body
    tr = simulate(prog, kin, setup, accel=False)
    return tr, simulate_material(tr, kin, setup)


def test_slot_volume(kin):
    _, r = run(kin, job(kin), "G1 Z-5. F500.\nG1 X20. F1000.\nG0 Z10.")
    # slot from outside the stock (x=-40) to x=20: 60 x 10 x 5, plus a half-disc at the end
    expected = 60 * 10 * 5 + 0.5 * np.pi * 25 * 5
    assert r.removed_per_sample.sum() == pytest.approx(expected, rel=0.04)
    assert r.issues == []


def test_rapid_into_material_but_not_out_of_a_pocket(kin):
    _, r = run(kin, job(kin), "G1 Z-5. F500.\nG1 X0 F1000.\nG0 Z10.\nG0 X10.\nG0 Z-3.\nG0 Z10.")
    kinds = [(i.line, i.kind) for i in r.issues]
    assert (9, "rapid_into_material") in kinds       # G0 Z-3 at X10 plunges into uncut stock
    assert not any(line == 7 for line, _ in kinds)    # G0 Z10 out of the slot is fine


def test_shank_contact_when_side_cutting_deeper_than_the_flutes(kin):
    # plunging is fine (the shank follows the hole the flutes cut) ...
    _, plunge = run(kin, job(kin, flute_length=5.0), "G0 X0\nG1 Z-8. F300.\nG0 Z10.")
    assert not any(i.kind == "shank_contact" for i in plunge.issues)
    # ... but moving sideways 8 deep with 5 mm of flutes drags the shank through material
    _, side = run(kin, job(kin, flute_length=5.0), "G0 X0\nG1 Z-8. F300.\nG1 X10. F500.\nG0 Z10.")
    assert any(i.kind == "shank_contact" and i.line == 7 for i in side.issues)


def test_cutting_with_spindle_off(kin):
    _, r = run(kin, job(kin), "M5\nG1 Z-2. F500.\nG1 X0 F1000.")
    assert any(i.kind == "spindle_off_cut" for i in r.issues)


def test_gouge_detection(kin):
    # finished part: the stock minus a 3 mm deep slot everywhere above z = TOP - 3
    part = Solid(type="box", size=[80, 40, 17], position=[0, 0, -50.8])
    _, ok = run(kin, job(kin, part), "G1 Z-3. F500.\nG1 X40. F1000.\nG0 Z10.")
    assert not any(i.kind == "gouge" for i in ok.issues)
    _, bad = run(kin, job(kin, part), "G1 Z-4. F500.\nG1 X40. F1000.\nG0 Z10.")
    assert any(i.kind == "gouge" for i in bad.issues)


def test_tool_tilted_by_tcpc_cuts_where_expected(kin):
    s = job(kin)
    _, r = run(kin, s, "G0 X0 Y0 Z10.\nG234 H1\nG0 B30.\nG1 Z-2. F300.\nG0 Z10.")
    g = r.grid
    cut = g.initial & ~g.material
    idx = np.argwhere(cut)
    centre = np.array([g.axes[i][idx[:, i]].mean() for i in range(3)])
    assert np.hypot(centre[0], centre[1]) < 3.0          # hole at the part zero, not smeared sideways
    assert r.removed_per_sample.sum() > 20


def test_mesh_voxelisation_matches_analytic():
    box = Solid(type="box", size=[20, 10, 6], position=[1, 2, 3])
    grid = VoxelGrid(box, resolution=0.5)
    mesh_solid = Solid(type="box", size=[20, 10, 6], position=[1, 2, 3])
    analytic = occupancy(box, grid)
    from_mesh = voxelize_mesh(mesh_solid.mesh, grid)
    assert (analytic != from_mesh).sum() <= 0.01 * analytic.sum()
    cyl = trimesh.creation.cylinder(radius=5, height=8, sections=64)
    assert voxelize_mesh(cyl, VoxelGrid(Solid(type="cylinder", size=[10, 8], position=[0, 0, -4]), 0.5)).sum() * 0.125 \
        == pytest.approx(np.pi * 25 * 8, rel=0.05)


def test_surface_mesh_is_closed():
    occ = np.zeros((6, 6, 6), dtype=bool)
    occ[1:5, 1:5, 1:4] = True
    occ[2:4, 2:4, 3] = False  # a pocket
    m = surface_mesh(occ, [0, 0, 0], 2.0)
    assert m.is_watertight
    assert m.volume == pytest.approx(occ.sum() * 8.0)
