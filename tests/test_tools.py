from pathlib import Path

import numpy as np
import pytest
import trimesh

from umc_twin import Kinematics, load_machine
from umc_twin.job import Tool, load_setup
from umc_twin.toollib import load_tool_library

DATA = Path(__file__).parent / "data"


def test_profiles():
    ball = Tool(1, type="ball", diameter=10, length=80)
    assert ball.flute_radius(0.0) == pytest.approx(0.0)
    assert ball.flute_radius(5.0) == pytest.approx(5.0)
    assert ball.flute_radius(1.0) == pytest.approx(np.sqrt(25 - 16))
    drill = Tool(2, type="drill", diameter=10, length=100)
    assert drill.flute_radius(1.0) == pytest.approx(np.tan(np.radians(59)))
    bull = Tool(3, type="bull", diameter=10, corner_radius=1, length=80)
    assert bull.flute_radius(0.0) == pytest.approx(4.0)
    flat = Tool(4, diameter=10, length=100, holder=[[20, 30, 40], [30, 60, 60]])
    assert flat.holder_length == 50 and flat.stick_out == 50
    assert flat.body_radius(10) == 5.0            # flutes
    assert flat.body_radius(60) == pytest.approx(17.5)  # halfway up the first holder segment
    assert flat.body_radius(90) == 30.0 and flat.body_radius(120) == 0.0


def test_holder_longer_than_tool_is_trimmed():
    t = Tool(1, length=40, holder_length=45)
    assert t.holder_length < t.length


def test_fusion_import():
    tools = load_tool_library(DATA / "fusion_tools.json")
    assert sorted(tools) == [5, 6, 7]
    t5 = tools[5]
    assert (t5.type, t5.diameter, t5.flute_length) == ("flat", 12, 26)
    assert t5.length == pytest.approx(45 + 20 + 40)
    assert t5.holder[0] == [20, 24, 32]
    assert tools[6].type == "ball" and tools[6].diameter == pytest.approx(6.35)
    assert tools[6].stick_out == pytest.approx(38.1) and tools[6].flute_length == pytest.approx(19.05)
    assert tools[7].type == "chamfer" and tools[7].tip_angle == 90


def test_csv_import():
    tools = load_tool_library(DATA / "tools.csv")
    assert tools[3].type == "drill" and tools[3].tip_angle == 118
    assert tools[8].diameter == pytest.approx(12.7) and tools[8].length == pytest.approx(101.6)


def test_setup_with_library_override_and_mesh_fixture(tmp_path):
    box = trimesh.creation.box(extents=[20, 30, 10])
    box.apply_translation([0, 0, 5])
    box.export(tmp_path / "block.stl")
    (tmp_path / "setup.yaml").write_text(
        f"tool_library: {DATA / 'fusion_tools.json'}\n"
        "tools:\n  5: {length: 110.0, diameter: 12, holder: [[20, 24, 32], [40, 50, 50]]}\n"
        "fixtures:\n  - {name: block, file: block.stl, position: [10, 0, -50.8], rotation: [0, 0, 90]}\n")
    setup = load_setup(tmp_path / "setup.yaml", Kinematics(load_machine()))
    assert setup.tools[5].length == 110.0 and 6 in setup.tools
    b = setup.fixtures[0].mesh.bounds
    np.testing.assert_allclose(b, [[-5, -10, -50.8], [25, 10, -40.8]], atol=1e-6)  # rotated 90 about Z
