"""Per-job setup: tool table, work offsets and stock. Loaded from a small YAML file.

Example (see examples/setup_demo.yaml):

    units: mm
    tools:
      1: {length: 95.0, diameter: 12.0, flute_length: 30.0, name: "12 mm flat endmill"}
    work_offsets:
      G54: {table_point: [0, 0, -0.8]}     # part zero as a point in the table frame
      G55: {X: -250.0, Y: -200.0, Z: -300.0}  # or machine coordinates, like the control shows
    stock: {type: box, size: [100, 80, 50], position: [0, 0, -50.8]}
    fixtures:                                # risers / vises -- collide with the tool like the table
      - {name: riser, type: cylinder, size: [80, 60], position: [0, 0, -50.8]}
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

from .kinematics import Kinematics


@dataclass
class Tool:
    number: int
    length: float = 100.0         # gauge line to tip (H offset), mm
    diameter: float = 10.0
    flute_length: float | None = None
    holder_diameter: float = 50.0  # simple cylinder standing in for the holder
    holder_length: float = 45.0
    name: str = ""

    @property
    def cutter_length(self) -> float:
        return max(self.length - self.holder_length, 1.0)


@dataclass
class Stock:
    """A simple solid on the table: the stock, or a fixture."""
    type: str = "box"                  # box | cylinder
    size: list[float] = field(default_factory=lambda: [100.0, 100.0, 50.0])  # box: x,y,z / cyl: d,h
    position: list[float] = field(default_factory=lambda: [0.0, 0.0, -50.8])  # bottom-centre, table frame

    name: str = "stock"

    @property
    def height(self) -> float:
        return self.size[2] if self.type == "box" else self.size[1]


@dataclass
class JobSetup:
    tools: dict[int, Tool] = field(default_factory=dict)
    work_offsets: dict[str, np.ndarray] = field(default_factory=dict)  # name -> X,Y,Z,B,C (machine)
    stock: Stock | None = None
    fixtures: list[Stock] = field(default_factory=list)
    default_tool: Tool = field(default_factory=lambda: Tool(number=0, name="default"))

    def tool(self, number: int) -> Tool:
        return self.tools.get(number, self.default_tool)

    def work_offset(self, name: str) -> np.ndarray:
        return self.work_offsets.get(name, np.zeros(5))


def default_setup(kin: Kinematics) -> JobSetup:
    """Part zero on the platter centre, top face; a generic tool; no stock."""
    wo = np.zeros(5)
    wo[:3] = kin.work_offset_from_table_point([0.0, 0.0, kin.m.raw["table"]["top_z"]])
    return JobSetup(work_offsets={"G54": wo})


def load_setup(path: str | Path, kin: Kinematics) -> JobSetup:
    raw = yaml.safe_load(Path(path).read_text()) or {}
    scale = 25.4 if raw.get("units", "mm") in ("in", "inch") else 1.0

    tools = {}
    for num, t in (raw.get("tools") or {}).items():
        t = dict(t)
        for k in ("length", "diameter", "flute_length", "holder_diameter", "holder_length"):
            if t.get(k) is not None:
                t[k] = float(t[k]) * scale
        tools[int(num)] = Tool(number=int(num), **t)

    offsets = {}
    for name, wo in (raw.get("work_offsets") or {}).items():
        v = np.zeros(5)
        if "table_point" in wo:
            v[:3] = kin.work_offset_from_table_point(np.array(wo["table_point"], dtype=float) * scale)
        else:
            v[:3] = [float(wo.get(a, 0.0)) * scale for a in "XYZ"]
        v[3:] = [float(wo.get(a, 0.0)) for a in "BC"]
        offsets[_normalise_offset_name(name)] = v
    if "G54" not in offsets:
        offsets["G54"] = default_setup(kin).work_offsets["G54"]

    def solid(s: dict, default_name: str) -> Stock:
        return Stock(
            type=s.get("type", "box"),
            size=[float(v) * scale for v in s["size"]],
            position=[float(v) * scale for v in s.get("position", [0.0, 0.0, kin.m.raw["table"]["top_z"] / scale])],
            name=s.get("name", default_name),
        )

    stock = solid(raw["stock"], "stock") if raw.get("stock") else None
    fixtures = [solid(f, f"fixture{i + 1}") for i, f in enumerate(raw.get("fixtures") or [])]
    setup = JobSetup(tools=tools, work_offsets=offsets, stock=stock, fixtures=fixtures)
    if "default_tool" in raw:
        setup.default_tool = Tool(number=0, **raw["default_tool"])
    return setup


def _normalise_offset_name(name: str) -> str:
    """'g54' -> 'G54', 'G154 P1' / 'G154P1' -> 'G154P1'."""
    return str(name).upper().replace(" ", "")
