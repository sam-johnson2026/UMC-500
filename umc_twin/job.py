"""Per-job setup: tool table, work offsets, stock, fixtures and (optionally) the finished part.

Example (see examples/setup_demo.yaml):

    units: mm
    tool_library: my_tools.json            # Fusion 360 export or CSV, relative to this file
    tools:                                 # additions / overrides
      1: {name: "12 mm flat endmill", type: flat, length: 95.0, diameter: 12.0, flute_length: 30.0}
      2: {name: "6 mm ball", type: ball, length: 80, diameter: 6,
          holder: [[40, 25, 25], [20, 40, 50]]}   # [height, lower dia, upper dia] from the tool up
    work_offsets:
      G54: {table_point: [0, 0, -0.8]}     # part zero as a point in the table frame
      G55: {X: -250.0, Y: -200.0, Z: -300.0}  # or machine coordinates, like the control shows
    stock: {type: box, size: [100, 80, 50], position: [0, 0, -50.8]}
    fixtures:                              # collide with the tool like the table does
      - {name: riser, type: cylinder, size: [80, 60], position: [0, 0, -50.8]}
      - {name: vise, file: fixtures/vise.stl, position: [0, 0, -50.8], rotation: [0, 0, 90]}
    part: {file: part.stl, position: [0, 0, -50.8]}   # finished part, for gouge checks
    material: {name: aluminum_6061, resolution: 1.0}  # cutting sim (see material.py)

Box / cylinder `position` is the bottom centre, in the table frame. For mesh files, `position`
and `rotation` (degrees, applied X then Y then Z) place the file's own origin in the table frame.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from functools import cached_property
from pathlib import Path

import numpy as np
import yaml

from .kinematics import Kinematics

# Default included tip angles (degrees) for pointed tools.
TIP_ANGLE = {"drill": 118.0, "spot": 90.0, "chamfer": 90.0}
TOOL_TYPES = ("flat", "ball", "bull", "drill", "spot", "chamfer")


@dataclass
class Tool:
    number: int
    length: float = 100.0              # gauge line to tip (H offset), mm
    diameter: float = 10.0
    type: str = "flat"                 # flat | ball | bull | drill | spot | chamfer
    flute_length: float | None = None  # default: 3 x diameter, capped by the stick-out
    corner_radius: float = 0.0         # bull nose
    tip_angle: float | None = None     # included angle, pointed tools
    tip_diameter: float = 0.0          # chamfer mills with a flat tip
    shank_diameter: float | None = None
    # holder as stacked frustums from the tool end upward: [height, lower dia, upper dia]
    holder: list[list[float]] | None = None
    holder_diameter: float = 50.0      # shorthand for a single-cylinder holder
    holder_length: float = 45.0
    name: str = ""
    d_offset: float | None = None      # cutter-comp radius (the D register); None = the tool's radius.
                                       # 0 when CAM already offsets the path and D holds wear only.

    def __post_init__(self):
        if self.type not in TOOL_TYPES:
            self.type = "flat"
        if self.holder is None:
            self.holder = [[self.holder_length, self.holder_diameter, self.holder_diameter]]
        self.holder = [[float(h), float(a), float(b)] for h, a, b in self.holder if h > 0]
        total = sum(h for h, _, _ in self.holder)
        if total >= self.length:  # holder can't reach the tip: shorten it from the tool end
            excess = total - self.length + 1.0
            while excess > 0 and self.holder:
                h = self.holder[0][0]
                if h <= excess:
                    self.holder.pop(0)
                    excess -= h
                else:
                    self.holder[0][0] = h - excess
                    excess = 0
        self.holder_length = sum(h for h, _, _ in self.holder)
        if self.holder:
            self.holder_diameter = max(max(a, b) for _, a, b in self.holder)
        if self.shank_diameter is None:
            self.shank_diameter = self.diameter
        stick_out = self.length - self.holder_length
        if self.flute_length is None:
            self.flute_length = min(3.0 * self.diameter, stick_out)
        self.flute_length = float(min(self.flute_length, stick_out))
        if self.tip_angle is None and self.type in TIP_ANGLE:
            self.tip_angle = TIP_ANGLE[self.type]

    @property
    def radius(self) -> float:
        return self.diameter / 2.0

    @property
    def stick_out(self) -> float:
        return self.length - self.holder_length

    def flute_radius(self, h):
        """Cutting radius at height h above the tip (array in, array out), within the flutes."""
        h = np.asarray(h, dtype=float)
        R = self.radius
        if self.type == "ball":
            hh = np.clip(h, 0.0, R)
            return np.where(h < R, np.sqrt(np.maximum(R * R - (R - hh) ** 2, 0.0)), R)
        if self.type == "bull" and self.corner_radius > 0:
            rc = min(self.corner_radius, R)
            hh = np.clip(h, 0.0, rc)
            return np.where(h < rc, R - rc + np.sqrt(np.maximum(rc * rc - (rc - hh) ** 2, 0.0)), R)
        if self.tip_angle:
            return np.minimum(R, self.tip_diameter / 2.0 + h * np.tan(np.radians(self.tip_angle) / 2.0))
        return np.full_like(h, R)

    def body_radius(self, h):
        """Radius of the whole tool + holder at height h above the tip (0 outside it)."""
        h = np.asarray(h, dtype=float)
        r = np.where(h <= self.flute_length, self.flute_radius(h), self.shank_diameter / 2.0)
        base = self.stick_out
        for seg_h, d0, d1 in self.holder:
            f = np.clip((h - base) / seg_h, 0.0, 1.0)
            r = np.where((h > base) & (h <= base + seg_h), (d0 + (d1 - d0) * f) / 2.0, r)
            base += seg_h
        return np.where((h < 0) | (h > self.length), 0.0, r)

    def profile(self) -> list[tuple[float, float]]:
        """(radius, height-above-tip) outline of the tool itself (flutes + shank), tip to holder."""
        n = 12 if self.type in ("ball", "bull") or self.tip_angle else 1
        hs = list(np.linspace(0.0, self.flute_length, n + 1))
        if self.tip_angle:  # make sure the cone/cylinder knee is on the outline
            knee = (self.radius - self.tip_diameter / 2) / np.tan(np.radians(self.tip_angle) / 2)
            if 0 < knee < self.flute_length:
                hs = sorted(set(hs + [knee]))
        pts = [(0.0, 0.0)] + [(float(self.flute_radius(h)), float(h)) for h in hs]
        if self.stick_out > self.flute_length:
            pts += [(self.shank_diameter / 2, self.flute_length), (self.shank_diameter / 2, self.stick_out)]
        pts.append((0.0, pts[-1][1]))
        return _dedupe(pts)

    def holder_profile(self) -> list[tuple[float, float]]:
        pts, base = [(0.0, self.stick_out)], self.stick_out
        for seg_h, d0, d1 in self.holder:
            pts += [(d0 / 2, base), (d1 / 2, base + seg_h)]
            base += seg_h
        pts.append((0.0, base))
        return _dedupe(pts)


def _dedupe(pts):
    out = []
    for p in pts:
        if not out or abs(p[0] - out[-1][0]) > 1e-9 or abs(p[1] - out[-1][1]) > 1e-9:
            out.append((round(p[0], 4), round(p[1], 4)))
    return out


@dataclass
class Solid:
    """Something on the table: the stock, a fixture, or the finished part."""
    type: str = "box"                  # box | cylinder | mesh
    size: list[float] = field(default_factory=lambda: [100.0, 100.0, 50.0])  # box: x,y,z / cyl: d,h
    position: list[float] = field(default_factory=lambda: [0.0, 0.0, -50.8])
    name: str = "stock"
    file: str | None = None            # mesh: path (resolved)
    rotation: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    file_scale: float = 1.0            # mesh units -> mm

    @property
    def height(self) -> float:
        if self.type == "mesh":
            b = self.mesh.bounds
            return float(b[1][2] - b[0][2])
        return self.size[2] if self.type == "box" else self.size[1]

    @cached_property
    def mesh(self):
        """The solid as a trimesh in the table frame."""
        import trimesh

        if self.type == "mesh":
            from .cadio import load_mesh_file

            mesh = load_mesh_file(self.file)
            mesh.apply_scale(self.file_scale)
            T = trimesh.transformations.euler_matrix(*np.radians(self.rotation), axes="sxyz")
            T[:3, 3] = self.position
            mesh.apply_transform(T)
            return mesh
        if self.type == "cylinder":
            mesh = trimesh.creation.cylinder(radius=self.size[0] / 2, height=self.size[1], sections=64)
        else:
            mesh = trimesh.creation.box(extents=self.size)
        mesh.apply_translation(np.array(self.position) + [0, 0, self.height / 2])
        return mesh


Stock = Solid  # backwards-compatible name


@dataclass
class JobSetup:
    tools: dict[int, Tool] = field(default_factory=dict)
    work_offsets: dict[str, np.ndarray] = field(default_factory=dict)  # name -> X,Y,Z,B,C (machine)
    stock: Solid | None = None
    fixtures: list[Solid] = field(default_factory=list)
    part: Solid | None = None
    material: dict = field(default_factory=dict)
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


LENGTH_KEYS = ("length", "diameter", "flute_length", "corner_radius", "tip_diameter", "shank_diameter",
               "holder_diameter", "holder_length", "d_offset")


def tool_from_dict(num: int, t: dict, scale: float = 1.0) -> Tool:
    t = dict(t)
    for k in LENGTH_KEYS:
        if t.get(k) is not None:
            t[k] = float(t[k]) * scale
    if t.get("holder"):
        t["holder"] = [[float(v) * scale for v in seg] for seg in t["holder"]]
    return Tool(number=int(num), **t)


def load_setup(path: str | Path, kin: Kinematics) -> JobSetup:
    path = Path(path)
    return setup_from_dict(yaml.safe_load(path.read_text()) or {}, kin, base_dir=path.parent)


def setup_from_dict(raw: dict, kin: Kinematics, base_dir: Path = Path(".")) -> JobSetup:
    scale = 25.4 if raw.get("units", "mm") in ("in", "inch") else 1.0

    tools: dict[int, Tool] = {}
    if raw.get("tool_library"):
        from .toollib import load_tool_library

        tools.update(load_tool_library(base_dir / raw["tool_library"]))
    for num, t in (raw.get("tools") or {}).items():
        tools[int(num)] = tool_from_dict(num, t, scale)

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

    top_z = kin.m.raw["table"]["top_z"]

    def solid(s: dict, default_name: str) -> Solid:
        if s.get("file"):
            file_units = s.get("units", raw.get("units", "mm"))
            return Solid(type="mesh", name=s.get("name", default_name), file=str(base_dir / s["file"]),
                         position=[float(v) * scale for v in s.get("position", [0.0, 0.0, 0.0])],
                         rotation=[float(v) for v in s.get("rotation", [0.0, 0.0, 0.0])],
                         file_scale=25.4 if file_units in ("in", "inch") else 1.0, size=[])
        return Solid(
            type=s.get("type", "box"),
            size=[float(v) * scale for v in s["size"]],
            position=[float(v) * scale for v in s.get("position", [0.0, 0.0, top_z / scale])],
            name=s.get("name", default_name),
        )

    setup = JobSetup(
        tools=tools, work_offsets=offsets,
        stock=solid(raw["stock"], "stock") if raw.get("stock") else None,
        fixtures=[solid(f, f"fixture{i + 1}") for i, f in enumerate(raw.get("fixtures") or [])],
        part=solid(raw["part"], "part") if raw.get("part") else None,
        material=dict(raw.get("material") or {}),
    )
    if "default_tool" in raw:
        setup.default_tool = tool_from_dict(0, raw["default_tool"], scale)
    return setup


def _normalise_offset_name(name: str) -> str:
    """'g54' -> 'G54', 'G154 P1' / 'G154P1' -> 'G154P1'."""
    return str(name).upper().replace(" ", "")
