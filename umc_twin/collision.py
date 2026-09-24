"""Mesh collision checking along a trajectory (trimesh + python-fcl).

Bodies are grouped into a *moving head* set (spindle side: X/Y/Z links, holder, cutter) and a
*table/frame* set (base, trunnion, platter, stock). Only head-vs-table pairs are tested -- the
head parts can't hit each other, and neither can the table parts.

Pairs that already touch at the CAD pose (e.g. the X saddle sitting on the base ways) are
treated as adjacent and ignored, the same trick MoveIt uses for its allowed-collision matrix.
Cutter-vs-stock contact is cutting, not a crash. There is no material removal yet, so the
stock stays a full solid: a rapid is only flagged when it takes the cutter *into* the stock
from outside (retracting out of a machined pocket is not a crash).
"""
from __future__ import annotations

import json
from dataclasses import dataclass

import numpy as np
import trimesh

from .config import REPO_ROOT, Machine
from .gcode import MOTION, Trajectory
from .job import JobSetup, Stock, Tool
from .kinematics import Kinematics

ASSETS = REPO_ROOT / "web" / "assets"
HEAD_LINKS = {"X", "Y", "Z"}


@dataclass
class Collision:
    t: float
    line: int
    a: str
    b: str
    q: list[float]
    kind: str = "collision"  # collision | rapid_into_stock

    def as_dict(self):
        return {"t": round(self.t, 3), "line": self.line, "a": self.a, "b": self.b,
                "q": [round(v, 3) for v in self.q], "kind": self.kind}


def load_part_mesh(part_id: str) -> trimesh.Trimesh:
    scene = trimesh.load(ASSETS / "parts" / f"{part_id}.glb", force="scene")
    return trimesh.util.concatenate(scene.dump())


def tool_meshes(kin: Kinematics, tool: Tool) -> dict[str, trimesh.Trimesh]:
    """Holder + cutter cylinders in the world frame at the CAD pose (hanging off the gauge point)."""
    d = kin.tool_dir
    out = {}
    hl = min(tool.holder_length, tool.length - 1.0)
    if hl > 0:
        holder = trimesh.creation.cylinder(radius=tool.holder_diameter / 2, height=hl, sections=24)
        out["holder"] = _place_along(holder, kin.gauge, d, 0.0, hl)
    cut = trimesh.creation.cylinder(radius=tool.diameter / 2, height=tool.length - max(hl, 0), sections=24)
    out["cutter"] = _place_along(cut, kin.gauge, d, max(hl, 0), tool.length)
    return out


def stock_mesh(stock: Stock) -> trimesh.Trimesh:
    if stock.type == "cylinder":
        mesh = trimesh.creation.cylinder(radius=stock.size[0] / 2, height=stock.size[1], sections=48)
    else:
        mesh = trimesh.creation.box(extents=stock.size)
    mesh.apply_translation(np.array(stock.position) + [0, 0, stock.height / 2])
    return mesh


def _place_along(mesh, origin, direction, start, end):
    """trimesh cylinders are centred on the origin along +Z; move one to span [start, end] along direction."""
    z = np.array([0.0, 0.0, 1.0])
    T = trimesh.geometry.align_vectors(z, direction)
    T[:3, 3] = origin + direction * (start + end) / 2
    mesh.apply_transform(T)
    return mesh


class CollisionChecker:
    def __init__(self, machine: Machine, kin: Kinematics, setup: JobSetup | None = None,
                 extra_ignore: set[frozenset] | None = None):
        import fcl  # noqa: F401  -- fail early with a clear message if python-fcl is missing

        self.m, self.kin, self.setup = machine, kin, setup
        self.head = trimesh.collision.CollisionManager()
        self.table = trimesh.collision.CollisionManager()
        self.link_of: dict[str, str] = {}
        self.head_names: set[str] = set()
        for part in machine.active_parts():
            if not part.collide:
                continue
            mesh = load_part_mesh(part.id)
            mgr = self.head if part.link in HEAD_LINKS else self.table
            mgr.add_object(part.id, mesh)
            self.link_of[part.id] = part.link
            if mgr is self.head:
                self.head_names.add(part.id)
        if setup is not None:
            solids = ([setup.stock] if setup.stock else []) + setup.fixtures
            for i, solid in enumerate(solids):
                name = "stock" if i == 0 and setup.stock else f"fixture:{solid.name}"
                self.table.add_object(name, stock_mesh(solid))
                self.link_of[name] = machine.raw["table"]["link"]
        self.current_tool: int | None = None
        # Adjacent pairs: in contact at the CAD pose (no tool loaded).
        self._set_pose(kin.q0)
        _, pairs = self.head.in_collision_other(self.table, return_names=True)
        self.ignore = {frozenset(p) for p in pairs} | (extra_ignore or set())

    def _set_tool(self, number: int):
        if number == self.current_tool:
            return
        for name in ("holder", "cutter"):
            if name in self.link_of:
                self.head.remove_object(name)
                del self.link_of[name]
        self.current_tool = number
        if number and self.setup is not None:
            for name, mesh in tool_meshes(self.kin, self.setup.tool(number)).items():
                self.head.add_object(name, mesh)
                self.head_names.add(name)
                self.link_of[name] = self.m.raw["spindle"]["link"]

    def _set_pose(self, q):
        T = self.kin.link_transforms(q)
        for name, link in self.link_of.items():
            mgr = self.head if name in self.head_names else self.table
            mgr.set_transform(name, T[link])

    def check_pose(self, q, tool: int = 0, rapid: bool = False) -> list[tuple[str, str, str]]:
        """Colliding (a, b, kind) pairs at joint vector q."""
        self._set_tool(tool)
        self._set_pose(q)
        hit, pairs = self.head.in_collision_other(self.table, return_names=True)
        if not hit:
            return []
        out = []
        for a, b in pairs:
            if frozenset((a, b)) in self.ignore:
                continue
            if {a, b} == {"cutter", "stock"}:
                if rapid:
                    out.append((a, b, "rapid_into_stock"))
                continue
            out.append((a, b, "collision"))
        return out

    def check_trajectory(self, traj: Trajectory, linear_step: float = 2.0, rotary_step: float = 1.0,
                         max_reports: int = 50) -> list[Collision]:
        """Densify each segment to <= linear_step mm / rotary_step deg and test every pose.
        Reports the first contact of each (pair, line) so one crash doesn't flood the list."""
        t, q, _ = traj.arrays()
        seen, out = set(), []
        cutter_in_stock = False
        for k in range(1, len(t)):
            dq = q[k] - q[k - 1]
            n = int(max(1, np.ceil(max(np.max(np.abs(dq[:3])) / linear_step,
                                       np.max(np.abs(dq[3:])) / rotary_step))))
            rapid = traj.motion[k] in (MOTION["rapid"], MOTION["home"], MOTION["toolchange"])
            for f in np.linspace(1.0 / n, 1.0, n):
                qs = q[k - 1] + dq * f
                hits = self.check_pose(qs, traj.tool[k], rapid=True)
                in_stock = any(kind == "rapid_into_stock" for *_, kind in hits)
                entering = in_stock and not cutter_in_stock
                cutter_in_stock = in_stock
                for a, b, kind in hits:
                    if kind == "rapid_into_stock" and not (rapid and entering):
                        continue
                    key = (a, b, traj.line[k])
                    if key in seen:
                        continue
                    seen.add(key)
                    out.append(Collision(t[k - 1] + (t[k] - t[k - 1]) * f, traj.line[k], a, b, qs.tolist(), kind))
                    if len(out) >= max_reports:
                        return out
        return out


def load_manifest() -> dict:
    return json.loads((ASSETS / "machine.json").read_text())
