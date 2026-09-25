"""Material removal: a voxel model of the stock, cut by the tool along the trajectory.

The stock is a grid of voxels in the table frame (so it tilts and turns with the part). Each
move is swept in small steps; at every step the voxels inside the tool's *flutes* are removed
and stamped with the time they were cut. From that we get:

- the part as machined (final stock mesh, and the stock at any moment for the viewer);
- rapid-into-material: a rapid whose tool touches material that is still there;
- shank / holder contact: the non-cutting part of the tool touching material on a feed move;
- cutting with the spindle stopped;
- gouges: material removed that belongs to the finished part (when a part model is given);
- removed volume per trajectory sample -> material removal rate -> spindle load (physics.py).

Resolution trades accuracy for speed; features smaller than about one voxel are not resolved.
"""
from __future__ import annotations

import base64
import math
from dataclasses import dataclass, field

import numpy as np

from .gcode import MOTION, Trajectory
from .job import JobSetup, Solid, Tool
from .kinematics import Kinematics

RAPIDS = (MOTION["rapid"], MOTION["home"], MOTION["toolchange"])
BODY_STEP = 2.0  # mm between shank/holder contact checks
MAX_VOXELS = 12_000_000


@dataclass
class MaterialIssue:
    t: float
    line: int
    kind: str       # rapid_into_material | holder_contact | shank_contact | spindle_off_cut | gouge
    detail: str
    volume: float = 0.0

    def as_dict(self):
        return {"t": round(self.t, 3), "line": self.line, "kind": self.kind, "detail": self.detail,
                "volume_mm3": round(self.volume, 2)}


@dataclass
class MaterialResult:
    grid: "VoxelGrid"
    removed_per_sample: np.ndarray             # mm^3 removed during the motion into sample k
    issues: list[MaterialIssue] = field(default_factory=list)
    target: np.ndarray | None = None           # finished-part occupancy, when a part model is given
    cut_times: np.ndarray | None = None        # time of every cutting sub-step ...
    cut_volumes: np.ndarray | None = None      # ... and the volume it removed (mm^3)
    cut_spans: np.ndarray | None = None        # ... over this many seconds before cut_times

    def summary(self) -> dict:
        g = self.grid
        v = g.res ** 3
        return {
            "resolution_mm": g.res,
            "stock_volume_mm3": round(float(g.initial.sum()) * v, 1),
            "removed_volume_mm3": round(float(self.removed_per_sample.sum()), 1),
            "final_volume_mm3": round(float(g.material.sum()) * v, 1),
        }


class VoxelGrid:
    def __init__(self, solid: Solid, resolution: float | None = None):
        lo, hi = solid.mesh.bounds
        size = hi - lo
        if resolution is None:  # ~160 voxels along the longest side, within [0.25, 2] mm
            resolution = float(np.clip(size.max() / 160.0, 0.25, 2.0))
        n = np.ceil(size / resolution).astype(int) + 2
        if int(np.prod(n)) > MAX_VOXELS:
            resolution *= (np.prod(n) / MAX_VOXELS) ** (1 / 3)
            n = np.ceil(size / resolution).astype(int) + 2
        self.res = float(resolution)
        self.shape = tuple(int(v) for v in n)
        self.origin = lo - self.res  # centre of voxel (0,0,0) sits half a voxel inside this corner
        self.axes = [(self.origin[i] + (np.arange(self.shape[i]) + 0.5) * self.res).astype(np.float32) for i in range(3)]
        self.initial = occupancy(solid, self)
        self.material = self.initial.copy()
        self.removed_at = np.full(self.shape, np.inf, dtype=np.float32)

    @property
    def lo(self):
        return self.origin

    @property
    def hi(self):
        return self.origin + np.array(self.shape) * self.res

    def index_box(self, lo, hi):
        """Slice indices of voxels whose centres may lie in [lo, hi] (clipped), or None."""
        i0 = np.floor((np.asarray(lo) - self.origin) / self.res - 0.5).astype(int)
        i1 = np.ceil((np.asarray(hi) - self.origin) / self.res + 0.5).astype(int)
        i0 = np.maximum(i0, 0)
        i1 = np.minimum(i1, self.shape)
        if np.any(i1 <= i0):
            return None
        return tuple(slice(a, b) for a, b in zip(i0, i1))


def occupancy(solid: Solid, grid: VoxelGrid) -> np.ndarray:
    """Voxel centres inside the solid."""
    X = grid.axes[0][:, None, None]
    Y = grid.axes[1][None, :, None]
    Z = grid.axes[2][None, None, :]
    if solid.type == "box":
        c = np.array(solid.position) + [0, 0, solid.size[2] / 2]
        h = np.array(solid.size) / 2
        return (np.abs(X - c[0]) <= h[0]) & (np.abs(Y - c[1]) <= h[1]) & (np.abs(Z - c[2]) <= h[2])
    if solid.type == "cylinder":
        p = solid.position
        return (((X - p[0]) ** 2 + (Y - p[1]) ** 2) <= (solid.size[0] / 2) ** 2) & (Z >= p[2]) & (Z <= p[2] + solid.size[1])
    return voxelize_mesh(solid.mesh, grid)


def voxelize_mesh(mesh, grid: VoxelGrid) -> np.ndarray:
    """Parity scanline voxelisation: for every (x, y) column of voxel centres, find where a vertical
    ray crosses the surface and fill between pairs of crossings. Needs a closed mesh."""
    xs, ys, zs = grid.axes
    hits: dict[tuple[int, int], list[float]] = {}
    tri = mesh.triangles
    for a, b, c in tri:
        x0, x1 = min(a[0], b[0], c[0]), max(a[0], b[0], c[0])
        y0, y1 = min(a[1], b[1], c[1]), max(a[1], b[1], c[1])
        i0, i1 = np.searchsorted(xs, [x0, x1])
        j0, j1 = np.searchsorted(ys, [y0, y1])
        if i0 >= i1 or j0 >= j1:
            continue
        det = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
        if abs(det) < 1e-12:
            continue  # vertical triangle: no vertical ray crosses its interior
        px, py = np.meshgrid(xs[i0:i1], ys[j0:j1], indexing="ij")
        l1 = ((b[1] - c[1]) * (px - c[0]) + (c[0] - b[0]) * (py - c[1])) / det
        l2 = ((c[1] - a[1]) * (px - c[0]) + (a[0] - c[0]) * (py - c[1])) / det
        l3 = 1 - l1 - l2
        inside = (l1 >= 0) & (l2 >= 0) & (l3 > 0) | (l1 > 0) & (l2 > 0) & (l3 >= 0)
        if not inside.any():
            continue
        z = l1 * a[2] + l2 * b[2] + l3 * c[2]
        for ii, jj in zip(*np.nonzero(inside)):
            hits.setdefault((i0 + ii, j0 + jj), []).append(float(z[ii, jj]))
    occ = np.zeros(grid.shape, dtype=bool)
    for (i, j), zl in hits.items():
        zl = sorted(zl)
        for k in range(0, len(zl) - 1, 2):
            k0, k1 = np.searchsorted(zs, [zl[k], zl[k + 1]])
            occ[i, j, k0:k1] = True
    return occ


class MaterialSim:
    def __init__(self, kin: Kinematics, setup: JobSetup, resolution: float | None = None,
                 gouge_tolerance: float = 0.1, step_fraction: float = 0.75):
        if setup.stock is None:
            raise ValueError("material simulation needs a stock in the job setup")
        self.kin, self.setup = kin, setup
        self.grid = VoxelGrid(setup.stock, resolution or setup.material.get("resolution"))
        self.step = self.grid.res * step_fraction
        self.tol = float(setup.material.get("gouge_tolerance", gouge_tolerance))
        self.target = occupancy(setup.part, self.grid) if setup.part is not None else None
        g = self.grid
        self.voxel_volume = g.res ** 3

    # ------------------------------------------------------------------ sweep
    def run(self, traj: Trajectory) -> MaterialResult:
        t = np.asarray(traj.t)
        q = np.asarray(traj.q)
        removed = np.zeros(len(t))
        issues: list[MaterialIssue] = []
        seen: set = set()

        def report(kind, k, tt, detail, vol=0.0):
            key = (kind, traj.line[k])
            if key not in seen:
                seen.add(key)
                issues.append(MaterialIssue(tt, traj.line[k], kind, detail, vol))

        g = self.grid
        cut_t: list[float] = []
        cut_v: list[float] = []
        cut_s: list[float] = []
        for k in range(1, len(t)):
            tool_no = traj.tool[k]
            if tool_no == 0:
                continue  # no tool in the spindle
            tool = self.setup.tool(tool_no)
            poses = self._substeps(q[k - 1], q[k], tool)
            if not poses:
                continue
            rapid = traj.motion[k] in RAPIDS
            spindle_off = abs(traj.spindle[k]) < 1e-9
            # shank/holder contact needs far coarser spacing than cutting: every ~BODY_STEP mm
            every = max(1, int(BODY_STEP / self.step))
            prev_f = 0.0
            for i, (f, tip, axis) in enumerate(poses):
                span = float((t[k] - t[k - 1]) * (f - prev_f))
                prev_f = f
                tt = float(t[k - 1] + (t[k] - t[k - 1]) * f)
                check_body = (i % every == every - 1) or i == len(poses) - 1
                cut_n, body_hit, gouge_n = self._apply(tip, axis, tool, tt, check_body)
                if cut_n:
                    vol = cut_n * self.voxel_volume
                    removed[k] += vol
                    cut_t.append(tt)
                    cut_v.append(vol)
                    cut_s.append(span)
                    if rapid:
                        report("rapid_into_material", k, tt, f"T{tool_no} rapid through material", vol)
                    elif spindle_off:
                        report("spindle_off_cut", k, tt, f"T{tool_no} cutting with the spindle stopped", vol)
                if body_hit:
                    kind = "holder_contact" if body_hit == "holder" else "shank_contact"
                    report(kind, k, tt, f"T{tool_no} {body_hit} touches the material")
                if gouge_n:
                    report("gouge", k, tt, f"T{tool_no} cut into the finished part",
                           gouge_n * self.voxel_volume)
        return MaterialResult(g, removed, sorted(issues, key=lambda i: i.t), self.target,
                              np.array(cut_t), np.array(cut_v), np.array(cut_s))

    def _substeps(self, q0, q1, tool: Tool):
        """(fraction, tip, axis) poses along a segment, spaced <= self.step at the tip.
        Tips/axes are in the table frame; axis points from the tip up the tool."""
        kin = self.kin
        g = self.grid
        tip0 = kin.tool_tip_in_table(q0, tool.length)
        tip1 = kin.tool_tip_in_table(q1, tool.length)
        # quick reject: the whole tool stays clear of the stock's bounding box
        # (only valid when the rotaries don't move -- otherwise the tip path is curved)
        rot = float(np.max(np.abs(q1[3:] - q0[3:])))
        reach = tool.length + tool.holder_diameter
        if rot < 1e-9 and not self._segment_near_box(tip0, tip1, g.lo - reach, g.hi + reach):
            return []
        rot = float(np.max(np.abs(q1[3:] - q0[3:])))
        dist = float(np.linalg.norm(tip1 - tip0))
        if dist < 1e-9 and rot < 1e-9:
            return []
        if rot < 1e-9:
            axis = -kin.tool_axis_in_table(q0)
            n = max(1, math.ceil(dist / self.step))
            return [(i / n, tip0 + (tip1 - tip0) * (i / n), axis) for i in range(1, n + 1)]
        # rotaries move: the tip path is curved in the table frame -- evaluate the kinematics
        def poses(n):
            f = np.arange(1, n + 1) / n
            tips, axes = kin.tip_and_axis_in_table_batch(q0 + np.outer(f, q1 - q0), tool.length)
            return f, tips, axes

        n = max(1, math.ceil(max(dist / self.step, rot / 0.5)))
        f, tips, axes = poses(n)
        longest = float(np.max(np.linalg.norm(np.diff(np.vstack([tip0, tips]), axis=0), axis=1)))
        if longest > self.step * 1.5 and n < 20000:  # curved path longer than the chord: refine
            f, tips, axes = poses(min(20000, math.ceil(n * longest / self.step)))
        # a whole sweep that stays clear of the stock box does nothing
        if not self._segment_near_box(tips.min(0), tips.max(0), g.lo - reach, g.hi + reach):
            return []
        return list(zip(f, tips, axes))

    @staticmethod
    def _segment_near_box(a, b, lo, hi) -> bool:
        seg_lo, seg_hi = np.minimum(a, b), np.maximum(a, b)
        return bool(np.all(seg_hi >= lo) and np.all(seg_lo <= hi))

    def _apply(self, tip, axis, tool: Tool, t: float, check_body: bool = True):
        """Cut at one tool pose. Returns (voxels removed, body part touching material or None, gouge voxels)."""
        g = self.grid
        body_hit = None
        # 1) shank + holder: contact check only (these don't cut)
        top = tip + axis * tool.length
        start = tip + axis * tool.flute_length
        rmax = max(tool.shank_diameter, tool.holder_diameter) / 2
        box = g.index_box(np.minimum(start, top) - rmax, np.maximum(start, top) + rmax) if check_body else None
        if box is not None and g.material[box].any():
            h, rho = self._local(box, tip, axis)
            cand = g.material[box] & (h > tool.flute_length) & (h <= tool.length) & (rho <= rmax)
            if cand.any():
                hh = h[cand]
                touching = rho[cand] <= tool.body_radius(hh)
                if touching.any():
                    body_hit = "holder" if np.any(hh[touching] > tool.stick_out) else "shank"
        # 2) flutes: remove material
        fl_top = tip + axis * tool.flute_length
        r = tool.radius
        box = g.index_box(np.minimum(tip, fl_top) - r, np.maximum(tip, fl_top) + r)
        if box is None:
            return 0, body_hit, 0
        mat = g.material[box]
        if not mat.any():
            return 0, body_hit, 0
        h, rho = self._local(box, tip, axis)
        cut = (h >= 0) & (h <= tool.flute_length) & (rho <= tool.flute_radius(h)) & mat
        n = int(cut.sum())
        gouge_n = 0
        if n:
            if self.target is not None:
                shrunk = (h >= self.tol) & (h <= tool.flute_length) & (rho <= tool.flute_radius(h) - self.tol)
                gouge_n = int((cut & shrunk & self.target[box]).sum())
            g.material[box] &= ~cut
            ra = g.removed_at[box]
            ra[cut] = t
            g.removed_at[box] = ra
        return n, body_hit, gouge_n

    def _local(self, box, tip, axis):
        """Height along the tool axis above the tip, and radial distance, for voxel centres in box."""
        g = self.grid
        tip = tip.astype(np.float32)
        axis = axis.astype(np.float32)
        rx = g.axes[0][box[0]][:, None, None] - tip[0]
        ry = g.axes[1][box[1]][None, :, None] - tip[1]
        rz = g.axes[2][box[2]][None, None, :] - tip[2]
        h = rx * axis[0] + ry * axis[1] + rz * axis[2]
        rho2 = rx * rx + ry * ry + rz * rz - h * h
        return h, np.sqrt(np.maximum(rho2, 0.0))


# ---------------------------------------------------------------------- output
def surface_mesh(occ: np.ndarray, origin, res: float):
    """Blocky surface of an occupancy grid: one quad per exposed voxel face (as a trimesh)."""
    import trimesh

    padded = np.pad(occ, 1)
    verts, faces = [], []
    corners = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0], [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]])
    # (neighbour offset, 4 corner ids in CCW order seen from outside)
    dirs = [((1, 0, 0), (1, 2, 6, 5)), ((-1, 0, 0), (0, 4, 7, 3)), ((0, 1, 0), (2, 3, 7, 6)),
            ((0, -1, 0), (0, 1, 5, 4)), ((0, 0, 1), (4, 5, 6, 7)), ((0, 0, -1), (0, 3, 2, 1))]
    core = padded[1:-1, 1:-1, 1:-1]
    for (dx, dy, dz), quad in dirs:
        nb = padded[1 + dx:padded.shape[0] - 1 + dx, 1 + dy:padded.shape[1] - 1 + dy, 1 + dz:padded.shape[2] - 1 + dz]
        idx = np.argwhere(core & ~nb)
        if len(idx) == 0:
            continue
        base = sum(len(v) for v in verts)
        v = (idx[:, None, :] + corners[list(quad)][None, :, :]).reshape(-1, 3)
        verts.append(v)
        m = np.arange(len(idx))[:, None] * 4 + base
        faces.append(np.hstack([m, m + 1, m + 2]))
        faces.append(np.hstack([m, m + 2, m + 3]))
    if not verts:
        return trimesh.Trimesh()
    V = np.vstack(verts) * res + np.asarray(origin)
    return trimesh.Trimesh(vertices=V, faces=np.vstack(faces), process=True)


def viewer_payload(result: MaterialResult) -> dict:
    """Grid + removal times, compact, so the viewer can show the stock at any moment."""
    g = result.grid
    flat_init = g.initial.ravel(order="C")
    removed_idx = np.nonzero(np.isfinite(g.removed_at.ravel(order="C")))[0].astype("<u4")
    removed_t = g.removed_at.ravel(order="C")[removed_idx].astype("<f4")

    def b64(a):
        return base64.b64encode(np.ascontiguousarray(a).tobytes()).decode()

    return {
        "shape": list(g.shape), "origin": [round(float(v), 4) for v in g.origin], "res": g.res,
        "initial_bits": b64(np.packbits(flat_init)),
        "removed_index": b64(removed_idx), "removed_t": b64(removed_t),
        "target_bits": b64(np.packbits(result.target.ravel(order="C"))) if result.target is not None else None,
    }


def simulate_material(traj: Trajectory, kin: Kinematics, setup: JobSetup, **kw) -> MaterialResult:
    return MaterialSim(kin, setup, **kw).run(traj)
