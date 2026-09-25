"""High-level entry point: G-code in, checked trajectory + report out (JSON-ready)."""
from __future__ import annotations

import base64
from dataclasses import asdict
from pathlib import Path

import numpy as np

from .config import JOINT_ORDER, Machine, load_machine
from .gcode import MOTION, GCodeError, Trajectory, simulate
from .job import JobSetup, Solid, Tool, default_setup
from .kinematics import Kinematics


def limit_violations(traj: Trajectory, kin: Kinematics) -> list[dict]:
    """First travel-limit violation per (line, axis). Segments are linear in joint space
    between samples, so checking the samples is exact."""
    out, seen = [], set()
    for k, q in enumerate(traj.q):
        for msg in kin.limit_violations(q):
            key = (traj.line[k], msg.split("=")[0])
            if key not in seen:
                seen.add(key)
                out.append({"t": round(traj.t[k], 3), "line": traj.line[k], "message": msg})
    return out


def summarise(traj: Trajectory) -> dict:
    t = np.array(traj.t)
    dt = np.diff(t, prepend=0.0)
    motion = np.array(traj.motion)
    ideal = traj.t_ideal[-1] if traj.t_ideal else (float(t[-1]) if len(t) else 0.0)
    return {
        "duration_s": round(float(t[-1]) if len(t) else 0.0, 2),
        "ideal_duration_s": round(float(ideal), 2),
        "rapid_s": round(float(dt[(motion == MOTION["rapid"]) | (motion == MOTION["home"])].sum()), 2),
        "cutting_s": round(float(dt[(motion == MOTION["feed"]) | (motion == MOTION["arc"])].sum()), 2),
        "toolchange_s": round(float(dt[motion == MOTION["toolchange"]].sum()), 2),
        "dwell_s": round(float(dt[motion == MOTION["dwell"]].sum()), 2),
        "tool_changes": sum(1 for e in traj.events if e["type"] == "toolchange"),
        "samples": len(t),
    }


def tool_payload(tool: Tool) -> dict:
    d = asdict(tool)
    d["profile"] = tool.profile()
    d["holder_profile"] = tool.holder_profile()
    return d


def mesh_payload(mesh) -> dict:
    """Compact mesh for the viewer: base64 float32 vertices / uint32 faces."""
    return {"v": base64.b64encode(np.asarray(mesh.vertices, dtype="<f4").tobytes()).decode(),
            "f": base64.b64encode(np.asarray(mesh.faces, dtype="<u4").tobytes()).decode()}


def solid_payload(solid: Solid | None) -> dict | None:
    if solid is None:
        return None
    d = asdict(solid)
    if solid.type not in ("box", "cylinder"):
        d["mesh"] = mesh_payload(solid.mesh)
        if solid.file:
            d["file"] = Path(solid.file).name
    return d


def viewer_indices(traj, max_samples: int = 30000) -> list[int]:
    """Samples to send to the viewer. Long CAM programs have far more samples than a picture
    needs; keep every change of motion type / tool / spindle and otherwise one sample per
    `tol` mm of tool-tip travel (or 0.5 deg of rotary), with `tol` grown until it fits.
    The analysis (times, cutting, collisions) always uses every sample."""
    n = len(traj.t)
    if n <= max_samples:
        return list(range(n))
    tip = np.asarray(traj.tip)
    q = np.asarray(traj.q)
    tol = 0.05
    while True:
        keep = [0]
        last = 0
        for k in range(1, n):
            if (traj.motion[k] != traj.motion[last] or traj.tool[k] != traj.tool[last]
                    or traj.spindle[k] != traj.spindle[last] or k == n - 1
                    or np.linalg.norm(tip[k] - tip[last]) >= tol or np.max(np.abs(q[k, 3:] - q[last, 3:])) >= 0.5):
                keep.append(k)
                last = k
        if len(keep) <= max_samples or tol > 50:
            return keep
        tol *= 2


def _bucket_starts(keep: list[int]) -> list[int]:
    """reduceat starts so that bucket i sums the samples (keep[i-1], keep[i]]."""
    return [0] + [k + 1 for k in keep[:-1]] if len(keep) > 1 else [0]


def run_job(gcode: str, machine: Machine | None = None, setup: JobSetup | None = None,
            check_collisions: bool = True, material: bool | None = None,
            stock_out: str | Path | None = None, search_paths: list | None = None,
            max_samples: int = 30000) -> dict:
    """Simulate a program. material=None runs the cutting sim whenever the setup has a stock
    (unless the setup says `material: {enabled: false}`)."""
    machine = machine or load_machine()
    kin = Kinematics(machine)
    setup = setup or default_setup(kin)
    result: dict = {"version": 1, "joints": list(JOINT_ORDER), "options": machine.options}
    try:
        traj = simulate(gcode, kin, setup, search_paths=search_paths)
    except GCodeError as e:
        result["error"] = str(e)
        return result

    if material is None:
        material = setup.stock is not None and setup.material.get("enabled", True)
    mat = load = None
    if material and setup.stock is not None:
        from .material import simulate_material, surface_mesh, viewer_payload

        mat = simulate_material(traj, kin, setup)
        from .physics import spindle_load

        load = spindle_load(traj, mat, setup, machine.raw["spindle"])
        if stock_out:
            surface_mesh(mat.grid.material, mat.grid.origin, mat.grid.res).export(str(stock_out))

    collisions = []
    if check_collisions:
        from .collision import CollisionChecker

        # with the cutting sim, tool-vs-stock is judged against the material actually left
        extra = {frozenset((t, "stock")) for t in ("cutter", "holder")} if mat else set()
        collisions = [c.as_dict() for c in CollisionChecker(machine, kin, setup, extra_ignore=extra).check_trajectory(traj)]

    r3 = lambda a: np.round(np.asarray(a, dtype=float), 3).tolist()  # noqa: E731
    keep = viewer_indices(traj, max_samples)
    pick = lambda a: [a[i] for i in keep]  # noqa: E731
    result.update({
        "summary": summarise(traj),
        "samples_total": len(traj.t),
        "t": r3(pick(traj.t)),
        "q": r3(pick(traj.q)),
        "tip": r3(pick(traj.tip)),
        "line": pick(traj.line),
        "motion": pick(traj.motion),
        "motion_codes": MOTION,
        "tool": pick(traj.tool),
        "spindle": r3(pick(traj.spindle)),
        "events": traj.events,
        "warnings": traj.warnings,
        "limits": limit_violations(traj, kin),
        "collisions": collisions,
        "collision_checked": check_collisions,
        "setup": {
            "tools": {n: tool_payload(t) for n, t in setup.tools.items()},
            "default_tool": tool_payload(setup.default_tool),
            "stock": solid_payload(setup.stock),
            "fixtures": [solid_payload(f) for f in setup.fixtures],
            "part": solid_payload(setup.part),
            "work_offsets": {k: r3(v) for k, v in setup.work_offsets.items()},
        },
        "material": None if mat is None else {
            **mat.summary(),
            "issues": [i.as_dict() for i in mat.issues],
            "removed_per_sample": r3(np.add.reduceat(mat.removed_per_sample, _bucket_starts(keep))),
            "grid": viewer_payload(mat),
        },
        "spindle_load": load,
        "program": gcode.splitlines(),
    })
    return result


def format_report(result: dict) -> str:
    if "error" in result:
        return f"ERROR: {result['error']}"
    s = result["summary"]
    lines = [
        f"cycle time   {s['duration_s']:.1f} s  (cutting {s['cutting_s']:.1f}, rapid {s['rapid_s']:.1f}, "
        f"tool change {s['toolchange_s']:.1f}, dwell {s['dwell_s']:.1f}); "
        f"{s['ideal_duration_s']:.1f} s without acceleration",
        f"tool changes {s['tool_changes']}",
    ]
    m = result.get("material")
    if m:
        lines.append(f"material     {m['removed_volume_mm3'] / 1000:.1f} cm3 removed of {m['stock_volume_mm3'] / 1000:.1f} "
                     f"(voxel {m['resolution_mm']:.2f} mm); {len(m['issues']) or 'no'} cutting issues")
        for it in m["issues"][:20]:
            lines.append(f"  line {it['line']:5d}  t={it['t']:8.2f}s  {it['kind']}: {it['detail']}")
    sl = result.get("spindle_load")
    if sl:
        p = sl["peak"]
        lines.append(f"spindle      peak {p['load_pct']:.0f}% load, {p['power_kw']:.2f} kW, {p['torque_nm']:.1f} Nm, "
                     f"MRR {p['mrr_cm3_min']:.1f} cm3/min ({sl['material']}, estimate)")
        for it in sl["issues"]:
            lines.append(f"  line {it['line']:5d}  t={it['t']:8.2f}s  {it['kind']}: {it['detail']}")
    for key, title in (("limits", "travel limits"), ("collisions", "collisions"), ("warnings", "warnings")):
        items = result[key]
        if key == "collisions" and not result["collision_checked"]:
            lines.append("collisions   not checked")
            continue
        lines.append(f"{title:12s} {len(items) or 'none'}")
        for it in items[:20]:
            if key == "collisions":
                what = f"{it['a']} x {it['b']}" + (" (rapid into stock)" if it["kind"] == "rapid_into_stock" else "")
                lines.append(f"  line {it['line']:5d}  t={it['t']:8.2f}s  {what}")
            else:
                lines.append(f"  line {it['line']:5d}  {it['message']}")
        if len(items) > 20:
            lines.append(f"  ... {len(items) - 20} more")
    return "\n".join(lines)
