"""High-level entry point: G-code in, checked trajectory + report out (JSON-ready)."""
from __future__ import annotations

from dataclasses import asdict

import numpy as np

from .config import JOINT_ORDER, Machine, load_machine
from .gcode import MOTION, GCodeError, Trajectory, simulate
from .job import JobSetup, default_setup
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


def run_job(gcode: str, machine: Machine | None = None, setup: JobSetup | None = None,
            check_collisions: bool = True) -> dict:
    machine = machine or load_machine()
    kin = Kinematics(machine)
    setup = setup or default_setup(kin)
    result: dict = {"version": 1, "joints": list(JOINT_ORDER), "options": machine.options}
    try:
        traj = simulate(gcode, kin, setup)
    except GCodeError as e:
        result["error"] = str(e)
        return result

    collisions = []
    if check_collisions:
        from .collision import CollisionChecker
        collisions = [c.as_dict() for c in CollisionChecker(machine, kin, setup).check_trajectory(traj)]

    r3 = lambda a: np.round(np.asarray(a, dtype=float), 3).tolist()  # noqa: E731
    result.update({
        "summary": summarise(traj),
        "t": r3(traj.t),
        "q": r3(traj.q),
        "tip": r3(traj.tip),
        "line": traj.line,
        "motion": traj.motion,
        "motion_codes": MOTION,
        "tool": traj.tool,
        "spindle": r3(traj.spindle),
        "events": traj.events,
        "warnings": traj.warnings,
        "limits": limit_violations(traj, kin),
        "collisions": collisions,
        "collision_checked": check_collisions,
        "setup": {
            "tools": {n: asdict(t) for n, t in setup.tools.items()},
            "default_tool": asdict(setup.default_tool),
            "stock": asdict(setup.stock) if setup.stock else None,
            "fixtures": [asdict(f) for f in setup.fixtures],
            "work_offsets": {k: r3(v) for k, v in setup.work_offsets.items()},
        },
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
