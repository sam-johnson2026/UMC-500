"""Commanded vs actual: line a recorded run up against the simulation of the same program.

    python -m umc_twin compare PROGRAM --setup JOB --recording run.jsonl

A recording is JSON lines of MachineState (from `umc-twin record` or `serve --log`). States
are matched to the program by the line number the control reports, so the comparison doesn't
depend on the two clocks agreeing:

- time      actual vs simulated time spent on each line, and in total; the split between
            cutting and rapid lines hints whether feeds or rapids/accelerations need tuning;
- position  distance from each actual position to the simulated path of the same line --
            large values point at a wrong work offset, tool length or calibration;
- load      actual spindle load vs the predicted load on the same lines, with the factor that
            would make the prediction match (apply it to material.specific_energy).

Assumes the control reports *program line numbers* (as the viewer shows them). If it reports
N-numbers instead, lines won't match and the report says so.
"""
from __future__ import annotations

import json
from collections import defaultdict
from pathlib import Path

import numpy as np

from .gcode import MOTION, Trajectory

CUTTING = (MOTION["feed"], MOTION["arc"])


def load_recording(path: str | Path) -> list[dict]:
    states = [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]
    return sorted(states, key=lambda s: s["timestamp"])


def _active(states: list[dict]) -> list[dict]:
    """The part of the recording where a program was running."""
    idx = [i for i, s in enumerate(states)
           if s.get("line") is not None and (s.get("execution") in (None, "ACTIVE") or s.get("mode") == "SIM")]
    return states[idx[0]:idx[-1] + 1] if idx else []


def _sim_line_stats(traj: Trajectory):
    t = np.asarray(traj.t)
    dt = np.diff(t, prepend=t[0] if len(t) else 0.0)
    time = defaultdict(float)
    kind = defaultdict(lambda: "other")
    for k in range(1, len(t)):
        ln = traj.line[k]
        time[ln] += dt[k]
        if traj.motion[k] in CUTTING:
            kind[ln] = "cutting"
        elif kind[ln] == "other" and traj.motion[k] in (MOTION["rapid"], MOTION["home"]):
            kind[ln] = "rapid"
    return time, kind


def _dist_to_polyline(p: np.ndarray, pts: np.ndarray) -> float:
    if len(pts) == 1:
        return float(np.linalg.norm(p - pts[0]))
    a, b = pts[:-1], pts[1:]
    ab = b - a
    denom = np.maximum((ab * ab).sum(1), 1e-12)
    f = np.clip(((p - a) * ab).sum(1) / denom, 0, 1)
    return float(np.min(np.linalg.norm(a + ab * f[:, None] - p, axis=1)))


def compare(traj: Trajectory, states: list[dict], predicted_load: dict | None = None) -> dict:
    run = _active(states)
    if len(run) < 2:
        return {"error": "the recording has no running program (no states with a line number)"}
    ts = np.array([s["timestamp"] for s in run])
    actual_total = float(ts[-1] - ts[0])
    sim_time, sim_kind = _sim_line_stats(traj)
    sim_total = float(sum(sim_time.values()))

    # --- time per line (each state holds until the next one) -------------------------
    act_time = defaultdict(float)
    for s, dt in zip(run[:-1], np.diff(ts)):
        act_time[s["line"]] += float(dt)
    lines = sorted(set(act_time) | {ln for ln in sim_time if sim_time[ln] > 0})
    matched = [ln for ln in act_time if ln in sim_time]
    if not matched:
        return {"error": "no line numbers in the recording match the program -- does the control report "
                         "N-numbers instead of program lines?"}

    def ratio(kind):
        a = sum(act_time[ln] for ln in matched if sim_kind[ln] == kind)
        s = sum(sim_time[ln] for ln in matched if sim_kind[ln] == kind)
        return round(a / s, 3) if s > 0.05 else None

    per_line = []
    for ln in lines:
        a, s = act_time.get(ln, 0.0), sim_time.get(ln, 0.0)
        per_line.append({"line": ln, "actual_s": round(a, 3), "sim_s": round(s, 3), "diff_s": round(a - s, 3),
                         "kind": sim_kind[ln]})

    # --- position: distance to the simulated path of the same line -------------------
    q = np.asarray(traj.q)
    by_line = defaultdict(list)
    for k in range(1, len(q)):
        by_line[traj.line[k]].append(k)
    dev_lin = defaultdict(list)
    dev_rot = defaultdict(list)
    for s in run:
        ks = by_line.get(s["line"])
        if not ks:
            continue
        ks = [ks[0] - 1] + ks          # include where the line starts
        pts = q[ks]
        p = np.asarray(s["q"], dtype=float)
        dev_lin[s["line"]].append(_dist_to_polyline(p[:3], pts[:, :3]))
        dev_rot[s["line"]].append(_dist_to_polyline(p[3:], pts[:, 3:]))
    for row in per_line:
        ln = row["line"]
        if dev_lin.get(ln):
            row["max_dev_mm"] = round(max(dev_lin[ln]), 4)
            row["max_dev_deg"] = round(max(dev_rot[ln]), 4)
    all_lin = [v for vs in dev_lin.values() for v in vs]
    all_rot = [v for vs in dev_rot.values() for v in vs]
    worst = max(dev_lin, key=lambda ln: max(dev_lin[ln])) if dev_lin else None

    # --- spindle load -------------------------------------------------------------------
    load = None
    act_load = defaultdict(list)
    for s in run:
        if s.get("spindle_load") is not None:
            act_load[s["line"]].append(float(s["spindle_load"]))
    if act_load and predicted_load:
        pred_line = _predicted_load_by_line(traj, predicted_load)
        pairs = [(float(np.mean(act_load[ln])), pred_line[ln]) for ln in act_load
                 if ln in pred_line and pred_line[ln] > 0.5]
        if pairs:
            a, p = np.array(pairs).T
            factor = float((a @ p) / (p @ p))           # least-squares scale: actual ~ factor * predicted
            load = {"lines_compared": len(pairs), "actual_mean_pct": round(float(a.mean()), 1),
                    "predicted_mean_pct": round(float(p.mean()), 1), "scale_factor": round(factor, 3),
                    "hint": f"multiply material.specific_energy by {factor:.2f} to match this machine"}
        for row in per_line:
            ln = row["line"]
            if ln in act_load:
                row["actual_load_pct"] = round(float(np.mean(act_load[ln])), 1)
                row["predicted_load_pct"] = round(pred_line.get(ln, 0.0), 1)

    per_line.sort(key=lambda r: -abs(r["diff_s"]))
    return {
        "cycle_time": {"actual_s": round(actual_total, 2), "sim_s": round(sim_total, 2),
                       "ratio": round(actual_total / sim_total, 3) if sim_total else None,
                       "cutting_ratio": ratio("cutting"), "rapid_ratio": ratio("rapid")},
        "position": {"states": len(all_lin),
                     "max_dev_mm": round(max(all_lin), 4) if all_lin else None,
                     "mean_dev_mm": round(float(np.mean(all_lin)), 4) if all_lin else None,
                     "max_dev_deg": round(max(all_rot), 4) if all_rot else None,
                     "worst_line": worst},
        "spindle_load": load,
        "lines_in_recording": len(act_time), "lines_matched": len(matched),
        "per_line": per_line,
    }


def _predicted_load_by_line(traj: Trajectory, predicted: dict) -> dict[int, float]:
    t = np.asarray(traj.t)
    b = predicted["bin_s"]
    vals = np.asarray(predicted["load_pct"])
    centres = (np.arange(len(vals)) + 0.5) * b
    k = np.clip(np.searchsorted(t, centres), 1, len(t) - 1)
    acc = defaultdict(list)
    for kk, v in zip(k, vals):
        acc[traj.line[kk]].append(v)
    return {ln: float(np.mean(v)) for ln, v in acc.items()}


def format_report(r: dict, top: int = 10) -> str:
    if "error" in r:
        return f"ERROR: {r['error']}"
    c, p = r["cycle_time"], r["position"]
    lines = [f"cycle time   actual {c['actual_s']:.1f} s vs sim {c['sim_s']:.1f} s (x{c['ratio']}); "
             f"cutting lines x{c['cutting_ratio']}, rapid lines x{c['rapid_ratio']}",
             f"position     max {p['max_dev_mm']} mm / {p['max_dev_deg']} deg from the simulated path "
             f"(mean {p['mean_dev_mm']} mm; worst on line {p['worst_line']})",
             f"lines        {r['lines_matched']} of {r['lines_in_recording']} recorded lines match the program"]
    if r["spindle_load"]:
        sl = r["spindle_load"]
        lines.append(f"spindle load actual {sl['actual_mean_pct']}% vs predicted {sl['predicted_mean_pct']}% "
                     f"on {sl['lines_compared']} lines -> {sl['hint']}")
    lines.append(f"biggest time differences (top {top}):")
    for row in r["per_line"][:top]:
        extra = f"  dev {row['max_dev_mm']} mm" if "max_dev_mm" in row else ""
        lines.append(f"  line {row['line']:5d} {row['kind']:8s} actual {row['actual_s']:7.2f} s  "
                     f"sim {row['sim_s']:7.2f} s  ({row['diff_s']:+.2f}){extra}")
    return "\n".join(lines)


def synthetic_recording(traj: Trajectory, dt: float = 0.1, time_scale: float = 1.0,
                        offset=(0, 0, 0, 0, 0), load: dict | None = None, load_scale: float = 1.0,
                        t0: float = 1.8e9) -> list[dict]:
    """A recording as a machine *like* the simulation would produce it -- slower by `time_scale`,
    shifted by `offset`, with spindle load = predicted x `load_scale`. For tests and demos."""
    t = np.asarray(traj.t)
    q = np.asarray(traj.q)
    out = []
    for tt in np.arange(0.0, t[-1] + 1e-9, dt):
        k = int(np.clip(np.searchsorted(t, tt, side="right"), 1, len(t) - 1))
        f = 0.0 if t[k] == t[k - 1] else (tt - t[k - 1]) / (t[k] - t[k - 1])
        pos = q[k - 1] + (q[k] - q[k - 1]) * f + np.asarray(offset, dtype=float)
        state = {"q": pos.round(4).tolist(), "source": "synthetic", "timestamp": t0 + tt * time_scale,
                 "line": traj.line[k], "tool": traj.tool[k], "spindle_rpm": traj.spindle[k],
                 "execution": "ACTIVE", "mode": "AUTOMATIC", "program": "O00000"}
        if load:
            i = min(int(tt / load["bin_s"]), len(load["load_pct"]) - 1)
            state["spindle_load"] = round(load["load_pct"][i] * load_scale, 2)
        out.append(state)
    return out
