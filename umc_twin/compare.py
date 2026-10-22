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


def load_recording(path: str | Path, units: str = "auto") -> list[dict]:
    """JSON lines from `record` / `serve --log`, or a CSV log (see importers.py)."""
    if str(path).lower().endswith(".csv"):
        from .importers import read_csv_log

        return read_csv_log(path, units)[0]
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
    if not any(s.get("line") is not None for s in states):
        return compare_by_path(traj, states, predicted_load)
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


def _densify(traj: Trajectory, step: float = 0.5):
    """Simulated machine path as points every ~step mm / deg, with the sim time of each point."""
    t = np.asarray(traj.t)
    q = np.asarray(traj.q)
    pts, times = [q[:1]], [t[:1]]
    for k in range(1, len(t)):
        d = q[k] - q[k - 1]
        n = int(max(1, np.ceil(np.max(np.abs(d)) / step)))
        f = np.arange(1, n + 1) / n
        pts.append(q[k - 1] + np.outer(f, d))
        times.append(t[k - 1] + (t[k] - t[k - 1]) * f)
    return np.vstack(pts), np.concatenate(times)


def _main_run(states: list[dict]) -> tuple[list[dict], list[dict]]:
    """The longest unbroken stretch of ACTIVE states, and the other ACTIVE stretches.

    A feed hold or stop splits the log: what runs after it is often not the program carrying
    on (a reset then a move out of the way, a restart from the top), so matching it to the
    path would be guesswork. The other stretches are reported, not compared."""
    runs, cur = [], []
    for st in states:
        if st.get("execution") in (None, "ACTIVE"):
            cur.append(st)
        elif cur:
            runs.append(cur)
            cur = []
    if cur:
        runs.append(cur)
    if not runs:
        return [], []
    main = max(runs, key=lambda r: r[-1]["timestamp"] - r[0]["timestamp"])
    return main, [r for r in runs if r is not main]


def compare_by_path(traj: Trajectory, states: list[dict], predicted_load: dict | None = None,
                    max_jump_mm: float = 25.0) -> dict:
    """Compare a log without line numbers by matching positions to the simulated path.

    The first logged position is matched anywhere in the program (a log can start mid-run);
    after that the match only moves forward, a little further than the machine moved, so
    repeated passes over the same path (finishing laps, spring passes) stay in order. Each
    logged moment then has a simulated moment, which gives time per program line, coverage,
    position deviation and spindle load per line."""
    from scipy.spatial import cKDTree

    run, ignored = _main_run(states)
    if len(run) < 2:
        return {"error": "the recording has no running program (no ACTIVE states)"}
    path, ptime = _densify(traj)
    plen = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))])
    tree = cKDTree(path)
    actual = np.array([s["q"] for s in run], dtype=float)
    at = np.array([s["timestamp"] for s in run])

    idx = np.empty(len(run), dtype=int)
    _, idx[0] = tree.query(actual[0])
    for i in range(1, len(run)):
        moved = float(np.linalg.norm(actual[i] - actual[i - 1]))
        lo = idx[i - 1]
        hi = int(np.searchsorted(plen, plen[lo] + 3 * moved + 20.0, side="right"))
        hi = min(max(hi, lo + 2), len(path))
        d = np.linalg.norm(path[lo:hi] - actual[i], axis=1)
        j = int(np.argmin(d))
        if d[j] > max_jump_mm:   # lost the thread (a skipped section): nearest point anywhere ahead
            ahead = np.linalg.norm(path[lo:] - actual[i], axis=1)
            j = int(np.argmin(ahead))
        idx[i] = lo + j
    dev = np.array([_dist_to_polyline(actual[i, :3], path[max(idx[i] - 3, 0):idx[i] + 4, :3])
                    for i in range(len(run))])
    dev_rot = np.array([_dist_to_polyline(actual[i, 3:], path[max(idx[i] - 3, 0):idx[i] + 4, 3:])
                        for i in range(len(run))])

    # actual time as a function of sim time, through the matched points
    st_m = ptime[idx]
    keep = np.concatenate([[True], np.diff(st_m) > 1e-9])
    st_m, at_m = st_m[keep], at[keep]
    t = np.asarray(traj.t)
    sim_from, sim_to = float(st_m[0]), float(st_m[-1])
    actual_of = lambda x: np.interp(x, st_m, at_m)  # noqa: E731
    sim_time, sim_kind = _sim_line_stats(traj)
    act_time, cov_sim = defaultdict(float), defaultdict(float)
    for k in range(1, len(t)):
        a, b = max(t[k - 1], sim_from), min(t[k], sim_to)
        if b > a:
            ln = traj.line[k]
            act_time[ln] += float(actual_of(b) - actual_of(a))
            cov_sim[ln] += float(b - a)
    lines_cov = [ln for ln in cov_sim if cov_sim[ln] > 0]
    if not lines_cov:
        return {"error": "couldn't place the recording on the simulated path"}

    def ratio(kind):
        a = sum(act_time[ln] for ln in lines_cov if sim_kind[ln] == kind)
        s_ = sum(cov_sim[ln] for ln in lines_cov if sim_kind[ln] == kind)
        return round(a / s_, 3) if s_ > 0.05 else None

    dev_line = defaultdict(float)
    k_of = np.clip(np.searchsorted(t, ptime[idx], side="left"), 1, len(t) - 1)
    for i, k in enumerate(k_of):
        ln = traj.line[k]
        dev_line[ln] = max(dev_line[ln], float(dev[i]))
    per_line = [{"line": ln, "actual_s": round(act_time[ln], 3), "sim_s": round(cov_sim[ln], 3),
                 "diff_s": round(act_time[ln] - cov_sim[ln], 3), "kind": sim_kind[ln],
                 "max_dev_mm": round(dev_line.get(ln, 0.0), 4)} for ln in sorted(lines_cov)]

    load = None
    if predicted_load and any(s.get("spindle_load") is not None for s in run):
        pred_line = _predicted_load_by_line(traj, predicted_load)
        act_load = defaultdict(list)
        for i, k in enumerate(k_of):
            if run[i].get("spindle_load") is not None:
                act_load[traj.line[k]].append(float(run[i]["spindle_load"]))
        pairs = [(float(np.mean(v)), pred_line[ln]) for ln, v in act_load.items() if pred_line.get(ln, 0) > 0.5]
        if pairs:
            a, p = np.array(pairs).T
            factor = float((a @ p) / (p @ p))
            load = {"lines_compared": len(pairs), "actual_mean_pct": round(float(a.mean()), 1),
                    "predicted_mean_pct": round(float(p.mean()), 1), "scale_factor": round(factor, 3),
                    "hint": f"multiply material.specific_energy by {factor:.2f} to match this machine"}
    worst_i = int(np.argmax(dev))
    first_line = traj.line[int(k_of[0])]
    last_line = traj.line[int(k_of[-1])]
    actual_total = float(at[-1] - at[0])
    per_line.sort(key=lambda r: -abs(r["diff_s"]))
    sample = float(np.median(np.diff(at)))
    return {
        "alignment": "path",
        "coverage": {"sim_from_s": round(sim_from, 2), "sim_to_s": round(sim_to, 2),
                     "fraction": round((sim_to - sim_from) / max(float(t[-1]), 1e-9), 3),
                     "first_line": first_line, "last_line": last_line, "sample_s": round(sample, 3)},
        "cycle_time": {"actual_s": round(actual_total, 2), "sim_s": round(sim_to - sim_from, 2),
                       "ratio": round(actual_total / (sim_to - sim_from), 3) if sim_to > sim_from else None,
                       "cutting_ratio": ratio("cutting"), "rapid_ratio": ratio("rapid")},
        "position": {"states": len(run), "max_dev_mm": round(float(dev.max()), 4),
                     "mean_dev_mm": round(float(dev.mean()), 4), "max_dev_deg": round(float(dev_rot.max()), 4),
                     "worst_line": traj.line[int(k_of[worst_i])],
                     "note": (f"logged every {sample:.2f} s, so corners between samples are cut short in the "
                              "log -- deviations up to the distance travelled in one sample are expected")
                     if sample > 0.2 else None},
        "spindle_load": load,
        "lines_in_recording": len(lines_cov), "lines_matched": len(lines_cov),
        "per_line": per_line,
        "ignored_runs": [{"from_s": round(r[0]["timestamp"] - at[0], 1),
                          "seconds": round(r[-1]["timestamp"] - r[0]["timestamp"], 1), "states": len(r)}
                         for r in ignored],
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


def _x(ratio) -> str:
    return "n/a" if ratio is None else f"x{ratio}"


def format_report(r: dict, top: int = 10) -> str:
    if "error" in r:
        return f"ERROR: {r['error']}"
    c, p = r["cycle_time"], r["position"]
    lines = []
    if r.get("alignment") == "path":
        cv = r["coverage"]
        lines.append(f"alignment    by path (no line numbers in the log); the log starts on line {cv['first_line']}, "
                     f"ends on line {cv['last_line']} and covers {cv['fraction'] * 100:.0f}% of the simulated run "
                     f"({cv['sim_from_s']:.0f}-{cv['sim_to_s']:.0f} s)")
    lines += [f"cycle time   actual {c['actual_s']:.1f} s vs sim {c['sim_s']:.1f} s (x{c['ratio']}); "
             f"cutting lines {_x(c['cutting_ratio'])}, rapid lines {_x(c['rapid_ratio'])}",
             f"position     max {p['max_dev_mm']} mm / {p['max_dev_deg']} deg from the simulated path "
             f"(mean {p['mean_dev_mm']} mm; worst on line {p['worst_line']})",
             f"lines        {r['lines_matched']} of {r['lines_in_recording']} recorded lines match the program"]
    if p.get("note"):
        lines.append(f"             note: {p['note']}")
    for ig in r.get("ignored_runs", []):
        lines.append(f"ignored      {ig['states']} states ({ig['seconds']:.0f} s) running {ig['from_s']:+.0f} s from the "
                     "compared stretch, after a feed hold / stop; not compared")
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
