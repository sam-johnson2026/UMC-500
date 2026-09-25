"""Servo lag: how far the tool strays from the programmed path because the axes lag behind.

Each axis is a position loop with gain Kv (1/s) and velocity feed-forward ff (0..1):

    following error at speed v  ~  (1 - ff) * v / Kv

On a straight line that lag only delays the tool along the path. On curves and corners the
axes lag by different amounts and the tool cuts inside the programmed path -- contour error.
For a circle of radius R at speed v (equal gains, x = v / (R Kv)) the radius shrinks by about
(1 - ff) R x^2 - ((1 - ff) R x)^2 / (2 R); a ballbar test measures exactly this.

The model runs the commanded joint motion through each axis's loop (1 ms steps), maps the
lagged joints back through the kinematics, and measures the tool-tip error perpendicular to the
commanded path in the part frame. Gains in config/umc500.yaml (`servo:`) are placeholders --
tune them from a ballbar test or from `compare` on a recorded run.
"""
from __future__ import annotations

import numpy as np
from scipy.signal import lfilter

from .config import JOINT_ORDER, Machine
from .gcode import MOTION, Trajectory
from .kinematics import Kinematics

DT = 0.001  # s


def _distance_to_path(pts: np.ndarray, idx: np.ndarray, path: np.ndarray, window: int,
                      chunk: int = 20000) -> np.ndarray:
    """Distance from pts[i] to the commanded path polyline near sample idx[i] (segments
    idx-window .. idx+2): the contour error, independent of how far behind the tool is."""
    out = np.empty(len(pts))
    offs = np.arange(-window, 3)
    for c0 in range(0, len(pts), chunk):
        p = pts[c0:c0 + chunk]
        j = np.clip(idx[c0:c0 + chunk, None] + offs[None, :], 0, len(path) - 2)
        a, b = path[j], path[j + 1]
        ab = b - a
        denom = np.maximum((ab * ab).sum(-1), 1e-18)
        f = np.clip(((p[:, None, :] - a) * ab).sum(-1) / denom, 0, 1)
        d = np.linalg.norm(a + ab * f[..., None] - p[:, None, :], axis=-1)
        out[c0:c0 + chunk] = d.min(axis=1)
    return out


def servo_params(machine: Machine) -> tuple[np.ndarray, float]:
    cfg = machine.raw.get("servo") or {}
    kv = cfg.get("kv", {})
    gains = np.array([float(kv.get(a, 40.0)) if isinstance(kv, dict) else float(kv) for a in JOINT_ORDER])
    return gains, float(cfg.get("feedforward", 0.8))


def contour_error(traj: Trajectory, kin: Kinematics, tool_lengths: dict[int, float],
                  max_points: int = 400_000, cut_times: np.ndarray | None = None) -> dict:
    """Contour error along the cutting moves. With `cut_times` (from the cutting sim) only the
    moments the tool is actually removing material count -- error in the air doesn't matter."""
    t = np.asarray(traj.t)
    q = np.asarray(traj.q)
    if len(t) < 2 or t[-1] <= 0:
        return {"max_contour_mm": 0.0, "per_line": [], "max_following_mm": {}}
    dt = max(DT, t[-1] / max_points)
    ts = np.arange(0.0, t[-1], dt)
    from .timing import fractions_at

    k_seg, frac = fractions_at(traj, ts)                  # follow the planner's accel profile in each move
    cmd = q[k_seg - 1] + (q[k_seg] - q[k_seg - 1]) * frac[:, None]
    gains, ff = servo_params(kin.m)
    act = np.empty_like(cmd)
    for i in range(5):
        a = min(gains[i] * dt, 1.0)
        lag = lfilter([a], [1.0, -(1.0 - a)], cmd[:, i] - cmd[0, i]) + cmd[0, i]
        act[:, i] = cmd[:, i] - (1.0 - ff) * (cmd[:, i] - lag)
    k_of = k_seg
    motion = np.asarray(traj.motion)[k_of]
    tools = np.asarray(traj.tool)[k_of]
    lines = np.asarray(traj.line)[k_of]
    following = np.abs(act - cmd).max(axis=0)

    cutting = np.isin(motion, (MOTION["feed"], MOTION["arc"]))
    # after a rapid the control waits for in-position before cutting; the axes settle in about
    # 4 time constants -- not counted as contour error
    rapid_end = t[1:][np.isin(np.asarray(traj.motion)[1:], (MOTION["rapid"], MOTION["home"], MOTION["toolchange"]))]
    if len(rapid_end):
        j = np.searchsorted(rapid_end, ts, side="right") - 1
        since = ts - rapid_end[np.clip(j, 0, None)]
        cutting &= ~((j >= 0) & (since < 4.0 / max(float(gains.min()), 1e-6)))
    if cut_times is not None:
        ct = np.sort(np.asarray(cut_times, dtype=float))
        if len(ct):
            j = np.clip(np.searchsorted(ct, ts), 1, len(ct) - 1)
            near = np.minimum(np.abs(ct[j] - ts), np.abs(ct[j - 1] - ts))
            cutting &= near <= max(5 * dt, 0.02)
        else:
            cutting &= False
    err = np.zeros(len(ts))
    # the actual tool is behind the command by about lag_steps samples; search a little wider
    lag_s = (1.0 - ff) / max(float(gains.min()), 1e-6)
    window = int(np.ceil(4 * lag_s / dt)) + 3
    for tool in set(tools[cutting].tolist()):
        sel = np.nonzero(cutting & (tools == tool))[0]
        L = tool_lengths.get(tool, 0.0)
        tip_c, _ = kin.tip_and_axis_in_table_batch(cmd, L) if len(sel) else (None, None)
        tip_a, _ = kin.tip_and_axis_in_table_batch(act[sel], L)
        err[sel] = _distance_to_path(tip_a, sel, tip_c, window)
    per_line: dict[int, float] = {}
    for ln, v in zip(lines[cutting], err[cutting]):
        if v > per_line.get(ln, 0.0):
            per_line[int(ln)] = float(v)
    worst = sorted(per_line.items(), key=lambda kv: -kv[1])
    i = int(np.argmax(err)) if len(err) else 0
    return {
        "kv": gains.tolist(), "feedforward": ff,
        "in_material_only": cut_times is not None,
        "max_contour_mm": round(float(err.max(initial=0.0)), 5),
        "at": {"t": round(float(ts[i]), 3), "line": int(lines[i])} if len(err) else None,
        "per_line": [{"line": ln, "max_contour_mm": round(v, 5)} for ln, v in worst[:50]],
        "max_following_mm": {a: round(float(v), 4) for a, v in zip(JOINT_ORDER, following)},
    }
