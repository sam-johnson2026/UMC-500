"""Acceleration-aware re-timing of a trajectory (a small look-ahead planner).

The interpreter times each move from feed / rapid rate alone, as if the axes could change speed
instantly. Here that "ideal" timing is turned into something closer to what the control does:

- every segment gets a trapezoidal speed profile limited by each axis's acceleration;
- at a junction between segments the speed is limited so that the sudden change of direction
  stays within what the servos absorb in `motion.corner_time` seconds at full acceleration
  (sharp corners slow down, gentle ones don't);
- along finely sampled curves (arcs, TCPC moves) the speed is also limited by centripetal
  acceleration, estimated from how the direction changes across neighbouring segments;
- dwell / tool change / program stops come to a full stop, and so does every switch between a
  rapid and a cutting move (the control's in-position check).

Path speed is measured in a weighted joint space (each axis divided by its max velocity), so
linear (mm) and rotary (deg) axes mix correctly and every limit stays per-axis.
The acceleration and corner-time values in the config are placeholders until they are tuned
against real cycle times -- exactly the kind of loop the twin exists for.
"""
from __future__ import annotations

import numpy as np

from .config import Machine

EPS = 1e-12


def _trapezoid_time(s: float, v0: float, v1: float, vmax: float, a: float) -> float:
    s_acc = max(vmax * vmax - v0 * v0, 0.0) / (2 * a)
    s_dec = max(vmax * vmax - v1 * v1, 0.0) / (2 * a)
    if s_acc + s_dec <= s:
        return (vmax - v0) / a + (vmax - v1) / a + (s - s_acc - s_dec) / vmax
    vp = np.sqrt(max((2 * a * s + v0 * v0 + v1 * v1) / 2, 0.0))
    return max((vp - v0) / a + (vp - v1) / a, 0.0)


def retime(t, q, machine: Machine, profile: dict | None = None, motion=None) -> np.ndarray:
    """New sample times for joint samples `q` whose ideal times are `t`. If `profile` is a dict,
    it is filled with each segment's speed profile (entry/exit/cruise speed, accel, length) so the
    motion *within* a segment can be reconstructed (see `fractions_at`)."""
    t = np.asarray(t, dtype=float)
    q = np.asarray(q, dtype=float)
    n = len(t)
    if n < 2:
        return t.copy()
    joints = [machine.joints[k] for k in ("X", "Y", "Z", "B", "C")]
    vmax = np.array([j.max_velocity for j in joints]) / 60.0   # per second
    amax = np.array([j.max_accel for j in joints])
    tau = float(machine.raw.get("motion", {}).get("corner_time", 0.015))

    dq = np.diff(q, axis=0)
    dt0 = np.diff(t)
    s = np.sqrt(((dq / vmax) ** 2).sum(axis=1))           # weighted length ("seconds at full speed")
    moving = s > 1e-9
    d = np.zeros_like(dq)
    d[moving] = dq[moving] / s[moving, None]
    with np.errstate(divide="ignore", invalid="ignore"):
        V = np.where(moving & (dt0 > EPS), s / np.maximum(dt0, EPS), 0.0)   # cruise speed from the ideal timing
        A = np.where(moving, np.min(np.where(np.abs(d) > EPS, amax / np.abs(d), np.inf), axis=1), 0.0)

    # junction speed limits; J[k] is the speed at sample k (between segment k-1 and k)
    J = np.zeros(n)
    for k in range(1, n - 1):
        a, b = k - 1, k
        if not (moving[a] and moving[b]):
            continue
        dd = np.abs(d[b] - d[a])
        mask = dd > 1e-9
        if not mask.any():
            J[k] = min(V[a], V[b])
            continue
        corner = np.min(amax[mask] * tau / dd[mask])
        s_avg = 0.5 * (s[a] + s[b])
        curve = np.min(np.sqrt(amax[mask] * s_avg / dd[mask]))
        J[k] = min(corner, curve, V[a], V[b])

    # in-position check: rapids come to a stop before a cutting move and vice versa, as on the control
    if motion is not None:
        rapid = np.isin(np.asarray(motion), (0, 4, 5))   # rapid, tool change, home
        for k in range(1, n - 1):
            if rapid[k] != rapid[k + 1]:
                J[k] = 0.0

    # backward then forward pass: reachable speeds given accel over each segment
    for k in range(n - 2, -1, -1):
        if moving[k]:
            J[k] = min(J[k], np.sqrt(J[k + 1] ** 2 + 2 * A[k] * s[k]))
    for k in range(n - 1):
        if moving[k]:
            J[k + 1] = min(J[k + 1], np.sqrt(J[k] ** 2 + 2 * A[k] * s[k]))

    new_dt = np.empty(n - 1)
    for k in range(n - 1):
        if not moving[k]:
            new_dt[k] = dt0[k]                              # dwell, tool change, zero-length
        elif V[k] <= EPS:
            new_dt[k] = 0.0
        else:
            new_dt[k] = max(_trapezoid_time(s[k], J[k], J[k + 1], V[k], A[k]), dt0[k])
    if profile is not None:
        profile.update(v0=J[:-1].copy(), v1=J[1:].copy(), vc=V, a=A, s=s, moving=moving)
    return np.concatenate([[t[0]], t[0] + np.cumsum(new_dt)])


def fractions_at(traj, ts: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """For times `ts`: the segment k (motion from sample k-1 to k) and how far along it the
    machine is (0..1), following the planner's trapezoidal speed profile. Without a profile
    (accel=False) the motion is taken as uniform within each segment."""
    t = np.asarray(traj.t)
    k = np.clip(np.searchsorted(t, ts, side="right"), 1, len(t) - 1)
    tau = np.clip(ts - t[k - 1], 0.0, None)
    T = t[k] - t[k - 1]
    prof = getattr(traj, "profile", None)
    if not prof:
        return k, np.where(T > 0, np.clip(tau / np.where(T > 0, T, 1), 0, 1), 1.0)
    i = k - 1
    v0, v1, vc, a, S = (prof[n][i] for n in ("v0", "v1", "vc", "a", "s"))
    moving = prof["moving"][i] & (vc > EPS) & (a > 0)
    a_ = np.where(moving, a, 1.0)
    vc_ = np.where(moving, vc, 1.0)
    s_acc = np.maximum(vc_ ** 2 - v0 ** 2, 0) / (2 * a_)
    s_dec = np.maximum(vc_ ** 2 - v1 ** 2, 0) / (2 * a_)
    peak = np.where(s_acc + s_dec <= S, vc_, np.sqrt(np.maximum((2 * a_ * S + v0 ** 2 + v1 ** 2) / 2, 0)))
    s_acc = np.maximum(peak ** 2 - v0 ** 2, 0) / (2 * a_)
    s_dec = np.maximum(peak ** 2 - v1 ** 2, 0) / (2 * a_)
    t_acc = np.maximum(peak - v0, 0) / a_
    s_cru = np.maximum(S - s_acc - s_dec, 0)
    t_cru = s_cru / np.maximum(peak, EPS)
    td = tau - t_acc - t_cru
    dist = np.where(tau < t_acc, v0 * tau + a_ * tau ** 2 / 2,
                    np.where(tau < t_acc + t_cru, s_acc + peak * (tau - t_acc),
                             s_acc + s_cru + peak * td - a_ * td ** 2 / 2))
    f = np.where(moving, np.clip(dist / np.where(S > 0, S, 1), 0, 1),
                 np.where(T > 0, np.clip(tau / np.where(T > 0, T, 1), 0, 1), 1.0))
    return k, f


def apply(traj, machine: Machine) -> None:
    """Re-time a Trajectory in place; the feed-rate-only times are kept as `traj.t_ideal`."""
    t_old = np.array(traj.t)
    profile: dict = {}
    t_new = retime(t_old, np.array(traj.q), machine, profile, motion=traj.motion)
    traj.profile = profile
    traj.t_ideal = t_old.tolist()
    traj.t = t_new.tolist()
    if len(t_old) > 1:
        for e in traj.events:
            e["t"] = float(np.interp(e["t"], t_old, t_new))
