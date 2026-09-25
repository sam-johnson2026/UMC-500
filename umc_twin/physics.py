"""Spindle power / load estimate from the cutting sim -- the first physics layer.

    cutting power  = material removal rate x specific cutting energy of the work material
    spindle power  = cutting power / drive efficiency
    load %         = spindle power / power available at that rpm
                     (available = min(rated power, rated torque x angular speed))
    torque         = spindle power / angular speed
    cutting force  = cutting power / cutting speed at the tool's diameter

Specific cutting energies are handbook mid-range values (unit power at the tool); the spindle
ratings in the config are placeholders. Both get tuned once the machine reports its real
spindle load -- comparing this prediction with the live value is the point of the exercise.
"""
from __future__ import annotations

import math

import numpy as np

from .gcode import Trajectory
from .job import JobSetup

# J/mm^3 (= W*s/mm^3). Typical mid-range values; override per job with material.specific_energy.
SPECIFIC_ENERGY = {
    "aluminum": 0.8, "aluminum_6061": 0.8, "aluminum_7075": 0.9, "aluminum_cast": 0.7,
    "brass": 1.8, "copper": 2.8, "magnesium": 0.45,
    "steel_mild": 2.5, "steel_1018": 2.5, "steel_4140": 3.5, "steel_tool": 5.0,
    "stainless": 3.5, "stainless_304": 3.6, "stainless_316": 3.8, "stainless_17_4": 4.0,
    "cast_iron": 1.8, "titanium": 3.5, "titanium_6al4v": 3.5, "inconel": 6.0,
    "plastic": 0.2, "delrin": 0.2,
}
BIN_S = 0.05          # s per bin of the load series
SMOOTH_S = 0.2        # s moving-average window
MAX_POINTS = 6000


def _spread(bins: np.ndarray, t0: float, t1: float, v: float, dt_bin: float) -> None:
    nb = len(bins)
    if t1 - t0 <= 1e-12:
        bins[min(max(int(t1 / dt_bin), 0), nb - 1)] += v
        return
    b0, b1 = max(int(t0 / dt_bin), 0), min(int(math.ceil(t1 / dt_bin)), nb)
    if b1 <= b0:
        bins[min(b0, nb - 1)] += v
        return
    edges = np.clip(np.arange(b0, b1 + 1) * dt_bin, t0, t1)
    bins[b0:b1] += v * np.diff(edges) / (t1 - t0)


def specific_energy(setup: JobSetup) -> tuple[str, float]:
    mat = setup.material or {}
    name = str(mat.get("name", "aluminum_6061")).lower()
    if "specific_energy" in mat:
        return name, float(mat["specific_energy"])
    if name not in SPECIFIC_ENERGY:
        raise ValueError(f"unknown material {name!r}; known: {', '.join(sorted(SPECIFIC_ENERGY))} "
                         f"(or give material.specific_energy in J/mm^3)")
    return name, SPECIFIC_ENERGY[name]


def spindle_load(traj: Trajectory, removed, setup: JobSetup, spindle_cfg: dict) -> dict:
    """`removed` is a MaterialResult (exact cut times) or a per-sample removed-volume array."""
    t = np.asarray(traj.t, dtype=float)
    cut_times = getattr(removed, "cut_times", None)
    cut_volumes = getattr(removed, "cut_volumes", None)
    cut_spans = getattr(removed, "cut_spans", None)
    if hasattr(removed, "removed_per_sample"):
        removed = removed.removed_per_sample
    removed = np.asarray(removed, dtype=float)
    duration = float(t[-1]) if len(t) else 0.0
    name, u = specific_energy(setup)
    p_max = float(spindle_cfg.get("max_power_kw", 22.4)) * 1000.0
    t_max = float(spindle_cfg.get("max_torque_nm", 122.0))
    eff = float(spindle_cfg.get("efficiency", 0.8))
    rpm_max = float(spindle_cfg.get("max_rpm", 15000.0))

    dt_bin = max(BIN_S, duration / MAX_POINTS) if duration > 0 else BIN_S
    nb = max(1, int(math.ceil(duration / dt_bin)))
    vol = np.zeros(nb)
    if cut_times is not None and len(cut_times):
        # each sub-step removed its volume evenly over [t - span, t]
        for te, v, sp in zip(cut_times, cut_volumes, cut_spans):
            _spread(vol, te - sp, te, v, dt_bin)
    # otherwise spread each segment's removed volume evenly over its time span
    for k in (np.nonzero(removed > 0)[0] if cut_times is None else []):
        t0, t1 = t[k - 1], t[k]
        if t1 - t0 <= 1e-12:
            vol[min(int(t0 / dt_bin), nb - 1)] += removed[k]
            continue
        _spread(vol, t0, t1, removed[k], dt_bin)
    mrr = vol / dt_bin                                              # mm^3/s
    w = max(1, int(round(SMOOTH_S / dt_bin)))
    mrr = np.convolve(mrr, np.ones(w) / w, mode="same")

    centres = (np.arange(nb) + 0.5) * dt_bin
    k_at = np.clip(np.searchsorted(t, centres), 1, max(len(t) - 1, 1))
    rpm = np.abs(np.asarray(traj.spindle, dtype=float)[k_at]) if len(t) > 1 else np.zeros(nb)
    tool_d = np.array([setup.tool(traj.tool[k]).diameter for k in k_at]) if len(t) > 1 else np.zeros(nb)

    p_cut = mrr * u                                                 # W at the tool
    p_spindle = p_cut / eff
    omega = rpm * 2 * math.pi / 60.0
    with np.errstate(divide="ignore", invalid="ignore"):
        available = np.where(omega > 0, np.minimum(p_max, t_max * omega), 0.0)
        load = np.where(available > 0, p_spindle / available * 100.0, np.where(p_spindle > 0, np.inf, 0.0))
        torque = np.where(omega > 0, p_spindle / omega, 0.0)
        v_c = math.pi * tool_d / 1000.0 * rpm / 60.0                # m/s
        force = np.where(v_c > 0, p_cut / v_c, 0.0)                 # N

    issues = []
    over = np.nonzero(np.isfinite(load) & (load > 100.0))[0]
    if len(over):
        k = int(k_at[over[0]])
        issues.append({"t": round(float(centres[over[0]]), 2), "line": traj.line[k], "kind": "spindle_overload",
                       "detail": f"predicted spindle load {float(load[over].max()):.0f}% (peak)"})
    fast = [k for k in range(len(t)) if abs(traj.spindle[k]) > rpm_max + 1e-6]
    if fast:
        k = fast[0]
        issues.append({"t": round(float(t[k]), 2), "line": traj.line[k], "kind": "rpm_over_max",
                       "detail": f"S{abs(traj.spindle[k]):g} above the spindle's {rpm_max:g} rpm"})

    finite = np.where(np.isfinite(load), load, 0.0)
    r1 = lambda a: np.round(a, 1).tolist()  # noqa: E731
    return {
        "material": name, "specific_energy_j_mm3": u,
        "bin_s": dt_bin,
        "mrr_mm3_s": r1(mrr), "power_kw": np.round(p_spindle / 1000.0, 3).tolist(),
        "load_pct": r1(finite), "torque_nm": r1(torque), "force_n": r1(force),
        "peak": {
            "mrr_cm3_min": round(float(mrr.max(initial=0)) * 60 / 1000, 2),
            "power_kw": round(float(p_spindle.max(initial=0)) / 1000, 2),
            "load_pct": round(float(finite.max(initial=0)), 1),
            "torque_nm": round(float(torque.max(initial=0)), 1),
            "force_n": round(float(force.max(initial=0)), 0),
        },
        "issues": issues,
    }
