"""Turn measurements taken at the machine into config/calibration.yaml.

The CAD gives the geometry; the machine gives where that geometry sits relative to home.
Everything here writes an *overlay*: the CAD-derived config/umc500.yaml is never edited, and
deleting calibration.yaml returns to the uncalibrated model.

Inputs, all optional (give what you have measured):

    nose_to_platter   Z at home, B0: spindle nose (gauge line) to platter top.
    mrzp              Haas settings 255 / 256 / 257 (machine rotary zero point): machine X and Y of
                      the C-axis centre and machine Z of the B axis, as the control shows them.
                      More precise than the tape measure -- takes precedence for Z when both are given.
    b_direction       "right" if jogging B+ at home turns the platter face toward +X, else "left".
    c_direction       "cw" if jogging C+ turns the table clockwise seen from above, else "ccw".
    rotary_rapid      {"B": deg/min, "C": deg/min}
    tool_change_time  seconds; tool_change_position {"X":..,"Y":..,"Z":..} machine coords.
    options           {"spindle": "hsk", ...} -- your machine's build.
"""
from __future__ import annotations

import datetime as _dt
from pathlib import Path

import numpy as np
import yaml

from .config import DEFAULT_CALIBRATION, Machine, deep_merge, load_machine

INCH = 25.4
# How far a measurement may be from the CAD estimate before we call it suspicious (mm).
SANITY_MM = 60.0


class CalibrationError(ValueError):
    pass


def compute_overlay(machine: Machine, *, units: str = "mm", nose_to_platter: float | None = None,
                    mrzp: list[float] | None = None, b_direction: str | None = None,
                    c_direction: str | None = None, rotary_rapid: dict | None = None,
                    tool_change_time: float | None = None, tool_change_position: dict | None = None,
                    options: dict | None = None) -> tuple[dict, list[str]]:
    """Return (overlay dict, notes). `machine` should be the *uncalibrated* machine."""
    scale = INCH if units in ("in", "inch") else 1.0
    raw = machine.raw
    gauge = np.array(raw["spindle"]["gauge_point"], dtype=float)
    top_z = float(raw["table"]["top_z"])
    pose = dict(raw["cad"]["pose"])
    overlay: dict = {}
    notes: list[str] = []

    if nose_to_platter is not None:
        m = nose_to_platter * scale
        pose["Z"] = round(float(gauge[2] - top_z - m), 3)
        notes.append(f"nose-to-platter {m:.2f} mm -> CAD drawn at Z {pose['Z']:+.3f}")
        if abs(pose["Z"]) > SANITY_MM:
            notes.append(f"WARNING: that is {abs(pose['Z']):.0f} mm from the CAD estimate "
                         f"({gauge[2] - top_z:.1f} mm); double-check it was measured at Z home, B0")

    if mrzp is not None:
        if len(mrzp) != 3:
            raise CalibrationError("mrzp needs three values: settings 255, 256, 257")
        q = np.array(mrzp, dtype=float) * scale
        cad_estimate = -gauge  # machine coords that put the gauge point on the pivot
        for axis, v, est in zip("XYZ", q, cad_estimate):
            new = round(float(gauge["XYZ".index(axis)] + v), 3)
            if nose_to_platter is not None and axis == "Z":
                notes.append(f"MRZP Z overrides the tape measure: CAD drawn at Z {new:+.3f} "
                             f"(tape said {pose['Z']:+.3f}, difference {abs(new - pose['Z']):.2f} mm)")
            pose[axis] = new
            if abs(v - est) > SANITY_MM:
                notes.append(f"WARNING: MRZP {axis} = {v:.2f} mm is {abs(v - est):.0f} mm from the CAD estimate "
                             f"{est:.2f}; check units and sign (Haas shows these as negative machine coords)")
        notes.append("MRZP -> CAD pose X {X:+.3f} Y {Y:+.3f} Z {Z:+.3f}".format(**pose))

    if pose != raw["cad"]["pose"]:
        overlay["cad"] = {"pose": pose}

    joints: dict = {}
    if b_direction is not None:
        if b_direction not in ("right", "left"):
            raise CalibrationError("b_direction must be 'right' or 'left'")
        joints["B"] = {"axis": [0, 1, 0] if b_direction == "right" else [0, -1, 0]}
        if b_direction == "left":
            notes.append("B reversed from the CAD-based guess: re-run examples/make_demo.py and check that "
                         "tilting at home is still collision-free (python -m umc_twin pose 0 0 0 110 0)")
    if c_direction is not None:
        if c_direction not in ("cw", "ccw"):
            raise CalibrationError("c_direction must be 'cw' or 'ccw'")
        joints["C"] = {"axis": [0, 0, -1] if c_direction == "cw" else [0, 0, 1]}
    for axis, v in (rotary_rapid or {}).items():
        if v:
            joints.setdefault(axis, {})["max_velocity"] = float(v)
    if joints:
        overlay["joints"] = joints

    tc: dict = {}
    if tool_change_time is not None:
        tc["time"] = float(tool_change_time)
    if tool_change_position:
        tc["position"] = {k: float(v) * scale for k, v in tool_change_position.items()}
    if tc:
        overlay["tool_change"] = tc

    for k, v in (options or {}).items():
        if k not in raw["options"] or v not in raw["options"][k]["choices"]:
            raise CalibrationError(f"unknown option {k}={v}")
        overlay.setdefault("options", {})[k] = {"default": v}
    return overlay, notes


def save_overlay(overlay: dict, path: Path = DEFAULT_CALIBRATION, merge: bool = True) -> dict:
    """Write (merging with any existing calibration) and return the full overlay."""
    existing = (yaml.safe_load(path.read_text()) or {}) if (merge and path.exists()) else {}
    full = deep_merge(existing, overlay)
    header = (f"# UMC-500 calibration overlay -- merged over config/umc500.yaml.\n"
              f"# Written by `umc-twin calibrate` on {_dt.date.today().isoformat()}. Delete to return to the CAD model.\n")
    path.write_text(header + yaml.safe_dump(full, sort_keys=False))
    return full


def summary(machine: Machine) -> dict:
    """Numbers worth eyeballing after calibrating."""
    from .kinematics import Kinematics

    kin = Kinematics(machine)
    home_tip = kin.tool_tip_world(np.zeros(5))
    mrzp = kin.linear_for_tip(np.zeros(3), 0.0)
    return {
        "nose_to_platter_at_home_mm": round(float(home_tip[2] - machine.raw["table"]["top_z"]), 3),
        "mrzp_mm": [round(float(v), 3) for v in mrzp],
        "mrzp_in": [round(float(v) / INCH, 4) for v in mrzp],
        "b_axis": machine.raw["joints"]["B"]["axis"],
        "c_axis": machine.raw["joints"]["C"]["axis"],
        "options": machine.options,
        "calibrated": "calibrated_from" in machine.raw,
    }


def calibrate(save: bool = True, path: Path = DEFAULT_CALIBRATION, **inputs) -> dict:
    base = load_machine(calibration=None)
    overlay, notes = compute_overlay(base, **inputs)
    full = save_overlay(overlay, path) if save else deep_merge(
        (yaml.safe_load(path.read_text()) or {}) if path.exists() else {}, overlay)
    tmp = path.with_suffix(".preview.yaml")
    tmp.write_text(yaml.safe_dump(full))
    try:
        result = summary(load_machine(calibration=tmp))
    finally:
        tmp.unlink()
    return {"overlay": full, "notes": notes, "summary": result, "saved": save}
