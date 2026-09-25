"""Machine definition loaded from config/umc500.yaml."""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG = REPO_ROOT / "config" / "umc500.yaml"
# Machine-specific measurements (written by `umc-twin calibrate`), merged over DEFAULT_CONFIG.
DEFAULT_CALIBRATION = REPO_ROOT / "config" / "calibration.yaml"

JOINT_ORDER = ("X", "Y", "Z", "B", "C")


@dataclass
class Joint:
    name: str
    type: str  # "linear" | "rotary"
    parent: str
    axis: np.ndarray
    origin: np.ndarray
    limits: tuple[float, float] | None
    max_velocity: float  # mm/min or deg/min
    max_accel: float = 3000.0  # mm/s^2 or deg/s^2

    @property
    def rotary(self) -> bool:
        return self.type == "rotary"


@dataclass
class Part:
    id: str
    step: str
    link: str
    color: str = "#888888"
    opacity: float = 1.0
    collide: bool = True
    visible: bool = True
    option: dict[str, str] = field(default_factory=dict)


@dataclass
class Machine:
    raw: dict
    joints: dict[str, Joint]
    parts: list[Part]
    options: dict[str, str]  # selected option values

    # --- convenience accessors -------------------------------------------------
    @property
    def cad_pose(self) -> np.ndarray:
        pose = self.raw["cad"]["pose"]
        return np.array([pose[j] for j in JOINT_ORDER], dtype=float)

    @property
    def gauge_point(self) -> np.ndarray:
        return np.array(self.raw["spindle"]["gauge_point"], dtype=float)

    @property
    def tool_direction(self) -> np.ndarray:
        return np.array(self.raw["spindle"]["direction"], dtype=float)

    @property
    def max_cutting_feed(self) -> float:
        return float(self.raw["max_cutting_feed"])

    @property
    def tool_change(self) -> dict:
        return self.raw["tool_change"]

    def part_enabled(self, part: Part) -> bool:
        """A part is present if it has no option tag or its option value is selected."""
        return all(self.options.get(k) == v for k, v in part.option.items())

    def active_parts(self) -> list[Part]:
        return [p for p in self.parts if self.part_enabled(p)]


def load_machine(path: str | Path = DEFAULT_CONFIG, options: dict[str, str] | None = None,
                 calibration: str | Path | None | bool = True) -> Machine:
    """Load the machine definition.

    calibration: True (default) merges config/calibration.yaml when it exists; a path merges that
    file; None/False uses the CAD-derived config only.
    """
    raw = yaml.safe_load(Path(path).read_text())
    if calibration is True:
        calibration = DEFAULT_CALIBRATION if DEFAULT_CALIBRATION.exists() else None
    if calibration:
        overlay = yaml.safe_load(Path(calibration).read_text()) or {}
        raw = deep_merge(raw, overlay)
        raw["calibrated_from"] = str(calibration)
    joints = {}
    for name in JOINT_ORDER:
        j = raw["joints"][name]
        joints[name] = Joint(
            name=name,
            type=j["type"],
            parent=j["parent"],
            axis=_unit(j["axis"]),
            origin=np.array(j.get("origin", [0, 0, 0]), dtype=float),
            limits=tuple(j["limits"]) if j.get("limits") else None,
            max_velocity=float(j["max_velocity"]),
            max_accel=float(j.get("max_accel", 3000.0)),
        )
    parts = [Part(**p) for p in raw["parts"]]

    selected = {k: v["default"] for k, v in raw["options"].items()}
    for k, v in (options or {}).items():
        if k not in raw["options"]:
            raise KeyError(f"unknown option {k!r}; known: {sorted(raw['options'])}")
        if v not in raw["options"][k]["choices"]:
            raise ValueError(f"option {k}={v!r} not in {raw['options'][k]['choices']}")
        selected[k] = v
    return Machine(raw=raw, joints=joints, parts=parts, options=selected)


def deep_merge(base: dict, overlay: dict) -> dict:
    """Recursively merge `overlay` into a copy of `base` (dicts merge, everything else replaces)."""
    out = dict(base)
    for k, v in overlay.items():
        out[k] = deep_merge(out[k], v) if isinstance(v, dict) and isinstance(out.get(k), dict) else v
    return out


def _unit(v) -> np.ndarray:
    a = np.array(v, dtype=float)
    return a / np.linalg.norm(a)
