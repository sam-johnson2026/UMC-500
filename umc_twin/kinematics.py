"""Forward / inverse kinematics for the UMC-500 (spindle on X-Y-Z, table on B-C trunnion).

Joint vectors are always ordered (X, Y, Z, B, C) in Haas machine coordinates (mm, deg).
Meshes are stored in the world frame at `cad.pose`, so every link transform is the motion
*relative to that pose* -- the same convention the web viewer uses.
"""
from __future__ import annotations

import numpy as np

from .config import JOINT_ORDER, Machine

LINEAR = (0, 1, 2)
ROTARY = (3, 4)


def rotation_matrix(axis: np.ndarray, angle_deg: float) -> np.ndarray:
    """Right-handed rotation about a unit axis (Rodrigues)."""
    a = np.radians(angle_deg)
    x, y, z = axis
    c, s, t = np.cos(a), np.sin(a), 1 - np.cos(a)
    return np.array([
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ])


class Kinematics:
    def __init__(self, machine: Machine):
        self.m = machine
        self.joints = [machine.joints[n] for n in JOINT_ORDER]
        self.q0 = machine.cad_pose
        self.gauge = machine.gauge_point
        self.tool_dir = machine.tool_direction
        # Linear axes: machine displacement = A @ dq  (columns are the axis directions)
        self.A = np.column_stack([self.joints[i].axis for i in LINEAR])
        self.A_inv = np.linalg.inv(self.A)
        self.spindle_link = machine.raw["spindle"]["link"]
        self.table_link = machine.raw["table"]["link"]

    # --- forward ----------------------------------------------------------------
    def joint_transform(self, i: int, value: float) -> np.ndarray:
        j = self.joints[i]
        d = value - self.q0[i]
        T = np.eye(4)
        if j.rotary:
            R = rotation_matrix(j.axis, d)
            T[:3, :3] = R
            T[:3, 3] = j.origin - R @ j.origin
        else:
            T[:3, 3] = j.axis * d
        return T

    def link_transforms(self, q) -> dict[str, np.ndarray]:
        """World transform of every link (relative to the CAD pose)."""
        out = {"base": np.eye(4)}
        for i, j in enumerate(self.joints):  # JOINT_ORDER lists parents before children
            out[j.name] = out[j.parent] @ self.joint_transform(i, q[i])
        return out

    def table_transform(self, q) -> np.ndarray:
        return self.link_transforms(q)[self.table_link]

    def tool_tip_world(self, q, tool_length: float = 0.0) -> np.ndarray:
        T = self.link_transforms(q)[self.spindle_link]
        return (T @ np.append(self.gauge + self.tool_dir * tool_length, 1.0))[:3]

    def tool_tip_in_table(self, q, tool_length: float = 0.0) -> np.ndarray:
        """Tool tip expressed in the table frame (= world frame with B, C at the CAD pose).

        This is the position of the tool relative to the part: what TCPC programs describe.
        """
        T = self.table_transform(q)
        tip = np.append(self.tool_tip_world(q, tool_length), 1.0)
        return (np.linalg.inv(T) @ tip)[:3]

    def tool_axis_in_table(self, q) -> np.ndarray:
        """Tool direction (spindle -> tip) as seen from the part, i.e. in the table frame."""
        R_tab = self.table_transform(q)[:3, :3]
        R_sp = self.link_transforms(q)[self.spindle_link][:3, :3]
        return R_tab.T @ (R_sp @ self.tool_dir)

    # --- inverse ----------------------------------------------------------------
    def linear_for_tip(self, tip_world: np.ndarray, tool_length: float) -> np.ndarray:
        """Machine X/Y/Z that put the tool tip at `tip_world` (spindle has no rotation)."""
        gauge_target = tip_world - self.tool_dir * tool_length
        return self.q0[:3] + self.A_inv @ (gauge_target - self.gauge)

    def inverse_tcp(self, p_table: np.ndarray, b: float, c: float, tool_length: float) -> np.ndarray:
        """Joint vector placing the tip at `p_table` (table frame) with rotaries at b, c."""
        q = np.array([*self.q0[:3], b, c], dtype=float)
        tip_world = (self.table_transform(q) @ np.append(p_table, 1.0))[:3]
        q[:3] = self.linear_for_tip(tip_world, tool_length)
        return q

    # --- work offsets -------------------------------------------------------------
    def work_offset_from_table_point(self, point_table) -> np.ndarray:
        """Haas-style G54 X/Y/Z (machine coords, zero-length tool) for a part zero given as a
        point in the table frame -- e.g. [0, 0, -50.8] is the platter centre, top face."""
        return self.linear_for_tip(np.asarray(point_table, dtype=float), 0.0)

    def table_point_from_work_offset(self, wo_xyz) -> np.ndarray:
        """Inverse of `work_offset_from_table_point` (valid for B, C at the CAD pose)."""
        return self.gauge + self.A @ (np.asarray(wo_xyz, dtype=float) - self.q0[:3])

    # --- limits -------------------------------------------------------------------
    def limit_violations(self, q) -> list[str]:
        out = []
        for i, j in enumerate(self.joints):
            if j.limits is None:
                continue
            lo, hi = j.limits
            if q[i] < lo - 1e-6 or q[i] > hi + 1e-6:
                out.append(f"{j.name}={q[i]:.3f} outside [{lo}, {hi}]")
        return out
