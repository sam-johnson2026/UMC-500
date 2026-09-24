"""Live machine state -- the seam where real machine data will plug in.

The viewer subscribes to `/api/live` (server-sent events) and poses the model from whatever
`LiveSource` the server was started with. Today that's `ReplaySource`, which plays a simulated
program back in real time as if it were a running machine. When the machine is connected,
add a source (e.g. an MTConnect agent poller -- Haas NGC controls can serve MTConnect) that
returns the same `MachineState`, and nothing downstream has to change.
"""
from __future__ import annotations

import time
from dataclasses import asdict, dataclass, field
from typing import Protocol

import numpy as np

from .gcode import Trajectory


@dataclass
class MachineState:
    q: list[float]                    # X Y Z B C, machine coordinates (mm, deg)
    source: str
    timestamp: float = field(default_factory=time.time)
    line: int | None = None           # program line being executed, if known
    tool: int | None = None
    spindle_rpm: float | None = None
    mode: str | None = None           # e.g. "AUTOMATIC", "MANUAL", "SIM"

    def as_dict(self) -> dict:
        d = asdict(self)
        d["q"] = [round(v, 4) for v in self.q]
        return d


class LiveSource(Protocol):
    name: str

    def read(self) -> MachineState | None:
        """Latest machine state, or None if nothing is available right now."""


class ReplaySource:
    """Plays a simulated trajectory in wall-clock time (looping) -- a stand-in for a live machine."""

    name = "replay"

    def __init__(self, traj: Trajectory, speed: float = 1.0, loop: bool = True):
        self.t, self.q, _ = traj.arrays()
        self.traj = traj
        self.speed, self.loop = speed, loop
        self.start = time.monotonic()

    def read(self) -> MachineState:
        now = (time.monotonic() - self.start) * self.speed
        dur = float(self.t[-1]) if len(self.t) else 0.0
        if dur > 0:
            now = now % dur if self.loop else min(now, dur)
        k = int(np.searchsorted(self.t, now, side="right"))
        k = min(max(k, 1), len(self.t) - 1)
        span = self.t[k] - self.t[k - 1]
        f = 0.0 if span <= 0 else (now - self.t[k - 1]) / span
        q = self.q[k - 1] + (self.q[k] - self.q[k - 1]) * f
        return MachineState(q=q.tolist(), source=self.name, line=self.traj.line[k], tool=self.traj.tool[k],
                            spindle_rpm=self.traj.spindle[k], mode="SIM")
