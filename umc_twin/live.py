"""Live machine state -- the seam where real machine data will plug in.

The viewer subscribes to `/api/live` (server-sent events) and poses the model from whatever
`LiveSource` the server was started with:

- `MTConnectSource` (mtconnect.py) -- the real machine, through an MTConnect agent;
- `ReplaySource` -- a simulated program played back in real time, as if it were the machine;
- `LogReplaySource` -- a recording made with `JsonlLogger` / `umc-twin record`.

Every source returns the same `MachineState`, so nothing downstream cares which one it is.
"""
from __future__ import annotations

import json
import threading
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
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
    spindle_load: float | None = None  # % as the control reports it
    program: str | None = None
    execution: str | None = None      # ACTIVE, READY, STOPPED, INTERRUPTED, ...

    def as_dict(self) -> dict:
        d = asdict(self)
        d["q"] = [round(v, 4) for v in self.q]
        return d


class LiveSource(Protocol):
    name: str

    def read(self) -> MachineState | None:
        """Latest machine state, or None if nothing is available right now."""


class JsonlLogger:
    """Wraps a LiveSource and appends every new state to a JSON-lines file (one object per line)."""

    def __init__(self, source: LiveSource, path, min_interval: float = 0.05):
        self.source, self.name = source, source.name
        self.path = Path(path)
        self.min_interval = min_interval
        self._last = 0.0
        self._last_ts = None
        self._lock = threading.Lock()

    def read(self) -> MachineState | None:
        st = self.source.read()
        now = time.monotonic()
        if st is not None and st.timestamp != self._last_ts and now - self._last >= self.min_interval:
            with self._lock, self.path.open("a") as f:
                f.write(json.dumps(st.as_dict()) + "\n")
            self._last, self._last_ts = now, st.timestamp
        return st


class LogReplaySource:
    """Plays back a JSON-lines recording (from JsonlLogger / `umc-twin record`) in real time."""

    name = "recording"

    def __init__(self, path, speed: float = 1.0, loop: bool = True):
        self.states = [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]
        if not self.states:
            raise ValueError(f"{path} has no states")
        t0 = self.states[0]["timestamp"]
        self.t = np.array([s["timestamp"] - t0 for s in self.states])
        self.speed, self.loop = speed, loop
        self.start = time.monotonic()

    def read(self) -> MachineState:
        now = (time.monotonic() - self.start) * self.speed
        dur = float(self.t[-1])
        if dur > 0:
            now = now % dur if self.loop else min(now, dur)
        k = int(np.clip(np.searchsorted(self.t, now, side="right") - 1, 0, len(self.t) - 1))
        d = dict(self.states[k])
        d["source"] = self.name
        d["timestamp"] = time.time()
        known = {f for f in MachineState.__dataclass_fields__}
        return MachineState(**{k2: v for k2, v in d.items() if k2 in known})


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
