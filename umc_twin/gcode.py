"""Haas-flavoured G-code interpreter -> time-stamped machine trajectory.

Supported
  motion      G0 G1 G2 G3 (G17/G18/G19, IJK or R, helical), G4 dwell
  modes       G20/G21, G90/G91, G93/G94, G54-G59, G154 Pn, G43/G49 (H), G53, G28
  5-axis      G234 (TCPC) and G254 (DWO) -- XYZ are programmed in the part frame and the
              control compensates for B/C; G49 cancels G234, G255 cancels G254
  cycles      G73 G81 G82 G83 G84 G85 G86 G89 with G98/G99, L repeats, G80 cancel
  M-codes     M6 (with T), M3/M4/M5 (S), M8/M9, M0/M1 (recorded), M2/M30 end

Anything else is ignored with a warning, so the sim never silently pretends to understand
a code. Moves are first timed from feed / rapid rate alone; `simulate()` then re-times them
with acceleration and cornering limits (timing.py).
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass, field

import numpy as np

from .job import JobSetup, _normalise_offset_name
from .kinematics import Kinematics

WORD_RE = re.compile(r"([A-Z])\s*([-+]?(?:\d+\.?\d*|\.\d+))")
MOTION = {"rapid": 0, "feed": 1, "arc": 2, "dwell": 3, "toolchange": 4, "home": 5}
CYCLES = {73, 81, 82, 83, 84, 85, 86, 89}
G73_RETRACT = 1.27   # mm, Haas setting 22 default (0.050 in)
G83_CLEARANCE = 1.27  # mm, Haas setting 52-ish: rapid back down to just above the last peck


class GCodeError(Exception):
    pass


@dataclass
class Trajectory:
    """Samples of the machine state. Sample k is reached at t[k]; the motion from k-1 to k
    belongs to line[k] with type motion[k]."""
    t: list[float] = field(default_factory=list)
    q: list[np.ndarray] = field(default_factory=list)
    tip: list[np.ndarray] = field(default_factory=list)   # tool tip in the table frame
    line: list[int] = field(default_factory=list)
    motion: list[int] = field(default_factory=list)
    tool: list[int] = field(default_factory=list)
    spindle: list[float] = field(default_factory=list)
    events: list[dict] = field(default_factory=list)
    warnings: list[dict] = field(default_factory=list)
    t_ideal: list[float] | None = None                    # feed-rate-only times, before re-timing

    @property
    def duration(self) -> float:
        return self.t[-1] if self.t else 0.0

    def arrays(self):
        return np.array(self.t), np.array(self.q), np.array(self.tip)


@dataclass
class _State:
    q: np.ndarray                  # machine joints (truth)
    p: np.ndarray                  # program position X Y Z B C (mm/deg) in the active frame
    units: float = 1.0             # 25.4 in G20
    absolute: bool = True
    plane: int = 17
    inverse_time: bool = False
    motion: int = 0                # modal G0/G1/G2/G3 or cycle number
    feed: float = 0.0              # mm/min (G94) or 1/min (G93)
    work: str = "G54"
    tlo: bool = False
    h: int = 0
    tcp: bool = False              # G234 or G254 active
    tool: int = 0
    next_tool: int | None = None
    spindle: float = 0.0
    spindle_speed: float = 0.0
    retract_initial: bool = True   # G98 (True) / G99
    cycle_r: float | None = None
    cycle_z: float | None = None
    cycle_q: float | None = None
    cycle_p: float = 0.0
    cycle_initial_z: float | None = None


class Interpreter:
    def __init__(self, kin: Kinematics, setup: JobSetup, start: np.ndarray | None = None,
                 arc_tolerance: float = 0.01, rotary_step: float = 0.5):
        self.kin = kin
        self.setup = setup
        self.arc_tol = arc_tolerance
        self.rot_step = rotary_step
        self.vmax = np.array([j.max_velocity for j in kin.joints])  # per minute
        q = np.zeros(5) if start is None else np.asarray(start, dtype=float)
        self.s = _State(q=q.copy(), p=np.zeros(5))
        self.s.p = self._prog_from_q(q)
        self.traj = Trajectory()
        self._record(0, MOTION["rapid"], 0.0)
        self._warned: set[str] = set()
        self._decimal: set[str] = set()
        self.line_no = 0

    # ------------------------------------------------------------------ frames
    def _tool_length(self) -> float:
        return self.setup.tool(self.s.h).length if self.s.tlo else 0.0

    def _wo(self) -> np.ndarray:
        name = self.s.work
        if name not in self.setup.work_offsets:
            self._warn(f"work offset {name} not in setup; using zeros")
            self.setup.work_offsets[name] = np.zeros(5)
        return self.setup.work_offsets[name]

    def _q_from_prog(self, p: np.ndarray) -> np.ndarray:
        wo, L = self._wo(), self._tool_length()
        b, c = p[3] + wo[3], p[4] + wo[4]
        if self.s.tcp:
            origin = self.kin.table_point_from_work_offset(wo[:3])
            return self.kin.inverse_tcp(origin + p[:3], b, c, L)
        q = np.empty(5)
        q[:3] = p[:3] + wo[:3] + self.kin.A_inv @ (-self.kin.tool_dir * L)
        q[3:] = b, c
        return q

    def _prog_from_q(self, q: np.ndarray) -> np.ndarray:
        wo, L = self._wo(), self._tool_length()
        p = np.empty(5)
        p[3:] = q[3:] - wo[3:]
        if self.s.tcp:
            origin = self.kin.table_point_from_work_offset(wo[:3])
            p[:3] = self.kin.tool_tip_in_table(q, L) - origin
        else:
            p[:3] = q[:3] - wo[:3] - self.kin.A_inv @ (-self.kin.tool_dir * L)
        return p

    def _resync(self):
        """Mode change (offset, TLO, TCPC): machine stays put, program position is re-derived."""
        self.s.p = self._prog_from_q(self.s.q)

    # ------------------------------------------------------------------ recording
    def _record(self, line: int, motion: int, dt: float):
        tr = self.traj
        tr.t.append((tr.t[-1] if tr.t else 0.0) + dt)
        tr.q.append(self.s.q.copy())
        tr.tip.append(self.kin.tool_tip_in_table(self.s.q, self.setup.tool(self.s.tool).length))
        tr.line.append(line)
        tr.motion.append(motion)
        tr.tool.append(self.s.tool)
        tr.spindle.append(self.s.spindle)

    def _warn(self, msg: str, once_key: str | None = None):
        key = once_key or msg
        if key in self._warned:
            return
        self._warned.add(key)
        self.traj.warnings.append({"line": self.line_no, "message": msg})

    def _event(self, kind: str, text: str):
        self.traj.events.append({"t": self.traj.duration, "line": self.line_no, "type": kind, "text": text})

    # ------------------------------------------------------------------ motion primitives
    def _min_time(self, qs: list[np.ndarray]) -> list[float]:
        """Per-step minimum time (s) from axis velocity limits."""
        out, prev = [], self.s.q
        for q in qs:
            out.append(float(np.max(np.abs(q - prev) / self.vmax)) * 60.0)
            prev = q
        return out

    def _follow(self, ps: list[np.ndarray], rapid: bool, motion: int | None = None,
                machine_qs: list[np.ndarray] | None = None):
        """Move through program-space points `ps` (or explicit machine joints)."""
        qs = machine_qs if machine_qs is not None else [self._q_from_prog(p) for p in ps]
        tmin = self._min_time(qs)
        if rapid:
            steps = tmin
        else:
            if self.s.feed <= 0:
                raise GCodeError(f"line {self.line_no}: feed move with no F word")
            if self.s.inverse_time:
                total = 60.0 / self.s.feed
                w = np.array(tmin) if sum(tmin) > 0 else np.ones(len(qs))
                steps = list(total * w / w.sum())
            else:
                prev_p = self.s.p if machine_qs is None else None
                steps = []
                for k, q in enumerate(qs):
                    if prev_p is not None:
                        lin = float(np.linalg.norm(ps[k][:3] - prev_p[:3]))
                        rot = float(np.max(np.abs(ps[k][3:] - prev_p[3:])))
                        prev_p = ps[k]
                    else:
                        lin, rot = 0.0, 0.0
                    dist = lin if lin > 1e-9 else rot
                    feed = min(self.s.feed, self.kin.m.max_cutting_feed) if lin > 1e-9 else self.s.feed
                    steps.append(max(dist / feed * 60.0, tmin[k]))
        for q, dt in zip(qs, steps):
            self.s.q = q
            self._record(self.line_no, motion if motion is not None else (0 if rapid else 1), dt)
        if machine_qs is None:
            self.s.p = ps[-1].copy()
        else:
            self._resync()

    def _linear(self, target: np.ndarray, rapid: bool):
        p0 = self.s.p
        rot_delta = float(np.max(np.abs(target[3:] - p0[3:])))
        n = 1
        if self.s.tcp and rot_delta > 0:
            n = max(1, math.ceil(rot_delta / self.rot_step))
        ps = [p0 + (target - p0) * (k / n) for k in range(1, n + 1)]
        self._follow(ps, rapid)

    def _arc(self, target: np.ndarray, words: dict, clockwise: bool):
        u, v, w = {17: (0, 1, 2), 18: (2, 0, 1), 19: (1, 2, 0)}[self.s.plane]
        p0 = self.s.p
        start = np.array([p0[u], p0[v]])
        end = np.array([target[u], target[v]])
        if "R" in words:
            r = words["R"] * self.s.units
            chord = end - start
            d = np.linalg.norm(chord)
            if d < 1e-9:
                raise GCodeError(f"line {self.line_no}: R arc with coincident start/end")
            if d > 2 * abs(r) + 1e-6:
                raise GCodeError(f"line {self.line_no}: arc radius {abs(r):.4f} too small for chord {d:.4f}")
            h = math.sqrt(max(r * r - d * d / 4, 0.0))
            mid = (start + end) / 2
            perp = np.array([-chord[1], chord[0]]) / d
            # CW with positive R -> centre on the right of the chord
            sign = -1 if clockwise else 1
            if r < 0:
                sign = -sign
            centre = mid + sign * h * perp
        else:
            ijk = [words.get(k, 0.0) * self.s.units for k in "IJK"]
            centre = start + np.array([ijk[u], ijk[v]])
        r0 = np.linalg.norm(start - centre)
        r1 = np.linalg.norm(end - centre)
        if abs(r0 - r1) > 0.01 + 1e-3 * r0:
            self._warn(f"arc end radius differs from start radius by {abs(r0 - r1):.4f} mm")
        a0 = math.atan2(*(start - centre)[::-1])
        a1 = math.atan2(*(end - centre)[::-1])
        sweep = a1 - a0
        if clockwise:
            if sweep >= -1e-9:
                sweep -= 2 * math.pi
        elif sweep <= 1e-9:
            sweep += 2 * math.pi
        radius = max(r0, 1e-6)
        dtheta = 2 * math.acos(max(-1.0, 1 - self.arc_tol / radius)) if radius > self.arc_tol else math.pi / 2
        n = max(4, math.ceil(abs(sweep) / max(dtheta, 1e-3)))
        ps = []
        for k in range(1, n + 1):
            f = k / n
            a = a0 + sweep * f
            p = p0 + (target - p0) * f  # helical / rotary components interpolate linearly
            p[u] = centre[0] + radius * math.cos(a)
            p[v] = centre[1] + radius * math.sin(a)
            ps.append(p)
        ps[-1] = target.copy()
        self._follow(ps, rapid=False, motion=MOTION["arc"])

    def _machine_move(self, q_target: np.ndarray, motion: int = MOTION["rapid"]):
        if np.allclose(q_target, self.s.q, atol=1e-9):
            return
        self._follow([], rapid=True, motion=motion, machine_qs=[np.asarray(q_target, dtype=float)])

    def _seconds(self, p: float) -> float:
        """Haas P dwell: seconds when written with a decimal point, milliseconds otherwise."""
        return p if "P" in self._decimal else p / 1000.0

    def _dwell(self, seconds: float, kind: int = MOTION["dwell"]):
        self._record(self.line_no, kind, max(seconds, 0.0))

    # ------------------------------------------------------------------ blocks
    def run(self, text: str) -> Trajectory:
        for i, raw in enumerate(text.splitlines(), start=1):
            self.line_no = i
            if self._block(raw) == "end":
                break
        return self.traj

    def _parse(self, raw: str) -> tuple[list[float], list[float], dict]:
        line = re.sub(r"\(.*?\)", " ", raw.upper())
        line = line.split(";", 1)[0].strip()
        if not line or line.startswith("%") or line.startswith("/"):
            return [], [], {}
        if "#" in line or "[" in line:
            self._warn("macro variables / expressions are not supported; block ignored", "macro")
            return [], [], {}
        gs, ms, words = [], [], {}
        self._decimal = {letter for letter, value in WORD_RE.findall(line) if "." in value}
        rest = WORD_RE.sub(" ", line).strip()
        if rest:
            self._warn(f"could not parse {rest!r}")
        for letter, value in WORD_RE.findall(line):
            val = float(value)
            if letter == "G":
                gs.append(round(val, 1))
            elif letter == "M":
                ms.append(int(val))
            elif letter in words and letter not in "NO":
                self._warn(f"duplicate {letter} word; using the last one")
                words[letter] = val
            else:
                words[letter] = val
        return gs, ms, words

    def _block(self, raw: str):
        gs, ms, w = self._parse(raw)
        if not gs and not ms and not w:
            return None
        s = self.s
        mode_changed = False

        # --- modal settings -------------------------------------------------------
        for g in gs:
            if g in (20, 21):
                s.units = 25.4 if g == 20 else 1.0
            elif g in (90, 91):
                s.absolute = g == 90
            elif g in (17, 18, 19):
                s.plane = int(g)
            elif g in (93, 94):
                s.inverse_time = g == 93
            elif g in (98, 99):
                s.retract_initial = g == 98
            elif 54 <= g <= 59 or g == 154:
                name = f"G{int(g)}" if g != 154 else f"G154P{int(w.get('P', 1))}"
                if name != s.work:
                    s.work = _normalise_offset_name(name)
                    mode_changed = True
        if "F" in w:
            s.feed = w["F"] * (1.0 if s.inverse_time else s.units)
        if "S" in w:
            s.spindle_speed = w["S"]
            if s.spindle:
                s.spindle = math.copysign(s.spindle_speed, s.spindle)
        if "T" in w:
            s.next_tool = int(w["T"])

        # --- tool change / spindle / coolant ------------------------------------------
        if 6 in ms:
            self._tool_change()
        for m in ms:
            if m in (3, 4):
                s.spindle = s.spindle_speed if m == 3 else -s.spindle_speed
                self._event("spindle", f"M{m} S{s.spindle_speed:g}")
            elif m == 5:
                s.spindle = 0.0
                self._event("spindle", "M5")
            elif m in (8, 9):
                self._event("coolant", f"M{m}")
            elif m in (0, 1):
                self._event("stop", f"M{m:02d} program stop")
            elif m in (2, 30):
                self._event("end", f"M{m}")
                return "end"
            elif m != 6:
                self._warn(f"M{m} ignored", f"M{m}")

        # --- length comp / 5-axis modes -------------------------------------------------
        for g in gs:
            if g == 43:
                s.tlo, s.h = True, int(w.get("H", s.tool))
                mode_changed = True
            elif g == 234:
                s.tlo, s.tcp = True, True
                s.h = int(w.get("H", s.h or s.tool))
                mode_changed = True
            elif g == 254:
                s.tcp = True
                mode_changed = True
            elif g == 49:
                s.tlo, s.tcp = False, False
                mode_changed = True
            elif g == 255:
                s.tcp = False
                mode_changed = True
        if mode_changed:
            self._resync()

        # --- non-modal ------------------------------------------------------------------
        axes = {a: w[a] for a in "XYZBC" if a in w}
        if "A" in w:
            self._warn("A axis word ignored (UMC-500 has B/C only)", "A")
        for g in gs:
            if g == 4:
                self._dwell(self._seconds(w.get("P", 0.0)))
                return None
            if g == 53:
                q = s.q.copy()
                for a, v in axes.items():
                    i = "XYZBC".index(a)
                    q[i] = v * (s.units if i < 3 else 1.0)
                self._machine_move(q)
                return None
            if g == 28:
                self._home(axes)
                return None
            if g in (10, 41, 42, 40, 68, 69, 51, 50, 187, 103, 65, 47, 150):
                if g not in (40, 69, 50, 187):
                    self._warn(f"G{g:g} ignored", f"G{g:g}")
                if g == 10:
                    return None

        # --- motion ---------------------------------------------------------------------
        for g in gs:
            if g in (0, 1, 2, 3, 80) or g in CYCLES:
                s.motion = int(g)
            elif g not in (4, 10, 17, 18, 19, 20, 21, 28, 40, 41, 42, 43, 47, 49, 50, 51, 53, 54, 55, 56, 57, 58,
                           59, 65, 68, 69, 80, 90, 91, 93, 94, 98, 99, 103, 150, 154, 187, 234, 254, 255):
                self._warn(f"G{g:g} not supported; ignored", f"G{g:g}")
        if s.motion in CYCLES:
            if any(g in CYCLES for g in gs):
                s.cycle_initial_z = s.p[2]
                s.cycle_q = w["Q"] * s.units if "Q" in w else s.cycle_q
                s.cycle_p = self._seconds(w["P"]) if "P" in w else s.cycle_p
                if "R" in w:
                    s.cycle_r = w["R"] * s.units
                if "Z" in w:
                    s.cycle_z = w["Z"] * s.units
            elif "Q" in w:
                s.cycle_q = w["Q"] * s.units
            if "Z" in w and not any(g in CYCLES for g in gs):
                s.cycle_z = w["Z"] * s.units
            if "R" in w and not any(g in CYCLES for g in gs):
                s.cycle_r = w["R"] * s.units
            if any(a in axes for a in "XYBC") or any(g in CYCLES for g in gs):
                self._cycle(axes, int(w.get("L", 1)))
            return None
        if not axes:
            return None
        target = self._target(axes)
        if s.motion == 0:
            self._linear(target, rapid=True)
        elif s.motion == 1:
            self._linear(target, rapid=False)
        elif s.motion in (2, 3):
            self._arc(target, w, clockwise=s.motion == 2)
        return None

    def _target(self, axes: dict) -> np.ndarray:
        s = self.s
        t = s.p.copy()
        for a, v in axes.items():
            i = "XYZBC".index(a)
            val = v * (s.units if i < 3 else 1.0)
            t[i] = val if s.absolute else t[i] + val
        return t

    # ------------------------------------------------------------------ compound ops
    def _home(self, axes: dict):
        """G28: via the (optional) intermediate point, then the named axes (or all) to zero."""
        if axes:
            inter = self._target(axes)
            if np.any(np.abs(inter - self.s.p) > 1e-9):
                self._linear(inter, rapid=True)
            idx = ["XYZBC".index(a) for a in axes]
        else:
            idx = [0, 1, 2, 3, 4]
        q = self.s.q.copy()
        if 2 in idx:  # Z first, like the control
            q[2] = 0.0
            self._machine_move(q, MOTION["home"])
        for i in idx:
            q[i] = 0.0
        self._machine_move(q, MOTION["home"])

    def _tool_change(self):
        s = self.s
        if s.next_tool is None:
            self._warn("M6 without a T word")
            return
        pos = self.kin.m.tool_change["position"]
        q = s.q.copy()
        if "Z" in pos:
            q[2] = pos["Z"]
            self._machine_move(q, MOTION["toolchange"])
        for a in "XY":
            if a in pos:
                q["XY".index(a)] = pos[a]
        self._machine_move(q, MOTION["toolchange"])
        if s.next_tool not in self.setup.tools:
            self._warn(f"T{s.next_tool} not in the tool table; using default length "
                       f"{self.setup.default_tool.length:g} mm", f"T{s.next_tool}")
        s.tool = s.next_tool
        s.spindle = 0.0
        self._event("toolchange", f"T{s.tool} {self.setup.tool(s.tool).name}".strip())
        self._dwell(float(self.kin.m.tool_change["time"]), MOTION["toolchange"])
        self._resync()

    def _cycle(self, axes: dict, repeats: int):
        s = self.s
        if s.cycle_r is None or s.cycle_z is None:
            raise GCodeError(f"line {self.line_no}: canned cycle needs R and Z")
        for _ in range(max(repeats, 1)):
            xy = {a: v for a, v in axes.items() if a in "XYBC"}
            target = self._target(xy) if xy else s.p.copy()
            initial = s.cycle_initial_z if s.cycle_initial_z is not None else s.p[2]
            if s.absolute:
                r_abs, z_abs = s.cycle_r, s.cycle_z
            else:  # G91: R is relative to the initial level, Z relative to R
                r_abs = initial + s.cycle_r
                z_abs = r_abs + s.cycle_z
            target[2] = s.p[2]
            self._linear(target, rapid=True)
            self._goto_z(r_abs, rapid=True)
            code = s.motion
            if code in (83, 73) and s.cycle_q:
                depth = r_abs
                while depth > z_abs + 1e-9:
                    nxt = max(depth - s.cycle_q, z_abs)
                    if code == 83 and depth < r_abs:
                        self._goto_z(depth + G83_CLEARANCE, rapid=True)
                    self._goto_z(nxt, rapid=False)
                    depth = nxt
                    if depth > z_abs + 1e-9:
                        self._goto_z(r_abs if code == 83 else depth + G73_RETRACT, rapid=True)
            else:
                self._goto_z(z_abs, rapid=False)
            if code in (82, 89) and s.cycle_p:
                self._dwell(s.cycle_p)
            retract = initial if s.retract_initial else r_abs
            if code in (84, 85, 89):
                self._goto_z(r_abs, rapid=False)
                if retract != r_abs:
                    self._goto_z(retract, rapid=True)
            else:
                self._goto_z(retract, rapid=True)
            if s.absolute:
                break  # L repeats only make sense in G91

    def _goto_z(self, z: float, rapid: bool):
        t = self.s.p.copy()
        t[2] = z
        if abs(t[2] - self.s.p[2]) > 1e-9:
            self._linear(t, rapid=rapid)


def simulate(text: str, kin: Kinematics, setup: JobSetup, accel: bool = True, **kw) -> Trajectory:
    """Interpret `text`. With accel=True (default) the times include acceleration and cornering
    (see timing.py); the feed-rate-only times are kept in `traj.t_ideal`."""
    traj = Interpreter(kin, setup, **kw).run(text)
    if accel:
        from .timing import apply

        apply(traj, kin.m)
    return traj
