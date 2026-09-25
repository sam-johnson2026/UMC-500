"""Haas-flavoured G-code interpreter -> time-stamped machine trajectory.

Supported
  motion      G0 G1 G2 G3 (G17/G18/G19, IJK or R, helical), G4 dwell
  modes       G20/G21, G90/G91, G93/G94, G54-G59, G154 Pn, G43/G49 (H), G53, G28
  5-axis      G234 (TCPC) and G254 (DWO) -- XYZ are programmed in the part frame and the
              control compensates for B/C; G49 cancels G234, G255 cancels G254
  cycles      G73 G81 G82 G83 G84 G85 G86 G89 with G98/G99, L repeats, G80 cancel
  cutter comp G41/G42 with D (G17 plane), G40 cancel -- see _comp_* below
  rotation    G68 X Y R / G69
  offsets     G10 L2/L20 (work offsets), L10-L13 (tool length / diameter, geometry and wear)
  programs    M97 P (local N label), M98 P / M98 "file" (O-number in this file or a file next to
              it), M99 (and M99 P), G65 P macro calls with arguments, L repeats
  macros      #variables, expressions, IF / GOTO / WHILE (see macro.py) and system variables
              for positions, work offsets and tool offsets
  M-codes     M6 (with T), M3/M4/M5 (S), M8/M9, M0/M1 (recorded), M2/M30 end

Anything else is ignored with a warning, so the sim never silently pretends to understand
a code. Moves are first timed from feed / rapid rate alone; `simulate()` then re-times them
with acceleration and cornering limits (timing.py).

G10 and macro writes change the job setup's offsets in place, like the control's offset pages.
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from . import macro
from .job import JobSetup, _normalise_offset_name
from .kinematics import Kinematics

WORD_RE = re.compile(r"([A-Z])\s*([-+]?(?:\d+\.?\d*|\.\d+))")
MOTION = {"rapid": 0, "feed": 1, "arc": 2, "dwell": 3, "toolchange": 4, "home": 5}
CYCLES = {73, 81, 82, 83, 84, 85, 86, 89}
G73_RETRACT = 1.27   # mm, Haas setting 22 default (0.050 in)
G83_CLEARANCE = 1.27  # mm, Haas setting 52-ish: rapid back down to just above the last peck
MAX_BLOCKS = 2_000_000  # a runaway macro loop stops here
PROGRAM_SUFFIXES = ("", ".nc", ".NC", ".ngc", ".tap", ".txt")
KNOWN_G = {0, 1, 2, 3, 4, 10, 17, 18, 19, 20, 21, 28, 40, 41, 42, 43, 47, 49, 50, 51, 53, 54, 55, 56, 57, 58,
           59, 65, 68, 69, 80, 90, 91, 93, 94, 98, 99, 103, 150, 154, 187, 234, 254, 255} | CYCLES
WORK_VARS = {5221: "G54", 5241: "G55", 5261: "G56", 5281: "G57", 5301: "G58", 5321: "G59"}
VAR_AXIS = {0: 0, 1: 1, 2: 2, 4: 3, 5: 4}   # offset within a Haas variable block (X Y Z A B C) -> our X Y Z B C


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
    d: int = 0
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
    comp: int = 40                 # 40 / 41 / 42
    rotation: tuple[float, float, float] | None = None   # G68: centre x, y, angle (deg)


@dataclass
class _Program:
    name: str
    raw: list[str]
    clean: list[str]
    main: bool
    labels: dict[int, int] = field(default_factory=dict)   # N number -> line index
    onums: dict[int, int] = field(default_factory=dict)    # O number -> line index
    loops: dict[int, int] = field(default_factory=dict)    # WHILE <-> END line indexes

    @classmethod
    def parse(cls, name: str, text: str, main: bool) -> "_Program":
        raw = text.splitlines()
        clean = [_clean(r) for r in raw]
        p = cls(name, raw, clean, main)
        stack: dict[int, list[int]] = {}
        for i, c in enumerate(clean):
            m = re.match(r"^N\s*(\d+)", c)
            if m:
                p.labels.setdefault(int(m.group(1)), i)
            m = re.match(r"^[O:]\s*(\d+)", c)
            if m:
                p.onums.setdefault(int(m.group(1)), i)
            body = _strip_n(c)
            m = macro.WHILE_RE.match(body)
            if m:
                stack.setdefault(int(m.group(2)), []).append(i)
            m = macro.END_RE.match(body)
            if m and stack.get(int(m.group(1))):
                w = stack[int(m.group(1))].pop()
                p.loops[w], p.loops[i] = i, w
        return p


@dataclass
class _Frame:
    prog: _Program
    pc: int
    start_pc: int = 0
    repeats: int = 1
    call_line: int = 0             # display line of the caller (for code in other files)
    macro_call: bool = False


def _clean(raw: str) -> str:
    line = re.sub(r"\(.*?\)", " ", raw.upper())
    return line.split(";", 1)[0].strip()


def _strip_n(clean: str) -> str:
    return re.sub(r"^N\s*\d+\s*", "", clean)


class Interpreter:
    def __init__(self, kin: Kinematics, setup: JobSetup, start: np.ndarray | None = None,
                 arc_tolerance: float = 0.01, rotary_step: float = 0.5,
                 search_paths: list[str | Path] | None = None):
        self.kin = kin
        self.setup = setup
        self.arc_tol = arc_tolerance
        self.rot_step = rotary_step
        self.search_paths = [Path(p) for p in (search_paths or [])]
        self.vmax = np.array([j.max_velocity for j in kin.joints])  # per minute
        q = np.zeros(5) if start is None else np.asarray(start, dtype=float)
        self.s = _State(q=q.copy(), p=np.zeros(5))
        self.s.p = self._prog_from_q(q)
        self.traj = Trajectory()
        self._record(0, MOTION["rapid"], 0.0)
        self._warned: set[str] = set()
        self._decimal: set[str] = set()
        self.line_no = 0
        self.vars = macro.Variables(self._sysvar_get, self._sysvar_set)
        self.frames: list[_Frame] = []
        self._raw_line = ""
        # cutter compensation
        self.prog_pos = self.s.p.copy()    # programmed (uncompensated) position while comp is on
        self._pending: dict | None = None  # the compensated move waiting for the next one
        self._deferred: list = []          # non-XY moves queued behind it
        self._comp_started = False
        self._alarm = False

    # ------------------------------------------------------------------ frames
    def _tool_length(self) -> float:
        return self.setup.tool(self.s.h).length if self.s.tlo else 0.0

    def _wo(self) -> np.ndarray:
        name = self.s.work
        if name not in self.setup.work_offsets:
            self._warn(f"work offset {name} not in setup; using zeros")
            self.setup.work_offsets[name] = np.zeros(5)
        return self.setup.work_offsets[name]

    def _rotate(self, xy, inverse: bool = False):
        cx, cy, ang = self.s.rotation
        a = math.radians(-ang if inverse else ang)
        dx, dy = xy[0] - cx, xy[1] - cy
        return cx + dx * math.cos(a) - dy * math.sin(a), cy + dx * math.sin(a) + dy * math.cos(a)

    def _q_from_prog(self, p: np.ndarray) -> np.ndarray:
        if self.s.rotation is not None:
            p = p.copy()
            p[0], p[1] = self._rotate(p[:2])
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
        if self.s.rotation is not None:
            p[0], p[1] = self._rotate(p[:2], inverse=True)
        return p

    def _resync(self):
        """Mode change (offset, TLO, TCPC): machine stays put, program position is re-derived.
        With cutter comp on, the programmed (uncompensated) position moves by the same amount."""
        new = self._prog_from_q(self.s.q)
        if getattr(self, "_pending", None) is None and self.s.comp == 40:
            self.prog_pos = new.copy()
        else:
            self.prog_pos = self.prog_pos + (new - self.s.p)
        self.s.p = new

    # ------------------------------------------------------------------ recording
    def _record(self, line: int, motion: int, dt: float):
        tr = self.traj
        tr.t.append((tr.t[-1] if tr.t else 0.0) + dt)
        tr.q.append(self.s.q.copy())  # tool tips are filled in, batched, at the end of run()
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
        if self.s.comp == 40 and self._pending is None:
            self.prog_pos = self.s.p.copy()

    def _linear(self, target: np.ndarray, rapid: bool):
        p0 = self.s.p
        rot_delta = float(np.max(np.abs(target[3:] - p0[3:])))
        n = 1
        if self.s.tcp and rot_delta > 0:
            n = max(1, math.ceil(rot_delta / self.rot_step))
        ps = [p0 + (target - p0) * (k / n) for k in range(1, n + 1)]
        self._follow(ps, rapid)

    def _arc_centre(self, p0: np.ndarray, target: np.ndarray, words: dict, clockwise: bool):
        u, v, _ = {17: (0, 1, 2), 18: (2, 0, 1), 19: (1, 2, 0)}[self.s.plane]
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
            return mid + sign * h * perp
        ijk = [words.get(k, 0.0) * self.s.units for k in "IJK"]
        return start + np.array([ijk[u], ijk[v]])

    def _arc(self, target: np.ndarray, words: dict, clockwise: bool):
        self._arc_about(target, self._arc_centre(self.s.p, target, words, clockwise), clockwise)

    def _arc_about(self, target: np.ndarray, centre: np.ndarray, clockwise: bool):
        u, v, _ = {17: (0, 1, 2), 18: (2, 0, 1), 19: (1, 2, 0)}[self.s.plane]
        p0 = self.s.p
        start = np.array([p0[u], p0[v]])
        end = np.array([target[u], target[v]])
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

    # ------------------------------------------------------------------ program flow
    def run(self, text: str, name: str = "main") -> Trajectory:
        main = _Program.parse(name, text, main=True)
        self.frames = [_Frame(main, 0)]
        blocks = 0
        while self.frames:
            fr = self.frames[-1]
            if fr.pc >= len(fr.prog.clean):
                if len(self.frames) == 1:
                    break
                self._return()
                continue
            idx = fr.pc
            fr.pc += 1
            blocks += 1
            if blocks > MAX_BLOCKS:
                raise GCodeError(f"line {self.line_no}: stopped after {MAX_BLOCKS} blocks -- endless loop?")
            self.line_no = idx + 1 if fr.prog.main else fr.call_line
            self._raw_line = fr.prog.raw[idx]
            try:
                action = self._execute(fr, idx)
            except macro.MacroError as e:
                raise GCodeError(f"line {self.line_no}: {e}") from None
            if action == "end":
                break
        self._comp_flush()
        self._fill_tips()
        return self.traj

    def _fill_tips(self):
        tr = self.traj
        q = np.asarray(tr.q)
        tools = np.asarray(tr.tool)
        tips = np.zeros((len(q), 3))
        for n in set(tools.tolist()):
            ks = np.nonzero(tools == n)[0]
            tips[ks], _ = self.kin.tip_and_axis_in_table_batch(q[ks], self.setup.tool(n).length)
        tr.tip = list(tips)

    def _execute(self, fr: _Frame, idx: int):
        line = fr.prog.clean[idx]
        if not line or line.startswith("%") or line.startswith("/"):
            return None
        body = _strip_n(line)
        if re.match(r"^[O:]\s*\d+", body):
            return None  # program header
        if body.startswith(("#", "IF", "GOTO", "WHILE", "END", "DO")):
            return self._statement(fr, idx, body)
        m98_file = re.search(r'M\s*98\s*"([^"]+)"', line)
        if m98_file:
            return self._call(self._find_program(m98_file.group(1)), 1, macro_args=None)
        if "#" in line or "[" in line:
            line = macro.substitute(line, self.vars)
        gs, ms, w = self._parse(line)
        if not gs and not ms and not w:
            return None
        action = self._block(gs, ms, w)
        if isinstance(action, tuple):
            kind = action[0]
            if kind == "local":
                _, label, repeats = action
                if label not in fr.prog.labels:
                    raise GCodeError(f"line {self.line_no}: M97 P{label}: no N{label} in this program")
                start = fr.prog.labels[label]   # the N line itself carries code
                self.frames.append(_Frame(fr.prog, start, start, repeats, self.line_no))
                return None
            if kind == "external":
                _, number, repeats = action
                return self._call(self._find_program(number), repeats, None)
            if kind == "macro":
                _, number, repeats, args = action
                return self._call(self._find_program(number), repeats, args)
            if kind == "return":
                if len(self.frames) == 1:
                    self._warn("M99 in the main program (the control would loop forever); stopping here")
                    return "end"
                ret_label = action[1]
                self._return()
                if ret_label is not None:
                    caller = self.frames[-1]
                    if ret_label not in caller.prog.labels:
                        raise GCodeError(f"line {self.line_no}: M99 P{ret_label}: no N{ret_label}")
                    caller.pc = caller.prog.labels[ret_label]
                return None
        return action

    def _statement(self, fr: _Frame, idx: int, body: str):
        v = self.vars
        if (m := macro.WHILE_RE.match(body)):
            if not macro.evaluate(m.group(1), v):
                fr.pc = fr.prog.loops.get(idx, idx) + 1
            return None
        if macro.END_RE.match(body):
            if idx not in fr.prog.loops:
                raise GCodeError(f"line {self.line_no}: END without WHILE")
            fr.pc = fr.prog.loops[idx]
            return None
        if (m := macro.IF_GOTO_RE.match(body)):
            if macro.evaluate(m.group(1), v):
                self._goto(fr, m.group(2))
            return None
        if (m := macro.IF_THEN_RE.match(body)):
            if macro.evaluate(m.group(1), v):
                macro.assign(m.group(2).strip(), v)
            return None
        if (m := macro.GOTO_RE.match(body)):
            self._goto(fr, m.group(1))
            return None
        if macro.assign(body, v):
            return "end" if self._alarm else None
        raise GCodeError(f"line {self.line_no}: can't read macro statement {body!r}")

    def _goto(self, fr: _Frame, target: str):
        n = int(macro._n(macro.evaluate(target, self.vars)))
        if n not in fr.prog.labels:
            raise GCodeError(f"line {self.line_no}: GOTO {n}: no N{n} in this program")
        fr.pc = fr.prog.labels[n]

    def _find_program(self, ref) -> tuple[_Program, int]:
        """(program, start index) for an O-number or file name: this file first, then files."""
        main = self.frames[0].prog
        if isinstance(ref, int) and ref in main.onums:
            return main, main.onums[ref] + 1
        names = [str(ref)] if not isinstance(ref, int) else [f"O{ref:05d}", f"O{ref}", str(ref)]
        dirs = self.search_paths or [Path(".")]
        for d in dirs:
            for n in names:
                for suffix in PROGRAM_SUFFIXES:
                    f = d / f"{n}{suffix}"
                    if f.is_file():
                        prog = _Program.parse(f.name, f.read_text(errors="replace"), main=False)
                        return prog, 0
        raise GCodeError(f"line {self.line_no}: subprogram {ref} not found (looked in this file and "
                         f"{', '.join(str(d) for d in dirs)})")

    def _call(self, found: tuple[_Program, int], repeats: int, macro_args: dict | None):
        prog, start = found
        if len(self.frames) > 20:
            raise GCodeError(f"line {self.line_no}: subprograms nested deeper than 20")
        if macro_args is not None:
            self.vars.push(macro_args)
        self.frames.append(_Frame(prog, start, start, max(repeats, 1), self.line_no, macro_args is not None))
        return None

    def _return(self):
        fr = self.frames.pop()
        if fr.repeats > 1:
            fr.repeats -= 1
            fr.pc = fr.start_pc
            self.frames.append(fr)
            return
        if fr.macro_call:
            self.vars.pop()

    # ------------------------------------------------------------------ system variables
    def _sysvar_get(self, n: int):
        s = self.s
        if 5021 <= n <= 5025:
            v = s.q[n - 5021]
            return v / s.units if n - 5021 < 3 else v
        if 5041 <= n <= 5045:
            v = self.prog_pos[n - 5041]
            return v / s.units if n - 5041 < 3 else v
        wo = self._work_var(n)
        if wo is not None:
            name, i = wo
            v = self.setup.work_offset(name)[i]
            return v / s.units if i < 3 else v
        tool = self._tool_var(n)
        if tool is not None:
            kind, t = tool
            if kind == "length":
                return t.length / s.units
            if kind == "diameter":
                return (2 * t.d_offset if t.d_offset is not None else t.diameter) / s.units
            return 0.0  # wear
        modal = {4001: s.motion if s.motion < 4 else s.motion, 4003: 90 if s.absolute else 91,
                 4006: 20 if s.units != 1.0 else 21, 4014: _offset_code(s.work), 4109: s.feed / s.units,
                 4111: s.h, 4107: s.d, 4119: s.spindle_speed, 4120: s.tool,
                 3001: self.traj.duration * 1000.0, 3002: self.traj.duration / 3600.0}
        if n in modal:
            return float(modal[n])
        self._warn(f"system variable #{n} not simulated; reads as vacant", f"#{n}")
        return None

    def _sysvar_set(self, n: int, v) -> bool:
        s = self.s
        val = macro._n(v)
        if n == 3000:
            self._alarm = True
            msg = re.search(r"\((.*?)\)", self._raw_line)
            self._event("alarm", f"#3000 alarm {int(val)}" + (f": {msg.group(1)}" if msg else ""))
            self._warn(f"program raised alarm #3000 = {int(val)}" + (f" ({msg.group(1)})" if msg else ""))
            return True
        if n == 3006:
            msg = re.search(r"\((.*?)\)", self._raw_line)
            self._event("stop", "#3006 stop" + (f": {msg.group(1)}" if msg else ""))
            return True
        wo = self._work_var(n)
        if wo is not None:
            name, i = wo
            off = self.setup.work_offsets.setdefault(name, np.zeros(5))
            off[i] = val * (s.units if i < 3 else 1.0)
            self._resync()
            return True
        tool = self._tool_var(n)
        if tool is not None:
            kind, t = tool
            if kind == "length":
                t.length = val * s.units
            elif kind == "diameter":
                t.d_offset = val * s.units / 2.0
            else:
                self._warn(f"#{n}: tool wear offsets are not simulated", f"#{n}")
            self._resync()
            return True
        return False

    @staticmethod
    def _work_var(n: int):
        for base, name in WORK_VARS.items():
            if base <= n <= base + 5 and (n - base) in VAR_AXIS:
                return name, VAR_AXIS[n - base]
        if 7001 <= n <= 8985 and (n - 7001) % 20 <= 5 and ((n - 7001) % 20) in VAR_AXIS:
            return f"G154P{(n - 7001) // 20 + 1}", VAR_AXIS[(n - 7001) % 20]
        return None

    def _tool_var(self, n: int):
        for base, kind in ((2001, "length"), (2201, "wear"), (2401, "diameter"), (2601, "wear")):
            if base <= n < base + 200:
                num = n - base + 1
                if num not in self.setup.tools:
                    self.setup.tools[num] = self.setup.tool(num).__class__(number=num)
                return kind, self.setup.tools[num]
        return None

    # ------------------------------------------------------------------ blocks
    def _parse(self, line: str) -> tuple[list[float], list[float], dict]:
        line = line.strip()
        if not line or line.startswith("%") or line.startswith("/"):
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

    def _block(self, gs, ms, w):
        s = self.s
        mode_changed = False
        if 65 in gs:  # macro call: every other word is an argument (#1-#26), not a modal word
            args = {macro.G65_ARGS[k]: v for k, v in w.items() if k in macro.G65_ARGS}
            return ("macro", int(w.get("P", 0)), int(w.get("L", 1)), args)

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
                    self._comp_flush()
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
        if "D" in w:
            s.d = int(w["D"])

        # --- G10 / G65 / G68 take their words as data, not motion ----------------------
        if 10 in gs:
            self._g10(gs, w)
            return None
        if 68 in gs:
            self._comp_flush()
            cx = w.get("X", self.prog_pos[0] / s.units) * s.units
            cy = w.get("Y", self.prog_pos[1] / s.units) * s.units
            s.rotation = (cx, cy, w.get("R", 0.0))
            self._resync()
            return None
        if 69 in gs and s.rotation is not None:
            self._comp_flush()
            s.rotation = None
            mode_changed = True

        # --- tool change / spindle / coolant ------------------------------------------
        if 6 in ms:
            self._comp_flush()
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
                self._comp_flush()
                self._event("end", f"M{m}")
                return "end"
            elif m not in (6, 97, 98, 99):
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
            self._comp_flush()
            self._resync()

        # --- non-modal ------------------------------------------------------------------
        axes = {a: w[a] for a in "XYZBC" if a in w}
        if "A" in w:
            self._warn("A axis word ignored (UMC-500 has B/C only)", "A")
        for g in gs:
            if g == 4:
                if self._pending is not None:
                    secs = self._seconds(w.get("P", 0.0))
                    self._deferred.append(lambda secs=secs: self._dwell(secs))
                else:
                    self._dwell(self._seconds(w.get("P", 0.0)))
                return self._sub_action(ms, w)
            if g == 53:
                self._comp_flush()
                q = s.q.copy()
                for a, v in axes.items():
                    i = "XYZBC".index(a)
                    q[i] = v * (s.units if i < 3 else 1.0)
                self._machine_move(q)
                return self._sub_action(ms, w)
            if g == 28:
                self._comp_flush()
                self._home(axes)
                return self._sub_action(ms, w)
            if g in (51, 50, 187, 103, 47, 150):
                if g not in (50, 187):
                    self._warn(f"G{g:g} ignored", f"G{g:g}")

        # --- cutter compensation mode ---------------------------------------------------
        comp_off = 40 in gs and (s.comp != 40 or self._pending is not None)
        for g in gs:
            if g in (41, 42):
                if s.tcp:
                    self._warn("cutter compensation with G234/G254 is not simulated; ignored", "comp-tcp")
                    continue
                if s.comp == 40:
                    self._comp_started = False
                s.comp = int(g)

        # --- motion ---------------------------------------------------------------------
        for g in gs:
            if g in (0, 1, 2, 3, 80) or g in CYCLES:
                s.motion = int(g)
            elif g not in KNOWN_G:
                self._warn(f"G{g:g} not supported; ignored", f"G{g:g}")
        if s.motion in CYCLES:
            if s.comp != 40:
                self._warn("canned cycle with cutter compensation on; compensation ignored for the cycle")
                self._comp_flush()
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
            return self._sub_action(ms, w)

        if comp_off:
            target = self._target(axes) if axes else None
            self._comp_flush()
            s.comp = 40
            if target is not None:
                self._move(target, w)
            self.prog_pos = self.s.p.copy()
            return self._sub_action(ms, w)
        if axes:
            target = self._target(axes)
            if s.comp != 40 and s.plane == 17 and s.motion in (0, 1, 2, 3):
                self._comp_move(target, w)
            else:
                self._move(target, w)
        return self._sub_action(ms, w)

    def _sub_action(self, ms, w):
        if 97 in ms:
            return ("local", int(w.get("P", 0)), int(w.get("L", 1)))
        if 98 in ms:
            return ("external", int(w.get("P", 0)), int(w.get("L", 1)))
        if 99 in ms:
            self._comp_flush()
            return ("return", int(w["P"]) if "P" in w else None)
        return None

    def _move(self, target, w):
        s = self.s
        if s.motion == 0:
            self._linear(target, rapid=True)
        elif s.motion == 1:
            self._linear(target, rapid=False)
        elif s.motion in (2, 3):
            self._arc(target, w, clockwise=s.motion == 2)

    def _target(self, axes: dict) -> np.ndarray:
        s = self.s
        t = (self.prog_pos if (s.comp != 40 or self._pending is not None) else s.p).copy()
        for a, v in axes.items():
            i = "XYZBC".index(a)
            val = v * (s.units if i < 3 else 1.0)
            t[i] = val if s.absolute else t[i] + val
        return t

    def _g10(self, gs, w):
        s = self.s
        L = int(w.get("L", 2))
        P = int(w.get("P", 0))
        if L in (2, 20):
            if L == 2 and P == 0:
                self._warn("G10 L2 P0 (common offset) not simulated", "g10-p0")
                return
            name = f"G{53 + P}" if L == 2 else f"G154P{P}"
            off = self.setup.work_offsets.setdefault(name, np.zeros(5))
            for a, i in (("X", 0), ("Y", 1), ("Z", 2), ("B", 3), ("C", 4)):
                if a in w:
                    v = w[a] * (s.units if i < 3 else 1.0)
                    off[i] = v if s.absolute else off[i] + v
        elif L in (1, 10, 11, 12, 13):
            if P not in self.setup.tools:
                self.setup.tools[P] = self.setup.tool(P).__class__(number=P)
            t = self.setup.tools[P]
            if "R" in w:
                v = w["R"] * s.units
                if L in (1, 10):
                    t.length = v if s.absolute else t.length + v
                elif L == 11:
                    t.length += v   # length wear
                elif L == 12:
                    t.d_offset = v / 2 if s.absolute else (t.d_offset if t.d_offset is not None else t.radius) + v / 2
                else:
                    self._warn("G10 L13 (diameter wear) not simulated", "g10-l13")
        else:
            self._warn(f"G10 L{L} not simulated", f"g10-l{L}")
            return
        self._comp_flush()
        self._resync()

    # ------------------------------------------------------------------ cutter compensation
    # Compensated moves are held one block back: where a move must end depends on the next one
    # (inside corners are trimmed to the intersection of the two offset paths, outside corners
    # get an arc around the corner). Moves without XY motion are queued behind the held move.
    def _comp_radius(self) -> float:
        t = self.setup.tool(self.s.d or self.s.tool)
        return t.d_offset if t.d_offset is not None else t.radius

    def _comp_move(self, target, w):
        s = self.s
        start = self.prog_pos[:2].copy()
        end = target[:2].copy()
        if np.linalg.norm(end - start) < 1e-9 and s.motion in (0, 1):
            # no XY motion (Z / rotary only): runs after the held move
            if self._pending is None:
                self._move_with_comp_xy(target, s.motion == 0)
            else:
                self._deferred.append(lambda tgt=target.copy(), rapid=s.motion == 0:
                                      self._move_with_comp_xy(tgt, rapid))
            self.prog_pos = target.copy()
            return
        seg = {"motion": s.motion, "start": start, "end": end, "target": target.copy(), "line": self.line_no,
               "feed": s.feed, "inverse": s.inverse_time, "startup": not self._comp_started,
               "side": 1.0 if s.comp == 41 else -1.0, "r": self._comp_radius()}
        if s.motion in (2, 3):
            seg["centre"] = self._arc_centre(self.prog_pos, target, w, s.motion == 2)
            seg["cw"] = s.motion == 2
        self._comp_started = True
        self.prog_pos = target.copy()
        if self._pending is not None:
            self._comp_emit(self._pending, seg)
        self._pending = seg

    def _move_with_comp_xy(self, target, rapid):
        t = target.copy()
        t[:2] = self.s.p[:2]  # stay on the compensated XY position
        self._linear(t, rapid)

    @staticmethod
    def _tangent(seg, at_end: bool) -> np.ndarray:
        if seg["motion"] in (2, 3):
            p = seg["end"] if at_end else seg["start"]
            rv = p - seg["centre"]
            t = np.array([rv[1], -rv[0]]) if seg["cw"] else np.array([-rv[1], rv[0]])
        else:
            t = seg["end"] - seg["start"]
        n = np.linalg.norm(t)
        return t / n if n > 1e-12 else np.array([1.0, 0.0])

    @staticmethod
    def _normal(seg, t) -> np.ndarray:
        return seg["side"] * np.array([-t[1], t[0]]) * seg["r"]   # left of travel for G41

    def _comp_emit(self, seg, nxt):
        """Emit the held move `seg`; `nxt` is the following compensated move (or None)."""
        s = self.s
        saved = (self.line_no, s.feed, s.inverse_time)
        self.line_no, s.feed, s.inverse_time = seg["line"], seg["feed"], seg["inverse"]
        try:
            E = seg["end"]
            tA = self._tangent(seg, at_end=True)
            nA = self._normal(seg, tA)
            rapid = seg["motion"] == 0
            if seg["startup"]:
                n = self._normal(seg, self._tangent(nxt, at_end=False)) if nxt else nA
                if seg["motion"] in (2, 3):
                    self._warn("cutter compensation start-up on an arc; treated as a straight move")
                self._to_xy(seg, E + n, rapid)
            elif nxt is None:
                self._seg_to(seg, E + nA)
            else:
                tB = self._tangent(nxt, at_end=False)
                nB = self._normal(nxt, tB)
                cross = tA[0] * tB[1] - tA[1] * tB[0]
                dot = float(tA @ tB)
                if abs(cross) < 1e-6 and dot > 0:          # tangent (or straight on)
                    self._seg_to(seg, E + nA)
                    if np.linalg.norm(nA - nB) > 1e-6:
                        self._to_xy(seg, E + nB, False)
                elif seg["side"] * cross < 0 or (abs(cross) < 1e-6 and dot < 0):   # outside corner
                    self._seg_to(seg, E + nA)
                    t = seg["target"].copy()
                    t[:2] = E + nB
                    self._arc_about(t, E, clockwise=seg["side"] > 0)
                elif seg["motion"] not in (2, 3) and nxt["motion"] not in (2, 3):   # inside, two lines
                    A = np.column_stack([tA, -tB])
                    a, _ = np.linalg.solve(A, nB - nA)
                    J = E + nA + a * tA
                    if a > 0 or (np.linalg.norm(E - seg["start"]) + a) < 0:
                        self._warn(f"cutter compensation: move shorter than the tool radius "
                                   f"({seg['r']:.3f} mm) -- the tool would gouge", f"comp-short-{seg['line']}")
                    self._to_xy(seg, J, rapid)
                else:
                    self._warn("cutter compensation at an inside corner with an arc is approximated",
                               "comp-arc-inside")
                    self._seg_to(seg, E + nA)
                    self._to_xy(seg, E + nB, False)
            for fn in self._deferred:
                fn()
            self._deferred = []
        finally:
            self.line_no, s.feed, s.inverse_time = saved

    def _to_xy(self, seg, xy, rapid):
        t = seg["target"].copy()
        t[:2] = xy
        self._linear(t, rapid)

    def _seg_to(self, seg, xy):
        """Move along the offset version of `seg` to `xy` (on the offset path)."""
        if seg["motion"] in (2, 3):
            C = seg["centre"]
            if np.linalg.norm(xy - C) < 1e-6:
                self._warn(f"cutter compensation: arc radius smaller than the tool radius ({seg['r']:.3f} mm)",
                           f"comp-arc-{seg['line']}")
                self._to_xy(seg, xy, False)
                return
            t = seg["target"].copy()
            t[:2] = xy
            self._arc_about(t, C, seg["cw"])
        else:
            self._to_xy(seg, xy, seg["motion"] == 0)

    def _comp_flush(self):
        if self._pending is not None:
            seg, self._pending = self._pending, None
            self._comp_emit(seg, None)
        elif self._deferred:
            for fn in self._deferred:
                fn()
            self._deferred = []

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


def _offset_code(name: str) -> float:
    if name.startswith("G154P"):
        return 154.0
    return float(name[1:]) if name[1:].isdigit() else 54.0


def simulate(text: str, kin: Kinematics, setup: JobSetup, accel: bool = True,
             search_paths: list[str | Path] | None = None, **kw) -> Trajectory:
    """Interpret `text`. With accel=True (default) the times include acceleration and cornering
    (see timing.py); the feed-rate-only times are kept in `traj.t_ideal`. `search_paths` are
    folders searched for M98 / G65 subprograms that aren't in `text` itself."""
    traj = Interpreter(kin, setup, search_paths=search_paths, **kw).run(text)
    if accel:
        from .timing import apply

        apply(traj, kin.m)
    return traj
