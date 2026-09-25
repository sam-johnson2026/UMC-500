"""MTConnect client: turns an MTConnect agent into a LiveSource for the twin.

Haas NGC controls can serve MTConnect (an agent built into the control, or a separate agent
talking to it). The client:

1. reads the agent's /probe document and picks the data items it needs automatically --
   actual machine positions of the Linear X/Y/Z and Rotary B/C axes, spindle speed and load,
   program line, program, tool number, execution state and controller mode;
2. polls /current on a background thread and serves the latest `MachineState`.

Everything it chose is shown by `python -m umc_twin mtconnect-probe URL`. When a control names
things differently, pin the ids in config (overrides the automatic choice):

    live:
      mtconnect:
        url: http://192.168.1.50:8082
        device: UMC500            # only if the agent serves several devices
        data_items: {X: Xabs, Z: Zabs, spindle_load: Sload}
        scale: {X: 25.4}          # only if an adapter reports inches (the standard says mm)

It has been tested against the fake agent in mtconnect_agent.py (same document structure as
the MTConnect standard), not yet against this machine's agent.
"""
from __future__ import annotations

import threading
import time
import urllib.request
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

from .live import MachineState

AXES = ("X", "Y", "Z", "B", "C")
UNAVAILABLE = "UNAVAILABLE"


def _local(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def _camel_to_type(tag: str) -> str:
    """Streams use CamelCase element names (RotaryVelocity) for types (ROTARY_VELOCITY)."""
    out = []
    for i, ch in enumerate(tag):
        if ch.isupper() and i and not tag[i - 1].isupper():
            out.append("_")
        out.append(ch.upper())
    return "".join(out)


@dataclass
class DataItemInfo:
    id: str
    type: str
    sub_type: str = ""
    category: str = ""
    coordinate_system: str = ""
    units: str = ""
    name: str = ""
    component: str = ""        # component element type: Linear, Rotary, Path, Controller...
    component_name: str = ""
    native_name: str = ""
    device: str = ""


@dataclass
class Mapping:
    items: dict[str, str] = field(default_factory=dict)   # role -> dataItemId
    device: str = ""
    notes: list[str] = field(default_factory=list)

    def describe(self) -> str:
        lines = [f"device: {self.device or '(any)'}"]
        for role, did in self.items.items():
            lines.append(f"  {role:14s} <- {did}")
        lines += [f"  note: {n}" for n in self.notes]
        return "\n".join(lines)


def parse_probe(xml_text: str) -> list[DataItemInfo]:
    root = ET.fromstring(xml_text)
    out: list[DataItemInfo] = []

    def walk(el, device, comp_type, comp_name, native):
        tag = _local(el.tag)
        if tag == "Device":
            device = el.get("name", "")
        if tag not in ("DataItems", "DataItem", "Components", "Devices", "MTConnectDevices", "Header",
                       "Configuration", "Description", "Compositions", "References"):
            if el.get("id") and tag != "DataItem":
                comp_type, comp_name, native = tag, el.get("name", ""), el.get("nativeName", "")
        if tag == "DataItem":
            out.append(DataItemInfo(
                id=el.get("id", ""), type=el.get("type", ""), sub_type=el.get("subType", ""), category=el.get("category", ""),
                coordinate_system=el.get("coordinateSystem", ""), units=el.get("units", ""), name=el.get("name", ""),
                component=comp_type, component_name=comp_name, native_name=native, device=device))
        for child in el:
            walk(child, device, comp_type, comp_name, native)

    walk(root, "", "", "", "")
    return out


def auto_mapping(items: list[DataItemInfo], device: str | None = None,
                 overrides: dict[str, str] | None = None) -> Mapping:
    m = Mapping(device=device or "")
    if device:
        items = [i for i in items if i.device == device]
    devices = sorted({i.device for i in items if i.device})
    if not device and len(devices) > 1:
        m.notes.append(f"several devices ({', '.join(devices)}); using {devices[0]} -- set live.mtconnect.device")
        items = [i for i in items if i.device == devices[0]]
        m.device = devices[0]
    elif devices:
        m.device = devices[0]

    def score_position(i: DataItemInfo) -> int:
        s = 0
        s += 4 if i.sub_type in ("ACTUAL", "") else 0
        s += {"MACHINE": 3, "": 1}.get(i.coordinate_system, 0)
        return s

    def named(i: DataItemInfo, axis: str) -> bool:
        return axis in (i.component_name.upper(), i.native_name.upper()) or i.name.upper().startswith(axis)

    for axis in AXES:
        kind, comp = ("POSITION", "Linear") if axis in "XYZ" else ("ANGLE", "Rotary")
        cands = [i for i in items if i.type == kind and i.component == comp and named(i, axis)]
        if not cands:
            cands = [i for i in items if i.type == kind and named(i, axis)]
        if cands:
            best = max(cands, key=score_position)
            m.items[axis] = best.id
            if best.coordinate_system == "WORK":
                m.notes.append(f"{axis}: only WORK coordinates found -- the twin needs MACHINE positions")
        else:
            m.notes.append(f"{axis}: no {kind} data item found")

    speeds = [i for i in items if i.type in ("ROTARY_VELOCITY", "SPINDLE_SPEED")]
    spindle = [i for i in speeds if i.component_name.upper() in ("S", "S1", "SPINDLE", "C2")] or \
              [i for i in speeds if "spindle" in (i.component_name + i.name).lower()] or \
              [i for i in speeds if i.component == "Rotary" and i.component_name.upper() not in ("B", "C")]
    if spindle:
        best = max(spindle, key=lambda i: i.sub_type == "ACTUAL")
        m.items["spindle_rpm"] = best.id
        loads = [i for i in items if i.type == "LOAD" and i.component_name == best.component_name]
        if loads:
            m.items["spindle_load"] = loads[0].id
    for role, types in (("line", ("LINE_NUMBER", "LINE")), ("program", ("PROGRAM",)),
                        ("execution", ("EXECUTION",)), ("tool", ("TOOL_NUMBER", "TOOL_ASSET_ID")),
                        ("mode", ("CONTROLLER_MODE",))):
        cands = [i for i in items if i.type in types]
        if role == "line":
            cands.sort(key=lambda i: i.sub_type != "ABSOLUTE")  # prefer the absolute line
        if cands:
            m.items[role] = cands[0].id
    for role, did in (overrides or {}).items():
        m.items[role] = did
    return m


def parse_current(xml_text: str) -> dict[str, str]:
    """dataItemId -> latest value text."""
    root = ET.fromstring(xml_text)
    out = {}
    for el in root.iter():
        did = el.get("dataItemId")
        if did is not None:
            out[did] = (el.text or "").strip()
    return out


def _num(v: str | None) -> float | None:
    if v is None or v == "" or v == UNAVAILABLE:
        return None
    try:
        return float(v.split()[0])
    except ValueError:
        return None


def state_from_values(values: dict[str, str], mapping: Mapping, scale: dict | None = None,
                      last_q: list[float] | None = None) -> MachineState | None:
    q = []
    for i, axis in enumerate(AXES):
        v = _num(values.get(mapping.items.get(axis, "")))
        if v is None:
            if last_q is None:
                return None  # no pose yet
            v = last_q[i]
        else:
            v *= (scale or {}).get(axis, 1.0)
        q.append(v)
    line = _num(values.get(mapping.items.get("line", "")))
    tool = _num(values.get(mapping.items.get("tool", "")))
    exec_ = values.get(mapping.items.get("execution", ""))
    mode = values.get(mapping.items.get("mode", ""))
    return MachineState(
        q=q, source="mtconnect",
        line=int(line) if line is not None else None,
        tool=int(tool) if tool is not None else None,
        spindle_rpm=_num(values.get(mapping.items.get("spindle_rpm", ""))),
        spindle_load=_num(values.get(mapping.items.get("spindle_load", ""))),
        program=values.get(mapping.items.get("program", "")) or None,
        execution=None if exec_ in (None, UNAVAILABLE) else exec_,
        mode=None if mode in (None, UNAVAILABLE) else mode,
    )


def fetch(url: str, timeout: float = 3.0) -> str:
    with urllib.request.urlopen(url, timeout=timeout) as r:  # noqa: S310 - operator-configured URL
        return r.read().decode("utf-8", "replace")


class MTConnectSource:
    """LiveSource polling an MTConnect agent's /current on a background thread."""

    name = "mtconnect"

    def __init__(self, url: str, device: str | None = None, data_items: dict | None = None,
                 scale: dict | None = None, interval: float = 0.1, timeout: float = 3.0):
        self.base = url.rstrip("/")
        self.interval, self.timeout = interval, timeout
        self.scale = scale or {}
        dev = f"/{device}" if device else ""
        self.mapping = auto_mapping(parse_probe(fetch(f"{self.base}{dev}/probe", timeout)), device, data_items)
        self.current_url = f"{self.base}{dev}/current"
        self._state: MachineState | None = None
        self.error: str | None = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True, name="mtconnect-poll")
        self._thread.start()

    def _loop(self):
        while not self._stop.is_set():
            t0 = time.monotonic()
            try:
                values = parse_current(fetch(self.current_url, self.timeout))
                st = state_from_values(values, self.mapping, self.scale, self._state.q if self._state else None)
                if st is not None:
                    self._state = st
                self.error = None
            except Exception as e:  # keep polling; the agent may come back
                self.error = f"{type(e).__name__}: {e}"
            self._stop.wait(max(0.0, self.interval - (time.monotonic() - t0)))

    def read(self) -> MachineState | None:
        return self._state

    def close(self):
        self._stop.set()
