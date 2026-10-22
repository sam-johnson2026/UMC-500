"""Read machine logs that weren't written by the twin itself, into its recording format.

    python -m umc_twin import-log run.csv --out run.jsonl

Supported: the shop's CSV logger (pc_time, x_mach .. c_mach, spindle_rpm, spindle_load_pct,
tool_in_spindle, Execution, Program, Line / LineNumber, ...). Column names are matched
loosely, so close variants work too.

Units: Haas reports positions in the control's units. `units="auto"` treats a log as inches
when every X/Y/Z value fits inside the machine's travel in inches (20 x 16 x 14) -- a metric
log of any real job goes outside that. Pass units="in" or "mm" to be explicit.
"""
from __future__ import annotations

import csv
import datetime as _dt
import io
from pathlib import Path

INCH = 25.4
TRAVEL_IN = {"x": 20.5, "y": 16.5, "z": 14.5}

ALIASES = {
    "time": ("pc_time", "timestamp", "time", "datetime"),
    "x": ("x_mach", "x_machine", "xmach", "x"), "y": ("y_mach", "y_machine", "ymach", "y"),
    "z": ("z_mach", "z_machine", "zmach", "z"), "b": ("b_mach", "b_machine", "bmach", "b"),
    "c": ("c_mach", "c_machine", "cmach", "c"),
    "rpm": ("spindle_rpm", "spindlespeed", "s"), "load": ("spindle_load_pct", "spindle_load", "load"),
    "tool": ("tool_in_spindle", "tool", "tool_number"), "execution": ("execution",),
    "program": ("program",), "mode": ("controllermode", "mode"),
    "line": ("linenumber", "line", "line_number"),
    "feed_override": ("pathfeedrateoverride", "feed_override"),
}


def _col(header: list[str], role: str) -> str | None:
    low = {h.lower().replace(" ", ""): h for h in header}
    for alias in ALIASES[role]:
        if alias in low:
            return low[alias]
    return None


def _num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def _time(v: str) -> float:
    v = v.strip()
    try:
        return float(v)
    except ValueError:
        return _dt.datetime.fromisoformat(v.replace("Z", "+00:00")).timestamp()


def read_csv_log(path: str | Path, units: str = "auto") -> tuple[list[dict], dict]:
    """(states, info). States are MachineState-style dicts (mm, deg); info says what was found."""
    with open(path, newline="", encoding="utf-8-sig") as f:
        return read_csv_text(f.read(), units, name=str(path))


def read_csv_text(text: str, units: str = "auto", name: str = "log") -> tuple[list[dict], dict]:
    reader = csv.DictReader(io.StringIO(text.lstrip("\ufeff")))
    header = reader.fieldnames or []
    rows = list(reader)
    cols = {role: _col(header, role) for role in ALIASES}
    missing = [r for r in ("time", "x", "y", "z") if cols[r] is None]
    if missing:
        raise ValueError(f"{name}: no column for {', '.join(missing)} (columns: {', '.join(header)})")

    if units == "auto":
        fits = all(abs(_num(r[cols[a]]) or 0.0) <= TRAVEL_IN[a] for r in rows for a in "xyz")
        units = "in" if fits else "mm"
    scale = INCH if units in ("in", "inch") else 1.0

    states = []
    for r in rows:
        q = [(_num(r[cols[a]]) or 0.0) * scale for a in "xyz"]
        q += [(_num(r[cols[a]]) or 0.0) if cols[a] else 0.0 for a in "bc"]
        line = _num(r.get(cols["line"], "")) if cols["line"] else None
        st = {
            "q": [round(v, 4) for v in q], "source": "csv", "timestamp": _time(r[cols["time"]]),
            "line": int(line) if line is not None else None,
            "tool": int(_num(r[cols["tool"]])) if cols["tool"] and _num(r[cols["tool"]]) is not None else None,
            "spindle_rpm": _num(r[cols["rpm"]]) if cols["rpm"] else None,
            "spindle_load": _num(r[cols["load"]]) if cols["load"] else None,
            "execution": (r.get(cols["execution"]) or None) if cols["execution"] else None,
            "program": (r.get(cols["program"]) or None) if cols["program"] else None,
            "mode": (r.get(cols["mode"]) or None) if cols["mode"] else None,
        }
        if cols["feed_override"]:
            st["feed_override"] = _num(r[cols["feed_override"]])
        states.append(st)
    states.sort(key=lambda s: s["timestamp"])
    dts = [b["timestamp"] - a["timestamp"] for a, b in zip(states, states[1:])]
    info = {
        "rows": len(states), "units": "in" if scale == INCH else "mm",
        "seconds": round(states[-1]["timestamp"] - states[0]["timestamp"], 1) if states else 0.0,
        "sample_s": round(sorted(dts)[len(dts) // 2], 3) if dts else None,
        "has_line_numbers": any(s["line"] is not None for s in states),
        "programs": sorted({s["program"] for s in states if s["program"]}),
        "columns_used": {k: v for k, v in cols.items() if v},
    }
    return states, info
