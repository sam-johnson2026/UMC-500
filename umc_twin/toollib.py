"""Import tool libraries into the twin's Tool model.

Formats:
  .json   Fusion 360 tool library export (Manage > Tool Library > Export). The mapping below
          follows Fusion's export format (DC diameter, LCF flute length, LB length below holder,
          RE corner radius, SIG point angle, TA taper angle, SFDM shank, holder segments,
          post-process number). It has not yet been tried on this shop's own export --
          check the first import with `python -m umc_twin tools <file>`.
  .csv    One row per tool, header names = Tool fields:
          number,name,type,length,diameter,flute_length,corner_radius,tip_angle,holder_diameter,holder_length
          Add a `units` column with "in" for inch rows.

Tool length (the H offset) = length below holder + holder height, i.e. gauge line to tip.
When the machine's measured offsets are available, they should win: give them in the setup's
`tools:` block (same number) and only `length` is overridden.
"""
from __future__ import annotations

import csv
import json
from pathlib import Path

from .job import TOOL_TYPES, Tool, tool_from_dict

DEFAULT_HOLDER = (45.0, 50.0)  # height, diameter (mm) when a library entry has no holder

FUSION_TYPES = {
    "flat end mill": "flat", "face mill": "flat", "slot mill": "flat", "tapered mill": "flat",
    "ball end mill": "ball", "lollipop mill": "ball",
    "bull nose end mill": "bull", "radius mill": "bull",
    "drill": "drill", "center drill": "spot", "spot drill": "spot",
    "chamfer mill": "chamfer", "counter sink": "chamfer",
}


def load_tool_library(path: str | Path) -> dict[int, Tool]:
    path = Path(path)
    if path.suffix.lower() == ".csv":
        return _load_csv(path)
    return _load_fusion(json.loads(path.read_text()))


def _load_csv(path: Path) -> dict[int, Tool]:
    out = {}
    with path.open(newline="") as f:
        for row in csv.DictReader(f):
            row = {k.strip(): v.strip() for k, v in row.items() if k and v is not None and v.strip() != ""}
            if "number" not in row:
                continue
            scale = 25.4 if row.pop("units", "mm") in ("in", "inch") else 1.0
            num = int(float(row.pop("number")))
            data = {k: (v if k in ("name", "type") else float(v)) for k, v in row.items()}
            out[num] = tool_from_dict(num, data, scale)
    return out


def _load_fusion(doc: dict) -> dict[int, Tool]:
    out = {}
    for entry in doc.get("data", doc if isinstance(doc, list) else []):
        post = entry.get("post-process") or {}
        num = post.get("number")
        if num is None or entry.get("type") in ("holder", "probe"):
            continue
        g = entry.get("geometry") or {}
        scale = 25.4 if str(entry.get("unit", "millimeters")).startswith("inch") else 1.0
        ttype = FUSION_TYPES.get(str(entry.get("type", "")).lower(), "flat")
        holder = [[s.get("height", 0.0) * scale, s.get("lower-diameter", 0.0) * scale, s.get("upper-diameter", 0.0) * scale]
                  for s in (entry.get("holder") or {}).get("segments", [])]
        holder = [seg for seg in holder if seg[0] > 0]
        if not holder:  # no holder in the library: assume the default one (the measured H offset should win)
            holder = [[DEFAULT_HOLDER[0], DEFAULT_HOLDER[1], DEFAULT_HOLDER[1]]]
        dia = float(g.get("DC", 10.0)) * scale
        below = g.get("LB") or g.get("OAL") or 3 * dia / scale
        length = float(below) * scale + sum(seg[0] for seg in holder)
        tip_angle = None
        if ttype in ("drill", "spot") and g.get("SIG"):
            tip_angle = float(g["SIG"])
        elif ttype == "chamfer" and g.get("TA"):
            tip_angle = 2.0 * float(g["TA"])  # Fusion's taper angle is measured from the axis
        data = {
            "holder": holder,
            "name": entry.get("description") or entry.get("product-id") or "",
            "type": ttype if ttype in TOOL_TYPES else "flat",
            "length": length, "diameter": dia,
            "flute_length": float(g["LCF"]) * scale if g.get("LCF") else None,
            "corner_radius": float(g.get("RE", 0.0)) * scale if ttype == "bull" else 0.0,
            "tip_angle": tip_angle,
            "shank_diameter": float(g["SFDM"]) * scale if g.get("SFDM") else None,
        }
        out[int(num)] = tool_from_dict(int(num), data)
    return out
