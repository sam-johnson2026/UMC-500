"""Pre-flight check for a batch of programs: `umc-twin check FILES_OR_FOLDERS --setup JOB --html report.html`.

Every program is simulated (motion, cutting, spindle load, collisions, travel limits) and
graded:

  fail  error, over-travel, collision, gouge, rapid into material, spindle overload
  warn  interpreter warnings, shank/holder contact, cutting with the spindle stopped, over-speed,
        estimated contour error above the job's `material.tolerance` (when given)
  pass  nothing to report

A job setup next to a program with the same name (part12.nc + part12.yaml) is used for that
program; otherwise --setup, otherwise the default (part zero on the platter, generic tool).
"""
from __future__ import annotations

import datetime as _dt
import html
from pathlib import Path

from .config import Machine
from .job import default_setup, load_setup
from .kinematics import Kinematics
from .sim import run_job

PROGRAM_GLOBS = ("*.nc", "*.NC", "*.ngc", "*.tap", "*.gcode")
FAIL_KINDS = {"gouge", "rapid_into_material", "spindle_overload"}


def find_programs(paths: list[str | Path]) -> list[Path]:
    out: list[Path] = []
    for p in map(Path, paths):
        if p.is_dir():
            for g in PROGRAM_GLOBS:
                out += sorted(p.glob(g))
        elif p.is_file():
            out.append(p)
    seen, uniq = set(), []
    for p in out:
        if p.resolve() not in seen:
            seen.add(p.resolve())
            uniq.append(p)
    return uniq


def grade(result: dict) -> tuple[str, list[str]]:
    if "error" in result:
        return "fail", [result["error"]]
    notes, status = [], "pass"

    def bump(level):
        nonlocal status
        if level == "fail" or (level == "warn" and status == "pass"):
            status = level

    for lim in result["limits"]:
        bump("fail")
        notes.append(f"line {lim['line']}: over-travel {lim['message']}")
    for c in result["collisions"]:
        bump("fail")
        what = "rapid into stock" if c["kind"] == "rapid_into_stock" else f"collision {c['a']} x {c['b']}"
        notes.append(f"line {c['line']}: {what}")
    for it in (result.get("servo") or {}).get("issues", []):
        bump("warn")
        notes.append(f"line {it['line']}: {it['detail']}")
    for group in ("material", "spindle_load"):
        for it in (result.get(group) or {}).get("issues", []):
            bump("fail" if it["kind"] in FAIL_KINDS else "warn")
            notes.append(f"line {it['line']}: {it['detail']}")
    for w in result["warnings"]:
        bump("warn")
        notes.append(f"line {w['line']}: {w['message']}")
    notes.sort(key=lambda n: int(n.split(":")[0].split()[1]) if n.startswith("line ") else 0)
    return status, notes


def check(paths, machine: Machine, setup_path: str | Path | None = None) -> list[dict]:
    kin = Kinematics(machine)
    rows = []
    for prog in find_programs(paths):
        own = next((prog.with_suffix(s) for s in (".yaml", ".yml") if prog.with_suffix(s).is_file()), None)
        setup_file = own or (Path(setup_path) if setup_path else None)
        setup = load_setup(setup_file, kin) if setup_file else default_setup(kin)
        try:
            result = run_job(prog.read_text(errors="replace"), machine, setup, search_paths=[prog.resolve().parent])
        except Exception as e:  # a broken program must not stop the batch
            result = {"error": f"{type(e).__name__}: {e}"}
        status, notes = grade(result)
        s = result.get("summary") or {}
        rows.append({
            "program": str(prog), "setup": str(setup_file) if setup_file else "(default)", "status": status,
            "cycle_s": s.get("duration_s"), "tool_changes": s.get("tool_changes"),
            "peak_load_pct": ((result.get("spindle_load") or {}).get("peak") or {}).get("load_pct"),
            "removed_cm3": round(result["material"]["removed_volume_mm3"] / 1000, 2) if result.get("material") else None,
            "notes": notes,
        })
    return rows


def format_text(rows: list[dict]) -> str:
    out = []
    for r in rows:
        ct = f"{r['cycle_s']:.1f} s" if r["cycle_s"] is not None else "-"
        out.append(f"{r['status'].upper():4s}  {r['program']}  ({ct}, setup {r['setup']})")
        out += [f"        {n}" for n in r["notes"][:8]]
        if len(r["notes"]) > 8:
            out.append(f"        ... {len(r['notes']) - 8} more")
    counts = {k: sum(r["status"] == k for r in rows) for k in ("pass", "warn", "fail")}
    out.append(f"{len(rows)} programs: {counts['pass']} pass, {counts['warn']} warn, {counts['fail']} fail")
    return "\n".join(out)


def format_html(rows: list[dict], machine_name: str) -> str:
    counts = {k: sum(r["status"] == k for r in rows) for k in ("pass", "warn", "fail")}
    esc = html.escape
    body = []
    for r in rows:
        notes = "".join(f"<li>{esc(n)}</li>" for n in r["notes"]) or "<li class='ok'>Nothing to report</li>"
        fmt = lambda v, f: "–" if v is None else f.format(v)  # noqa: E731
        body.append(
            f"<tr class='{r['status']}'><td><span class='pill {r['status']}'>{r['status']}</span></td>"
            f"<td class='mono'>{esc(r['program'])}<div class='sub'>{esc(r['setup'])}</div></td>"
            f"<td class='num'>{fmt(r['cycle_s'], '{:.1f} s')}</td><td class='num'>{fmt(r['tool_changes'], '{}')}</td>"
            f"<td class='num'>{fmt(r['peak_load_pct'], '{:.0f}%')}</td><td class='num'>{fmt(r['removed_cm3'], '{:.1f}')}</td>"
            f"<td><ul>{notes}</ul></td></tr>")
    return f"""<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><title>Pre-flight report</title>
<style>
:root {{ --bg:#f6f7f9; --panel:#fff; --text:#1f2328; --muted:#5f6873; --border:#d9dde2;
  --pass:#2e7d32; --warn:#b26a00; --fail:#c62828; }}
@media (prefers-color-scheme: dark) {{ :root {{ color-scheme: dark; --bg:#111418; --panel:#1a1e24; --text:#e6e8eb;
  --muted:#9aa3ad; --border:#2c323a; --pass:#66bb6a; --warn:#f0a202; --fail:#ef5350; }} }}
body {{ background:var(--bg); color:var(--text); font:14px/1.45 system-ui, sans-serif; margin:0; padding:24px 16px; }}
main {{ max-width:1100px; margin:0 auto; }}
h1 {{ font-size:20px; margin:0 0 4px; }} .sub {{ color:var(--muted); font-size:12.5px; }}
.summary {{ display:flex; gap:10px; margin:14px 0; flex-wrap:wrap; }}
.summary div {{ background:var(--panel); border:1px solid var(--border); border-radius:8px; padding:8px 14px; }}
.summary b {{ font-size:18px; display:block; }}
.wrap {{ overflow-x:auto; }} table {{ width:100%; border-collapse:collapse; background:var(--panel); }}
th, td {{ text-align:left; padding:8px 10px; border-bottom:1px solid var(--border); vertical-align:top; }}
th {{ font-size:12px; color:var(--muted); text-transform:uppercase; letter-spacing:.04em; }}
.num {{ text-align:right; font-variant-numeric:tabular-nums; white-space:nowrap; }}
.mono {{ font-family:ui-monospace, Menlo, Consolas, monospace; font-size:12.5px; }}
ul {{ margin:0; padding-left:16px; font-size:12.5px; }} li.ok {{ color:var(--pass); list-style:none; margin-left:-16px; }}
.pill {{ font-size:11.5px; font-weight:700; text-transform:uppercase; padding:2px 8px; border-radius:999px; border:1px solid currentColor; }}
.pill.pass {{ color:var(--pass); }} .pill.warn {{ color:var(--warn); }} .pill.fail {{ color:var(--fail); }}
tr.fail td:first-child {{ box-shadow: inset 3px 0 var(--fail); }} tr.warn td:first-child {{ box-shadow: inset 3px 0 var(--warn); }}
</style></head><body><main>
<h1>Pre-flight report</h1>
<div class="sub">{esc(machine_name)} · {_dt.datetime.now().strftime('%Y-%m-%d %H:%M')} · simulated by umc-twin</div>
<div class="summary"><div><b>{len(rows)}</b>programs</div><div style="color:var(--pass)"><b>{counts['pass']}</b>pass</div>
<div style="color:var(--warn)"><b>{counts['warn']}</b>warn</div><div style="color:var(--fail)"><b>{counts['fail']}</b>fail</div></div>
<div class="wrap"><table><thead><tr><th>Status</th><th>Program / setup</th><th class="num">Cycle</th>
<th class="num">Tools</th><th class="num">Peak load</th><th class="num">Removed cm³</th><th>Findings</th></tr></thead>
<tbody>{''.join(body)}</tbody></table></div>
<p class="sub">Cycle times include acceleration; spindle load is an estimate from material removal rate.
Values marked VERIFY in the machine config are still spec-sheet placeholders.</p>
</main></body></html>"""
