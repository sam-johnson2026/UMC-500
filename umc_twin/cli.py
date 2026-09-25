"""Command line: `python -m umc_twin <command>` (or `umc-twin` once installed)."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from .config import JOINT_ORDER, load_machine
from .job import default_setup, load_setup
from .kinematics import Kinematics


def _options(pairs: list[str]) -> dict:
    out = {}
    for p in pairs or []:
        k, _, v = p.partition("=")
        out[k] = v
    return out


def _machine_and_setup(args):
    machine = load_machine(options=_options(args.option))
    kin = Kinematics(machine)
    setup = load_setup(args.setup, kin) if args.setup else default_setup(kin)
    return machine, kin, setup


def cmd_simulate(args):
    from .sim import format_report, run_job

    machine, _, setup = _machine_and_setup(args)
    result = run_job(Path(args.program).read_text(), machine, setup, check_collisions=not args.no_collisions)
    print(format_report(result))
    if args.out:
        Path(args.out).write_text(json.dumps(result))
        print(f"trajectory written to {args.out} (open it in the viewer)")
    bad = "error" in result or result["limits"] or result["collisions"]
    return 1 if bad and args.strict else 0


def cmd_serve(args):
    from .live import ReplaySource
    from .server import serve

    live = None
    if args.replay:
        from .gcode import simulate

        _, kin, setup = _machine_and_setup(args)
        live = ReplaySource(simulate(Path(args.replay).read_text(), kin, setup), speed=args.speed)
    serve(port=args.port, live=live, host=args.host)
    return 0


def cmd_pose(args):
    """Where is the tool, and does anything collide, at a given machine position?"""
    import numpy as np

    machine, kin, setup = _machine_and_setup(args)
    q = np.array(args.q, dtype=float)
    tool = setup.tool(args.tool)
    print("joints      ", dict(zip(JOINT_ORDER, q.tolist())))
    print("tip (world) ", np.round(kin.tool_tip_world(q, tool.length), 3).tolist())
    print("tip (table) ", np.round(kin.tool_tip_in_table(q, tool.length), 3).tolist())
    print("limits      ", kin.limit_violations(q) or "ok")
    if not args.no_collisions:
        from .collision import CollisionChecker

        hits = CollisionChecker(machine, kin, setup).check_pose(q, tool=args.tool)
        print("collisions  ", [f"{a} x {b}" for a, b, _ in hits] or "none")
    return 0


def cmd_info(args):
    machine = load_machine(options=_options(args.option))
    print(machine.raw["machine"]["name"])
    for name, j in machine.joints.items():
        lim = f"[{j.limits[0]:g}, {j.limits[1]:g}]" if j.limits else "continuous"
        print(f"  {name}: {j.type:6s} parent={j.parent:4s} limits={lim:16s} vmax={j.max_velocity:g}/min")
    print("options:", ", ".join(f"{k}={v}" for k, v in machine.options.items()))
    print("active parts:", ", ".join(p.id for p in machine.active_parts()))
    return 0


def cmd_calibrate(args):
    from .calibration import calibrate

    inputs = {"units": args.units}
    if args.nose_to_platter is not None:
        inputs["nose_to_platter"] = args.nose_to_platter
    if args.mrzp:
        inputs["mrzp"] = args.mrzp
    for key in ("b_dir", "c_dir"):
        if getattr(args, key):
            inputs[key.replace("dir", "direction")] = getattr(args, key)
    rapid = {k: v for k, v in (("B", args.b_rapid), ("C", args.c_rapid)) if v}
    if rapid:
        inputs["rotary_rapid"] = rapid
    if args.tc_time is not None:
        inputs["tool_change_time"] = args.tc_time
    if args.option:
        inputs["options"] = _options(args.option)
    if len(inputs) == 1 and not args.show:
        print("nothing to calibrate -- pass a measurement (see --help), or --show")
        return 1
    result = calibrate(save=not (args.dry_run or args.show), **inputs)
    for n in result["notes"]:
        print(n)
    s = result["summary"]
    print(f"nose to platter at home: {s['nose_to_platter_at_home_mm']:.2f} mm "
          f"({s['nose_to_platter_at_home_mm'] / 25.4:.3f} in)")
    print("MRZP (settings 255-257): X {:.3f} Y {:.3f} Z {:.3f} mm  =  {:.4f} {:.4f} {:.4f} in".format(
        *s["mrzp_mm"], *s["mrzp_in"]))
    print(f"B axis {s['b_axis']}  C axis {s['c_axis']}  options {s['options']}")
    print("saved config/calibration.yaml" if result["saved"] else "(not saved)")
    return 0


def cmd_refresh_manifest(args):
    from .manifest import MANIFEST, refresh_manifest

    m = refresh_manifest()
    print(f"updated config in {MANIFEST.relative_to(MANIFEST.parents[2])}")
    if m.get("missing_meshes"):
        print("parts without meshes (run tools/build_assets.py):", ", ".join(m["missing_meshes"]))
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(prog="umc-twin", description="Haas UMC-500 kinematic digital twin")
    sub = ap.add_subparsers(dest="cmd", required=True)

    def common(p):
        p.add_argument("--setup", help="job setup YAML (tools, work offsets, stock)")
        p.add_argument("--option", action="append", metavar="KEY=VALUE",
                       help="machine option, e.g. spindle=hsk (repeatable)")

    p = sub.add_parser("simulate", help="run a G-code program and report time / limits / collisions")
    p.add_argument("program")
    common(p)
    p.add_argument("--out", help="write the trajectory JSON here (loadable in the viewer)")
    p.add_argument("--no-collisions", action="store_true")
    p.add_argument("--strict", action="store_true", help="exit 1 on errors, limit violations or collisions")
    p.set_defaults(func=cmd_simulate)

    p = sub.add_parser("serve", help="serve the 3D viewer + simulation API")
    common(p)
    p.add_argument("--port", type=int, default=8000)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--replay", metavar="PROGRAM", help="stream this program as a fake live machine")
    p.add_argument("--speed", type=float, default=1.0, help="replay speed multiplier")
    p.set_defaults(func=cmd_serve)

    p = sub.add_parser("pose", help="tool position / limits / collisions at one machine position")
    p.add_argument("q", nargs=5, type=float, metavar=("X", "Y", "Z", "B", "C"))
    common(p)
    p.add_argument("--tool", type=int, default=0)
    p.add_argument("--no-collisions", action="store_true")
    p.set_defaults(func=cmd_pose)

    p = sub.add_parser("info", help="print the machine definition")
    p.add_argument("--option", action="append", metavar="KEY=VALUE")
    p.set_defaults(func=cmd_info)

    p = sub.add_parser("calibrate", help="record measurements from the machine in config/calibration.yaml")
    p.add_argument("--units", choices=["mm", "in"], default="mm", help="units of the lengths you enter")
    p.add_argument("--nose-to-platter", type=float, help="Z home, B0: spindle nose to platter top")
    p.add_argument("--mrzp", type=float, nargs=3, metavar=("S255", "S256", "S257"),
                   help="Haas settings 255/256/257 (MRZP X, Y, Z) as shown on the control")
    p.add_argument("--b-dir", choices=["right", "left"], help="which way the platter face turns on B+ at home")
    p.add_argument("--c-dir", choices=["cw", "ccw"], help="C+ table rotation seen from above")
    p.add_argument("--b-rapid", type=float, help="B max speed, deg/min")
    p.add_argument("--c-rapid", type=float, help="C max speed, deg/min")
    p.add_argument("--tc-time", type=float, help="tool change time, s")
    p.add_argument("--option", action="append", metavar="KEY=VALUE", help="machine option, e.g. spindle=hsk")
    p.add_argument("--dry-run", action="store_true", help="show the result without saving")
    p.add_argument("--show", action="store_true", help="show the current calibration")
    p.set_defaults(func=cmd_calibrate)

    p = sub.add_parser("refresh-manifest", help="copy config/umc500.yaml into web/assets/machine.json")
    p.set_defaults(func=cmd_refresh_manifest)

    args = ap.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
