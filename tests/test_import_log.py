"""CSV machine logs without line numbers: import, path-matched compare, position check."""
import csv
from pathlib import Path

import numpy as np
import pytest

from umc_twin import Kinematics, load_machine
from umc_twin.cli import main
from umc_twin.compare import compare, format_report
from umc_twin.gcode import simulate
from umc_twin.importers import read_csv_log
from umc_twin.job import load_setup

DATA = Path(__file__).resolve().parent / "data"
HEADER = ["pc_time", "spindle_load_pct", "tool_in_spindle", "spindle_rpm", "feed_cmd", "x_mach", "y_mach",
          "z_mach", "b_mach", "c_mach", "Execution", "ControllerMode", "Program", "Line", "LineNumber",
          "PathFeedrateOverride"]


@pytest.fixture(scope="module")
def traj():
    kin = Kinematics(load_machine())
    setup = load_setup(DATA / "rect_setup.yaml", kin)
    return simulate((DATA / "rect_laps.nc").read_text(), kin, setup)


def write_log(path, traj, start_s, stop_s, time_scale=1.0, dt=1.0, offset_mm=(0, 0, 0)):
    """A log as the shop's logger writes it: inches, once a second, no line numbers, started
    and stopped partway, then a feed hold, a reset and a move out of the way."""
    t, q = np.asarray(traj.t), np.asarray(traj.q)
    rows = []

    def row(tt, pos, execution):
        rows.append({"pc_time": f"{1.79e9 + tt:.3f}", "spindle_load_pct": "1", "tool_in_spindle": "1",
                     "spindle_rpm": "2000", "feed_cmd": "0", "x_mach": f"{pos[0] / 25.4:.4f}",
                     "y_mach": f"{pos[1] / 25.4:.4f}", "z_mach": f"{pos[2] / 25.4:.4f}", "b_mach": "0",
                     "c_mach": "0", "Execution": execution, "ControllerMode": "AUTOMATIC",
                     "Program": "TEST.NC", "Line": "", "LineNumber": "", "PathFeedrateOverride": "3.0"})

    for st in np.arange(start_s, stop_s, dt / time_scale):
        pos = np.array([np.interp(st, t, q[:, a]) for a in range(3)]) + offset_mm
        row((st - start_s) * time_scale, pos, "ACTIVE")
    end = (stop_s - start_s) * time_scale
    row(end + 1, pos, "FEED_HOLD")
    row(end + 2, pos, "STOPPED")
    row(end + 3, pos + (0, -60, 0), "ACTIVE")
    row(end + 4, pos + (0, -120, 0), "ACTIVE")
    row(end + 5, pos + (0, -120, 0), "STOPPED")
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, HEADER)
        w.writeheader()
        w.writerows(rows)


def test_reads_inch_log(tmp_path, traj):
    p = tmp_path / "run.csv"
    write_log(p, traj, 20, 120)
    states, info = read_csv_log(p)
    assert info["units"] == "in" and not info["has_line_numbers"]
    assert info["sample_s"] == pytest.approx(1.0) and info["programs"] == ["TEST.NC"]
    assert all(s["line"] is None for s in states)
    # back in mm, on the simulated path
    assert states[0]["q"][0] == pytest.approx(np.interp(20, traj.t, np.asarray(traj.q)[:, 0]), abs=0.01)
    mm, info_mm = read_csv_log(p, units="mm")
    assert info_mm["units"] == "mm" and mm[0]["q"][0] == pytest.approx(states[0]["q"][0] / 25.4, abs=1e-3)


def test_metric_log_detected(tmp_path):
    p = tmp_path / "mm.csv"
    p.write_text("pc_time,x_mach,y_mach,z_mach\n0,-250.0,-100,-50\n1,-251.0,-100,-50\n")
    states, info = read_csv_log(p)
    assert info["units"] == "mm" and states[0]["q"][:3] == [-250.0, -100.0, -50.0]


def test_missing_columns(tmp_path):
    p = tmp_path / "bad.csv"
    p.write_text("time,a,b\n0,1,2\n")
    with pytest.raises(ValueError, match="x, y, z"):
        read_csv_log(p)


def test_partial_log_matched_by_path(tmp_path, traj):
    """Starts in the third lap, stops in the sixth, 5% slower; the move after the stop is ignored."""
    p = tmp_path / "run.csv"
    write_log(p, traj, 100, 330, time_scale=1.05)
    states, _ = read_csv_log(p)
    r = compare(traj, states, None)
    assert r["alignment"] == "path"
    cv = r["coverage"]
    assert cv["sim_from_s"] == pytest.approx(100, abs=1.5) and cv["sim_to_s"] == pytest.approx(330, abs=1.5)
    assert r["cycle_time"]["ratio"] == pytest.approx(1.05, abs=0.02)
    assert r["position"]["max_dev_mm"] < 0.01
    assert len(r["ignored_runs"]) == 1
    assert "ignored" in format_report(r)


def test_offset_shows_as_deviation(tmp_path, traj):
    p = tmp_path / "run.csv"
    write_log(p, traj, 30, 200, offset_mm=(0, 0, 0.2))
    r = compare(traj, read_csv_log(p)[0], None)
    assert r["position"]["mean_dev_mm"] == pytest.approx(0.2, abs=0.01)
    assert r["cycle_time"]["ratio"] == pytest.approx(1.0, abs=0.02)


def test_cli_import_compare_and_check(tmp_path, traj, capsys):
    p = tmp_path / "run.csv"
    write_log(p, traj, 50, 250)
    out = tmp_path / "run.jsonl"
    assert main(["import-log", str(p), "--out", str(out)]) == 0
    assert "match it to the program by path" in capsys.readouterr().out
    assert len(out.read_text().splitlines()) > 150
    assert main(["compare", str(DATA / "rect_laps.nc"), "--setup", str(DATA / "rect_setup.yaml"),
                 "--recording", str(p)]) == 0
    assert "by path" in capsys.readouterr().out
    assert main(["check-recording", str(p), "--setup", str(DATA / "rect_setup.yaml")]) == 0
    text = capsys.readouterr().out
    assert "travel limits ok" in text and "collisions   none" in text
