import shutil
from pathlib import Path

from umc_twin import load_machine
from umc_twin.check import check, find_programs, format_html

EXAMPLES = Path(__file__).resolve().parent.parent / "examples"


def test_batch_check_grades_and_pairs_setups(tmp_path):
    for name in ("demo_5axis.nc", "crash_demo.nc"):
        shutil.copy(EXAMPLES / name, tmp_path / name)
    shutil.copy(EXAMPLES / "setup_demo.yaml", tmp_path / "crash_demo.yaml")   # paired by name
    (tmp_path / "broken.nc").write_text("G1 X10.\n")                          # feed move with no F
    assert len(find_programs([tmp_path])) == 3
    rows = {Path(r["program"]).name: r for r in check([tmp_path], load_machine(), EXAMPLES / "setup_demo.yaml")}
    assert rows["demo_5axis.nc"]["status"] == "pass"
    crash = rows["crash_demo.nc"]
    assert crash["status"] == "fail" and crash["setup"].endswith("crash_demo.yaml")
    lines = [int(n.split(":")[0].split()[1]) for n in crash["notes"]]
    assert lines == sorted(lines)
    assert rows["broken.nc"]["status"] == "fail" and "no F word" in rows["broken.nc"]["notes"][0]
    page = format_html(list(rows.values()), "UMC")
    assert "<title>Pre-flight report</title>" in page and "crash_demo.nc" in page
