#!/usr/bin/env python3
"""Pre-compute the example programs into web/assets/examples/*.json so the viewer's
"Load demo" buttons work even when it is hosted statically (no Python server)."""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from umc_twin.config import load_machine  # noqa: E402
from umc_twin.job import load_setup  # noqa: E402
from umc_twin.kinematics import Kinematics  # noqa: E402
from umc_twin.sim import format_report, run_job  # noqa: E402

EXAMPLES = {"demo_5axis": "setup_demo.yaml", "crash_demo": "setup_demo.yaml", "macro_comp_demo": "setup_demo.yaml"}

out_dir = ROOT / "web" / "assets" / "examples"
out_dir.mkdir(parents=True, exist_ok=True)
machine = load_machine()
kin = Kinematics(machine)
for name, setup_file in EXAMPLES.items():
    setup_path = ROOT / "examples" / setup_file
    result = run_job((ROOT / "examples" / f"{name}.nc").read_text(), machine, load_setup(setup_path, kin))
    result["setup_yaml"] = setup_path.read_text()
    (out_dir / f"{name}.json").write_text(json.dumps(result, separators=(",", ":")))
    print(f"== {name}\n{format_report(result)}")
