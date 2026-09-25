"""Long CAM-style programs: viewer thinning keeps the shape; analysis sees every sample."""
import subprocess
import sys
from pathlib import Path

from umc_twin import Kinematics, load_machine
from umc_twin.job import load_setup
from umc_twin.sim import run_job

BENCH = Path(__file__).resolve().parent.parent / "examples" / "bench"


def test_surface_program_thinned_for_viewer():
    text = subprocess.run([sys.executable, str(BENCH / "make_surface.py"), "4.0"], capture_output=True,
                          text=True, check=True).stdout
    m = load_machine()
    setup = load_setup(BENCH / "setup_bench.yaml", Kinematics(m))
    full = run_job(text, m, setup, check_collisions=False, max_samples=10**9)
    thin = run_job(text, m, load_setup(BENCH / "setup_bench.yaml", Kinematics(m)), check_collisions=False,
                   max_samples=800)
    assert len(thin["t"]) <= 800 < thin["samples_total"] == len(full["t"])
    assert thin["t"][-1] == full["t"][-1] and thin["summary"] == full["summary"]
    assert abs(sum(thin["material"]["removed_per_sample"]) - sum(full["material"]["removed_per_sample"])) < 1.0
    # every change of motion type survives the thinning
    changes = lambda r: sum(a != b for a, b in zip(r["motion"], r["motion"][1:]))  # noqa: E731
    assert changes(thin) == changes(full)
