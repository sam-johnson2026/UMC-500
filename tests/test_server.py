import json
import socket
import threading
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

import pytest

from umc_twin import Kinematics, load_machine
from umc_twin.compare import synthetic_recording
from umc_twin.gcode import simulate
from umc_twin.job import load_setup
from umc_twin.server import make_handler

EXAMPLES = Path(__file__).resolve().parent.parent / "examples"
PROGRAM = "G21 G90\nT1 M6\nG54 G43 H1 S5000 M3\nG0 X0 Y0 Z5.\nG1 Z-1. F500.\nG1 X20. F1000.\nG0 Z20.\nM30"
SETUP = "tools:\n  1: {length: 100, diameter: 10}\nstock: {type: box, size: [60, 60, 20], position: [0, 0, -70.8]}\n"


@pytest.fixture(scope="module")
def base_url():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    httpd = ThreadingHTTPServer(("127.0.0.1", port), make_handler(None))
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    yield f"http://127.0.0.1:{port}"
    httpd.shutdown()


def post(url, body):
    req = urllib.request.Request(url, json.dumps(body).encode(), {"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=60) as r:
            return json.loads(r.read())
    except urllib.error.HTTPError as e:
        return json.loads(e.read())


def get(url):
    with urllib.request.urlopen(url, timeout=30) as r:
        return r.read()


def test_status_manifest_and_viewer(base_url):
    assert json.loads(get(base_url + "/api/status")) == {"live_source": None}
    manifest = json.loads(get(base_url + "/assets/machine.json"))
    assert "joints" in manifest["config"] and "base_casting" in manifest["parts"]
    assert b"UMC-500 Twin" in get(base_url + "/")


def test_simulate(base_url):
    r = post(base_url + "/api/simulate", {"gcode": PROGRAM, "setup": SETUP})
    assert r["summary"]["duration_s"] > 0 and r["material"]["removed_volume_mm3"] > 0
    bad = post(base_url + "/api/simulate", {"gcode": "G1 X10."})
    assert "no F word" in bad["error"]


def test_compare(base_url):
    m = load_machine()
    kin = Kinematics(m)
    setup = load_setup(EXAMPLES / "setup_demo.yaml", kin)
    text = (EXAMPLES / "demo_5axis.nc").read_text()
    rec = synthetic_recording(simulate(text, kin, setup), time_scale=1.1)
    r = post(base_url + "/api/compare", {"gcode": text, "setup": (EXAMPLES / "setup_demo.yaml").read_text(),
                                         "recording": "\n".join(json.dumps(s) for s in rec)})
    assert r["cycle_time"]["ratio"] == pytest.approx(1.1, abs=0.01) and "cycle time" in r["text"]


def test_calibrate_preview_does_not_save(base_url, tmp_path, monkeypatch):
    import umc_twin.calibration as cal
    monkeypatch.setattr(cal, "DEFAULT_CALIBRATION", tmp_path / "cal.yaml")
    r = post(base_url + "/api/calibrate", {"nose_to_platter": 250.0, "save": False})
    assert r["summary"]["nose_to_platter_at_home_mm"] == pytest.approx(250.0) and not r["saved"]
    assert "error" in post(base_url + "/api/calibrate", {"b_direction": "up"})
