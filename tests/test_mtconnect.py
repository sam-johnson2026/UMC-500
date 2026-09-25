import socket
import threading
import time
from pathlib import Path

import pytest

from umc_twin import Kinematics, load_machine
from umc_twin.gcode import simulate
from umc_twin.job import load_setup
from umc_twin.live import JsonlLogger, LogReplaySource, MachineState, ReplaySource
from umc_twin.mtconnect import MTConnectSource, auto_mapping, parse_probe, state_from_values
from umc_twin.mtconnect_agent import make_server, probe_xml

EXAMPLES = Path(__file__).resolve().parent.parent / "examples"


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture(scope="module")
def agent():
    kin = Kinematics(load_machine(calibration=None))
    setup = load_setup(EXAMPLES / "setup_demo.yaml", kin)
    replay = ReplaySource(simulate((EXAMPLES / "demo_5axis.nc").read_text(), kin, setup), speed=5.0)
    port = free_port()
    httpd = make_server(replay, port=port)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    yield f"http://127.0.0.1:{port}", replay
    httpd.shutdown()


def test_auto_mapping_prefers_machine_coordinates():
    m = auto_mapping(parse_probe(probe_xml()))
    assert m.items["X"] == "Xpos"          # not the WORK position Xwork
    assert {"Y", "Z", "B", "C", "spindle_rpm", "spindle_load", "line", "tool", "execution"} <= set(m.items)
    assert m.notes == []


def test_overrides_and_missing_axes():
    m = auto_mapping(parse_probe(probe_xml()), overrides={"X": "Xwork"})
    assert m.items["X"] == "Xwork"
    xml = probe_xml().replace('name="C"', 'name="A"')
    m = auto_mapping(parse_probe(xml))
    assert any("C: no ANGLE" in n for n in m.notes)


def test_unavailable_values():
    m = auto_mapping(parse_probe(probe_xml()))
    vals = {"Xpos": "UNAVAILABLE", "Ypos": "1", "Zpos": "2", "Bang": "3", "Cang": "4"}
    assert state_from_values(vals, m) is None                       # no pose yet
    st = state_from_values(vals, m, last_q=[9, 0, 0, 0, 0])
    assert st.q == [9, 1, 2, 3, 4] and st.line is None


def test_client_follows_the_fake_agent(agent):
    url, replay = agent
    src = MTConnectSource(url, interval=0.05)
    try:
        deadline = time.time() + 5
        while src.read() is None and time.time() < deadline:
            time.sleep(0.05)
        st = src.read()
        assert st is not None and src.error is None
        truth = replay.read()
        assert abs(st.q[0] - truth.q[0]) < 60        # replay runs at 5x; allow for poll lag
        assert st.program == "O01000" and st.execution == "ACTIVE"
    finally:
        src.close()


def test_record_and_replay(tmp_path):
    class Fake:
        name = "fake"

        def __init__(self):
            self.i = 0

        def read(self):
            self.i += 1
            return MachineState(q=[float(self.i), 0, 0, 0, 0], source="fake", timestamp=1000.0 + self.i * 0.1, line=self.i)

    log = tmp_path / "run.jsonl"
    rec = JsonlLogger(Fake(), log, min_interval=0.0)
    for _ in range(5):
        rec.read()
    assert len(log.read_text().splitlines()) == 5
    rep = LogReplaySource(log, speed=1.0, loop=False)
    first = rep.read()
    assert first.source == "recording" and first.q[0] == 1.0 and first.line == 1
