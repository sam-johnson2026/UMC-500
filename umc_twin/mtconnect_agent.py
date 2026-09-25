"""A small fake MTConnect agent that serves any LiveSource (usually a simulated program).

It exists to test the twin's MTConnect client end to end without the machine, and to demo the
Live tab:

    python -m umc_twin fake-agent --replay examples/demo_5axis.nc --setup examples/setup_demo.yaml --port 5000
    python -m umc_twin serve --mtconnect http://127.0.0.1:5000

The documents follow the MTConnect standard's structure (Devices / Streams, one Linear
component per X/Y/Z, Rotary for B/C and the spindle, a Path with line / program / execution /
tool). A real agent has much more in it; the client only uses what it finds.
"""
from __future__ import annotations

import datetime as _dt
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from xml.sax.saxutils import escape

from .live import LiveSource

DEVICE = "UMC500"
NS_DEV = "urn:mtconnect.org:MTConnectDevices:1.7"
NS_STR = "urn:mtconnect.org:MTConnectStreams:1.7"

# (component element, component name, [(data item id, type, subType, category, extra attrs)])
LAYOUT = [
    ("Linear", "X", [("Xpos", "POSITION", "ACTUAL", "SAMPLE", 'coordinateSystem="MACHINE" units="MILLIMETER"'),
                     ("Xwork", "POSITION", "ACTUAL", "SAMPLE", 'coordinateSystem="WORK" units="MILLIMETER"')]),
    ("Linear", "Y", [("Ypos", "POSITION", "ACTUAL", "SAMPLE", 'coordinateSystem="MACHINE" units="MILLIMETER"')]),
    ("Linear", "Z", [("Zpos", "POSITION", "ACTUAL", "SAMPLE", 'coordinateSystem="MACHINE" units="MILLIMETER"')]),
    ("Rotary", "B", [("Bang", "ANGLE", "ACTUAL", "SAMPLE", 'units="DEGREE"')]),
    ("Rotary", "C", [("Cang", "ANGLE", "ACTUAL", "SAMPLE", 'units="DEGREE"')]),
    ("Rotary", "S", [("Sspeed", "ROTARY_VELOCITY", "ACTUAL", "SAMPLE", 'units="REVOLUTION/MINUTE"'),
                     ("Sload", "LOAD", "", "SAMPLE", 'units="PERCENT"')]),
    ("Path", "path", [("line", "LINE_NUMBER", "ABSOLUTE", "EVENT", ""), ("program", "PROGRAM", "", "EVENT", ""),
                      ("exec", "EXECUTION", "", "EVENT", ""), ("tool", "TOOL_NUMBER", "", "EVENT", ""),
                      ("mode", "CONTROLLER_MODE", "", "EVENT", "")]),
]


def probe_xml() -> str:
    comps = []
    for i, (el, name, items) in enumerate(LAYOUT):
        dis = "".join(
            f'<DataItem id="{did}" type="{typ}"' + (f' subType="{sub}"' if sub else "") +
            f' category="{cat}" {extra}/>' for did, typ, sub, cat, extra in items)
        comps.append(f'<{el} id="c{i}" name="{name}"><DataItems>{dis}</DataItems></{el}>')
    return (f'<?xml version="1.0" encoding="UTF-8"?><MTConnectDevices xmlns="{NS_DEV}"><Header creationTime="{_now()}" '
            f'sender="umc-twin" instanceId="1" version="1.7" bufferSize="1"/><Devices>'
            f'<Device id="d1" name="{DEVICE}" uuid="umc500-twin"><Components>{"".join(comps)}</Components></Device>'
            f'</Devices></MTConnectDevices>')


def current_xml(source: LiveSource, seq: int) -> str:
    st = source.read()
    q = st.q if st else None
    values = {
        "Xpos": q and q[0], "Ypos": q and q[1], "Zpos": q and q[2], "Bang": q and q[3], "Cang": q and q[4],
        "Xwork": None, "Sspeed": st and st.spindle_rpm, "Sload": st and st.spindle_load,
        "line": st and st.line, "program": st and (st.program or "O01000"),
        "exec": st and ("ACTIVE" if st.mode == "SIM" else st.execution), "tool": st and st.tool,
        "mode": "AUTOMATIC" if st else None,
    }
    ts = _now()
    comps = []
    for el, name, items in LAYOUT:
        groups = {"SAMPLE": [], "EVENT": []}
        for did, typ, sub, cat, _ in items:
            v = values.get(did)
            text = "UNAVAILABLE" if v is None else (f"{v:.4f}" if isinstance(v, float) else escape(str(v)))
            tag = "".join(p.capitalize() for p in typ.split("_"))
            sub_attr = f' subType="{sub}"' if sub else ""
            groups[cat].append(f'<{tag} dataItemId="{did}" timestamp="{ts}" sequence="{seq}"{sub_attr}>{text}</{tag}>')
        body = "".join(f"<{'Samples' if c == 'SAMPLE' else 'Events'}>{''.join(v)}</{'Samples' if c == 'SAMPLE' else 'Events'}>"
                       for c, v in groups.items() if v)
        comps.append(f'<ComponentStream component="{el}" name="{name}" componentId="c">{body}</ComponentStream>')
    return (f'<?xml version="1.0" encoding="UTF-8"?><MTConnectStreams xmlns="{NS_STR}"><Header creationTime="{ts}" '
            f'sender="umc-twin" instanceId="1" version="1.7" bufferSize="1" nextSequence="{seq + 1}"/><Streams>'
            f'<DeviceStream name="{DEVICE}" uuid="umc500-twin">{"".join(comps)}</DeviceStream></Streams></MTConnectStreams>')


def _now() -> str:
    return _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")


def make_server(source: LiveSource, port: int = 5000, host: str = "127.0.0.1") -> ThreadingHTTPServer:
    counter = {"seq": 0}

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            path = self.path.split("?")[0].rstrip("/")
            if path in ("/probe", f"/{DEVICE}/probe", ""):
                body = probe_xml()
            elif path in ("/current", f"/{DEVICE}/current"):
                counter["seq"] += 1
                body = current_xml(source, counter["seq"])
            else:
                self.send_error(404)
                return
            data = body.encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/xml")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

    httpd = ThreadingHTTPServer((host, port), Handler)
    httpd.daemon_threads = True
    return httpd
