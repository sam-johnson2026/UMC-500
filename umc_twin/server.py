"""Tiny local server: serves the web viewer and a JSON API. Standard library only.

    GET  /                 the viewer (web/index.html)
    POST /api/simulate     {"gcode": str, "setup": yaml str?, "options": {..}?, "collisions": bool?}
    GET  /api/live         server-sent events: MachineState at ~20 Hz from the live source
    GET  /api/status       {"live_source": name | null}
"""
from __future__ import annotations

import json
import tempfile
import time
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from .config import REPO_ROOT, load_machine
from .job import default_setup, load_setup
from .kinematics import Kinematics
from .live import LiveSource
from .manifest import current_manifest
from .sim import run_job

WEB_ROOT = REPO_ROOT / "web"


def make_handler(live: LiveSource | None):
    class Handler(SimpleHTTPRequestHandler):
        def __init__(self, *a, **kw):
            super().__init__(*a, directory=str(WEB_ROOT), **kw)

        def log_message(self, fmt, *args):  # quieter: only API calls
            if self.path.startswith("/api/"):
                super().log_message(fmt, *args)

        def _json(self, obj, status=HTTPStatus.OK):
            body = json.dumps(obj).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path.split("?")[0] == "/assets/machine.json":
                return self._json(current_manifest())  # always reflects config/umc500.yaml
            if self.path == "/api/status":
                return self._json({"live_source": live.name if live else None})
            if self.path == "/api/live":
                if live is None:
                    return self._json({"error": "no live source configured"}, HTTPStatus.NOT_FOUND)
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                try:
                    while True:
                        state = live.read()
                        if state is not None:
                            self.wfile.write(f"data: {json.dumps(state.as_dict())}\n\n".encode())
                            self.wfile.flush()
                        time.sleep(0.05)
                except (BrokenPipeError, ConnectionResetError):
                    return
            return super().do_GET()

        def do_POST(self):
            if self.path != "/api/simulate":
                return self._json({"error": "not found"}, HTTPStatus.NOT_FOUND)
            try:
                req = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))))
                machine = load_machine(options=req.get("options") or None)
                kin = Kinematics(machine)
                if req.get("setup"):
                    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
                        f.write(req["setup"])
                    setup = load_setup(f.name, kin)
                    Path(f.name).unlink()
                else:
                    setup = default_setup(kin)
                result = run_job(req["gcode"], machine, setup, check_collisions=req.get("collisions", True))
                self._json(result)
            except Exception as e:  # report to the UI rather than dropping the connection
                self._json({"error": f"{type(e).__name__}: {e}"}, HTTPStatus.BAD_REQUEST)

    return Handler


def serve(port: int = 8000, live: LiveSource | None = None, host: str = "127.0.0.1"):
    httpd = ThreadingHTTPServer((host, port), make_handler(live))
    httpd.daemon_threads = True
    print(f"UMC-500 twin viewer on http://{host}:{port}/" + (f"  (live source: {live.name})" if live else ""))
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
