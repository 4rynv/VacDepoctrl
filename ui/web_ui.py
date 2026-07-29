# web_ui.py — Browser dashboard server, running in parallel with the
# Tkinter GUI (never instead of it).
#
# Design constraints, in priority order:
#
# 1. The local Tkinter GUI is the fallback of record: this server is a
#    second *view* of the same process, so a network problem can only ever
#    cost the browser page -- the control loop and the local GUI don't
#    know or care whether anyone is connected here.
# 2. Nothing here may crash the app: a busy port degrades to a logged
#    warning (app runs on without the web UI), request-handler exceptions
#    answer that one request with an error, and every server thread is a
#    daemon.
# 3. Stdlib only (http.server + Server-Sent Events), matching this repo's
#    no-dependency ethos (hand-rolled Modbus, pure-Tkinter graphs): there
#    is nothing to pip-install on the Pi for this to work.
#
# Wire protocol:
#   GET  /          -> the dashboard page (web/index.html, self-contained)
#   GET  /events    -> SSE stream; one JSON state snapshot per interval
#   POST /command   -> {"action": <name>, "value": <optional>} dispatched
#                      through the injected commands dict; every accepted
#                      command is written to the persistent event log with
#                      the client's IP (audit trail).
#
# SSE (one-way server->browser push over plain HTTP) rather than
# websockets: telemetry is inherently one-way, commands fit ordinary POSTs,
# EventSource reconnects automatically, and it needs no library on either
# end. One server thread per connected browser -- fine for the handful of
# clients a lab rig sees.
#
# SECURITY: no authentication -- anyone on the LAN who can reach the port
# can operate the rig, the same trust model as the Pi's VNC/SSH access.
# See the README's Web Dashboard section before exposing the Pi to a
# network you don't control.

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


class WebUI:
    def __init__(self, get_snapshot, commands, page_path, port,
                 update_interval, log=None):
        """
        get_snapshot     -- () -> JSON-serializable dict of live state
        commands         -- {action_name: callable(value) -> (ok, message)};
                            callables must be thread-safe (they run on
                            request threads, exactly like Tk button
                            callbacks already run off the poll thread)
        page_path        -- filesystem path of the dashboard HTML
        port             -- TCP port to serve on (0 = ephemeral, for tests)
        update_interval  -- seconds between SSE snapshots
        log              -- optional callable(msg) for the audit trail
        """
        self.get_snapshot = get_snapshot
        self.commands = commands
        self.page_path = page_path
        self.port = port
        self.update_interval = update_interval
        self.log = log or (lambda msg: None)
        self._server = None

    def start(self):
        """Bind and serve on a daemon thread. Returns True if serving,
        False if the port couldn't be bound (already logged) -- the caller
        should carry on either way."""
        ui = self

        class _Handler(BaseHTTPRequestHandler):
            # Silence the default per-request stderr line -- the poll
            # loop's console is for hardware events, not HTTP chatter.
            def log_message(self, fmt, *args):
                pass

            def _send_json(self, obj, status=200):
                body = json.dumps(obj).encode()
                self.send_response(status)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                if self.path in ("/", "/index.html"):
                    self._serve_page()
                elif self.path == "/events":
                    self._serve_events()
                else:
                    self.send_error(404)

            def _serve_page(self):
                try:
                    with open(ui.page_path, "rb") as f:
                        body = f.read()
                except OSError as e:
                    self.send_error(500, f"dashboard page unavailable: {e}")
                    return
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _serve_events(self):
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                try:
                    while True:
                        try:
                            data = json.dumps(ui.get_snapshot())
                        except Exception:
                            # One bad snapshot must not kill the stream --
                            # skip the tick; the next one usually succeeds.
                            time.sleep(ui.update_interval)
                            continue
                        self.wfile.write(b"data: " + data.encode() + b"\n\n")
                        self.wfile.flush()
                        time.sleep(ui.update_interval)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    pass  # browser navigated away / network dropped

            def do_POST(self):
                if self.path != "/command":
                    self.send_error(404)
                    return
                try:
                    length = int(self.headers.get("Content-Length", 0))
                    payload = json.loads(self.rfile.read(length) or b"{}")
                except (ValueError, TypeError):
                    self._send_json({"ok": False, "message": "Malformed JSON body."}, status=400)
                    return

                action = payload.get("action")
                value = payload.get("value")
                fn = ui.commands.get(action)
                if fn is None:
                    self._send_json({"ok": False, "message": f"Unknown action: {action!r}"}, status=400)
                    return

                client = self.client_address[0]
                ui.log(f"[web {client}] command: {action}"
                       + (f" = {value!r}" if value is not None else ""))
                try:
                    ok, msg = fn(value)
                except Exception as e:
                    # A handler bug answers this request with an error --
                    # it must never take down the server (or the app).
                    self._send_json({"ok": False, "message": f"Command failed: {e}"}, status=500)
                    return
                self._send_json({"ok": bool(ok), "message": msg})

        try:
            self._server = ThreadingHTTPServer(("", self.port), _Handler)
        except OSError as e:
            # Port taken (a second instance?) or bind refused: the web UI
            # is optional -- warn and let the app run without it.
            self.log(f"Web UI disabled: could not bind port {self.port} ({e}).")
            print(f"[WEB UI] disabled: could not bind port {self.port} ({e})")
            return False

        self._server.daemon_threads = True
        self.port = self._server.server_address[1]  # resolves port 0 for tests
        # poll_interval only affects how quickly shutdown() is noticed (it
        # is not a request-handling rate) -- the default 0.5s made each
        # start/stop cycle in the test suite take ~1s for no benefit.
        threading.Thread(target=lambda: self._server.serve_forever(poll_interval=0.05),
                         daemon=True, name="web-ui").start()
        self.log(f"Web UI serving on port {self.port}.")
        return True

    def stop(self):
        """Clean shutdown -- used by tests; the app itself just exits and
        lets the daemon threads die."""
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
            self._server = None
