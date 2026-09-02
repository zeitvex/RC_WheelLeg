"""Minimal HTTP + SSE server for the sim2real web console."""

from __future__ import annotations

import argparse
import json
import os
import queue
import sys
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from web.session import RobotSession  # noqa: E402


SESSION: "RobotSession" = None  # type: ignore


def make_real_factory():
    def outer():
        sim2real_root = Path(__file__).resolve().parents[1]
        workspace_root = sim2real_root.parent.parent
        for path in (workspace_root,):
            path_str = str(path)
            if path_str not in sys.path and path.exists():
                sys.path.append(path_str)

        from dm_socket.sim2real_factory import driver_factory

        return driver_factory

    return outer


def make_dry_factory():
    def outer():
        class MockMotor:
            def __init__(self):
                class State:
                    position = 0.0
                    velocity = 0.0
                    torque = 0.0
                    update_count = 0

                self.state = State()

        class MockDriver:
            def __init__(self, port, debug):
                self.port = port
                self.motors = {}

            def connect(self):
                pass

            def disconnect(self):
                pass

            def add_motor(self, name, motor_id, model):
                self.motors[name] = MockMotor()

            def enable(self, name):
                pass

            def disable(self, name):
                pass

            def clear_warnings(self, name):
                pass

            def process_messages(self):
                for motor in self.motors.values():
                    motor.state.update_count += 1

            def control_mit(self, name, q, dq, kp, kd, tau):
                if name in self.motors:
                    self.motors[name].state.position = q
                    self.motors[name].state.velocity = dq
                    self.motors[name].state.torque = tau

        def factory(can1_port, can2_port, debug):
            return MockDriver(can1_port, debug), MockDriver(can2_port, debug)

        return factory

    return outer


def _send_json(handler: BaseHTTPRequestHandler, code: int, obj):
    body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
    handler.send_response(code)
    handler.send_header("Content-Type", "application/json; charset=utf-8")
    handler.send_header("Content-Length", str(len(body)))
    handler.send_header("Cache-Control", "no-store")
    handler.end_headers()
    handler.wfile.write(body)


def _send_static(handler: BaseHTTPRequestHandler, path: Path, content_type: str):
    if not path.exists():
        handler.send_error(404, str(path))
        return
    body = path.read_bytes()
    handler.send_response(200)
    handler.send_header("Content-Type", content_type)
    handler.send_header("Content-Length", str(len(body)))
    handler.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
    handler.send_header("Pragma", "no-cache")
    handler.send_header("Expires", "0")
    handler.end_headers()
    handler.wfile.write(body)


class Handler(BaseHTTPRequestHandler):
    server_version = "Sim2RealConsole/1.1"

    def log_message(self, fmt, *args):
        if "GET /events" in (fmt % args):
            return
        super().log_message(fmt, *args)

    def do_GET(self):
        url = urlparse(self.path)
        if url.path in ("/", "/index.html"):
            return _send_static(self, Path(__file__).parent / "static" / "index.html", "text/html; charset=utf-8")
        if url.path == "/static/app.js":
            return _send_static(self, Path(__file__).parent / "static" / "app.js", "application/javascript; charset=utf-8")
        if url.path == "/static/style.css":
            return _send_static(self, Path(__file__).parent / "static" / "style.css", "text/css; charset=utf-8")
        if url.path.startswith("/static/viewer/"):
            viewer_file = url.path.split("/static/viewer/", 1)[1]
            viewer_path = Path(__file__).parent / "static" / "viewer" / viewer_file
            content_type = "text/javascript" if not viewer_file.endswith(".css") else "text/css"
            return _send_static(self, viewer_path, content_type)
        if url.path.startswith("/meshes/"):
            mesh_name = url.path.split("/meshes/", 1)[1]
            mesh_path = Path(__file__).resolve().parents[1] / "mjcf" / "meshes" / mesh_name
            if not mesh_path.exists():
                return self.send_error(404, f"mesh not found: {mesh_name}")
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Content-Length", str(mesh_path.stat().st_size))
            self.send_header("Cache-Control", "max-age=3600")
            self.end_headers()
            with open(mesh_path, "rb") as file_obj:
                while True:
                    chunk = file_obj.read(64 * 1024)
                    if not chunk:
                        break
                    self.wfile.write(chunk)
            return
        if url.path.startswith("/mjcf/"):
            mjcf_name = url.path.split("/mjcf/", 1)[1]
            mjcf_path = Path(__file__).resolve().parents[1] / "mjcf" / mjcf_name
            if not mjcf_path.exists():
                return self.send_error(404, f"mjcf not found: {mjcf_name}")
            self.send_response(200)
            self.send_header("Content-Type", "application/xml; charset=utf-8")
            self.send_header("Content-Length", str(mjcf_path.stat().st_size))
            self.end_headers()
            self.wfile.write(mjcf_path.read_bytes())
            return
        if url.path == "/api/status":
            return _send_json(self, 200, SESSION.get_status())
        if url.path == "/api/debug":
            return _send_json(self, 200, SESSION.get_debug_snapshot())
        if url.path == "/api/logs":
            return _send_json(self, 200, {"sessions": SESSION.list_logs()})
        if url.path.startswith("/api/logs/"):
            parts = url.path.split("/")
            if len(parts) >= 5:
                session_id = parts[3]
                filename = parts[4]
                file_path = Path(SESSION.cfg.get("log_dir", "logs")) / session_id / filename
                if file_path.exists() and filename in ("state.csv", "events.jsonl"):
                    self.send_response(200)
                    self.send_header(
                        "Content-Type",
                        "text/csv" if filename.endswith("csv") else "application/json",
                    )
                    self.send_header("Content-Disposition", f'attachment; filename="{session_id}_{filename}"')
                    self.send_header("Content-Length", str(file_path.stat().st_size))
                    self.end_headers()
                    with open(file_path, "rb") as file_obj:
                        while True:
                            chunk = file_obj.read(64 * 1024)
                            if not chunk:
                                break
                            self.wfile.write(chunk)
                    return
            return self.send_error(404)
        if url.path == "/events":
            return self._handle_sse()
        return self.send_error(404, self.path)

    def do_POST(self):
        url = urlparse(self.path)
        try:
            length = int(self.headers.get("Content-Length", "0"))
            body = self.rfile.read(length) if length else b""
            data = json.loads(body) if body else {}
        except Exception as exc:
            SESSION.note_api_error()
            return _send_json(self, 400, {"error": f"bad body: {exc}"})

        try:
            result = self._handle_post(url.path, data)
        except Exception as exc:
            SESSION.note_api_error()
            return _send_json(
                self,
                500,
                {
                    "error": f"{type(exc).__name__}: {exc}",
                    "traceback": traceback.format_exc(),
                },
            )
        if result is None:
            return self.send_error(404)
        return _send_json(self, 200, {"ok": True, **(result if isinstance(result, dict) else {})})

    def _handle_post(self, path: str, data: dict):
        if path == "/api/connect":
            return {"queued": SESSION.connect(dry_run=bool(data.get("dry_run", False)))}
        if path == "/api/disconnect":
            return {"queued": SESSION.disconnect()}
        if path == "/api/enable":
            return {"queued": SESSION.enable_motors()}
        if path == "/api/disable":
            return {"queued": SESSION.disable_motors()}
        if path == "/api/test_motor":
            return {
                "queued": SESSION.test_motor(
                    leg=data["leg"],
                    joint=data["joint"],
                    delta_rad=float(data.get("delta_rad", 0.1)),
                    kp=float(data.get("kp", 5.0)),
                    kd=float(data.get("kd", 1.0)),
                    duration_s=float(data.get("duration_s", 1.0)),
                )
            }
        if path == "/api/calibrate_offsets":
            return {
                "queued": SESSION.calibrate_offsets(
                    target_pose_name=data.get("target_pose", "stand"),
                    samples=int(data.get("samples", 100)),
                )
            }
        if path == "/api/startup":
            return {"queued": SESSION.startup()}
        if path == "/api/runtime/start":
            return {"queued": SESSION.runtime_start(policy_path=data.get("policy_path"))}
        if path == "/api/runtime/stop":
            return {"queued": SESSION.runtime_stop()}
        if path == "/api/cmd":
            SESSION.set_command(
                vx=float(data.get("vx", 0.0)),
                vy=float(data.get("vy", 0.0)),
                yaw=float(data.get("yaw", 0.0)),
            )
            return {}
        if path == "/api/remote_takeover":
            return {"ok": SESSION.set_remote_takeover(enabled=bool(data.get("enabled", False)))}
        if path == "/api/estop":
            SESSION.estop()
            return {}
        if path == "/api/reset_estop":
            SESSION.reset_estop()
            return {}
        return None

    def _handle_sse(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()

        event_queue: "queue.Queue" = queue.Queue(maxsize=1024)
        SESSION.add_listener(event_queue)
        try:
            initial = {"kind": "STATUS_FULL", **SESSION.get_status()}
            self.wfile.write(f"data: {json.dumps(initial, ensure_ascii=False)}\n\n".encode())
            self.wfile.flush()
            last_keepalive = time.time()
            while True:
                try:
                    event = event_queue.get(timeout=1.0)
                    self.wfile.write(f"data: {json.dumps(event, ensure_ascii=False)}\n\n".encode())
                    self.wfile.flush()
                except queue.Empty:
                    if time.time() - last_keepalive > 15:
                        self.wfile.write(b": keepalive\n\n")
                        self.wfile.flush()
                        last_keepalive = time.time()
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            SESSION.remove_listener(event_queue)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--config", default=str(Path(__file__).resolve().parents[1] / "config.yaml"))
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    cfg_path = Path(args.config)
    with open(cfg_path, "r", encoding="utf-8") as file_obj:
        cfg = yaml.safe_load(file_obj)

    global SESSION
    SESSION = RobotSession(
        cfg=cfg,
        cfg_path=cfg_path,
        driver_factory_real=make_real_factory(),
        driver_factory_dry=make_dry_factory(),
    )

    def _pulse():
        while True:
            try:
                SESSION._broadcast({"kind": "PULSE", **SESSION.get_status()})
            except Exception:
                pass
            time.sleep(1.0)

    threading.Thread(target=_pulse, daemon=True).start()

    httpd = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"\n[Web] sim2real console -> http://{args.host}:{args.port}\n")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[Web] Ctrl+C received, shutting down...")
    finally:
        if SESSION is not None and SESSION.status.stage == "RUNTIME":
            try:
                SESSION.runtime_stop()
            except Exception:
                pass
        try:
            SESSION._do_disconnect()
        except Exception:
            pass
        httpd.server_close()
        os._exit(0)


if __name__ == "__main__":
    main()
