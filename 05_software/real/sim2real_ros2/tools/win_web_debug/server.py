#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import socket
import threading
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional

STATE_LOCK = threading.Lock()
LATEST_STATE: dict = {"type": "state", "connected": False}
NANO_ADDR: tuple[str, int]
UDP_SOCK: socket.socket


class Handler(SimpleHTTPRequestHandler):
    def do_GET(self) -> None:
        if self.path == "/api/state":
            with STATE_LOCK:
                data = json.dumps(LATEST_STATE).encode("utf-8")
            self._json(200, data)
            return
        super().do_GET()

    def do_POST(self) -> None:
        if self.path not in ("/api/control", "/api/state"):
            self.send_error(404)
            return
        length = int(self.headers.get("Content-Length", "0"))
        body = self.rfile.read(length) if length else b"{}"
        try:
            payload = json.loads(body.decode("utf-8"))
            send_udp(payload)
            self._json(200, b'{"ok":true}')
        except Exception as exc:
            self._json(400, json.dumps({"ok": False, "error": str(exc)}).encode())

    def _json(self, code: int, data: bytes) -> None:
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, format: str, *args: object) -> None:
        return


def send_udp(payload: dict) -> None:
    data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
    UDP_SOCK.sendto(data, NANO_ADDR)


def udp_rx_loop(sock: socket.socket) -> None:
    global LATEST_STATE
    while True:
        try:
            data, _ = sock.recvfrom(65535)
            payload = json.loads(data.decode("utf-8"))
            payload["connected"] = True
            payload["local_receive_time"] = time.time()
            with STATE_LOCK:
                LATEST_STATE = payload
        except Exception:
            time.sleep(0.01)


def heartbeat_loop() -> None:
    while True:
        try:
            send_udp({"type": "ping", "stamp": time.time()})
        except Exception:
            pass
        time.sleep(0.5)


def main() -> None:
    global NANO_ADDR, UDP_SOCK
    parser = argparse.ArgumentParser(description="Windows local web debug UI for sim2real_ros2")
    parser.add_argument("--nano-host", required=True, help="Nano IP address")
    parser.add_argument("--nano-port", type=int, default=15000, help="Nano UDP listen port")
    parser.add_argument("--listen-host", default="0.0.0.0", help="Local HTTP host")
    parser.add_argument("--http-port", type=int, default=8088, help="Local HTTP port")
    parser.add_argument("--udp-port", type=int, default=15001, help="Local UDP receive port")
    args = parser.parse_args()

    NANO_ADDR = (args.nano_host, args.nano_port)
    UDP_SOCK = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    UDP_SOCK.bind(("0.0.0.0", args.udp_port))

    threading.Thread(target=udp_rx_loop, args=(UDP_SOCK,), daemon=True).start()
    threading.Thread(target=heartbeat_loop, daemon=True).start()

    static_dir = Path(__file__).resolve().parent / "static"
    handler = lambda *a, **kw: Handler(*a, directory=str(static_dir), **kw)
    httpd = ThreadingHTTPServer((args.listen_host, args.http_port), handler)
    print(f"Open http://127.0.0.1:{args.http_port}")
    print(f"UDP  Nano={args.nano_host}:{args.nano_port}  local={args.udp_port}")
    httpd.serve_forever()


if __name__ == "__main__":
    main()
