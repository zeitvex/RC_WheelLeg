#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import socket
import threading
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sim2real_interfaces.msg import RuntimeState, RuntimeTarget
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


class WebUdpBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("sim2real_web_udp_bridge_node", allow_undeclared_parameters=True)

        self.enabled = bool(self.declare_parameter("web_bridge_enabled", True).value)
        self.listen_host = str(self.declare_parameter("web_udp_listen_host", "0.0.0.0").value)
        self.listen_port = int(self.declare_parameter("web_udp_listen_port", 15000).value)
        self.remote_host = str(self.declare_parameter("web_udp_remote_host", "").value)
        self.remote_port = int(self.declare_parameter("web_udp_remote_port", 15001).value)
        self.state_hz = float(self.declare_parameter("web_udp_state_hz", 20.0).value)
        self.cmd_timeout_ms = float(self.declare_parameter("web_udp_cmd_timeout_ms", 300.0).value)
        self.max_packet_bytes = int(self.declare_parameter("web_udp_max_packet_bytes", 8192).value)
        self.max_vx = float(self.declare_parameter("web_udp_max_vx", 0.8).value)
        self.max_vy = float(self.declare_parameter("web_udp_max_vy", 0.3).value)
        self.max_yaw = float(self.declare_parameter("web_udp_max_yaw_rate", 0.5).value)
        self.estop_on_timeout = bool(self.declare_parameter("web_udp_estop_on_timeout", False).value)

        self.http_host = str(self.declare_parameter("web_http_host", "0.0.0.0").value)
        self.http_port = int(self.declare_parameter("web_http_port", 18080).value)
        self.web_static_dir = str(self.declare_parameter("web_static_dir", "").value)
        self.nav_map_frame = str(self.declare_parameter("nav_map_frame", "map").value)
        self.nav_base_frame = str(self.declare_parameter("nav_base_frame", "base_link").value)
        self.pcd_nav_file = str(self.declare_parameter("pcd_nav_file", "").value)
        self.pcd_floor_z_min = float(self.declare_parameter("pcd_floor_z_min", -1.6).value)
        self.pcd_floor_z_max = float(self.declare_parameter("pcd_floor_z_max", 0.4).value)
        self.pcd_sample_step = max(1, int(self.declare_parameter("pcd_sample_step", 25).value))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.sock.bind((self.listen_host, self.listen_port))

        self.client_addr: Optional[tuple[str, int]] = None
        if self.remote_host:
            self.client_addr = (self.remote_host, self.remote_port)

        self.latest_target: Optional[RuntimeTarget] = None
        self.latest_state: Optional[RuntimeState] = None
        self.latest_cmd = Twist()
        self.latest_mode = "UNKNOWN"
        self.latest_mux_status = ""
        self.latest_nav_status = ""
        self.estop = False
        self.web_enabled = False
        self.last_cmd_time = self.get_clock().now()
        self.timeout_estop_sent = False

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel_web", 10)
        self.nav_cmd_pub = self.create_publisher(String, "/simple_nav/cmd", 10)
        self.estop_pub = self.create_publisher(Bool, "/safety/estop", 10)
        self.web_enabled_pub = self.create_publisher(Bool, "web/enabled", 10)
        self.remote_enabled_pub = self.create_publisher(Bool, "remote/enabled", 10)
        self.nav_enabled_pub = self.create_publisher(Bool, "nav/enabled", 10)
        self.mode_pub = self.create_publisher(String, "control/mode", 10)

        self.create_subscription(RuntimeTarget, "runtime/target", self.on_target, 10)
        self.create_subscription(RuntimeState, "runtime/state", self.on_state, 10)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(Bool, "/safety/estop", self.on_estop, 10)
        self.create_subscription(String, "control/mode_state", self.on_mode_state, 10)
        self.create_subscription(String, "control/mux_status", self.on_mux_status, 10)
        self.create_subscription(String, "simple_nav/status", self.on_nav_status, 10)

        self.map_points = self.load_filtered_pcd(Path(self.pcd_nav_file)) if self.pcd_nav_file else []
        self.http_server: Optional[ThreadingHTTPServer] = None
        self.http_thread: Optional[threading.Thread] = None
        self.static_dir = Path(self.web_static_dir) if self.web_static_dir else Path(__file__).resolve().parents[3] / "tools" / "win_web_debug" / "static"
        self.start_http_server()

        self.rx_timer = self.create_timer(0.01, self.on_rx_timer)
        self.state_timer = self.create_timer(1.0 / self.state_hz if self.state_hz > 0.0 else 0.05, self.on_state_timer)
        self.guard_timer = self.create_timer(0.05, self.on_guard_timer)

        self.get_logger().info(f"Web UDP bridge listening on {self.listen_host}:{self.listen_port}")
        self.get_logger().info(f"Web HTTP UI serving on http://{self.http_host}:{self.http_port}")

    def load_filtered_pcd(self, path: Path) -> list[list[float]]:
        if not path.exists():
            self.get_logger().warn(f"PCD file not found: {path}")
            return []

        points: list[list[float]] = []
        data_started = False
        with path.open("r", encoding="utf-8") as f:
            for line in f:
                stripped = line.strip()
                if not stripped:
                    continue
                if data_started:
                    parts = stripped.split()
                    if len(parts) < 3:
                        continue
                    try:
                        x = float(parts[0])
                        y = float(parts[1])
                        z = float(parts[2])
                    except ValueError:
                        continue
                    if self.pcd_floor_z_min <= z <= self.pcd_floor_z_max:
                        points.append([round(x, 3), round(y, 3)])
                elif stripped.upper().startswith("DATA"):
                    data_started = True
        return points[:: self.pcd_sample_step]

    def lookup_pose(self) -> Optional[dict[str, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(self.nav_map_frame, self.nav_base_frame, rclpy.time.Time())
        except TransformException:
            return None
        t = transform.transform.translation
        q = transform.transform.rotation
        yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        return {"x": round(float(t.x), 3), "y": round(float(t.y), 3), "yaw": round(float(yaw), 6)}

    def start_http_server(self) -> None:
        node = self
        static_dir = self.static_dir

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path in ["/", "/index.html"]:
                    self.serve_file(static_dir / "index.html", "text/html; charset=utf-8")
                elif self.path == "/app.js":
                    self.serve_file(static_dir / "app.js", "application/javascript; charset=utf-8")
                elif self.path == "/style.css":
                    self.serve_file(static_dir / "style.css", "text/css; charset=utf-8")
                elif self.path == "/api/state":
                    self.send_json(node.build_state_packet())
                elif self.path == "/api/map":
                    self.send_json({
                        "points": node.map_points,
                        "map_frame": node.nav_map_frame,
                        "base_frame": node.nav_base_frame,
                        "pose": node.lookup_pose(),
                    })
                else:
                    self.send_error(HTTPStatus.NOT_FOUND)

            def do_POST(self):
                if self.path != "/api/control":
                    self.send_error(HTTPStatus.NOT_FOUND)
                    return
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    raw = self.rfile.read(length)
                    payload = json.loads(raw.decode("utf-8"))
                    node.handle_http_control(payload)
                    self.send_json({"ok": True})
                except Exception as exc:
                    self.send_json({"ok": False, "error": str(exc)}, status=HTTPStatus.BAD_REQUEST)

            def serve_file(self, path: Path, content_type: str):
                if not path.exists():
                    self.send_error(HTTPStatus.NOT_FOUND)
                    return
                data = path.read_bytes()
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def send_json(self, payload: dict[str, Any], status: HTTPStatus = HTTPStatus.OK):
                data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def log_message(self, format: str, *args):
                return

        self.http_server = ThreadingHTTPServer((self.http_host, self.http_port), Handler)
        self.http_thread = threading.Thread(target=self.http_server.serve_forever, daemon=True)
        self.http_thread.start()

    def handle_http_control(self, payload: dict[str, Any]) -> None:
        msg_type = str(payload.get("type", "")).lower()
        if msg_type in {"cmd_vel", "zero", "estop", "mode", "web_enable", "remote_enable", "nav_enable", "ping", "nav_cmd", "go_to", "go_rel"}:
            self.handle_packet(payload)
            return
        raise ValueError(f"unknown control type: {msg_type}")

    def on_target(self, msg: RuntimeTarget) -> None:
        self.latest_target = msg

    def on_state(self, msg: RuntimeState) -> None:
        self.latest_state = msg

    def on_cmd_vel(self, msg: Twist) -> None:
        self.latest_cmd = msg

    def on_estop(self, msg: Bool) -> None:
        self.estop = bool(msg.data)

    def on_mode_state(self, msg: String) -> None:
        self.latest_mode = msg.data

    def on_mux_status(self, msg: String) -> None:
        self.latest_mux_status = msg.data

    def on_nav_status(self, msg: String) -> None:
        self.latest_nav_status = msg.data

    def on_rx_timer(self) -> None:
        if not self.enabled:
            return
        while True:
            try:
                data, addr = self.sock.recvfrom(self.max_packet_bytes)
            except BlockingIOError:
                break
            except OSError as exc:
                self.get_logger().warn(f"UDP receive failed: {exc}")
                break
            self.client_addr = addr
            try:
                payload = json.loads(data.decode("utf-8"))
                self.handle_packet(payload)
            except Exception as exc:
                self.send_packet({"type": "error", "message": str(exc)})

    def handle_packet(self, payload: dict[str, Any]) -> None:
        msg_type = str(payload.get("type", "")).lower()
        if msg_type == "cmd_vel":
            cmd = self.parse_twist(payload)
            self.set_control_mode("WEB")
            self.cmd_pub.publish(cmd)
            self.last_cmd_time = self.get_clock().now()
            self.timeout_estop_sent = False
        elif msg_type == "zero":
            self.cmd_pub.publish(Twist())
            self.last_cmd_time = self.get_clock().now()
        elif msg_type == "estop":
            self.estop_pub.publish(Bool(data=bool(payload.get("data", True))))
        elif msg_type == "mode":
            mode = str(payload.get("mode", "DISABLED")).upper()
            self.set_control_mode(mode)
        elif msg_type == "web_enable":
            self.web_enabled = bool(payload.get("data", False))
            self.web_enabled_pub.publish(Bool(data=self.web_enabled))
            if self.web_enabled:
                self.set_control_mode("WEB")
        elif msg_type == "remote_enable":
            enabled = bool(payload.get("data", False))
            self.remote_enabled_pub.publish(Bool(data=enabled))
            if enabled:
                self.set_control_mode("REMOTE")
        elif msg_type == "nav_enable":
            enabled = bool(payload.get("data", False))
            self.nav_enabled_pub.publish(Bool(data=enabled))
            if enabled:
                self.set_control_mode("NAV")
        elif msg_type == "nav_cmd":
            command = str(payload.get("command", "")).strip()
            if command:
                self.set_control_mode("NAV")
                self.nav_cmd_pub.publish(String(data=command))
        elif msg_type == "go_to":
            x = float(payload.get("x", 0.0))
            y = float(payload.get("y", 0.0))
            self.set_control_mode("NAV")
            self.nav_cmd_pub.publish(String(data=f"go {x:.3f} {y:.3f}"))
        elif msg_type == "go_rel":
            dx = float(payload.get("dx", 0.0))
            dy = float(payload.get("dy", 0.0))
            self.set_control_mode("NAV")
            self.nav_cmd_pub.publish(String(data=f"go_rel {dx:.3f} {dy:.3f}"))
        elif msg_type == "ping":
            self.send_packet({"type": "pong", "stamp": self.now_sec()})
        elif msg_type == "map_request":
            self.send_packet({
                "type": "map",
                "points": self.map_points,
                "map_frame": self.nav_map_frame,
                "base_frame": self.nav_base_frame,
                "pose": self.lookup_pose(),
            })
        else:
            self.send_packet({"type": "error", "message": f"unknown packet type: {msg_type}"})

    def set_control_mode(self, mode: str) -> None:
        mode = str(mode).upper()
        self.mode_pub.publish(String(data=mode))
        self.web_enabled = mode == "WEB"
        self.web_enabled_pub.publish(Bool(data=mode == "WEB"))
        self.remote_enabled_pub.publish(Bool(data=mode == "REMOTE"))
        self.nav_enabled_pub.publish(Bool(data=mode == "NAV"))

    def parse_twist(self, payload: dict[str, Any]) -> Twist:
        cmd = Twist()
        linear = payload.get("linear", {}) or {}
        angular = payload.get("angular", {}) or {}
        cmd.linear.x = self.clamp(float(linear.get("x", 0.0)), -self.max_vx, self.max_vx)
        cmd.linear.y = self.clamp(float(linear.get("y", 0.0)), -self.max_vy, self.max_vy)
        cmd.angular.z = self.clamp(float(angular.get("z", 0.0)), -self.max_yaw, self.max_yaw)
        return cmd

    def on_guard_timer(self) -> None:
        age_ms = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1.0e6
        if age_ms > self.cmd_timeout_ms:
            self.cmd_pub.publish(Twist())
            if self.estop_on_timeout and not self.timeout_estop_sent:
                self.estop_pub.publish(Bool(data=True))
                self.timeout_estop_sent = True

    def on_state_timer(self) -> None:
        if not self.enabled:
            return
        self.send_packet(self.build_state_packet())

    def build_state_packet(self) -> dict[str, Any]:
        target = self.latest_target
        state = self.latest_state
        packet: dict[str, Any] = {
            "type": "state",
            "stamp": self.now_sec(),
            "mode": self.latest_mode,
            "mux_status": self.latest_mux_status,
            "nav_status": self.latest_nav_status,
            "estop": self.estop,
            "web_enabled": self.web_enabled,
            "connected": True,
            "local_receive_time": self.now_sec(),
            "cmd_vel": self.twist_to_dict(self.latest_cmd),
            "runtime": {},
            "robot": {},
            "nav": {"pose": self.lookup_pose(), "map_frame": self.nav_map_frame, "base_frame": self.nav_base_frame},
        }
        if target is not None:
            packet["runtime"] = {
                "target_source": str(target.target_source),
                "zero_command": bool(target.zero_command),
                "runtime_released": bool(target.runtime_released),
                "release_alpha": self._f(target.release_alpha),
                "command": [self._f(v) for v in target.command],
                "raw_command": [self._f(v) for v in target.raw_command],
            }
        if state is not None:
            packet["robot"] = {
                "joint_pos": [self._f(v) for v in state.joint_pos],
                "joint_vel": [self._f(v) for v in state.joint_vel],
                "joint_torque": [self._f(v) for v in state.joint_torque],
                "imu_gyro": [self._f(v) for v in state.imu_gyro],
                "imu_accel": [self._f(v) for v in state.imu_accel],
                "projected_gravity": [self._f(v) for v in state.projected_gravity],
                "quat_wxyz": [self._f(v) for v in state.quat_wxyz],
                "imu_age_ms": self._f(state.imu_age_ms),
                "imu_fresh": bool(state.imu_fresh),
                "odom_age_ms": self._f(state.odom_age_ms),
                "odom_fresh": bool(state.odom_fresh),
                "odom_local_pos": [self._f(v) for v in state.odom_local_pos],
                "odom_local_yaw": self._f(state.odom_local_yaw),
                "fresh_count": int(state.fresh_count),
                "holdover_count": int(state.holdover_count),
                "stale_max": int(state.stale_max),
                "update_counts": [int(v) for v in state.update_counts],
            }
        return packet

    def send_packet(self, payload: dict[str, Any]) -> None:
        if self.client_addr is None:
            return
        try:
            data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
            self.sock.sendto(data, self.client_addr)
        except OSError as exc:
            self.get_logger().warn(f"UDP send failed: {exc}")

    @staticmethod
    def _f(v: Any) -> float:
        try:
            f = float(v)
            if f != f:
                return 0.0
            return round(f, 6)
        except (TypeError, ValueError):
            return 0.0

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1.0e9

    @staticmethod
    def twist_to_dict(msg: Twist) -> dict[str, Any]:
        return {
            "linear": {"x": msg.linear.x, "y": msg.linear.y, "z": msg.linear.z},
            "angular": {"x": msg.angular.x, "y": msg.angular.y, "z": msg.angular.z},
        }

    @staticmethod
    def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = WebUdpBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.http_server is not None:
            node.http_server.shutdown()
            node.http_server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
