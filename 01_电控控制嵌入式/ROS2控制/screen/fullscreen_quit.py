#!/usr/bin/env python3
"""
Touch control panel for the 800x600 robot display area.

The screen panel starts the normal sim2real ROS2 launch command and then uses
the existing web bridge HTTP API for control. That keeps the button behavior in
sync with the browser UI without adding another ROS control path.
"""

from __future__ import annotations

import glob
import getpass
import json
import math
import os
import re
import shlex
import signal
import subprocess
import sys
import threading
import time
import tkinter as tk
from typing import Any, Callable, List, Optional
from urllib.error import URLError
from urllib.request import Request, urlopen


SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
BUTTON_WIDTH = 156
BUTTON_HEIGHT = 62
BUTTON_GAP_X = 18
BUTTON_GAP_Y = 12
INFO_BOX_WIDTH = 176
INFO_BOX_GAP_X = 4
TASK_LABEL_WIDTH = 350
TASK_BUTTON_WIDTH = 96
TASK_BUTTON_HEIGHT = 54
TASK_BUTTON_GAP_X = 20
TASK_ROW_GAP_Y = 8
TASK_ROW_WIDTH = TASK_LABEL_WIDTH + 3 * TASK_BUTTON_WIDTH + 3 * TASK_BUTTON_GAP_X
TASK_ROW_LEFT_PAD = 0
DEFAULT_DISPLAY = ":0"
DISPLAY_ATTEMPTS: list[str] = []

LOCAL_REPO_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
REPO_DIR = os.environ.get("SIM2REAL_REPO_DIR", LOCAL_REPO_DIR)
ODIN_CONTROL_CONFIG_REL = os.path.join("src", "odin_ros_driver", "config", "control_command.yaml")
ODIN_INSTALLED_CONTROL_CONFIG_RELS = (
    os.path.join("install", "odin_ros_driver", "share", "odin_ros_driver", "config", "control_command.yaml"),
    os.path.join("install", "share", "odin_ros_driver", "config", "control_command.yaml"),
)
ODIN_PROFILE_CONFIG_REL = {
    "odom": os.path.join("src", "odin_ros_driver", "config", "control_command_odom.yaml"),
    "relocal": os.path.join("src", "odin_ros_driver", "config", "control_command_relocal.yaml"),
}
LOCALIZATION_PROFILES = ("odom", "relocal")
RELOCAL_MODE_NAMES = {0: "odom", 1: "slam", 2: "relocal"}
LOCALIZATION_PROFILE_LABELS = {"odom": "ODOM", "relocal": "REL"}
CAN_BUS_MAP = [1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 1, 1, 2, 2]
CAN_BUS_LABELS = {1: "CAN0", 2: "CAN1"}
ACTION_COMMAND_PREFIX = (
    f"cd {shlex.quote(REPO_DIR)} && "
    "source install/setup.bash && "
    "ros2 launch sim2real_bringup sim2real_system.launch.py launch_nav2:=false"
)
HTTP_BASE = "http://127.0.0.1:18080"
HTTP_TIMEOUT_S = 0.35
MODE_CONFIRM_DELAY_S = 0.25
MODE_CONFIRM_ATTEMPTS = 8
ACTION_OUTPUT_QUIET_S = 1.0
ACTION_SIGINT_TIMEOUT_S = 15.0
ACTION_SIGTERM_TIMEOUT_S = 8.0
SYSTEM_PROCESS_PATTERNS = [
    "ros2 launch sim2real_bringup sim2real_system.launch.py",
    "sim2real_system.launch.py",
    "sim2real_hw_node",
    "sim2real_runtime_node",
    "sim2real_cmd_mux_node",
    "sim2real_web_udp_bridge_node",
    "sim2real_remote_uart_node",
    "sim2real_simple_nav_node",
    "host_sdk_sample",
]

COLOR_BG = "#101418"
COLOR_PANEL = "#18202a"
COLOR_TEXT = "#f5f7fb"
COLOR_MUTED = "#8d99a8"
COLOR_DISABLED = "#4a4f57"
COLOR_ACTION = "#2474a6"
COLOR_NAV = "#1e8449"
COLOR_REMOTE = "#7d3c98"
COLOR_KEEP = "#6f42c1"
COLOR_ODOM = "#b9770e"
COLOR_ESTOP = "#c0392b"
COLOR_QUIT = "#566573"
COLOR_QUIT_ACTIVE = "#d35400"
COLOR_WARN = "#ffd60a"
COLOR_OK = "#30d158"


def configure_display() -> None:
    if sys.platform.startswith("linux") and not os.environ.get("DISPLAY"):
        os.environ["DISPLAY"] = DEFAULT_DISPLAY


def current_user() -> str:
    try:
        return getpass.getuser()
    except OSError:
        return os.environ.get("USER", "unknown")


def xauth_candidates() -> List[Optional[str]]:
    candidates: List[Optional[str]] = [os.environ.get("XAUTHORITY")]

    if sys.platform.startswith("linux"):
        candidates.extend(xauth_from_x_processes())
        uid = os.getuid()
        candidates.extend(
            [
                os.path.expanduser("~/.Xauthority"),
                f"/run/user/{uid}/gdm/Xauthority",
                f"/run/user/{uid}/Xauthority",
            ]
        )
        candidates.extend(glob.glob(f"/run/user/{uid}/*Xauthority*"))

    candidates.append(None)

    unique_candidates: List[Optional[str]] = []
    seen = set()
    for candidate in candidates:
        key = candidate or "<unset>"
        if key in seen:
            continue
        seen.add(key)
        if candidate and not os.path.exists(candidate):
            continue
        unique_candidates.append(candidate)
    return unique_candidates


def xauth_from_x_processes() -> List[str]:
    try:
        result = subprocess.run(
            ["ps", "-eo", "args"],
            check=False,
            capture_output=True,
            text=True,
        )
    except OSError:
        return []

    candidates = []
    for line in result.stdout.splitlines():
        if "Xorg" not in line and "Xwayland" not in line:
            continue
        try:
            parts = shlex.split(line)
        except ValueError:
            parts = line.split()
        for index, part in enumerate(parts[:-1]):
            if part == "-auth":
                candidates.append(parts[index + 1])
    return candidates


def create_tk_root() -> tk.Tk:
    configure_display()
    last_error = None

    for candidate in xauth_candidates():
        if candidate:
            os.environ["XAUTHORITY"] = candidate
        else:
            os.environ.pop("XAUTHORITY", None)

        DISPLAY_ATTEMPTS.append(os.environ.get("XAUTHORITY", "<unset>"))
        try:
            return tk.Tk()
        except tk.TclError as exc:
            last_error = exc

    if last_error:
        raise last_error
    raise tk.TclError("Unable to open display")


class FullscreenControlApp:
    def __init__(self) -> None:
        self.root = create_tk_root()
        self.root.title("sim2real Screen")
        self.root.configure(bg=COLOR_BG)
        self.root.geometry(f"{SCREEN_WIDTH}x{SCREEN_HEIGHT}+0+0")
        self.root.overrideredirect(True)
        self.root.resizable(False, False)

        self.root.bind("<Escape>", self.close_app)
        self.root.bind("q", self.close_app)
        self.root.protocol("WM_DELETE_WINDOW", self.close_app)

        self.action_process: Optional[subprocess.Popen[str]] = None
        self.stopping_action = False
        self.action_output_reader_done = True
        self.last_action_output_time = 0.0
        self.latest_mode = "UNKNOWN"
        self.latest_model = "--"
        self.latest_nav_status = "--"
        self.latest_estop = "--"
        self.latest_odom_xy: Optional[tuple[float, float]] = None
        self.latest_odom_yaw: Optional[float] = None
        self.latest_re_xy: Optional[tuple[float, float]] = None
        self.latest_re_yaw: Optional[float] = None
        self.latest_nav_pose_available = False
        self.latest_relocal_text = "--"
        self.latest_relocal_level = "muted"
        self.latest_can_text = "--"
        self.latest_can_level = "muted"
        self.latest_error_text = "err: ok"
        self.latest_error_level = "ok"
        self.command_error_text = ""
        self.command_error_until = 0.0
        self.task_specs: list[dict[str, Any]] = []
        self.selected_task_index = 0
        self.current_task = ""
        self.task_selection_manual = False
        self.localization_profile = self.normalize_localization_profile(
            os.environ.get("SIM2REAL_LOCALIZATION_PROFILE", "relocal")
        )
        self.bridge_localization_mode = ""
        self.bridge_connected = False
        self.system_process_running = False
        self.manual_cleanup_required = False
        self.message = "ready"

        self.buttons: dict[str, tk.Button] = {}
        self.labels: dict[str, tk.Label] = {}

        self.build_ui()
        self.refresh_loop()

    def build_ui(self) -> None:
        self.header = tk.Frame(self.root, bg=COLOR_PANEL, height=74)
        self.header.pack(fill="x")
        self.header.pack_propagate(False)

        left = tk.Frame(self.header, bg=COLOR_PANEL)
        left.pack(side="left", padx=18, fill="y")
        tk.Label(
            left,
            text="MODEL",
            font=("Arial", 12, "bold"),
            fg=COLOR_MUTED,
            bg=COLOR_PANEL,
        ).pack(anchor="w", pady=(6, 0))
        self.labels["model"] = tk.Label(
            left,
            text="--",
            font=("Arial", 24, "bold"),
            fg="#30d158",
            bg=COLOR_PANEL,
            width=12,
            anchor="w",
        )
        self.labels["model"].pack(anchor="w")

        right = tk.Frame(self.header, bg=COLOR_PANEL)
        right.pack(side="right", padx=18, fill="y")
        self.labels["action_status"] = self.status_label(right, "ACTION", "idle")
        self.labels["mode_status"] = self.status_label(right, "MODE", "unknown")

        self.body = tk.Frame(self.root, bg=COLOR_BG)
        self.body.pack(fill="both", expand=True)

        title_row = tk.Frame(self.body, bg=COLOR_BG)
        title_row.pack(fill="x", padx=36, pady=(10, 2))
        title = tk.Label(
            title_row,
            text="sim2real touch control",
            font=("Arial", 22, "bold"),
            fg=COLOR_TEXT,
            bg=COLOR_BG,
        )
        title.pack(side="left")

        grid = tk.Frame(self.body, bg=COLOR_BG)
        grid.pack(pady=4)

        layout = [
            ("action", "action", self.action, COLOR_ACTION, 0, 0),
            ("nav", "nav", self.nav, COLOR_NAV, 0, 1),
            ("estop", "estop", self.estop, COLOR_ESTOP, 0, 2),
            ("loc_profile", "LOC REL", self.toggle_localization_profile, COLOR_NAV, 1, 0),
            ("odom", "odom", self.odom, COLOR_ODOM, 1, 1),
            ("quit", "quit", self.quit_action, COLOR_QUIT, 1, 2),
        ]
        for name, text, command, color, row, col in layout:
            cell = tk.Frame(grid, bg=COLOR_BG, width=BUTTON_WIDTH, height=BUTTON_HEIGHT)
            cell.grid(
                row=row,
                column=col,
                padx=BUTTON_GAP_X // 2,
                pady=BUTTON_GAP_Y // 2,
            )
            cell.grid_propagate(False)
            button = self.make_button(cell, text, command, color)
            if name == "loc_profile":
                button.configure(font=("Arial", 20, "bold"))
            button.pack(fill="both", expand=True)
            self.buttons[name] = button

        info = tk.Frame(self.body, bg=COLOR_BG)
        info.pack(fill="x", padx=36, pady=(10, 0))
        self.labels["bridge"] = self.info_label(info, "bridge", "--", 0)
        self.labels["estop"] = self.info_label(info, "estop", "--", 1)
        self.labels["relocal"] = self.info_label(info, "reloc", "--", 2)
        self.labels["can_status"] = self.info_label(info, "can", "--", 3)

        task_panel = tk.Frame(self.body, bg=COLOR_BG)
        task_panel.pack(fill="x", padx=36, pady=(12, 0))

        task_bar = tk.Frame(task_panel, bg=COLOR_BG, height=TASK_BUTTON_HEIGHT)
        task_bar.pack(anchor="w", padx=(TASK_ROW_LEFT_PAD, 0), pady=(0, TASK_ROW_GAP_Y))
        task_bar.configure(width=TASK_ROW_WIDTH)
        task_bar.pack_propagate(False)
        task_label_cell = tk.Frame(
            task_bar,
            bg=COLOR_PANEL,
            width=TASK_LABEL_WIDTH,
            height=TASK_BUTTON_HEIGHT,
        )
        task_label_cell.pack(side="left", padx=(0, TASK_BUTTON_GAP_X))
        task_label_cell.pack_propagate(False)
        self.labels["task"] = tk.Label(
            task_label_cell,
            text="task: --",
            font=("Arial", 18, "bold"),
            fg=COLOR_TEXT,
            bg=COLOR_PANEL,
            anchor="w",
            padx=12,
        )
        self.labels["task"].pack(fill="both", expand=True)
        top_task_layout = [
            ("task_prev", "prev", self.prev_task, COLOR_QUIT),
            ("task_next", "next", self.next_task, COLOR_QUIT),
            ("task_resume", "resume", self.resume_task, COLOR_NAV),
        ]
        for index, (name, text, command, color) in enumerate(top_task_layout):
            cell = tk.Frame(task_bar, bg=COLOR_BG, width=TASK_BUTTON_WIDTH, height=TASK_BUTTON_HEIGHT)
            pad_right = TASK_BUTTON_GAP_X if index < len(top_task_layout) - 1 else 0
            cell.pack(side="left", padx=(0, pad_right))
            cell.pack_propagate(False)
            button = self.make_small_button(cell, text, command, color)
            button.pack(fill="both", expand=True)
            self.buttons[name] = button

        odom_bar = tk.Frame(task_panel, bg=COLOR_BG, height=TASK_BUTTON_HEIGHT)
        odom_bar.pack(anchor="w", padx=(TASK_ROW_LEFT_PAD, 0))
        odom_bar.configure(width=TASK_ROW_WIDTH)
        odom_bar.pack_propagate(False)
        odom_label_cell = tk.Frame(
            odom_bar,
            bg=COLOR_PANEL,
            width=TASK_LABEL_WIDTH,
            height=TASK_BUTTON_HEIGHT,
        )
        odom_label_cell.pack(side="left", padx=(0, TASK_BUTTON_GAP_X))
        odom_label_cell.pack_propagate(False)
        self.labels["odom_xy"] = tk.Label(
            odom_label_cell,
            text="odom x=-- y=-- yaw=--\nre   x=-- y=-- yaw=--",
            font=("Arial", 13, "bold"),
            fg=COLOR_TEXT,
            bg=COLOR_PANEL,
            anchor="w",
            padx=12,
        )
        self.labels["odom_xy"].pack(fill="both", expand=True)
        bottom_task_layout = [
            ("keep", "keep", self.keep, COLOR_KEEP),
            ("task_stop", "stop", self.stop_task, COLOR_ODOM),
            ("estop_reset", "reset", self.reset_estop, COLOR_ACTION),
        ]
        for index, (name, text, command, color) in enumerate(bottom_task_layout):
            cell = tk.Frame(odom_bar, bg=COLOR_BG, width=TASK_BUTTON_WIDTH, height=TASK_BUTTON_HEIGHT)
            pad_right = TASK_BUTTON_GAP_X if index < len(bottom_task_layout) - 1 else 0
            cell.pack(side="left", padx=(0, pad_right))
            cell.pack_propagate(False)
            button = self.make_small_button(cell, text, command, color)
            button.pack(fill="both", expand=True)
            self.buttons[name] = button

        self.labels["message"] = tk.Label(
            self.body,
            text=self.message,
            font=("Arial", 15),
            fg=COLOR_MUTED,
            bg=COLOR_BG,
            wraplength=728,
        )
        self.labels["message"].pack(pady=(8, 0))
        self.labels["error"] = tk.Label(
            self.body,
            text=self.latest_error_text,
            font=("Arial", 14, "bold"),
            fg=COLOR_OK,
            bg=COLOR_BG,
            wraplength=728,
        )
        self.labels["error"].pack(pady=(4, 0))

    def status_label(self, parent: tk.Frame, name: str, value: str) -> tk.Label:
        row = tk.Frame(parent, bg=COLOR_PANEL)
        row.pack(anchor="e", pady=(4, 0))
        tk.Label(
            row,
            text=f"{name}: ",
            font=("Arial", 13, "bold"),
            fg=COLOR_MUTED,
            bg=COLOR_PANEL,
        ).pack(side="left")
        label = tk.Label(
            row,
            text=value,
            font=("Arial", 17, "bold"),
            fg=COLOR_TEXT,
            bg=COLOR_PANEL,
            width=10,
            anchor="e",
        )
        label.pack(side="left")
        return label

    def info_label(self, parent: tk.Frame, name: str, value: str, col: int) -> tk.Label:
        box = tk.Frame(parent, bg=COLOR_PANEL, width=INFO_BOX_WIDTH, height=58)
        box.grid(row=0, column=col, padx=INFO_BOX_GAP_X, sticky="nsew")
        box.grid_propagate(False)
        tk.Label(
            box,
            text=name,
            font=("Arial", 12, "bold"),
            fg=COLOR_MUTED,
            bg=COLOR_PANEL,
        ).pack(anchor="w", padx=12, pady=(5, 0))
        label = tk.Label(
            box,
            text=value,
            font=("Arial", 15, "bold"),
            fg=COLOR_TEXT,
            bg=COLOR_PANEL,
            anchor="w",
        )
        label.pack(anchor="w", padx=12)
        parent.grid_columnconfigure(col, weight=1)
        return label

    def make_button(
        self,
        parent: tk.Frame,
        text: str,
        command: Callable[[], None],
        background: str,
    ) -> tk.Button:
        return tk.Button(
            parent,
            text=text,
            command=command,
            font=("Arial", 22, "bold"),
            fg="#ffffff",
            bg=background,
            activeforeground="#ffffff",
            activebackground=background,
            disabledforeground="#c3c7ce",
            relief="flat",
            bd=0,
            cursor="hand2",
            highlightthickness=0,
        )

    def make_small_button(
        self,
        parent: tk.Frame,
        text: str,
        command: Callable[[], None],
        background: str,
    ) -> tk.Button:
        return tk.Button(
            parent,
            text=text,
            command=command,
            font=("Arial", 16, "bold"),
            fg="#ffffff",
            bg=background,
            activeforeground="#ffffff",
            activebackground=background,
            disabledforeground="#c3c7ce",
            relief="flat",
            bd=0,
            cursor="hand2",
            highlightthickness=0,
        )

    def set_button_state(self, name: str, enabled: bool, color: str) -> None:
        button = self.buttons.get(name)
        if not button:
            return
        if enabled:
            button.configure(state="normal", bg=color, activebackground=color, cursor="hand2")
        else:
            button.configure(state="disabled", bg=COLOR_DISABLED, activebackground=COLOR_DISABLED, cursor="arrow")

    def action_running(self) -> bool:
        return self.action_process is not None and self.action_process.poll() is None

    def action_busy(self) -> bool:
        return self.action_process is not None or self.stopping_action

    def action_output_quiet(self) -> bool:
        # The launch can be stopped while a child still holds stdout briefly.
        # Gate restart on visible terminal quiet time, not only pipe closure.
        return time.monotonic() - self.last_action_output_time >= ACTION_OUTPUT_QUIET_S

    def process_pids_for_pattern(self, pattern: str) -> list[int]:
        if not sys.platform.startswith("linux"):
            return []
        try:
            result = subprocess.run(
                ["pgrep", "-f", pattern],
                check=False,
                capture_output=True,
                text=True,
            )
        except OSError:
            return []

        pids: list[int] = []
        current_pid = os.getpid()
        for line in result.stdout.splitlines():
            try:
                pid = int(line.strip())
            except ValueError:
                continue
            if pid != current_pid:
                pids.append(pid)
        return pids

    def process_pattern_running(self, pattern: str) -> bool:
        return bool(self.process_pids_for_pattern(pattern))

    def system_process_pids(self) -> list[int]:
        pids: set[int] = set()
        for pattern in SYSTEM_PROCESS_PATTERNS:
            pids.update(self.process_pids_for_pattern(pattern))
        return sorted(pids)

    def any_system_process_running(self) -> bool:
        return bool(self.system_process_pids())

    def existing_system_running(self) -> bool:
        if self.fetch_state() is not None:
            return True
        return self.any_system_process_running()

    def build_action_command(self) -> str:
        profile = self.normalize_localization_profile(self.localization_profile)
        self.sync_legacy_odin_control_mode(profile)
        config_rel = self.profile_config_rel(profile)
        config_path = self.resolve_repo_file(config_rel)
        if not config_path:
            raise ValueError(f"{profile} config missing: {config_rel}")
        return (
            f"{ACTION_COMMAND_PREFIX} "
            f"localization_mode:={shlex.quote(profile)} "
            f"odin_config_file:={shlex.quote(config_path)}"
        )

    def action(self) -> None:
        if self.action_busy():
            self.set_message("wait for previous action to stop")
            return
        if self.bridge_connected or self.existing_system_running():
            self.set_message("system already running; do not launch twice")
            return
        try:
            action_command = self.build_action_command()
        except ValueError as exc:
            self.set_message(str(exc))
            return
        self.manual_cleanup_required = False
        self.stopping_action = False
        self.action_output_reader_done = False
        self.last_action_output_time = time.monotonic()
        try:
            kwargs: dict[str, Any] = {
                "shell": False,
                "text": True,
                "stdout": subprocess.PIPE,
                "stderr": subprocess.STDOUT,
                "bufsize": 1,
            }
            if sys.platform.startswith("linux"):
                kwargs["preexec_fn"] = os.setsid
            self.action_process = subprocess.Popen(["bash", "-lc", action_command], **kwargs)
            threading.Thread(
                target=self.read_action_output,
                args=(self.action_process,),
                daemon=True,
            ).start()
            self.set_message(f"launch started ({self.localization_profile})")
        except OSError as exc:
            self.action_process = None
            self.action_output_reader_done = True
            self.set_message(f"action failed: {exc}")
        self.update_ui_state()

    def read_action_output(self, proc: subprocess.Popen[str]) -> None:
        try:
            if proc.stdout is None:
                return
            for line in proc.stdout:
                self.last_action_output_time = time.monotonic()
                print(line, end="", flush=True)
        except Exception as exc:
            print(f"[screen] action output reader failed: {exc}", file=sys.stderr, flush=True)
        finally:
            if self.action_process is proc:
                self.action_output_reader_done = True

    def wait_for_action_output_quiet(self, timeout_s: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.action_output_quiet():
                return True
            time.sleep(0.05)
        return self.action_output_quiet()

    def quit_action(self) -> None:
        if self.stopping_action:
            self.set_message("action is stopping...")
            return
        if not self.action_process:
            if self.bridge_connected or self.system_process_running or self.existing_system_running():
                self.stopping_action = True
                self.set_message("stopping external system...")
                threading.Thread(target=self.stop_external_system_processes, daemon=True).start()
                self.update_ui_state()
                return
            self.manual_cleanup_required = False
            self.set_message("system already stopped")
            self.update_ui_state()
            return
        self.stopping_action = True
        self.set_message("stopping action...")
        threading.Thread(target=self.stop_action_process, daemon=True).start()

    def stop_external_system_processes(self) -> None:
        if not sys.platform.startswith("linux"):
            self.root.after(0, lambda: self.set_message("external cleanup is only available on linux"))
            self.root.after(0, self.update_ui_state)
            self.stopping_action = False
            return

        try:
            self.post_control({"type": "nav_stop_keep"})
            self.signal_system_processes(signal.SIGINT)
            deadline = time.monotonic() + ACTION_SIGINT_TIMEOUT_S
            while time.monotonic() < deadline and self.any_system_process_running():
                time.sleep(0.1)

            if self.any_system_process_running():
                self.signal_system_processes(signal.SIGTERM)
                deadline = time.monotonic() + ACTION_SIGTERM_TIMEOUT_S
                while time.monotonic() < deadline and self.any_system_process_running():
                    time.sleep(0.1)
        except Exception as exc:
            self.root.after(0, lambda: self.set_message(f"external stop failed: {exc}"))
            self.root.after(0, self.update_ui_state)
            return
        finally:
            self.stopping_action = False

        if self.any_system_process_running():
            self.manual_cleanup_required = True
            self.system_process_running = True
            self.root.after(
                0,
                lambda: self.set_message("cleanup still running; replug USB or run usbreset 2207:0019"),
            )
        else:
            self.manual_cleanup_required = False
            self.action_process = None
            self.bridge_connected = False
            self.system_process_running = False
            self.root.after(0, lambda: self.set_message("external system stopped"))
        self.root.after(0, self.update_ui_state)

    def signal_system_processes(self, sig: signal.Signals) -> None:
        for pid in self.system_process_pids():
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                continue
            except PermissionError:
                continue

    def stop_action_process(self) -> None:
        proc = self.action_process
        if not proc:
            return
        stopped = False
        try:
            if proc.poll() is None:
                if sys.platform.startswith("linux"):
                    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                else:
                    proc.terminate()
                try:
                    proc.wait(timeout=ACTION_SIGINT_TIMEOUT_S)
                    stopped = True
                except subprocess.TimeoutExpired:
                    if sys.platform.startswith("linux"):
                        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                    else:
                        proc.terminate()
                    try:
                        proc.wait(timeout=ACTION_SIGTERM_TIMEOUT_S)
                        stopped = True
                    except subprocess.TimeoutExpired:
                        # Odin's official driver cleanup path is registered only for
                        # SIGINT/SIGTERM. Do not send SIGKILL automatically; it can
                        # leave the libusb interface busy for the next run.
                        stopped = False
            else:
                stopped = True
        except Exception as exc:
            self.root.after(0, lambda: self.set_message(f"stop failed: {exc}"))
            return
        finally:
            self.stopping_action = False
        output_quiet = self.wait_for_action_output_quiet() if stopped else False
        if stopped and output_quiet:
            self.manual_cleanup_required = False
            self.action_process = None
            self.root.after(0, lambda: self.set_message("action stopped"))
        elif stopped:
            self.root.after(0, lambda: self.set_message("waiting for action output to finish"))
        else:
            self.manual_cleanup_required = True
            self.root.after(
                0,
                lambda: self.set_message("Odin cleanup still running; replug USB or run usbreset 2207:0019"),
            )
        self.root.after(0, self.update_ui_state)

    @staticmethod
    def normalize_localization_profile(value: Any) -> str:
        text = str(value or "").strip().lower()
        if text in {"relocal", "reloc", "relocalization", "localization"}:
            return "relocal"
        return "odom"

    @staticmethod
    def localization_profile_label(profile: str) -> str:
        return LOCALIZATION_PROFILE_LABELS.get(profile, profile.upper())

    @staticmethod
    def profile_config_rel(profile: str) -> str:
        return ODIN_PROFILE_CONFIG_REL.get(profile, ODIN_CONTROL_CONFIG_REL)

    def sync_legacy_odin_control_mode(self, profile: str) -> None:
        """Keep default Odin config compatible with installed launches that ignore odin_config_file."""
        mode = self.read_relocalization_mode(profile)
        if mode is None:
            mode = 2 if profile == "relocal" else 0

        target_paths = self.resolve_repo_files(
            [ODIN_CONTROL_CONFIG_REL, *ODIN_INSTALLED_CONTROL_CONFIG_RELS]
        )
        if not target_paths:
            raise ValueError("Odin default config missing: control_command.yaml")

        errors: list[str] = []
        changed = False
        for path in target_paths:
            try:
                with open(path, "r", encoding="utf-8") as f:
                    text = f.read()
                new_text, count = re.subn(
                    r"^(\s*)custom_map_mode\s*:.*$",
                    rf"\g<1>custom_map_mode: {mode}      # 0: Odometry mode 1: SLAM mode 2: Relocalization mode.",
                    text,
                    count=1,
                    flags=re.MULTILINE,
                )
                if count <= 0:
                    errors.append(f"{os.path.basename(path)} missing custom_map_mode")
                    continue
                if new_text != text:
                    with open(path, "w", encoding="utf-8", newline="") as f:
                        f.write(new_text)
                    changed = True
            except OSError as exc:
                errors.append(f"{path}: {exc}")

        if errors:
            raise ValueError("failed to sync Odin mode: " + "; ".join(errors[:2]))
        if changed:
            print(f"[screen] synced Odin custom_map_mode={mode} for LOC {profile}", flush=True)

    def display_localization_profile(self) -> str:
        if self.bridge_localization_mode:
            return self.normalize_localization_profile(self.bridge_localization_mode)
        return self.normalize_localization_profile(self.localization_profile)

    def relocal_ready(self) -> bool:
        if self.display_localization_profile() != "relocal":
            return True
        return self.latest_nav_pose_available and self.latest_relocal_text == "on/tf ok"

    def toggle_localization_profile(self) -> None:
        running = self.action_running() or self.bridge_connected or self.system_process_running
        if running or self.action_busy():
            self.set_message("stop system before switching LOC")
            return
        self.localization_profile = "relocal" if self.localization_profile == "odom" else "odom"
        self.bridge_localization_mode = ""
        self.update_relocal_status(None, {})
        self.set_message(f"LOC set to {self.localization_profile_label(self.localization_profile)}")
        self.update_ui_state()

    def remote(self) -> None:
        self.request_mode("REMOTE")

    def nav(self) -> None:
        self.request_mode("NAV")

    def keep(self) -> None:
        self.request_mode("KEEP")

    def request_mode(self, mode: str) -> None:
        mode = mode.upper()
        retry_payload = None
        if mode == "REMOTE":
            retry_payload = {"type": "remote_enable", "data": True}
        elif mode == "NAV":
            retry_payload = {"type": "nav_enable", "data": True}
        self.post_control_async(
            {"type": "mode", "mode": mode},
            f"{mode.lower()} requested",
            confirm_mode=mode,
            retry_payload=retry_payload,
        )

    def odom(self) -> None:
        if self.display_localization_profile() == "relocal":
            self.set_message("odom is disabled in RELOC mode")
            return
        if self.latest_mode != "NAV":
            self.set_message("odom is available only in NAV mode")
            return
        self.post_control_async({"type": "odom_task"}, "odom task requested")

    def estop(self) -> None:
        self.post_control_async({"type": "estop", "data": True}, "soft estop sent")

    def selected_task(self) -> Optional[dict[str, Any]]:
        if not self.task_specs:
            return None
        index = max(0, min(self.selected_task_index, len(self.task_specs) - 1))
        self.selected_task_index = index
        return self.task_specs[index]

    def selected_task_name(self) -> str:
        task = self.selected_task()
        return str(task.get("name", "")).strip() if task else ""

    def select_task_delta(self, delta: int) -> None:
        if not self.task_specs:
            self.set_message("no tasks loaded")
            return
        self.selected_task_index = (self.selected_task_index + delta) % len(self.task_specs)
        self.task_selection_manual = True
        self.update_ui_state()

    def prev_task(self) -> None:
        self.select_task_delta(-1)

    def next_task(self) -> None:
        self.select_task_delta(1)

    def stop_task(self) -> None:
        if self.latest_estop.lower() == "true":
            self.set_message("reset estop before stop/keep")
            return
        self.post_control_async(
            {"type": "nav_stop_keep"},
            "navigation stopped; keep requested",
            confirm_mode="KEEP",
        )

    def resume_task(self) -> None:
        task_name = self.selected_task_name()
        if not task_name:
            self.set_message("no task selected")
            return
        if self.latest_mode not in {"NAV", "KEEP"}:
            self.set_message("task resume is available only in NAV/KEEP mode")
            return
        if self.display_localization_profile() == "relocal" and not self.relocal_ready():
            self.set_message("waiting for reloc on/tf ok before resume")
            return
        self.post_control_async(
            {"type": "task_resume", "task": task_name},
            f"resume {task_name} requested",
        )

    def reset_estop(self) -> None:
        self.post_control_async(
            {"type": "estop_reset"},
            "soft estop reset; keep requested",
            confirm_mode="KEEP",
        )

    def post_control_async(
        self,
        payload: dict[str, Any],
        ok_message: str,
        confirm_mode: Optional[str] = None,
        retry_payload: Optional[dict[str, Any]] = None,
    ) -> None:
        def worker() -> None:
            ok, error = self.post_control(payload)
            if not ok:
                self.root.after(0, lambda: self.set_command_error(f"CMD failed: {error}"))
                self.root.after(0, lambda: self.set_message(f"control failed: {error}"))
                return

            if confirm_mode:
                confirmed, actual, mux_status = self.wait_for_mode(confirm_mode)
                if not confirmed and retry_payload is not None:
                    retry_ok, retry_error = self.post_control(retry_payload)
                    if not retry_ok:
                        self.root.after(0, lambda: self.set_command_error(f"CMD retry failed: {retry_error}"))
                        self.root.after(0, lambda: self.set_message(f"control retry failed: {retry_error}"))
                        return
                    confirmed, actual, mux_status = self.wait_for_mode(confirm_mode)
                if confirmed:
                    self.root.after(0, self.clear_command_error)
                    self.root.after(0, lambda: self.set_message(ok_message))
                else:
                    detail = f"{confirm_mode.lower()} not active; mode={actual}"
                    if mux_status:
                        detail = f"{detail}; {mux_status[:48]}"
                    self.root.after(0, lambda: self.set_command_error(f"MODE failed: {detail}"))
                    self.root.after(0, lambda: self.set_message(detail))
                return

            self.root.after(0, self.clear_command_error)
            self.root.after(0, lambda: self.set_message(ok_message))

        threading.Thread(target=worker, daemon=True).start()

    def wait_for_mode(self, expected_mode: str) -> tuple[bool, str, str]:
        expected_mode = expected_mode.upper()
        actual = "UNKNOWN"
        mux_status = ""
        for _ in range(MODE_CONFIRM_ATTEMPTS):
            time.sleep(MODE_CONFIRM_DELAY_S)
            state = self.fetch_state()
            if not state:
                continue
            actual = str(state.get("mode", "UNKNOWN")).upper()
            mux_status = str(state.get("mux_status", "") or "")
            if actual == expected_mode:
                return True, actual, mux_status
        return False, actual, mux_status

    def post_control(self, payload: dict[str, Any]) -> tuple[bool, str]:
        data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        req = Request(
            f"{HTTP_BASE}/api/control",
            data=data,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urlopen(req, timeout=HTTP_TIMEOUT_S) as response:
                raw = response.read()
            parsed = json.loads(raw.decode("utf-8")) if raw else {}
            if parsed.get("ok", True) is False:
                return False, str(parsed.get("error", "api error"))
            return True, ""
        except (OSError, URLError, json.JSONDecodeError) as exc:
            return False, str(exc)

    def fetch_state(self) -> Optional[dict[str, Any]]:
        try:
            with urlopen(f"{HTTP_BASE}/api/state", timeout=HTTP_TIMEOUT_S) as response:
                return json.loads(response.read().decode("utf-8"))
        except (OSError, URLError, json.JSONDecodeError):
            return None

    def refresh_loop(self) -> None:
        if self.action_process is not None and self.action_process.poll() is not None:
            if self.action_output_quiet():
                self.manual_cleanup_required = False
                self.action_process = None
                self.set_message("action process exited")
            else:
                self.set_message("waiting for action output to finish")

        state = self.fetch_state()
        if state:
            self.bridge_connected = True
            self.system_process_running = True
            self.latest_mode = str(state.get("mode", "UNKNOWN")).upper()
            self.latest_estop = str(state.get("estop", "--"))
            self.latest_nav_status = str(state.get("nav_status", "--") or "--")
            model = state.get("model") if isinstance(state.get("model"), dict) else {}
            self.latest_model = str(model.get("current_model", "--"))
            nav = state.get("nav") if isinstance(state.get("nav"), dict) else {}
            robot = state.get("robot") if isinstance(state.get("robot"), dict) else {}
            nav_mode = nav.get("localization_mode") if isinstance(nav, dict) else ""
            if nav_mode:
                self.bridge_localization_mode = self.normalize_localization_profile(nav_mode)
                self.localization_profile = self.bridge_localization_mode
            pose = nav.get("pose") if isinstance(nav, dict) else None
            self.latest_nav_pose_available = isinstance(pose, dict)
            self.latest_re_xy = (
                self.parse_xy(pose.get("x"), pose.get("y"))
                if isinstance(pose, dict)
                else None
            )
            self.latest_re_yaw = (
                self.safe_float(pose.get("yaw"))
                if isinstance(pose, dict)
                else None
            )
            odom_pos = robot.get("odom_local_pos") if isinstance(robot, dict) else None
            if not (isinstance(odom_pos, list) and len(odom_pos) >= 2):
                odom_pos = robot.get("odom_pos") if isinstance(robot, dict) else None
            if isinstance(odom_pos, list) and len(odom_pos) >= 2:
                self.latest_odom_xy = self.parse_xy(odom_pos[0], odom_pos[1])
            else:
                self.latest_odom_xy = None
            self.latest_odom_yaw = (
                self.safe_float(robot.get("odom_local_yaw"))
                if isinstance(robot, dict)
                else None
            )
            tasks = nav.get("task_specs") if isinstance(nav, dict) else None
            if isinstance(tasks, list):
                self.task_specs = [task for task in tasks if isinstance(task, dict) and task.get("name")]
                if self.selected_task_index >= len(self.task_specs):
                    self.selected_task_index = max(0, len(self.task_specs) - 1)
            current_task = nav.get("current_task") if isinstance(nav, dict) else ""
            if current_task:
                self.current_task = str(current_task)
                if self.task_specs and not self.task_selection_manual:
                    for index, task in enumerate(self.task_specs):
                        if str(task.get("name", "")) == self.current_task:
                            self.selected_task_index = index
                            break
            self.update_relocal_status(state, nav)
            self.latest_can_level, self.latest_can_text = self.build_can_status(robot)
            self.set_diagnostic_state(self.build_error_summary(state))
        else:
            self.bridge_connected = False
            self.system_process_running = self.any_system_process_running()
            self.latest_nav_pose_available = False
            self.latest_re_yaw = None
            self.bridge_localization_mode = ""
            self.update_relocal_status(None, {})
            self.latest_can_level = "warn" if self.system_process_running else "muted"
            self.latest_can_text = "waiting" if self.system_process_running else "--"
            if not self.action_running() and not self.system_process_running:
                self.latest_mode = "UNKNOWN"
                self.latest_model = "--"
                self.latest_estop = "--"
                self.latest_nav_status = "--"
                self.latest_odom_xy = None
                self.latest_odom_yaw = None
                self.latest_re_xy = None
                self.latest_re_yaw = None
                self.latest_nav_pose_available = False
                self.bridge_localization_mode = ""
                self.current_task = ""
                self.latest_can_level = "muted"
                self.latest_can_text = "--"
            self.set_diagnostic_state(self.build_error_summary(None))

        self.update_ui_state()
        self.root.after(500, self.refresh_loop)

    def update_ui_state(self) -> None:
        running = self.action_running() or self.bridge_connected or self.system_process_running
        busy = self.action_busy()
        action_text = (
            "working"
            if running and self.bridge_connected
            else "starting"
            if running
            else "stopping"
            if busy
            else "idle"
        )
        self.labels["action_status"].configure(
            text=action_text,
            fg="#30d158" if action_text == "working" else "#ffd60a" if action_text in {"starting", "stopping"} else COLOR_MUTED,
        )
        mode_text = self.latest_mode.lower() if self.latest_mode in {"REMOTE", "NAV", "WEB", "KEEP"} else "unknown"
        self.labels["mode_status"].configure(
            text=mode_text,
            fg="#30d158"
            if self.latest_mode == "NAV"
            else "#bf5af2"
            if self.latest_mode in {"REMOTE", "KEEP"}
            else COLOR_TEXT,
        )
        self.labels["model"].configure(text=self.latest_model)
        profile = self.display_localization_profile()
        profile_color = COLOR_NAV if profile == "relocal" else COLOR_ODOM
        loc_button = self.buttons.get("loc_profile")
        if loc_button:
            loc_button.configure(text=f"LOC {self.localization_profile_label(profile)}")
        self.labels["bridge"].configure(
            text="connected" if self.bridge_connected else "waiting" if running else "--",
            fg="#30d158" if self.bridge_connected else "#ffd60a" if running else COLOR_TEXT,
        )
        self.labels["estop"].configure(
            text=self.latest_estop,
            fg="#ff453a" if self.latest_estop.lower() == "true" else COLOR_TEXT,
        )
        self.labels["relocal"].configure(
            text=self.latest_relocal_text,
            fg=self.level_color(self.latest_relocal_level),
        )
        self.labels["can_status"].configure(
            text=self.latest_can_text,
            fg=self.level_color(self.latest_can_level),
        )

        task = self.selected_task()
        if task:
            first_id = task.get("first_id", task.get("start_index", ""))
            last_id = task.get("last_id", task.get("end_index", ""))
            range_text = f" {first_id}-{last_id}" if first_id or last_id else ""
            self.labels["task"].configure(
                text=self.compact_text(f"task: {task.get('name', '--')}{range_text}", 22),
                fg=COLOR_TEXT,
            )
        else:
            self.labels["task"].configure(text="task: --", fg=COLOR_MUTED)

        odom_xy = self.latest_odom_xy
        re_xy = self.latest_re_xy
        odom_yaw = self.format_yaw_deg(self.latest_odom_yaw)
        re_yaw = self.format_yaw_deg(self.latest_re_yaw)
        odom_text = (
            "odom x=-- y=-- yaw=--"
            if odom_xy is None
            else f"odom x={odom_xy[0]:.2f} y={odom_xy[1]:.2f} yaw={odom_yaw}"
        )
        re_text = (
            "re   x=-- y=-- yaw=--"
            if re_xy is None
            else f"re   x={re_xy[0]:.2f} y={re_xy[1]:.2f} yaw={re_yaw}"
        )
        self.labels["odom_xy"].configure(
            text=f"{odom_text}\n{re_text}",
            fg=COLOR_TEXT if odom_xy is not None or re_xy is not None else COLOR_MUTED,
        )

        bridge_ready = self.bridge_connected
        self.set_button_state("action", not busy and not running, COLOR_ACTION)
        self.set_button_state("quit", running or busy, COLOR_QUIT_ACTIVE)
        self.set_button_state("loc_profile", not busy and not running, profile_color)
        self.set_button_state("nav", bridge_ready, COLOR_NAV)
        self.set_button_state("odom", bridge_ready and profile == "odom" and self.latest_mode == "NAV", COLOR_ODOM)
        self.set_button_state("estop", bridge_ready, COLOR_ESTOP)
        task_ready = bridge_ready and bool(self.task_specs)
        nav_task_ready = task_ready and self.latest_mode in {"NAV", "KEEP"} and self.relocal_ready()
        self.set_button_state("task_prev", task_ready, COLOR_QUIT)
        self.set_button_state("task_next", task_ready, COLOR_QUIT)
        self.set_button_state("keep", bridge_ready, COLOR_KEEP)
        self.set_button_state("task_stop", bridge_ready, COLOR_ODOM)
        self.set_button_state("task_resume", nav_task_ready, COLOR_NAV)
        self.set_button_state("estop_reset", bridge_ready, COLOR_ACTION)

    def set_message(self, text: str) -> None:
        self.message = text
        label = self.labels.get("message")
        if label:
            label.configure(text=self.compact_text(text, 58))

    def set_command_error(self, text: str) -> None:
        self.command_error_text = self.compact_text(text, 54)
        self.command_error_until = time.monotonic() + 8.0
        self.set_diagnostic_state(("error", self.command_error_text))

    def clear_command_error(self) -> None:
        self.command_error_text = ""
        self.command_error_until = 0.0

    def set_diagnostic_state(self, diagnostic: tuple[str, str]) -> None:
        level, text = diagnostic
        now = time.monotonic()
        if self.command_error_text and now <= self.command_error_until:
            level = "error"
            text = self.command_error_text if text == "err: ok" else f"{self.command_error_text} | {text}"
        elif self.command_error_text and now > self.command_error_until:
            self.clear_command_error()

        self.latest_error_level = level
        self.latest_error_text = self.compact_text(text, 64)
        label = self.labels.get("error")
        if not label:
            return
        color = COLOR_OK if level == "ok" else COLOR_WARN if level == "warn" else "#ff453a"
        label.configure(text=self.latest_error_text, fg=color)

    @staticmethod
    def level_color(level: str) -> str:
        if level == "ok":
            return COLOR_OK
        if level == "warn":
            return COLOR_WARN
        if level == "error":
            return "#ff453a"
        if level == "muted":
            return COLOR_MUTED
        return COLOR_TEXT

    def update_relocal_status(self, state: Optional[dict[str, Any]], nav: dict[str, Any]) -> None:
        profile = self.display_localization_profile()
        mode = self.read_relocalization_mode(profile)
        if mode is None and profile == "relocal":
            mode = 2
        elif mode is None and profile == "odom":
            mode = 0
        if mode is None:
            self.latest_relocal_text = "cfg?"
            self.latest_relocal_level = "warn"
            return

        mode_name = RELOCAL_MODE_NAMES.get(mode, str(mode))
        if mode != 2:
            self.latest_relocal_text = f"off/{mode_name}"
            self.latest_relocal_level = "muted"
            return

        relocal = nav.get("relocalization") if isinstance(nav, dict) else {}
        relocal = relocal if isinstance(relocal, dict) else {}
        external_seen = bool(relocal.get("external_tf_seen"))
        external_age_s = self.safe_float(relocal.get("external_tf_age_s"))
        handoff_pending = bool(relocal.get("handoff_pending"))
        pose_available = isinstance(nav.get("pose"), dict)

        odom_fallback = nav.get("odom_fallback") if isinstance(nav, dict) else {}
        if isinstance(odom_fallback, dict):
            handoff_pending = handoff_pending or bool(odom_fallback.get("handoff_pending"))

        if (external_seen and (external_age_s is None or external_age_s <= 5.0)) or pose_available:
            self.latest_relocal_text = "on/tf ok"
            self.latest_relocal_level = "ok"
        elif handoff_pending:
            self.latest_relocal_text = "on/handoff"
            self.latest_relocal_level = "ok"
        elif state:
            self.latest_relocal_text = "on/no tf"
            self.latest_relocal_level = "warn"
        else:
            self.latest_relocal_text = "on/wait"
            self.latest_relocal_level = "warn"

    def read_relocalization_mode(self, profile: Optional[str] = None) -> Optional[int]:
        profile_key = self.normalize_localization_profile(profile or self.localization_profile)
        path = self.resolve_repo_file(self.profile_config_rel(profile_key))
        if not path and profile is None:
            path = self.resolve_repo_file(ODIN_CONTROL_CONFIG_REL)
        if not path:
            return None
        try:
            with open(path, "r", encoding="utf-8") as f:
                text = f.read()
        except OSError:
            return None

        match = re.search(r"^\s*custom_map_mode\s*:\s*['\"]?(\d+)['\"]?", text, re.MULTILINE)
        if not match:
            return None
        try:
            return int(match.group(1))
        except ValueError:
            return None

    @staticmethod
    def resolve_repo_file(relative_path: str) -> Optional[str]:
        matches = FullscreenControlApp.resolve_repo_files([relative_path])
        return matches[0] if matches else None

    @staticmethod
    def resolve_repo_files(relative_paths: list[str]) -> list[str]:
        candidates = [
            os.environ.get("SIM2REAL_REPO_DIR", ""),
            REPO_DIR,
            LOCAL_REPO_DIR,
        ]
        matches: list[str] = []
        seen: set[str] = set()
        for base in candidates:
            if not base:
                continue
            for relative_path in relative_paths:
                path = os.path.abspath(os.path.join(base, relative_path))
                if path in seen:
                    continue
                seen.add(path)
                if os.path.exists(path):
                    matches.append(path)
        return matches

    def build_can_status(self, robot: dict[str, Any]) -> tuple[str, str]:
        if not robot:
            return ("warn", "no state") if self.bridge_connected else ("muted", "--")

        update_counts = robot.get("update_counts")
        if not isinstance(update_counts, list) or len(update_counts) < len(CAN_BUS_MAP):
            return "warn", "no counts"

        counts = [self.safe_int(value) or 0 for value in update_counts[: len(CAN_BUS_MAP)]]
        if max(counts) <= 0:
            return "error", "CAN no frames"

        stale_max = self.safe_int(robot.get("stale_max"))
        if stale_max is not None and stale_max >= 20:
            return "error", f"stale {stale_max}"

        for bus_id in sorted(set(CAN_BUS_MAP)):
            indexes = [index for index, value in enumerate(CAN_BUS_MAP) if value == bus_id]
            bus_counts = [counts[index] for index in indexes]
            bus_label = CAN_BUS_LABELS.get(bus_id, f"CAN{bus_id}")
            if bus_counts and max(bus_counts) <= 0:
                return "error", f"{bus_label} no frames"

        for bus_id in sorted(set(CAN_BUS_MAP)):
            bus_label = CAN_BUS_LABELS.get(bus_id, f"CAN{bus_id}")
            missing = [
                index + 1
                for index, value in enumerate(counts)
                if CAN_BUS_MAP[index] == bus_id and value <= 0
            ]
            if missing:
                return "error", f"{bus_label} miss m{missing[0]}"

        return "ok", "ok"

    def build_error_summary(self, state: Optional[dict[str, Any]]) -> tuple[str, str]:
        issues: list[tuple[str, str]] = []

        if self.manual_cleanup_required:
            issues.append(("error", "PROC cleanup needed"))
        if self.latest_estop.lower() == "true":
            issues.append(("error", "ESTOP active"))
        if self.action_running() and not self.bridge_connected:
            issues.append(("warn", "BRIDGE waiting"))
        if not self.bridge_connected and self.system_process_running:
            issues.append(("warn", "PROC running no bridge"))

        if state:
            mux_status = str(state.get("mux_status", "") or "")
            nav_status = str(state.get("nav_status", "") or "")
            nav = state.get("nav") if isinstance(state.get("nav"), dict) else {}
            runtime = state.get("runtime") if isinstance(state.get("runtime"), dict) else {}
            robot = state.get("robot") if isinstance(state.get("robot"), dict) else {}

            if self.display_localization_profile() == "relocal" and not isinstance(nav.get("pose"), dict):
                issues.append(("warn", "RELOC no TF"))

            target_source = str(runtime.get("target_source", "") or "")
            if target_source in {"safety_brake", "safety_estop"}:
                issues.append(("error", f"SAFETY {target_source}"))
            elif target_source in {"timeout_hold", "boot_hold"}:
                issues.append(("warn", f"TARGET {target_source}"))

            if robot:
                imu_age = self.safe_float(robot.get("imu_age_ms"))
                if robot.get("imu_fresh") is False:
                    issues.append(("error", f"IMU stale {self.format_ms(imu_age)}"))
                odom_age = self.safe_float(robot.get("odom_age_ms"))
                if robot.get("odom_fresh") is False:
                    issues.append(("warn", f"ODOM stale {self.format_ms(odom_age)}"))

                can_level, can_text = self.build_can_status(robot)
                if can_level in {"warn", "error"} and can_text not in {"--", "no state"}:
                    issue_text = can_text if can_text.startswith("CAN") else f"CAN {can_text}"
                    issues.append((can_level, issue_text))
            elif self.bridge_connected:
                issues.append(("warn", "HW no runtime/state"))

            status_text = f"{nav_status} {mux_status}".lower()
            for marker in ("failed", "error", "fault", "unavailable"):
                if marker in status_text:
                    issues.append(("warn", self.compact_text(f"NAV {marker}: {nav_status or mux_status}", 40)))
                    break

        if not issues:
            return "ok", "err: ok"

        level = "error" if any(item[0] == "error" for item in issues) else "warn"
        text = " | ".join(item[1] for item in issues[:2])
        return level, text

    @staticmethod
    def parse_xy(x_value: Any, y_value: Any) -> Optional[tuple[float, float]]:
        try:
            return float(x_value), float(y_value)
        except (TypeError, ValueError):
            return None

    @staticmethod
    def safe_float(value: Any) -> Optional[float]:
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    @staticmethod
    def format_yaw_deg(yaw_rad: Optional[float]) -> str:
        if yaw_rad is None:
            return "--"
        yaw_deg = math.degrees(yaw_rad)
        while yaw_deg > 180.0:
            yaw_deg -= 360.0
        while yaw_deg <= -180.0:
            yaw_deg += 360.0
        return f"{yaw_deg:.0f}"

    @staticmethod
    def safe_int(value: Any) -> Optional[int]:
        try:
            return int(value)
        except (TypeError, ValueError):
            return None

    @staticmethod
    def format_ms(value: Optional[float]) -> str:
        return "--ms" if value is None else f"{value:.0f}ms"

    @staticmethod
    def compact_text(text: str, limit: int) -> str:
        text = str(text)
        if len(text) <= limit:
            return text
        return text[: max(0, limit - 3)] + "..."

    def close_app(self, _event: object = None) -> None:
        if self.manual_cleanup_required:
            self.set_message("manual Odin cleanup required before closing")
            return
        if self.action_running():
            self.quit_action()
            self.root.after(500, self.close_app)
            return
        if self.action_busy():
            self.set_message("waiting for action cleanup")
            self.root.after(500, self.close_app)
            return
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    try:
        FullscreenControlApp().run()
    except tk.TclError as exc:
        print("Failed to open the display.", file=sys.stderr)
        print(f"DISPLAY={os.environ.get('DISPLAY', '<unset>')}", file=sys.stderr)
        print("XAUTHORITY values tried:", file=sys.stderr)
        for attempt in DISPLAY_ATTEMPTS:
            print(f"  {attempt}", file=sys.stderr)
        print("If running over SSH on the Nano, try:", file=sys.stderr)
        print("  DISPLAY=:0 python3 fullscreen_quit.py", file=sys.stderr)
        print("If authorization still fails, run this once in the Nano desktop terminal:", file=sys.stderr)
        print(f"  DISPLAY=:0 xhost +SI:localuser:{current_user()}", file=sys.stderr)
        print("Also make sure a desktop session is running on the screen.", file=sys.stderr)
        raise SystemExit(1) from exc


if __name__ == "__main__":
    main()
