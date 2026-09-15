#!/usr/bin/env python3
"""
Print display-related diagnostics for launching the fullscreen quit screen.
"""

import glob
import getpass
import os
import shlex
import subprocess
from typing import List, Optional


def run(command: List[str]) -> str:
    try:
        result = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
        )
    except OSError as exc:
        return f"<failed: {exc}>"

    output = result.stdout.strip()
    error = result.stderr.strip()
    if error:
        return f"{output}\n{error}".strip()
    return output


def current_user() -> str:
    try:
        return getpass.getuser()
    except OSError:
        return os.environ.get("USER", "unknown")


def xauth_from_x_processes() -> List[str]:
    output = run(["ps", "-eo", "args"])
    candidates = []
    for line in output.splitlines():
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


def print_path_status(label: str, path: Optional[str]) -> None:
    if not path:
        print(f"{label}: <unset>")
        return
    exists = os.path.exists(path)
    readable = os.access(path, os.R_OK) if exists else False
    print(f"{label}: {path} exists={exists} readable={readable}")


def main() -> int:
    uid = os.getuid()
    print(f"user: {run(['whoami'])}")
    print(f"uid: {uid}")
    print(f"DISPLAY: {os.environ.get('DISPLAY', '<unset>')}")
    print(f"XAUTHORITY: {os.environ.get('XAUTHORITY', '<unset>')}")
    print()

    print("candidate XAUTHORITY files:")
    candidates = [
        os.environ.get("XAUTHORITY"),
        *xauth_from_x_processes(),
        os.path.expanduser("~/.Xauthority"),
        f"/run/user/{uid}/gdm/Xauthority",
        f"/run/user/{uid}/Xauthority",
        *glob.glob(f"/run/user/{uid}/*Xauthority*"),
    ]

    seen = set()
    for candidate in candidates:
        key = candidate or "<unset>"
        if key in seen:
            continue
        seen.add(key)
        print_path_status("  -", candidate)

    print()
    print("X server processes:")
    x_lines = [
        line
        for line in run(["ps", "-eo", "user,args"]).splitlines()
        if "Xorg" in line or "Xwayland" in line
    ]
    if x_lines:
        for line in x_lines:
            print(f"  {line}")
    else:
        print("  <none found>")

    print()
    print("recommended SSH launch:")
    print("  cd <workspace>/screen")
    print("  bash run_fullscreen_quit.sh")
    print()
    print("if authorization fails, run this on the Nano desktop terminal once:")
    print(f"  DISPLAY=:0 xhost +SI:localuser:{current_user()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
