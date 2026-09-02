from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from input_dev.remote_uart import RemoteCommandSource  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--max-vx", type=float, default=0.8)
    parser.add_argument("--max-vy", type=float, default=0.3)
    parser.add_argument("--max-yaw", type=float, default=0.5)
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--baudrate", type=int, default=100000)
    parser.add_argument("--timeout", type=float, default=0.02)
    parser.add_argument("--deadzone", type=int, default=50)
    args = parser.parse_args()

    remote = RemoteCommandSource(
        port=args.port,
        baudrate=args.baudrate,
        timeout=args.timeout,
        axis_deadzone=args.deadzone,
        active_threshold=args.deadzone,
        max_vx=args.max_vx,
        max_vy=args.max_vy,
        max_yaw=args.max_yaw,
    )
    remote.open()
    print(f"[remote-test] listening on {args.port}")
    try:
        period = 1.0 / max(args.hz, 1.0)
        while True:
            remote.poll()
            status = remote.get_status()
            print(
                "cmd=({:+.3f}, {:+.3f}, {:+.3f}) active={} estop={} raw=({}, {}, {}, {})".format(
                    status["cmd"][0],
                    status["cmd"][1],
                    status["cmd"][2],
                    status["command_active"],
                    status["estop_requested"],
                    status["ch1"],
                    status["ch2"],
                    status["ch3"],
                    status["ch4"],
                )
            )
            time.sleep(period)
    finally:
        remote.close()


if __name__ == "__main__":
    main()
