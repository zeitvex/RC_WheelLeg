#!/usr/bin/env python3
"""Run repeatable safety and sim2sim experiments for a route candidate."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run route safety + sim2sim experiment suite.")
    parser.add_argument("--points", type=Path, required=True)
    parser.add_argument("--terrain-xml", type=Path, default=PROJECT_ROOT / "tools/nav_tools/xml/1hao.xml")
    parser.add_argument("--onnx", type=Path, default=PROJECT_ROOT / "model_6800.onnx")
    parser.add_argument("--out-dir", type=Path, default=PROJECT_ROOT / "sim2sim/route_experiments")
    parser.add_argument("--start-yaw-offset-deg", type=float, default=-180.0)
    parser.add_argument("--heading-offset-deg", type=float, default=180.0)
    parser.add_argument("--start-z", type=float, default=0.75)
    parser.add_argument("--settle-steps", type=int, default=500)
    parser.add_argument("--follower", choices=("waypoint", "pure-pursuit"), default="pure-pursuit")
    parser.add_argument("--lookahead", type=float, default=0.45)
    parser.add_argument("--max-vx", type=float, default=0.22)
    parser.add_argument("--min-cmd-vx", type=float, default=0.04)
    parser.add_argument("--creep-cmd-vx", type=float, default=0.04)
    parser.add_argument("--yaw-stop-threshold-deg", type=float, default=45.0)
    parser.add_argument("--turn-in-place-enter-deg", type=float, default=70.0)
    parser.add_argument("--cmd-vx-scale", type=float, default=1.0)
    parser.add_argument("--no-local-safety", action="store_true")
    parser.add_argument("--include-reverse-vx", action="store_true")
    parser.add_argument("--quick", action="store_true", help="Use shorter timeouts for fast iteration.")
    return parser.parse_args()


def run_command(args: list[str], cwd: Path) -> tuple[int, str]:
    proc = subprocess.run(
        args,
        cwd=str(cwd),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    return proc.returncode, proc.stdout


def newest_report(out_dir: Path) -> Path | None:
    reports = sorted(out_dir.glob("route_check_*/report.json"), key=lambda p: p.stat().st_mtime, reverse=True)
    return reports[0] if reports else None


def load_report(path: Path | None) -> dict[str, Any]:
    if path is None:
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def run_sim_case(
    name: str,
    args: argparse.Namespace,
    extra: list[str],
    max_time: int,
    waypoint_timeout: int,
) -> dict[str, Any]:
    case_out = args.out_dir / name
    case_out.mkdir(parents=True, exist_ok=True)
    cmd = [
        sys.executable,
        "sim2sim/nav_route_sim2sim_check.py",
        "--terrain-xml",
        str(args.terrain_xml),
        "--points",
        str(args.points),
        "--onnx",
        str(args.onnx),
        "--policy-backend",
        "policy-runner",
        "--mission",
        "json",
        "--start-yaw-offset-deg",
        str(args.start_yaw_offset_deg),
        "--heading-offset-deg",
        str(args.heading_offset_deg),
        "--start-z",
        str(args.start_z),
        "--settle-steps",
        str(args.settle_steps),
        "--follower",
        str(args.follower),
        "--lookahead",
        str(args.lookahead),
        "--max-vx",
        str(args.max_vx),
        "--min-cmd-vx",
        str(args.min_cmd_vx),
        "--creep-cmd-vx",
        str(args.creep_cmd_vx),
        "--yaw-stop-threshold-deg",
        str(args.yaw_stop_threshold_deg),
        "--turn-in-place-enter-deg",
        str(args.turn_in_place_enter_deg),
        "--cmd-vx-scale",
        str(args.cmd_vx_scale),
        "--max-time",
        str(max_time),
        "--waypoint-timeout",
        str(waypoint_timeout),
        "--sample-every",
        "10",
        "--out-dir",
        str(case_out),
        *extra,
    ]
    if args.no_local_safety:
        cmd.append("--no-local-safety")
    code, output = run_command(cmd, PROJECT_ROOT)
    report_path = newest_report(case_out)
    report = load_report(report_path)
    return {
        "name": name,
        "exit_code": code,
        "success": bool(report.get("success", False)),
        "reason": report.get("reason", "no report"),
        "reached": f"{report.get('reached_count', '?')}/{report.get('waypoint_count', '?')}",
        "min_margin": report.get("min_margin"),
        "report": str(report_path) if report_path else None,
        "output_tail": "\n".join(output.splitlines()[-12:]),
    }


def main() -> int:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    suite_dir = args.out_dir / f"suite_{time.strftime('%Y%m%d_%H%M%S')}"
    suite_dir.mkdir(parents=True, exist_ok=True)
    args.out_dir = suite_dir

    safety_cmd = [
        sys.executable,
        "tools/nav_tools/route_safety_check.py",
        "--points",
        str(args.points),
        "--xml",
        str(args.terrain_xml),
        "--onnx",
        str(args.onnx),
        "--top",
        "12",
    ]
    safety_code, safety_output = run_command(safety_cmd, PROJECT_ROOT)
    cases = []
    full_time = 60 if args.quick else 180
    slice_time = 45 if args.quick else 90
    cases.append(run_sim_case("full", args, [], full_time, 20))
    cases.append(run_sim_case("start_1_8", args, ["--start-id", "1", "--end-id", "8"], slice_time, 20))
    cases.append(run_sim_case("mid_13_22", args, ["--start-id", "13", "--end-id", "22"], slice_time, 20))
    cases.append(run_sim_case("slalom_30_45", args, ["--start-id", "30", "--end-id", "45"], slice_time, 20))
    if args.include_reverse_vx:
        old_scale = args.cmd_vx_scale
        args.cmd_vx_scale = -abs(old_scale)
        cases.append(run_sim_case("reverse_start_1_8", args, ["--start-id", "1", "--end-id", "8"], slice_time, 20))
        cases.append(run_sim_case("reverse_slalom_30_45", args, ["--start-id", "30", "--end-id", "45"], slice_time, 20))
        args.cmd_vx_scale = old_scale

    summary = {
        "points": str(args.points),
        "terrain_xml": str(args.terrain_xml),
        "onnx": str(args.onnx),
        "follower": args.follower,
        "lookahead": args.lookahead,
        "max_vx": args.max_vx,
        "cmd_vx_scale": args.cmd_vx_scale,
        "no_local_safety": args.no_local_safety,
        "suite_dir": str(suite_dir),
        "safety_exit_code": safety_code,
        "safety_tail": "\n".join(safety_output.splitlines()[-18:]),
        "cases": cases,
    }
    summary_path = suite_dir / "summary.json"
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")

    print("Route experiment suite")
    print(f"  points: {args.points}")
    print(f"  suite: {suite_dir}")
    print(f"  safety: {'PASS' if safety_code == 0 else 'FAIL'}")
    for case in cases:
        print(
            f"  {case['name']}: success={case['success']} "
            f"reached={case['reached']} reason={case['reason']} "
            f"margin={case['min_margin']} report={case['report']}"
        )
    print(f"  summary: {summary_path}")
    return 0 if safety_code == 0 and all(case["success"] for case in cases) else 2


if __name__ == "__main__":
    raise SystemExit(main())
