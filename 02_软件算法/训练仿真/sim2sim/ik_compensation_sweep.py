#!/usr/bin/env python3
"""Sweep IK compensation parameters in the standalone sim2sim scene."""

from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from pathlib import Path
from typing import Any

THIS_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = THIS_DIR.parent
if str(THIS_DIR) not in sys.path:
    sys.path.insert(0, str(THIS_DIR))

SIM_PATH = THIS_DIR / "ik_slalom_sim2sim.py"
spec = importlib.util.spec_from_file_location("ik_slalom_sim2sim", SIM_PATH)
if spec is None or spec.loader is None:
    raise RuntimeError(f"Cannot load {SIM_PATH}")
sim = importlib.util.module_from_spec(spec)
sys.modules["ik_slalom_sim2sim"] = sim
spec.loader.exec_module(sim)


TRIALS = [
    {"name": "forward", "speed": 1.0, "yaw": 0.0, "target_vx": 1.0, "target_yaw": 0.0},
    {"name": "yaw", "speed": 0.0, "yaw": 1.0, "target_vx": 0.0, "target_yaw": 1.0},
    {"name": "arc", "speed": 1.0, "yaw": 1.0, "target_vx": 1.0, "target_yaw": 1.0},
]


def parse_float_list(text: str) -> list[float]:
    return [float(x.strip()) for x in text.split(",") if x.strip()]


def parse_bool_list(text: str) -> list[bool]:
    out: list[bool] = []
    for item in text.split(","):
        key = item.strip().lower()
        if not key:
            continue
        if key in {"1", "true", "on", "yes"}:
            out.append(True)
        elif key in {"0", "false", "off", "no"}:
            out.append(False)
        else:
            raise argparse.ArgumentTypeError(f"Invalid bool item: {item}")
    return out


def make_sim_args(args: argparse.Namespace, trial: dict[str, float | str], cfg: dict[str, Any]) -> argparse.Namespace:
    argv = [
        "ik_slalom_sim2sim.py",
        "--test",
        str(trial["name"]),
        "--duration",
        str(args.duration),
        "--settle",
        str(args.settle),
        "--speed",
        str(trial["speed"]),
        "--yaw-rate",
        str(trial["yaw"]),
        "--posture",
        "custom",
        "--custom-abduction",
        str(args.custom_abduction),
        "--custom-hip",
        str(args.custom_hip),
        "--custom-knee",
        str(args.custom_knee),
        "--wheel-model",
        "direct",
        "--linear-wheel-gain",
        str(args.linear_wheel_gain),
        "--direct-yaw-wheel-gain",
        str(args.direct_yaw_wheel_gain),
        "--max-wheel-speed",
        str(args.max_wheel_speed),
        "--wheel-accel-limit",
        str(args.wheel_accel_limit),
        "--yaw-rate-kp",
        str(cfg["yaw_rate_kp"]),
        "--encoder-posture-kp",
        str(cfg["encoder_posture_kp"]),
        "--encoder-posture-max",
        str(cfg["encoder_posture_max"]),
        "--roll-comp-gain",
        str(cfg["roll_comp_gain"]),
        "--pitch-comp-gain",
        str(cfg["pitch_comp_gain"]),
        "--no-realtime",
    ]
    argv.append("--imu-posture" if cfg["imu_posture"] else "--no-imu-posture")
    argv.append("--encoder-guard" if cfg["encoder_guard"] else "--no-encoder-guard")
    argv.append("--imu-guard" if cfg["imu_guard"] else "--no-imu-guard")
    old_argv = sys.argv
    try:
        sys.argv = argv
        return sim.parse_args()
    finally:
        sys.argv = old_argv


def score_trial(out: dict[str, Any], trial: dict[str, float | str]) -> dict[str, float]:
    vx = float(out["mean_body_vx_mps"])
    yaw = float(out["mean_yaw_rate_rad_s"])
    vx_err = abs(vx - float(trial["target_vx"]))
    yaw_err = abs(yaw - float(trial["target_yaw"]))
    return {
        "vx": vx,
        "yaw": yaw,
        "imu_gyro_z": float(out["mean_imu_gyro_z_rad_s"]),
        "vx_err": vx_err,
        "yaw_err": yaw_err,
        "err": vx_err + yaw_err,
    }


def run_sweep(args: argparse.Namespace) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for imu_posture in args.imu_posture_values:
        for encoder_guard in args.encoder_guard_values:
            for imu_guard in args.imu_guard_values:
                for encoder_posture_kp in args.encoder_posture_kps:
                    for encoder_posture_max in args.encoder_posture_maxs:
                        for yaw_rate_kp in args.yaw_rate_kps:
                            for roll_comp_gain in args.roll_comp_gains:
                                for pitch_comp_gain in args.pitch_comp_gains:
                                    cfg = {
                                        "imu_posture": imu_posture,
                                        "encoder_guard": encoder_guard,
                                        "imu_guard": imu_guard,
                                        "encoder_posture_kp": encoder_posture_kp,
                                        "encoder_posture_max": encoder_posture_max,
                                        "yaw_rate_kp": yaw_rate_kp,
                                        "roll_comp_gain": roll_comp_gain,
                                        "pitch_comp_gain": pitch_comp_gain,
                                    }
                                    detail: list[dict[str, Any]] = []
                                    speed_error = 0.0
                                    max_tilt = 0.0
                                    max_leg = 0.0
                                    mean_wheel_err = 0.0
                                    stable_all = True
                                    for trial in TRIALS:
                                        sim_args = make_sim_args(args, trial, cfg)
                                        out = sim.run_one(str(trial["name"]), sim_args)
                                        trial_score = score_trial(out, trial)
                                        trial_score["test"] = str(trial["name"])
                                        detail.append(trial_score)
                                        speed_error += trial_score["err"]
                                        max_tilt = max(max_tilt, float(out["max_tilt_deg"]))
                                        max_leg = max(max_leg, float(out["max_leg_encoder_error_rad"]))
                                        mean_wheel_err += float(out["mean_wheel_speed_error_rad_s"])
                                        stable_all = stable_all and bool(out["stable"])

                                    score = (
                                        speed_error
                                        + args.tilt_weight * max_tilt
                                        + args.leg_error_weight * max_leg
                                        + args.wheel_error_weight * (mean_wheel_err / len(TRIALS))
                                    )
                                    row = {
                                        **cfg,
                                        "score": round(score, 6),
                                        "speed_error_sum": round(speed_error, 6),
                                        "max_tilt_deg": round(max_tilt, 5),
                                        "max_leg_encoder_error_rad": round(max_leg, 6),
                                        "mean_wheel_speed_error_rad_s": round(mean_wheel_err / len(TRIALS), 6),
                                        "stable_all": stable_all,
                                        "detail": detail,
                                    }
                                    rows.append(row)
                                    print(
                                        "DONE "
                                        + json.dumps(
                                            {k: v for k, v in row.items() if k != "detail"},
                                            ensure_ascii=False,
                                        ),
                                        flush=True,
                                    )
    rows.sort(key=lambda r: float(r["score"]))
    return rows


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--settle", type=float, default=1.5)
    parser.add_argument("--custom-abduction", type=float, default=0.2)
    parser.add_argument("--custom-hip", type=float, default=1.697)
    parser.add_argument("--custom-knee", type=float, default=-2.650)
    parser.add_argument("--linear-wheel-gain", type=float, default=12.5)
    parser.add_argument("--direct-yaw-wheel-gain", type=float, default=8.0)
    parser.add_argument("--max-wheel-speed", type=float, default=12.0)
    parser.add_argument("--wheel-accel-limit", type=float, default=35.0)
    parser.add_argument("--imu-posture-values", type=parse_bool_list, default=[True, False])
    parser.add_argument("--encoder-guard-values", type=parse_bool_list, default=[True])
    parser.add_argument("--imu-guard-values", type=parse_bool_list, default=[True])
    parser.add_argument("--encoder-posture-kps", type=parse_float_list, default=[0.0, 0.05, 0.15, 0.30])
    parser.add_argument("--encoder-posture-maxs", type=parse_float_list, default=[0.03])
    parser.add_argument("--yaw-rate-kps", type=parse_float_list, default=[0.0, 0.4, 0.8])
    parser.add_argument("--roll-comp-gains", type=parse_float_list, default=[0.35])
    parser.add_argument("--pitch-comp-gains", type=parse_float_list, default=[0.35])
    parser.add_argument("--tilt-weight", type=float, default=0.02)
    parser.add_argument("--leg-error-weight", type=float, default=0.5)
    parser.add_argument("--wheel-error-weight", type=float, default=0.0)
    parser.add_argument("--top", type=int, default=12)
    parser.add_argument("--json", type=Path, default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rows = run_sweep(args)
    if args.json:
        args.json.write_text(json.dumps(rows, indent=2), encoding="utf-8")

    print("\nTop compensation parameter sets")
    print("rank score speed_err tilt leg_err wheel_err imu enc_kp yaw_kp enc_guard imu_guard")
    for i, row in enumerate(rows[: args.top], 1):
        print(
            f"{i:2d} {row['score']:7.4f} {row['speed_error_sum']:7.4f} "
            f"{row['max_tilt_deg']:5.2f} {row['max_leg_encoder_error_rad']:7.4f} "
            f"{row['mean_wheel_speed_error_rad_s']:7.4f} "
            f"{int(row['imu_posture'])} {row['encoder_posture_kp']:6.3f} "
            f"{row['yaw_rate_kp']:6.3f} {int(row['encoder_guard'])} {int(row['imu_guard'])}"
        )
        for d in row["detail"]:
            print(f"   {d['test']:<7} vx={d['vx']:+.3f} yaw={d['yaw']:+.3f} err={d['err']:.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
