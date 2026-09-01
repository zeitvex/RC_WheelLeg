"""Sweep wheel-mode IK postures for differential-drive tracking.

The sweep is intentionally small and reproducible:
  1. Generate ab=0 leg postures in the requested height range.
  2. Keep candidates with good static geometry from rl_friendly_opt.
  3. Simulate forward, yaw, and arc commands in MuJoCo.
  4. Rank by attitude, x-speed tracking, yaw-rate tracking, and wheel contact.

Usage:
    uv run python mujoco_sim/ik_diff_sweep.py --quick
    uv run python mujoco_sim/ik_diff_sweep.py --height-min 0.15 --height-max 0.42
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

THIS_DIR = Path(__file__).resolve().parent
if str(THIS_DIR) not in sys.path:
    sys.path.insert(0, str(THIS_DIR))

from config import LEG_JOINTS, LEG_NAMES, SCENE_XML, WHEEL_JOINT, WHEEL_RADIUS  # noqa: E402
from rl_friendly_opt import get_all, rl_cost  # noqa: E402
from robot import Robot  # noqa: E402


@dataclass(frozen=True)
class Candidate:
    height: float
    ab: float
    hip: float
    knee: float
    static_cost: float
    r_hip_x: float
    cond: float
    max_tau: float


@dataclass(frozen=True)
class Trial:
    name: str
    vx: float
    yaw_rate: float
    duration: float


def wrap_pi(x: float) -> float:
    return (x + math.pi) % (2.0 * math.pi) - math.pi


def body_track(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    wheel_bids = []
    for leg in LEG_NAMES:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{leg}_wheel_Link")
        wheel_bids.append(bid)
    y = data.xipos[wheel_bids, 1]
    return float(np.mean(y[[0, 2]]) - np.mean(y[[1, 3]]))


def build_maps(robot: Robot) -> tuple[dict[str, int], dict[str, int], dict[str, int]]:
    act: dict[str, int] = {}
    qadr: dict[str, int] = {}
    vadr: dict[str, int] = {}
    for leg in LEG_NAMES:
        for jt in (*LEG_JOINTS, WHEEL_JOINT):
            name = f"{leg}_{jt}"
            act[name] = mujoco.mj_name2id(robot.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            jid = mujoco.mj_name2id(robot.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            qadr[name] = robot.model.jnt_qposadr[jid]
            vadr[name] = robot.model.jnt_dofadr[jid]
    return act, qadr, vadr


def set_posture(robot: Robot, cand: Candidate, act: dict[str, int], qadr: dict[str, int]) -> None:
    mujoco.mj_resetData(robot.model, robot.data)
    for leg in LEG_NAMES:
        side_ab = cand.ab if leg[1] == "l" else -cand.ab
        for jt, val in zip(LEG_JOINTS, (side_ab, cand.hip, cand.knee)):
            name = f"{leg}_{jt}"
            robot.data.qpos[qadr[name]] = val
            robot.data.ctrl[act[name]] = val
        robot.data.ctrl[act[f"{leg}_{WHEEL_JOINT}"]] = 0.0
    robot.data.qpos[:3] = [0.0, 0.0, max(0.25, cand.height + 0.08)]
    robot.data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    robot.data.qvel[:] = 0.0
    mujoco.mj_forward(robot.model, robot.data)


def wheel_targets(
    vx: float,
    yaw_rate: float,
    track: float,
    max_wheel: float,
    yaw_gain: float,
    wheel_model: str,
    linear_gain: float,
    direct_yaw_gain: float,
    wheel_signs: np.ndarray,
) -> np.ndarray:
    if wheel_model == "direct":
        left = linear_gain * vx - direct_yaw_gain * yaw_rate
        right = linear_gain * vx + direct_yaw_gain * yaw_rate
    else:
        left = (vx - yaw_gain * 0.5 * track * yaw_rate) / WHEEL_RADIUS
        right = (vx + yaw_gain * 0.5 * track * yaw_rate) / WHEEL_RADIUS
    raw = np.array([left, right, left, right], dtype=float)
    return np.clip(raw * wheel_signs, -max_wheel, max_wheel)


def run_trial(robot: Robot, cand: Candidate, trial: Trial, args: argparse.Namespace) -> dict:
    act, qadr, vadr = build_maps(robot)
    set_posture(robot, cand, act, qadr)

    ctrl_dt = args.control_dt
    sim_dt = robot.model.opt.timestep
    steps_per_ctrl = max(1, int(round(ctrl_dt / sim_dt)))
    track = body_track(robot.model, robot.data) if args.track_source == "model" else args.track_width
    wheel_signs = np.array(args.wheel_signs, dtype=float)
    wheel_cmd = wheel_targets(
        trial.vx,
        trial.yaw_rate,
        track,
        args.max_wheel_speed,
        args.yaw_gain,
        args.wheel_model,
        args.linear_gain,
        args.direct_yaw_gain,
        wheel_signs,
    )

    for _ in range(int(round(args.settle / sim_dt))):
        for leg in LEG_NAMES:
            vals = (cand.ab if leg[1] == "l" else -cand.ab, cand.hip, cand.knee)
            for jt, val in zip(LEG_JOINTS, vals):
                robot.data.ctrl[act[f"{leg}_{jt}"]] = val
            robot.data.ctrl[act[f"{leg}_{WHEEL_JOINT}"]] = 0.0
        robot.step()

    state0 = robot.get_state()
    yaw0 = float(state0.rpy[2])
    x0 = float(state0.pos[0])

    max_roll = 0.0
    max_pitch = 0.0
    max_tilt = 0.0
    max_wheel_air = -1e9
    wheel_err_sum = 0.0
    samples = 0
    max_leg_err = 0.0
    body_vx_sum = 0.0
    yaw_unwrapped = 0.0
    last_yaw = yaw0
    leg_target = np.array([cand.ab, cand.hip, cand.knee] * 4, dtype=float)

    total_steps = int(round(trial.duration / sim_dt))
    cmd = np.zeros(4, dtype=float)
    max_delta = args.wheel_accel_limit * ctrl_dt
    wheel_body_ids = [
        mujoco.mj_name2id(robot.model, mujoco.mjtObj.mjOBJ_BODY, f"{leg}_wheel_Link")
        for leg in LEG_NAMES
    ]
    wheel_vadr = np.array([vadr[f"{leg}_{WHEEL_JOINT}"] for leg in LEG_NAMES], dtype=int)
    leg_qadr = np.array([qadr[f"{leg}_{jt}"] for leg in LEG_NAMES for jt in LEG_JOINTS], dtype=int)

    for step in range(total_steps):
        if step % steps_per_ctrl == 0:
            cmd = cmd + np.clip(wheel_cmd - cmd, -max_delta, max_delta)
            for leg in LEG_NAMES:
                vals = (cand.ab if leg[1] == "l" else -cand.ab, cand.hip, cand.knee)
                for jt, val in zip(LEG_JOINTS, vals):
                    robot.data.ctrl[act[f"{leg}_{jt}"]] = val
            for i, leg in enumerate(LEG_NAMES):
                robot.data.ctrl[act[f"{leg}_{WHEEL_JOINT}"]] = cmd[i]

        robot.step()

        if step % steps_per_ctrl == 0:
            st = robot.get_state()
            roll, pitch = float(st.rpy[0]), float(st.rpy[1])
            yaw_now = float(st.rpy[2])
            yaw_unwrapped += wrap_pi(yaw_now - last_yaw)
            last_yaw = yaw_now
            body_vx_sum += float(st.rot[:, 0].dot(st.lin_vel))
            max_roll = max(max_roll, abs(roll))
            max_pitch = max(max_pitch, abs(pitch))
            max_tilt = max(max_tilt, math.hypot(roll, pitch))
            wheel_air = robot.data.xipos[wheel_body_ids, 2] - WHEEL_RADIUS
            max_wheel_air = max(max_wheel_air, float(np.max(wheel_air)))
            wheel_err_sum += float(np.mean(np.abs(robot.data.qvel[wheel_vadr] - cmd)))
            max_leg_err = max(max_leg_err, float(np.max(np.abs(robot.data.qpos[leg_qadr] - leg_target))))
            samples += 1

    st = robot.get_state()
    elapsed = max(1e-6, float(st.time - state0.time))
    world_x_rate = (float(st.pos[0]) - x0) / elapsed
    x_rate = body_vx_sum / max(1, samples)
    yaw_rate = yaw_unwrapped / max(1e-6, samples * steps_per_ctrl * sim_dt)
    x_err = abs(x_rate - trial.vx)
    yaw_err = abs(yaw_rate - trial.yaw_rate)
    return {
        "trial": trial.name,
        "x_rate": x_rate,
        "world_x_rate": world_x_rate,
        "yaw_rate": yaw_rate,
        "x_err": x_err,
        "yaw_err": yaw_err,
        "max_roll_deg": math.degrees(max_roll),
        "max_pitch_deg": math.degrees(max_pitch),
        "max_tilt_deg": math.degrees(max_tilt),
        "max_wheel_air_m": max_wheel_air,
        "mean_wheel_err": wheel_err_sum / max(1, samples),
        "max_leg_err": max_leg_err,
        "track": track,
        "wheel_cmd": [float(x) for x in wheel_cmd],
    }


def generate_candidates(args: argparse.Namespace) -> list[Candidate]:
    if args.fixed_hip is not None or args.fixed_knee is not None:
        if args.fixed_hip is None or args.fixed_knee is None:
            raise SystemExit("--fixed-hip and --fixed-knee must be provided together")
        ab = float(args.fixed_ab)
        hip = float(args.fixed_hip)
        knee = float(args.fixed_knee)
        r = get_all(ab, hip, knee)
        return [
            Candidate(
                height=float(r["z"]),
                ab=ab,
                hip=hip,
                knee=knee,
                static_cost=float(rl_cost(r)),
                r_hip_x=float(r["r_hip_x_mag"]),
                cond=float(r["cond"]),
                max_tau=float(r["max_tau"]),
            )
        ]

    cands: list[Candidate] = []
    h_targets = np.arange(args.height_min, args.height_max + 0.5 * args.height_step, args.height_step)
    ab_values = [0.0] if args.ab_max <= 1e-9 else np.arange(0.0, args.ab_max + 1e-9, args.ab_step)
    hip_values = np.arange(args.hip_min, args.hip_max + 0.5 * args.hip_step, args.hip_step)
    knee_values = np.arange(args.knee_min, args.knee_max + 0.5 * args.knee_step, args.knee_step)
    for ht in h_targets:
        bucket: list[Candidate] = []
        for ab in ab_values:
            for hip in hip_values:
                for knee in knee_values:
                    r = get_all(float(ab), float(hip), float(knee))
                    if float(r["z"]) < args.height_min or float(r["z"]) > args.height_max:
                        continue
                    if abs(float(r["z"]) - float(ht)) > args.height_tol:
                        continue
                    if r["wz"] >= r["kz"]:
                        continue
                    if abs(r["r_hip_x_mag"]) > args.max_wheel_x:
                        continue
                    cost = float(rl_cost(r))
                    bucket.append(
                        Candidate(
                            height=float(r["z"]),
                            ab=float(ab),
                            hip=float(hip),
                            knee=float(knee),
                            static_cost=cost,
                            r_hip_x=float(r["r_hip_x_mag"]),
                            cond=float(r["cond"]),
                            max_tau=float(r["max_tau"]),
                        )
                    )
        bucket.sort(key=lambda c: c.static_cost)
        cands.extend(bucket[: args.top_per_height])
    return cands


def score_result(cand: Candidate, trials: list[dict]) -> float:
    score = 0.08 * cand.static_cost
    for t in trials:
        score += 8.0 * t["x_err"]
        score += 10.0 * t["yaw_err"]
        score += 0.08 * t["max_tilt_deg"]
        score += 0.03 * max(0.0, t["max_pitch_deg"] - 8.0) ** 2
        score += 20.0 * max(0.0, t["max_wheel_air_m"] - 0.015)
        score += 1.5 * t["mean_wheel_err"]
        score += 2.0 * max(0.0, t["max_leg_err"] - 0.35)
    return float(score)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--height-min", type=float, default=0.15)
    parser.add_argument("--height-max", type=float, default=0.42)
    parser.add_argument("--height-step", type=float, default=0.01)
    parser.add_argument("--height-tol", type=float, default=0.004)
    parser.add_argument("--top-per-height", type=int, default=1)
    parser.add_argument("--ab-max", type=float, default=0.0)
    parser.add_argument("--ab-step", type=float, default=0.04)
    parser.add_argument("--hip-min", type=float, default=0.25)
    parser.add_argument("--hip-max", type=float, default=1.05)
    parser.add_argument("--hip-step", type=float, default=0.025)
    parser.add_argument("--knee-min", type=float, default=-2.65)
    parser.add_argument("--knee-max", type=float, default=-0.85)
    parser.add_argument("--knee-step", type=float, default=0.025)
    parser.add_argument("--max-wheel-x", type=float, default=0.09)
    parser.add_argument("--fixed-ab", type=float, default=0.0)
    parser.add_argument("--fixed-hip", type=float, default=None)
    parser.add_argument("--fixed-knee", type=float, default=None)
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--settle", type=float, default=1.5)
    parser.add_argument("--control-dt", type=float, default=0.02)
    parser.add_argument("--vx", type=float, default=0.6)
    parser.add_argument("--yaw", type=float, default=0.3)
    parser.add_argument("--arc-yaw", type=float, default=0.15)
    parser.add_argument("--yaw-gain", type=float, default=1.0)
    parser.add_argument("--wheel-model", choices=("diff", "direct"), default="diff")
    parser.add_argument("--linear-gain", type=float, default=12.5)
    parser.add_argument("--direct-yaw-gain", type=float, default=8.0)
    parser.add_argument("--max-wheel-speed", type=float, default=12.0)
    parser.add_argument("--wheel-accel-limit", type=float, default=35.0)
    parser.add_argument("--track-source", choices=("model", "fixed"), default="model")
    parser.add_argument("--track-width", type=float, default=0.394)
    parser.add_argument(
        "--wheel-signs",
        type=float,
        nargs=4,
        default=[1.0, 1.0, 1.0, 1.0],
        metavar=("FL", "FR", "RL", "RR"),
        help="Per-wheel velocity sign multipliers in joint order.",
    )
    parser.add_argument("--quick", action="store_true")
    parser.add_argument("--json", type=Path, default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.quick:
        args.height_step = 0.02
        args.hip_step = 0.05
        args.knee_step = 0.05

    candidates = generate_candidates(args)
    if not candidates:
        raise SystemExit("No candidates found")

    trials = [
        Trial("forward", args.vx, 0.0, args.duration),
        Trial("yaw", 0.0, args.yaw, args.duration),
        Trial("arc", args.vx, args.arc_yaw, args.duration),
    ]
    robot = Robot(SCENE_XML)
    rows = []
    for i, cand in enumerate(candidates, 1):
        trial_rows = [run_trial(robot, cand, t, args) for t in trials]
        rows.append({"candidate": cand.__dict__, "trials": trial_rows, "score": score_result(cand, trial_rows)})
        if i % 10 == 0:
            print(f"tested {i}/{len(candidates)}")

    rows.sort(key=lambda r: r["score"])
    if args.json:
        args.json.write_text(json.dumps(rows, indent=2), encoding="utf-8")

    print("Top IK postures for differential drive tracking")
    print("rank score height ab hip knee static xhip cond tau | fwd_x yaw_wz arc_x arc_wz max_tilt max_pitch")
    for rank, row in enumerate(rows[:10], 1):
        c = row["candidate"]
        by = {t["trial"]: t for t in row["trials"]}
        max_tilt = max(t["max_tilt_deg"] for t in row["trials"])
        max_pitch = max(t["max_pitch_deg"] for t in row["trials"])
        print(
            f"{rank:>2} {row['score']:>7.2f} {c['height']:.3f} {c['ab']:.2f} {c['hip']:.3f} {c['knee']:.3f} "
            f"{c['static_cost']:.1f} {c['r_hip_x']:.3f} {c['cond']:.2f} {c['max_tau']:.2f} | "
            f"{by['forward']['x_rate']:.3f} {by['yaw']['yaw_rate']:.3f} {by['arc']['x_rate']:.3f} {by['arc']['yaw_rate']:.3f} "
            f"{max_tilt:.1f} {max_pitch:.1f}"
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
