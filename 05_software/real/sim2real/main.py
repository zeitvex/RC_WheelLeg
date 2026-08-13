"""CLI entrypoint for current sim2real deployment."""

import argparse
import os
import sys
import threading
import time
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))

from input_dev.keyboard import KeyboardCommandController
from interface.real_io import RealIO
from policy.policy_runner import PolicyRunner
from safety.runtime_guard import GuardLevel, RuntimeGuard
from safety.safety_monitor import SafetyLevel, SafetyMonitor
from startup.pose_initializer import PoseInitFailed, PoseInitializer, STAND_POSE
from startup.stand_balance import StandBalanceController
from tools.logger import LogBundle
from tools.math_utils import get_gravity_orientation

JOINT_LABELS = LogBundle.JOINT_LABELS


def make_real_driver_factory():
    def factory(can1_port, can2_port, debug):
        sim2real_root = Path(__file__).resolve().parent
        for path in (
            sim2real_root / "vendored",
            "/home/rc2/work/rcwork/control",
            "/home/rc2/work/rcwork",
        ):
            path_str = str(path)
            if path_str not in sys.path and Path(path).exists():
                sys.path.append(path_str)
        from drivers.motor_driver import RobStrideDriver  # type: ignore

        return RobStrideDriver(can1_port, debug), RobStrideDriver(can2_port, debug)

    return factory


def make_dry_driver_factory():
    class MockMotor:
        def __init__(self):
            class State:
                position = 0.0
                velocity = 0.0
                torque = 0.0

            self.state = State()

    class MockDriver:
        def __init__(self, port, debug):
            self.port = port
            self.motors = {}

        def connect(self): ...
        def disconnect(self): ...
        def add_motor(self, name, motor_id, model): self.motors[name] = MockMotor()
        def enable(self, name): ...
        def disable(self, name): ...
        def clear_warnings(self, name): ...
        def process_messages(self): ...
        def control_mit(self, *args, **kwargs): ...

    def factory(can1_port, can2_port, debug):
        return MockDriver(can1_port, debug), MockDriver(can2_port, debug)

    return factory


def _sleep_to(next_exec: float) -> float:
    slack = next_exec - time.perf_counter()
    if slack > 0:
        time.sleep(slack)
        return next_exec + 0.0
    return time.perf_counter()


def build_action_diag(
    *,
    joint_pos: np.ndarray,
    default_pose: np.ndarray,
    raw: np.ndarray,
    scaled: np.ndarray,
    tentative: np.ndarray,
    cmd: np.ndarray,
    zero_command: bool,
    runtime_released: bool,
    release_alpha: float,
    safety_details: dict | None = None,
) -> dict:
    details = dict(safety_details or {})
    joint_indices = list(details.get("joint_indices", []))
    pos_err = tentative - joint_pos
    leg_offset = tentative[:12] - default_pose[:12]
    diag = {
        "joint_indices": joint_indices,
        "joint_names": [JOINT_LABELS[i] for i in joint_indices if 0 <= i < len(JOINT_LABELS)],
        "cmd": cmd.tolist(),
        "zero_command": bool(zero_command),
        "runtime_released": bool(runtime_released),
        "release_alpha": float(release_alpha),
        "max_raw": float(np.max(np.abs(raw))) if raw.size else 0.0,
        "max_scaled": float(np.max(np.abs(scaled[:12]))) if scaled.size else 0.0,
        "max_target": float(np.max(np.abs(tentative[:12]))) if tentative.size else 0.0,
    }
    if joint_indices:
        primary = int(joint_indices[0])
        diag.update(
            {
                "primary_joint_index": primary,
                "primary_joint_name": JOINT_LABELS[primary],
                "primary_target": float(tentative[primary]),
                "primary_default": float(default_pose[primary]),
                "primary_measured": float(joint_pos[primary]),
                "primary_pos_err": float(pos_err[primary]),
                "primary_raw": float(raw[primary]),
                "primary_scaled": float(scaled[primary]),
            }
        )
        if primary < 12:
            diag["primary_leg_offset"] = float(leg_offset[primary])
    details.update(diag)
    return details


def policy_release_cfg(cfg: dict) -> dict[str, float]:
    policy_cfg = cfg.get("policy", {})
    return {
        "command_hold_s": max(float(policy_cfg.get("release_command_hold_s", 0.12)), 0.0),
        "posture_max_err": max(float(policy_cfg.get("release_posture_max_err", 0.35)), 0.0),
        "target_blend_s": max(float(policy_cfg.get("release_target_blend_s", 0.30)), 1e-3),
    }


def compute_release_metrics(runner: PolicyRunner, state: dict, hold_target: np.ndarray, cmd: np.ndarray) -> dict:
    joint_pos = np.asarray(state["joint_pos"], dtype=np.float32)
    default_pose = np.asarray(runner.default_dof_pos, dtype=np.float32)
    hold_target = np.asarray(hold_target, dtype=np.float32)
    planar_cmd, yaw_cmd = runner.command_activation_metrics(cmd)
    return {
        "planar_cmd": float(planar_cmd),
        "yaw_cmd": float(yaw_cmd),
        "max_hold_err": float(np.max(np.abs(joint_pos[:12] - hold_target[:12]))),
        "max_default_err": float(np.max(np.abs(joint_pos[:12] - default_pose[:12]))),
        "max_hold_default_gap": float(np.max(np.abs(hold_target[:12] - default_pose[:12]))),
    }


def blend_runtime_target(
    runner: PolicyRunner,
    hold_target: np.ndarray,
    policy_target: np.ndarray,
    release_alpha: float,
    target_blend_s: float,
    control_dt: float,
) -> np.ndarray:
    blend = min(1.0, release_alpha * (runner.command_release_s / max(target_blend_s, control_dt)))
    return ((1.0 - blend) * hold_target + blend * policy_target).astype(np.float32)


def compute_target_error_metrics(
    state: dict,
    hold_target: np.ndarray,
    policy_target: np.ndarray,
) -> dict[str, float]:
    joint_pos = np.asarray(state["joint_pos"], dtype=np.float32)
    hold_target = np.asarray(hold_target, dtype=np.float32)
    policy_target = np.asarray(policy_target, dtype=np.float32)
    return {
        "hold_target_max_err": float(np.max(np.abs(joint_pos[:12] - hold_target[:12]))),
        "policy_target_max_err": float(np.max(np.abs(joint_pos[:12] - policy_target[:12]))),
        "hold_policy_max_gap": float(np.max(np.abs(hold_target[:12] - policy_target[:12]))),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=str(Path(__file__).parent / "config.yaml"))
    parser.add_argument("--policy", default=None)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    with open(args.config, "r", encoding="utf-8") as file_obj:
        cfg = yaml.safe_load(file_obj)

    sim2real_root = Path(__file__).resolve().parent
    policy_path = Path(args.policy) if args.policy else sim2real_root / "policies" / "model_rough.pt"
    if not policy_path.exists():
        print(f"[Main] policy not found: {policy_path}")
        sys.exit(1)

    control_dt = 1.0 / float(cfg["control_freq"])
    driver_factory = make_dry_driver_factory() if args.dry_run else make_real_driver_factory()

    logger = LogBundle(cfg["log_dir"])
    logger.event(
        "CONFIG_LOADED",
        config_path=args.config,
        policy=str(policy_path),
        dry_run=args.dry_run,
        control_freq=cfg["control_freq"],
        motor_model=cfg["motor_model"],
    )

    io = RealIO(
        driver_factory=driver_factory,
        motor_model=cfg["motor_model"],
        can1_port=cfg["can1_port"],
        can2_port=cfg["can2_port"],
        imu_lib_path=cfg.get("imu_lib_path"),
        control_dt=control_dt,
        kp_leg=cfg["controller"]["kp_leg"],
        kd_leg=cfg["controller"]["kd_leg"],
        kd_wheel=cfg["controller"]["kd_wheel"],
        debug=cfg.get("debug", False),
    )
    runner = PolicyRunner(
        policy_path,
        enable_zero_cmd_suppression=cfg.get("policy", {}).get("enable_zero_cmd_suppression", True),
        hold_zero_command_pose=cfg.get("policy", {}).get("hold_zero_command_pose", True),
        command_release_s=cfg.get("policy", {}).get("command_release_s", 0.35),
        action_scale=np.asarray(
            cfg.get("policy", {}).get(
                "action_scale",
                [0.125, 0.25, 0.25, 0.125, 0.25, 0.25, 0.125, 0.25, 0.25, 0.125, 0.25, 0.25, 5.0, 5.0, 5.0, 5.0],
            ),
            dtype=np.float32,
        ),
        zero_cmd_use_yaw_rate=cfg.get("policy", {}).get("zero_cmd_use_yaw_rate", False),
    )
    require_active_command = cfg.get("policy", {}).get("require_active_command_to_release", True)
    keyboard = KeyboardCommandController(
        max_x_vel=cfg["controller"]["max_vx"],
        max_y_vel=cfg["controller"]["max_vy"],
        max_yaw_vel=cfg["controller"]["max_yaw_rate"],
    )
    safety = SafetyMonitor(
        max_target_offset=cfg["safety"]["max_target_offset"],
        max_ang_vel=cfg["safety"]["max_ang_vel"],
        max_tilt_z=cfg["safety"]["max_tilt_z"],
        clip_to_brake=cfg["safety"]["clip_to_brake"],
    )
    safety.reset()
    guard = RuntimeGuard(
        max_ang_vel=cfg["safety"]["max_ang_vel"],
        max_tilt_z=cfg["safety"]["max_tilt_z"],
        imu_age_warn_ms=cfg["safety"].get("imu_age_warn_ms", 60.0),
        imu_age_stop_ms=cfg["safety"].get("imu_age_stop_ms", 200.0),
    )
    initializer = PoseInitializer(
        io,
        control_dt=control_dt,
        transition_time_min=cfg["startup"].get("transition_time_min", 2.0),
        transition_time_max=cfg["startup"].get("transition_time_max", 6.0),
        transition_seconds_per_rad=cfg["startup"].get("transition_seconds_per_rad", 1.5),
        hold_time=cfg["startup"]["hold_time"],
        settle_pos_threshold=cfg["startup"]["settle_pos_threshold"],
        settle_vel_threshold=cfg["startup"]["settle_vel_threshold"],
        timeout_extra=cfg["startup"].get("timeout_extra", 3.0),
        progress_log_interval=cfg["startup"]["progress_log_interval"],
        ramp_kp_time=cfg["startup"].get("ramp_kp_time", 1.0),
        soft_hold_duration=cfg["startup"].get("soft_hold_duration", 1.0),
        max_dev_warn=cfg["startup"].get("max_dev_warn", 1.5),
        max_dev_abort=cfg["startup"].get("max_dev_abort", 3.0),
    )
    initializer.attach(logger=logger, guard=guard, keyboard=keyboard)
    stand_balance = StandBalanceController(cfg.get("stand_balance", {}), control_dt=control_dt)

    print("\n[Main] connecting hardware...")
    keyboard.start()
    try:
        io.connect()
        logger.event("CAN_IMU_CONNECTED", initial_gravity=io.imu.initial_gravity)
    except Exception as exc:
        logger.event("HARDWARE_CONNECT_FAILED", error=str(exc))
        keyboard.stop()
        logger.close()
        raise

    try:
        io.enable_motors()
        logger.event("MOTORS_ENABLED")
        time.sleep(0.5)

        target_pose = initializer.transition_to_stand_from_current(target_pose=STAND_POSE) if cfg["startup"]["enabled"] else STAND_POSE.copy()

        if stand_balance.enabled:
            logger.event("STAND_BALANCE_BEGIN")
            print("[Main] waiting for stand-balance to settle...")
            stand_balance.reset()
            next_exec = time.perf_counter()
            while True:
                state = io.read_state()
                target_pose = stand_balance.compute_target(state, np.zeros(3, dtype=np.float32))
                io.hold_pose(target_pose, kp_scale=1.0)
                debug = stand_balance.last_debug
                if stand_balance.is_stable():
                    logger.event(
                        "STAND_BALANCE_STABLE",
                        roll_deg=float(np.degrees(debug.roll)),
                        pitch_deg=float(np.degrees(debug.pitch)),
                    )
                    break
                next_exec += control_dt
                next_exec = _sleep_to(next_exec)
            logger.event("STAND_BALANCE_END")

        if cfg["startup"]["require_user_confirm"]:
            print("[Main] standing complete. Press Enter to release policy control...")
            done = threading.Event()

            def _wait():
                try:
                    input()
                except EOFError:
                    pass
                done.set()

            threading.Thread(target=_wait, daemon=True).start()
            if not initializer.hold_until_user_confirm(target_pose, done):
                raise PoseInitFailed("WAIT_USER interrupted")

        print("[Main] priming current observation...")
        logger.event("PRIME_BEGIN")
        zero_cmd = np.zeros(3, dtype=np.float32)
        next_exec = time.perf_counter()
        for index in range(1):
            if stand_balance.enabled:
                state = io.read_state()
                target_pose = stand_balance.compute_target(state, zero_cmd)
                io.hold_pose(target_pose, kp_scale=1.0)
            else:
                io.hold_pose(target_pose, kp_scale=1.0)
                state = io.read_state()
            obs = io.get_obs_policy(state, zero_cmd, runner.default_dof_pos, runner.last_actions)
            if index == 0:
                runner.reset(prime_obs=obs)
            logger.state(
                phase="PRIME",
                joint_pos=state["joint_pos"],
                joint_vel=state["joint_vel"],
                joint_torque=state.get("joint_torque", np.zeros(16, dtype=np.float32)),
                target_pose=target_pose,
                raw_action=None,
                gyro=state["imu_gyro"],
                accel=state["imu_accel"],
                quat=state["quat_wxyz"],
                proj_gravity=state["projected_gravity"],
                command=zero_cmd,
                imu_age_ms=float(state["imu_age_ms"]),
                loop_dt_ms=0.0,
                kp_scale=1.0,
            )
            next_exec += control_dt
            next_exec = _sleep_to(next_exec)
        logger.event("PRIME_END")

        print("[Main] entering 50Hz control loop... (space = estop)")
        logger.event("RUNTIME_BEGIN")
        next_exec = time.perf_counter()
        loop_count = 0
        last_print = next_exec
        log_every = int(cfg.get("log_every", 1))
        recent_dt_ms = []
        runtime_released = not require_active_command
        release_cfg = policy_release_cfg(cfg)
        release_active_time = 0.0

        while True:
            loop_t0 = time.perf_counter()
            cmd = keyboard.get_command()
            state = io.read_state()
            obs = io.get_obs_policy(state, cmd, runner.default_dof_pos, runner.last_actions)
            zero_command = runner._is_zero_command(cmd, state["imu_gyro"])

            obs_nan = bool(np.any(np.isnan(obs)) or np.any(np.isinf(obs)))
            if obs_nan:
                logger.event("OBS_NAN", obs_max=float(np.nanmax(obs)))
                io.damping_brake()
                break

            if not runtime_released and zero_command:
                raw = np.zeros(16, dtype=np.float32)
                scaled = np.zeros(16, dtype=np.float32)
                target_hold = stand_balance.compute_target(state, np.zeros(3, dtype=np.float32)) if stand_balance.enabled else runner.default_dof_pos.copy()
                actual_target = io.hold_pose(target_hold, kp_scale=1.0)
                policy_target = runner.default_dof_pos.copy()
                release_metrics = compute_release_metrics(runner, state, target_hold, cmd)
                target_metrics = compute_target_error_metrics(state, target_hold, policy_target)
                release_active_time = 0.0
                safety_decision = SafetyMonitor().check(
                    target_pose=target_hold,
                    default_pose=runner.default_dof_pos,
                    imu_gyro=state["imu_gyro"],
                    projected_gravity=state["projected_gravity"],
                    estop_triggered=keyboard.is_estop_triggered(),
                )
                guard_decision = guard.check(
                    imu_gyro=state["imu_gyro"],
                    projected_gravity=state["projected_gravity"],
                    imu_age_ms=float(state["imu_age_ms"]),
                    estop_triggered=keyboard.is_estop_triggered(),
                    extra_nan_arrays=(target_hold,),
                )
            else:
                target_hold = stand_balance.compute_target(state, np.zeros(3, dtype=np.float32)) if stand_balance.enabled else runner.default_dof_pos.copy()
                release_metrics = compute_release_metrics(runner, state, target_hold, cmd)
                if not runtime_released:
                    release_active_time += control_dt if runner.is_command_active(cmd) else 0.0
                    active_ready = release_active_time >= release_cfg["command_hold_s"]
                    posture_ready = release_metrics["max_hold_err"] <= release_cfg["posture_max_err"]
                    if active_ready and posture_ready:
                        runtime_released = True
                        logger.event(
                            "RUNTIME_COMMAND_RELEASED",
                            cmd=cmd.tolist(),
                            active_hold_s=release_active_time,
                            max_hold_err=release_metrics["max_hold_err"],
                            max_default_err=release_metrics["max_default_err"],
                            max_hold_default_gap=release_metrics["max_hold_default_gap"],
                        )
                    else:
                        reasons = []
                        if not active_ready:
                            reasons.append(f"cmd_hold<{release_cfg['command_hold_s']:.2f}s")
                        if not posture_ready:
                            reasons.append(f"hold_err>{release_cfg['posture_max_err']:.3f}")
                        logger.event(
                            "RUNTIME_RELEASE_BLOCKED",
                            reason=",".join(reasons),
                            cmd=cmd.tolist(),
                            active_hold_s=release_active_time,
                            max_hold_err=release_metrics["max_hold_err"],
                            max_default_err=release_metrics["max_default_err"],
                            max_hold_default_gap=release_metrics["max_hold_default_gap"],
                        )
                        raw = np.zeros(16, dtype=np.float32)
                        scaled = np.zeros(16, dtype=np.float32)
                        actual_target = io.hold_pose(target_hold, kp_scale=1.0)
                        policy_target = runner.default_dof_pos.copy()
                        target_metrics = compute_target_error_metrics(state, target_hold, policy_target)
                        safety_decision = SafetyMonitor().check(
                            target_pose=target_hold,
                            default_pose=runner.default_dof_pos,
                            imu_gyro=state["imu_gyro"],
                            projected_gravity=state["projected_gravity"],
                            estop_triggered=keyboard.is_estop_triggered(),
                        )
                        guard_decision = guard.check(
                            imu_gyro=state["imu_gyro"],
                            projected_gravity=state["projected_gravity"],
                            imu_age_ms=float(state["imu_age_ms"]),
                            estop_triggered=keyboard.is_estop_triggered(),
                            extra_nan_arrays=(target_hold,),
                        )
                        loop_dt_ms = (time.perf_counter() - loop_t0) * 1000.0
                        if log_every and (loop_count % log_every == 0):
                            motor_diag = state.get("motor_stale", {})
                            logger.state(
                                phase="RUNTIME",
                                joint_pos=state["joint_pos"],
                                joint_vel=state["joint_vel"],
                                joint_torque=state.get("joint_torque", np.zeros(16, dtype=np.float32)),
                                target_pose=actual_target,
                                raw_action=raw,
                                gyro=state["imu_gyro"],
                                accel=state["imu_accel"],
                                quat=state["quat_wxyz"],
                                proj_gravity=state["projected_gravity"],
                                command=cmd,
                                imu_age_ms=float(state["imu_age_ms"]),
                                loop_dt_ms=loop_dt_ms,
                                safety_level=int(safety_decision.level),
                                guard_level=int(guard_decision.level),
                                holdover=int(motor_diag.get("holdover_this_frame", 0)),
                                stale_max=int(motor_diag.get("stale_max", 0)),
                                fresh_count=int(motor_diag.get("fresh_count", 16)),
                                kp_scale=1.0,
                                nan_flag=0,
                                kp_leg_cmd=float(io.kp_leg),
                                kd_leg_cmd=float(io.kd_leg),
                                kd_wheel_cmd=float(io.kd_wheel),
                                runtime_release_alpha=0.0,
                                runtime_release_hold_s=release_active_time,
                                runtime_blend_ratio=0.0,
                                hold_target_max_err=target_metrics["hold_target_max_err"],
                                policy_target_max_err=target_metrics["policy_target_max_err"],
                                hold_policy_max_gap=target_metrics["hold_policy_max_gap"],
                                target_source="runtime_hold",
                                clip_primary_joint="",
                                safety_reason=f"release_blocked:{','.join(reasons)}",
                                guard_reason=guard_decision.reason,
                            )
                        next_exec += control_dt
                        next_exec = _sleep_to(next_exec)
                        loop_count += 1
                        continue
                scaled, raw = runner.step(obs)
                act_nan = bool(np.any(np.isnan(raw)) or np.any(np.isinf(raw)))
                if act_nan:
                    logger.event("ACTION_NAN")
                    io.damping_brake()
                    break

                policy_target = (scaled + runner.default_dof_pos).astype(np.float32)
                tentative = blend_runtime_target(
                    runner,
                    target_hold,
                    policy_target,
                    float(getattr(runner, "_command_release_alpha", 0.0)),
                    release_cfg["target_blend_s"],
                    control_dt,
                )
                scaled = tentative - runner.default_dof_pos
                target_metrics = compute_target_error_metrics(state, target_hold, policy_target)
                runtime_blend_ratio = min(
                    1.0,
                    float(getattr(runner, "_command_release_alpha", 0.0))
                    * (runner.command_release_s / max(release_cfg["target_blend_s"], control_dt)),
                )
                projected_gravity = get_gravity_orientation(state["quat_wxyz"])

                guard_decision = guard.check(
                    imu_gyro=state["imu_gyro"],
                    projected_gravity=projected_gravity,
                    imu_age_ms=float(state["imu_age_ms"]),
                    estop_triggered=keyboard.is_estop_triggered(),
                    extra_nan_arrays=(raw, tentative),
                )
                if guard_decision.level == GuardLevel.STOP:
                    logger.event("GUARD_STOP", phase="RUNTIME", reason=guard_decision.reason)
                    io.damping_brake()
                    break

                safety_decision = safety.check(
                    target_pose=tentative,
                    default_pose=runner.default_dof_pos,
                    imu_gyro=state["imu_gyro"],
                    projected_gravity=projected_gravity,
                    estop_triggered=keyboard.is_estop_triggered(),
                )
                if safety_decision.level == SafetyLevel.ESTOP:
                    logger.event("SAFETY_ESTOP", reason=safety_decision.message)
                    io.damping_brake()
                    break
                if safety_decision.level == SafetyLevel.BRAKE:
                    safety_diag = build_action_diag(
                        joint_pos=state["joint_pos"],
                        default_pose=runner.default_dof_pos,
                        raw=raw,
                        scaled=scaled,
                        tentative=tentative,
                        cmd=cmd,
                        zero_command=zero_command,
                        runtime_released=runtime_released,
                        release_alpha=float(getattr(runner, "_command_release_alpha", 0.0)),
                        safety_details=safety_decision.details,
                    )
                    logger.event(
                        "SAFETY_BRAKE",
                        reason=safety_decision.message,
                        details=safety_diag,
                        primary_joint=safety_diag.get("primary_joint_name"),
                        primary_offset=safety_diag.get("primary_leg_offset"),
                        primary_target=safety_diag.get("primary_target"),
                        primary_measured=safety_diag.get("primary_measured"),
                        primary_raw=safety_diag.get("primary_raw"),
                        primary_scaled=safety_diag.get("primary_scaled"),
                        cmd=cmd.tolist(),
                        release_alpha=float(getattr(runner, "_command_release_alpha", 0.0)),
                    )
                    io.damping_brake()
                    break
                if safety_decision.level == SafetyLevel.CLIP and safety_decision.clipped_target is not None:
                    scaled = safety_decision.clipped_target - runner.default_dof_pos
                    safety_diag = build_action_diag(
                        joint_pos=state["joint_pos"],
                        default_pose=runner.default_dof_pos,
                        raw=raw,
                        scaled=scaled,
                        tentative=tentative,
                        cmd=cmd,
                        zero_command=zero_command,
                        runtime_released=runtime_released,
                        release_alpha=float(getattr(runner, "_command_release_alpha", 0.0)),
                        safety_details=safety_decision.details,
                    )
                    logger.event(
                        "SAFETY_CLIP",
                        reason=safety_decision.message,
                        details=safety_diag,
                        primary_joint=safety_diag.get("primary_joint_name"),
                        primary_offset=safety_diag.get("primary_leg_offset"),
                        primary_target=safety_diag.get("primary_target"),
                        primary_measured=safety_diag.get("primary_measured"),
                        primary_raw=safety_diag.get("primary_raw"),
                        primary_scaled=safety_diag.get("primary_scaled"),
                        max_raw=float(np.max(np.abs(raw))),
                        cmd=cmd.tolist(),
                        release_alpha=float(getattr(runner, "_command_release_alpha", 0.0)),
                    )

                actual_target = io.send_actions(scaled, runner.default_dof_pos)
            loop_dt_ms = (time.perf_counter() - loop_t0) * 1000.0

            if log_every and (loop_count % log_every == 0):
                motor_diag = state.get("motor_stale", {})
                logger.state(
                    phase="RUNTIME",
                    joint_pos=state["joint_pos"],
                    joint_vel=state["joint_vel"],
                    joint_torque=state.get("joint_torque", np.zeros(16, dtype=np.float32)),
                    target_pose=actual_target,
                    raw_action=raw,
                    gyro=state["imu_gyro"],
                    accel=state["imu_accel"],
                    quat=state["quat_wxyz"],
                    proj_gravity=projected_gravity,
                    command=cmd,
                    imu_age_ms=float(state["imu_age_ms"]),
                    loop_dt_ms=loop_dt_ms,
                    safety_level=int(safety_decision.level),
                    guard_level=int(guard_decision.level),
                    holdover=int(motor_diag.get("holdover_this_frame", 0)),
                    stale_max=int(motor_diag.get("stale_max", 0)),
                    fresh_count=int(motor_diag.get("fresh_count", 16)),
                    kp_scale=1.0,
                    nan_flag=int(obs_nan or act_nan),
                    kp_leg_cmd=float(io.kp_leg),
                    kd_leg_cmd=float(io.kd_leg),
                    kd_wheel_cmd=float(io.kd_wheel),
                    runtime_release_alpha=float(getattr(runner, "_command_release_alpha", 0.0)),
                    runtime_release_hold_s=release_active_time,
                    runtime_blend_ratio=runtime_blend_ratio,
                    hold_target_max_err=target_metrics["hold_target_max_err"],
                    policy_target_max_err=target_metrics["policy_target_max_err"],
                    hold_policy_max_gap=target_metrics["hold_policy_max_gap"],
                    target_source="runtime_blend" if runtime_blend_ratio < 0.999 else "runtime_policy",
                    clip_primary_joint=str((safety_decision.details or {}).get("primary_joint_name", "")),
                    clip_primary_target=float((safety_decision.details or {}).get("primary_target", 0.0) or 0.0),
                    clip_primary_measured=float((safety_decision.details or {}).get("primary_measured", 0.0) or 0.0),
                    clip_primary_default=float((safety_decision.details or {}).get("primary_default", 0.0) or 0.0),
                    clip_primary_pos_err=float((safety_decision.details or {}).get("primary_pos_err", 0.0) or 0.0),
                    clip_primary_raw=float((safety_decision.details or {}).get("primary_raw", 0.0) or 0.0),
                    clip_primary_scaled=float((safety_decision.details or {}).get("primary_scaled", 0.0) or 0.0),
                    safety_reason=(
                        f"{safety_decision.message};zero_cmd={int(zero_command)};"
                        f"released={int(runtime_released)};alpha={getattr(runner, '_command_release_alpha', 0.0):.2f};"
                        f"max_raw={float(np.max(np.abs(raw))):.2f};"
                        f"clip={((safety_decision.details or {}).get('joint_indices', []))}"
                    ),
                    guard_reason=guard_decision.reason,
                )

            next_exec += control_dt
            slack = next_exec - time.perf_counter()
            if slack > 0:
                coarse = slack - 0.002
                if coarse > 0:
                    time.sleep(coarse)
                while time.perf_counter() < next_exec:
                    pass
            elif slack < -control_dt:
                logger.event("LOOP_OVERRUN", over_ms=-slack * 1000.0)
                next_exec = time.perf_counter()

            recent_dt_ms.append(loop_dt_ms)
            if len(recent_dt_ms) > 50:
                recent_dt_ms.pop(0)
            if len(recent_dt_ms) == 50:
                median_dt = float(np.median(recent_dt_ms))
                if median_dt > 22.0:
                    logger.event("SLOW_LOOP_TREND", median_dt_ms=median_dt)
                    recent_dt_ms.clear()

            loop_count += 1
            if time.perf_counter() - last_print > 1.0:
                print(
                    f"[Loop] cmd=[{cmd[0]:+.2f},{cmd[1]:+.2f},{cmd[2]:+.2f}] "
                    f"|raw|={float(np.max(np.abs(raw))):.2f} "
                    f"zero={int(zero_command)} rel={int(runtime_released)} "
                    f"alpha={getattr(runner, '_command_release_alpha', 0.0):.2f} "
                    f"imu_age={state['imu_age_ms']:.1f}ms "
                    f"holdover={io.hw.holdover_total} "
                    f"safety={int(safety_decision.level)}"
                )
                last_print = time.perf_counter()

    except PoseInitFailed as exc:
        print(f"[Main] startup aborted: {exc}")
        logger.event("POSE_INIT_FAILED", error=str(exc))
    except KeyboardInterrupt:
        print("\n[Main] Ctrl+C received, stopping...")
        logger.event("KEYBOARD_INTERRUPT")
    except Exception as exc:
        import traceback

        print(f"\n[Main] exception: {exc}")
        traceback.print_exc()
        logger.event("UNEXPECTED_ERROR", error=str(exc), traceback=traceback.format_exc())
    finally:
        print("[Main] cleaning up...")
        try:
            io.damping_brake()
            time.sleep(0.05)
            logger.event("DAMPING_BRAKE_APPLIED")
        except Exception as exc:
            logger.event("DAMPING_BRAKE_FAILED", error=str(exc))
        try:
            io.disconnect()
            logger.event("HARDWARE_DISCONNECTED")
        finally:
            keyboard.stop()
            logger.close()
        os._exit(0)


if __name__ == "__main__":
    main()
