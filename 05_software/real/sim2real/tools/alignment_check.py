"""Offline deployment alignment check for the current 53-D rough policy."""

import argparse
from pathlib import Path
import sys

import numpy as np
import torch
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from interface.motor_mapping import MotorMapping  # noqa: E402
from policy.policy_runner import PolicyRunner  # noqa: E402


def _load_manifest(manifest_path: Path) -> dict:
    with open(manifest_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def check(policy_path: Path, manifest_path: Path | None = None) -> int:
    issues: list[tuple[str, str]] = []
    manifest = _load_manifest(manifest_path) if manifest_path is not None else None

    expected = (
        ("fl", "hip_abduction"), ("fl", "hip_pitch"), ("fl", "knee"),
        ("fr", "hip_abduction"), ("fr", "hip_pitch"), ("fr", "knee"),
        ("rl", "hip_abduction"), ("rl", "hip_pitch"), ("rl", "knee"),
        ("rr", "hip_abduction"), ("rr", "hip_pitch"), ("rr", "knee"),
        ("fl", "wheel"), ("fr", "wheel"), ("rl", "wheel"), ("rr", "wheel"),
    )
    if MotorMapping.SIM_JOINT_ORDER != expected:
        issues.append(("joint_order", "MotorMapping.SIM_JOINT_ORDER mismatch"))
    else:
        print("[Check] joint order: PASS")

    manifest_enable_zero_cmd = True
    if manifest is not None:
        manifest_enable_zero_cmd = bool(
            manifest.get("model", {}).get("enable_zero_cmd_suppression", True)
        )

    runner = PolicyRunner(
        policy_path,
        device=torch.device("cpu"),
        enable_zero_cmd_suppression=manifest_enable_zero_cmd,
    )
    obs_mean = runner.policy.obs_mean.detach().cpu().numpy()
    obs_std = runner.policy.obs_std.detach().cpu().numpy()
    if np.allclose(obs_mean, 0.0) and np.allclose(obs_std, 1.0):
        print("[Check] obs normalizer: PASS (identity)")
    else:
        print(
            f"[Check] obs normalizer: PASS "
            f"(mean range=[{obs_mean.min():.3f},{obs_mean.max():.3f}], "
            f"std range=[{obs_std.min():.3f},{obs_std.max():.3f}])"
        )
    if (obs_std < 1e-6).any():
        issues.append(
            (
                "normalizer_zero_std",
                f"obs_std has near-zero entries: {np.where(obs_std < 1e-6)[0].tolist()}",
            )
        )

    raw_zero = np.zeros(runner.BASE_OBS_DIM, dtype=np.float32)
    runner.reset(prime_obs=raw_zero)
    _, raw = runner.step(raw_zero)
    if np.max(np.abs(raw)) > 5.0:
        issues.append(
            (
                "output_range",
                f"raw action too large under zero obs: {np.max(np.abs(raw)):.3f}",
            )
        )
    else:
        print(f"[Check] zero-obs output range: PASS (max|raw|={np.max(np.abs(raw)):.3f})")

    expected_default = np.array([0.0, 0.9, -1.8] * 4 + [0.0] * 4, dtype=np.float32)
    if not np.allclose(runner.default_dof_pos, expected_default):
        issues.append(("default_pose_mismatch", f"default_dof_pos mismatch: {runner.default_dof_pos}"))
    else:
        print("[Check] default_dof_pos: PASS")

    if runner.BASE_OBS_DIM != 53:
        issues.append(("obs_dim", f"base obs dim {runner.BASE_OBS_DIM} != 53"))
    else:
        print("[Check] actor obs dim: PASS (53)")

    if manifest is not None:
        declared_model = manifest.get("model", {})
        declared_action = manifest.get("action", {})
        declared_safety = manifest.get("safety", {})
        declared_control = manifest.get("control", {})

        if int(declared_model.get("obs_dim", -1)) != runner.policy.expected_obs_dim:
            issues.append(
                (
                    "manifest_obs_dim",
                    f"manifest obs_dim {declared_model.get('obs_dim')} != policy {runner.policy.expected_obs_dim}",
                )
            )
        else:
            print("[Check] manifest obs_dim: PASS")

        if int(declared_model.get("action_dim", -1)) != runner.policy.expected_action_dim:
            issues.append(
                (
                    "manifest_action_dim",
                    f"manifest action_dim {declared_model.get('action_dim')} != policy {runner.policy.expected_action_dim}",
                )
            )
        else:
            print("[Check] manifest action_dim: PASS")

        declared_default = np.asarray(declared_action.get("default_dof_pos", []), dtype=np.float32)
        if declared_default.shape != runner.default_dof_pos.shape or not np.allclose(
            declared_default, runner.default_dof_pos
        ):
            issues.append(("manifest_default_pose", "manifest default_dof_pos mismatch"))
        else:
            print("[Check] manifest default_dof_pos: PASS")

        declared_scale = np.asarray(declared_action.get("scale", []), dtype=np.float32)
        if declared_scale.shape != runner.action_scale.shape or not np.allclose(
            declared_scale, runner.action_scale
        ):
            issues.append(("manifest_action_scale", "manifest action scale mismatch"))
        else:
            print("[Check] manifest action scale: PASS")

        if float(declared_safety.get("zero_cmd_lin_thresh", -1.0)) != runner.zero_cmd_lin_thresh:
            issues.append(("manifest_zero_cmd_lin_thresh", "manifest zero_cmd_lin_thresh mismatch"))
        if float(declared_safety.get("zero_cmd_yaw_thresh", -1.0)) != runner.zero_cmd_yaw_thresh:
            issues.append(("manifest_zero_cmd_yaw_thresh", "manifest zero_cmd_yaw_thresh mismatch"))
        if float(declared_safety.get("zero_yaw_rate_thresh", -1.0)) != runner.zero_yaw_rate_thresh:
            issues.append(("manifest_zero_yaw_rate_thresh", "manifest zero_yaw_rate_thresh mismatch"))
        if bool(declared_model.get("enable_zero_cmd_suppression", True)) != runner.enable_zero_cmd_suppression:
            issues.append(("manifest_zero_cmd_switch", "manifest zero-command suppression switch mismatch"))
        else:
            print("[Check] manifest zero-command suppression: PASS")

        if int(declared_control.get("control_freq_hz", -1)) != 50:
            issues.append(("manifest_control_freq", "manifest control_freq_hz must be 50"))
        else:
            print("[Check] manifest control freq: PASS")

    if issues:
        print("\n" + "=" * 60)
        print(f"Alignment check failed: {len(issues)} issue(s)")
        for tag, msg in issues:
            print(f"  [{tag}] {msg}")
        return 1

    print("\n" + "=" * 60)
    print("All offline alignment checks passed.")
    return 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--policy", type=str, required=True, help="Path to policy .pt")
    parser.add_argument("--manifest", type=str, default=None, help="Optional deployment manifest yaml")
    args = parser.parse_args()
    manifest = Path(args.manifest) if args.manifest else None
    sys.exit(check(Path(args.policy), manifest))


if __name__ == "__main__":
    main()
