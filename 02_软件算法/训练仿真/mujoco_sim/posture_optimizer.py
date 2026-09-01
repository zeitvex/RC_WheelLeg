"""Posture optimizer for wheeled-leg standing/crawl height table.

The table is not a pure "minimum average torque" table. For crawl and low-bar
traversal, the wheel center should not be far from the hip/leg in sagittal X,
otherwise the robot is no longer really using the wheel as the support/drive
point. This is a soft guardrail, not a strict x=0 constraint. The score combines:

1. wheel center X offset from hip
2. peak single-motor holding torque
3. RMS torque, used as a proxy for I^2R heating

Usage:
    python posture_optimizer.py              # MuJoCo sweep
    python posture_optimizer.py --quick      # coarse MuJoCo sweep
    python posture_optimizer.py --analyze    # analytical-only sweep
"""

import argparse
import math
import time
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]

try:
    import mujoco
except ImportError:
    mujoco = None

SCENE_XML = REPO_ROOT / "mjcf" / "scene.xml"
WHEEL_RADIUS = 0.10
HIP_Z_OFFSET = 0.054
L1, L2 = 0.25, 0.20
ROBOT_MASS = 12.3
G = 9.81
F_PER_LEG = ROBOT_MASS * G / 4.0
MAX_TORQUE = 17.0

LEG_NAMES = ("fl", "fr", "rl", "rr")
LEG_JOINTS = ("hip_abduction_joint", "hip_pitch_joint", "knee_joint")

# 0.15m is not a good default table target for ab=0.0. 0.17m is the practical
# default crawl height, while lower crawl can be evaluated with abduction.
KNEE_MIN = -2.65
HEIGHT_MIN = 0.17
HEIGHT_MAX = 0.46
SOFT_WHEEL_X_OFFSET = 0.05
HARD_WHEEL_X_OFFSET = 0.08

_ACTUATOR_NAMES = [f"{leg}_{jt}" for leg in LEG_NAMES for jt in LEG_JOINTS]


def compute_fk(hip, knee):
    """Return wheel-center x offset and base height from (hip_pitch, knee)."""
    x = L1 * math.sin(hip) + L2 * math.sin(hip + knee)
    z = L1 * math.cos(hip) + L2 * math.cos(hip + knee)
    base_height = WHEEL_RADIUS + z - HIP_Z_OFFSET
    return x, base_height


def posture_cost(x_foot, torques):
    """Score one posture by support geometry, peak torque, and RMS torque."""
    tau = np.asarray(torques, dtype=float)
    peak_torque = float(np.max(np.abs(tau)))
    rms_torque = float(np.sqrt(np.mean(np.square(tau))))
    mean_i2r = float(np.mean(np.square(tau)))
    x_penalty = max(0.0, abs(x_foot) - SOFT_WHEEL_X_OFFSET)
    cost = (
        0.5 * (abs(x_foot) / SOFT_WHEEL_X_OFFSET) ** 2
        + 8.0 * (x_penalty / max(1e-6, HARD_WHEEL_X_OFFSET - SOFT_WHEEL_X_OFFSET)) ** 2
        + 3.0 * (peak_torque / MAX_TORQUE) ** 2
        + (rms_torque / MAX_TORQUE) ** 2
    )
    return cost, peak_torque, rms_torque, mean_i2r


def analyze_analytical():
    """Analytical sweep using static GRF moments."""
    hip_range = np.arange(0.0, 1.6, 0.002)
    knee_range = np.arange(KNEE_MIN, -0.4, 0.002)

    results = []
    for hip in hip_range:
        for knee in knee_range:
            x_foot, height = compute_fk(hip, knee)
            if height < HEIGHT_MIN or height > HEIGHT_MAX:
                continue

            tau_hip = F_PER_LEG * x_foot
            x_knee_to_foot = L2 * math.sin(hip + knee)
            tau_knee = F_PER_LEG * x_knee_to_foot
            tau_abduction = 0.0
            cost, peak, rms, mean_i2r = posture_cost(
                x_foot, (tau_abduction, tau_hip, tau_knee))

            results.append({
                "hip": float(hip),
                "knee": float(knee),
                "height": float(height),
                "x_foot": float(x_foot),
                "tau_abd": tau_abduction,
                "tau_hip": float(tau_hip),
                "tau_knee": float(tau_knee),
                "tau_peak": peak,
                "tau_rms": rms,
                "mean_i2r": mean_i2r,
                "cost": cost,
            })

    return results


def run_mujoco_sweep(quick=False):
    """MuJoCo sweep measuring actual actuator forces at steady state."""
    if mujoco is None:
        raise RuntimeError("mujoco is not installed; use --analyze for analytical mode")

    model = mujoco.MjModel.from_xml_path(str(SCENE_XML))
    data = mujoco.MjData(model)

    act_ids = {}
    for name in _ACTUATOR_NAMES:
        act_ids[name] = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)

    for i in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        model.actuator_biastype[i] = 1
        model.actuator_gaintype[i] = 0
        model.actuator_forcelimited[i] = 0
        if "wheel" not in name:
            model.actuator_gainprm[i, 0] = 60.0
            model.actuator_biasprm[i, 0] = 0.0
            model.actuator_biasprm[i, 1] = -60.0
            model.actuator_biasprm[i, 2] = -3.0
            model.actuator_ctrlrange[i] = [-3.14, 3.14]
        else:
            model.actuator_gainprm[i, 0] = 2.0
            model.actuator_biasprm[i, 0] = 0.0
            model.actuator_biasprm[i, 1] = 0.0
            model.actuator_biasprm[i, 2] = -2.0
            model.actuator_ctrlrange[i] = [-20.0, 20.0]

    jnt_ids = {}
    for name in _ACTUATOR_NAMES:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        jnt_ids[name] = model.jnt_qposadr[jid]

    if quick:
        hip_range = np.arange(0.3, 1.5, 0.10)
        knee_range = np.arange(KNEE_MIN, -0.6, 0.10)
    else:
        hip_range = np.arange(0.0, 1.6, 0.04)
        knee_range = np.arange(KNEE_MIN, -0.4, 0.04)

    results = []
    total = 0
    valid = 0

    for hip in hip_range:
        for knee in knee_range:
            total += 1
            x_foot, height = compute_fk(hip, knee)
            if height < HEIGHT_MIN or height > HEIGHT_MAX:
                continue

            mujoco.mj_resetData(model, data)

            for leg in LEG_NAMES:
                for jt, val in zip(LEG_JOINTS, (0.0, hip, knee)):
                    name = f"{leg}_{jt}"
                    data.qpos[jnt_ids[name]] = val

            data.qpos[2] = height
            data.qpos[3] = 1.0
            data.qpos[4:7] = 0.0

            for leg in LEG_NAMES:
                for jt, val in zip(LEG_JOINTS, (0.0, hip, knee)):
                    name = f"{leg}_{jt}"
                    data.ctrl[act_ids[name]] = val
                name_w = f"{leg}_wheel_joint"
                if name_w in act_ids:
                    data.ctrl[act_ids[name_w]] = 0.0

            mujoco.mj_forward(model, data)

            for _ in range(500):
                mujoco.mj_step(model, data)

            roll, pitch, _ = _get_rpy(data, model)
            if abs(roll) > 0.8 or abs(pitch) > 0.8:
                continue

            torque_buf = []
            for _ in range(100):
                mujoco.mj_step(model, data)
                torque_buf.append([data.actuator_force[act_ids[name]]
                                   for name in _ACTUATOR_NAMES])
            tau_avg = np.array(torque_buf).mean(axis=0)

            tau_hip_val = tau_avg[1]
            tau_knee_val = tau_avg[2]
            tau_abd_val = tau_avg[0]
            cost, peak, rms, mean_i2r = posture_cost(x_foot, tau_avg)

            q_hip_actual = float(data.qpos[jnt_ids["fl_hip_pitch_joint"]])
            q_knee_actual = float(data.qpos[jnt_ids["fl_knee_joint"]])

            valid += 1
            results.append({
                "hip": float(hip),
                "knee": float(knee),
                "height": float(f"{height:.4f}"),
                "x_foot": float(f"{x_foot:.4f}"),
                "tau_abd": float(f"{tau_abd_val:.4f}"),
                "tau_hip": float(f"{tau_hip_val:.4f}"),
                "tau_knee": float(f"{tau_knee_val:.4f}"),
                "tau_peak": float(f"{peak:.4f}"),
                "tau_rms": float(f"{rms:.4f}"),
                "mean_i2r": float(f"{mean_i2r:.4f}"),
                "cost": float(f"{cost:.4f}"),
                "q_hip_actual": float(f"{q_hip_actual:.4f}"),
                "q_knee_actual": float(f"{q_knee_actual:.4f}"),
            })

            if valid % 20 == 0:
                print(f"  [{valid}/{total}] hip={hip:.2f} knee={knee:.2f} "
                      f"h={height:.3f} x={x_foot:+.4f} "
                      f"peak={peak:.2f} rms={rms:.2f} cost={cost:.4f}")

    return results


def _get_rpy(data, model):
    """Extract roll and pitch from MuJoCo data."""
    base_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    rot = data.xmat[base_bid].reshape(3, 3)
    roll = math.atan2(rot[2, 1], rot[2, 2])
    pitch = math.atan2(-rot[2, 0], math.sqrt(rot[2, 1] ** 2 + rot[2, 2] ** 2))
    return roll, pitch, 0.0


def print_top_results(results, n=10):
    """Print the top-N results with lowest cost."""
    sorted_r = sorted(results, key=lambda r: r["cost"])

    print(f"\n{'=' * 104}")
    print(f"TOP {n} CONFIGURATIONS (soft wheel-X + peak/RMS torque score)")
    print(f"{'=' * 104}")
    print(f"{'Rank':>4} {'hip':>6} {'knee':>7} {'height':>7} {'x_foot':>8} "
          f"{'tau_abd':>8} {'tau_hip':>8} {'tau_knee':>8} "
          f"{'peak':>8} {'rms':>8} {'cost':>9}")
    print(f"{'-' * 104}")

    for i, r in enumerate(sorted_r[:n]):
        print(f"{i + 1:>4} {r['hip']:>6.3f} {r['knee']:>7.3f} "
              f"{r['height']:>7.3f} {r.get('x_foot', 0):>8.4f} "
              f"{r.get('tau_abd', 0):>8.3f} {r['tau_hip']:>8.3f} "
              f"{r['tau_knee']:>8.3f} {r.get('tau_peak', 0):>8.3f} "
              f"{r.get('tau_rms', 0):>8.3f} {r['cost']:>9.4f}")

    best = sorted_r[0]
    print(f"\nBEST: hip={best['hip']:.3f} knee={best['knee']:.3f} "
          f"z={best['height']:.3f}m x={best.get('x_foot', 0):+.4f}m "
          f"peak={best.get('tau_peak', 0):.3f}Nm "
          f"rms={best.get('tau_rms', 0):.3f}Nm cost={best['cost']:.4f}\n")

    return sorted_r


def compute_height_table(results):
    """Build height-to-angle lookup with soft wheel-X support guardrail."""
    sorted_r = sorted(
        (r for r in results if abs(r.get("x_foot", 999.0)) <= HARD_WHEEL_X_OFFSET),
        key=lambda r: r["height"],
    )
    if not sorted_r:
        raise RuntimeError("No candidates satisfy HARD_WHEEL_X_OFFSET")

    h_range = np.arange(0.17, 0.46, 0.02)
    table_h, table_hip, table_knee = [], [], []

    for h_target in h_range:
        candidates = [(r, abs(r["height"] - h_target)) for r in sorted_r]
        candidates.sort(key=lambda x: (x[1], x[0]["cost"]))
        best = candidates[0][0]
        table_h.append(best["height"])
        table_hip.append(best["hip"])
        table_knee.append(best["knee"])

    return {
        "height": [round(h, 3) for h in table_h],
        "hip": [round(h, 3) for h in table_hip],
        "knee": [round(k, 3) for k in table_knee],
    }


def export_calibrated_table(table):
    """Print the new height table in copy-paste format."""
    print(f"\n{'=' * 80}")
    print("CALIBRATED HEIGHT TABLE (soft wheel-X support guardrail)")
    print(f"{'=' * 80}")
    print(f"_H = {table['height']}")
    print(f"_HIP = {table['hip']}")
    print(f"_KNEE = {table['knee']}")
    print(f"{'=' * 80}\n")


def main():
    parser = argparse.ArgumentParser(description="Find wheeled-leg posture table")
    parser.add_argument("--quick", action="store_true", help="Coarse MuJoCo sweep")
    parser.add_argument("--analyze", action="store_true", help="Analytical only")
    parser.add_argument("--mujoco", action="store_true", default=True,
                        help="Run MuJoCo simulation when available")
    args = parser.parse_args()

    print("=" * 72)
    print("WHEELED-LEG POSTURE OPTIMIZER")
    print("=" * 72)
    print(f"Robot mass: {ROBOT_MASS} kg, F_per_leg: {F_PER_LEG:.1f} N")
    print(f"L1={L1}m, L2={L2}m, wheel_r={WHEEL_RADIUS}m")
    print(f"Height range: [{HEIGHT_MIN}, {HEIGHT_MAX}] m")
    print(f"Knee min hard limit: {KNEE_MIN} rad")
    print(f"Soft wheel X offset: {SOFT_WHEEL_X_OFFSET} m")
    print(f"Hard wheel X offset: {HARD_WHEEL_X_OFFSET} m\n")

    t0 = time.time()
    if args.analyze or mujoco is None:
        print("[Analytical mode]")
        results = analyze_analytical()
    else:
        print("[MuJoCo simulation mode]")
        results = run_mujoco_sweep(quick=args.quick)

    elapsed = time.time() - t0
    print(f"Evaluated {len(results)} valid configurations in {elapsed:.1f}s")
    if not results:
        print("No valid configurations found")
        return

    best_results = print_top_results(results, n=15)

    x_def, h_def = compute_fk(0.666, -1.546)
    print(f"Current config default: hip=0.666, knee=-1.546 "
          f"=> z={h_def:.3f}m, x={x_def:+.4f}m")

    table = compute_height_table(best_results)
    export_calibrated_table(table)

    best = best_results[0]
    print("=" * 72)
    print("RECOMMENDED DEFAULT")
    print("=" * 72)
    print(f"hip_abduction: 0.0")
    print(f"hip_pitch:     {best['hip']:.3f}")
    print(f"knee:          {best['knee']:.3f}")
    print(f"height:        {best['height']:.3f} m")
    print(f"x_foot:        {best.get('x_foot', 0):+.4f} m")
    print(f"tau_peak:      {best.get('tau_peak', 0):.3f} Nm")
    print(f"tau_rms:       {best.get('tau_rms', 0):.3f} Nm")


if __name__ == "__main__":
    main()
