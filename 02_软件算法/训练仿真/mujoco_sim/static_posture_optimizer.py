
































"""MuJoCo static posture optimizer for wheeled-leg standing defaults.

The old posture tools are mostly analytical. This script keeps the fast MJCF
kinematics as a candidate generator, then evaluates the best candidates in
MuJoCo with gravity and floor contact enabled.

The score is meant for RL default pose / real deployment:
  - low peak and RMS standing torque, so one hot motor is not hidden by average
  - wheel contact point close to the hip in X for wheel speed tracking
  - COM projection margin inside the four-wheel support rectangle
  - non-singular leg Jacobian for posture control authority
  - underbody and knee clearance for obstacle tolerance

Usage:
    uv run python mujoco_sim/static_posture_optimizer.py
    uv run python mujoco_sim/static_posture_optimizer.py --quick
    uv run python mujoco_sim/static_posture_optimizer.py --ab-max 0.08
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from mujoco_sim.rl_friendly_opt import get_all  # noqa: E402

SCENE_XML = REPO_ROOT / "mjcf" / "scene.xml"

LEG_NAMES = ("fl", "fr", "rl", "rr")
LEG_JOINTS = ("hip_abduction_joint", "hip_pitch_joint", "knee_joint")
WHEEL_JOINT = "wheel_joint"
LEG_ACTUATORS = [f"{leg}_{jt}" for leg in LEG_NAMES for jt in LEG_JOINTS]

ROBOT_MASS = 12.3
G = 9.81
MAX_TORQUE = 17.0
WHEEL_RADIUS = 0.10

HIP_MIN, HIP_MAX = -2.58, 2.58
KNEE_MIN, KNEE_MAX = -2.65, 2.65
HIP_SCAN = (0.20, 1.15)
KNEE_SCAN = (-1.90, -0.65)

SOFT_WHEEL_X = 0.045
HARD_WHEEL_X = 0.085
MIN_COM_MARGIN = 0.045
MIN_KNEE_CLEARANCE = 0.105
MIN_UNDERBODY_CLEARANCE = 0.33
MAX_COND = 3.7


@dataclass(frozen=True)
class Candidate:
    ab: float
    hip: float
    knee: float
    z_fk: float
    analytic_cost: float


@dataclass
class StaticResult:
    z_target: float
    z: float
    ab: float
    hip: float
    knee: float
    cost: float
    peak_tau: float
    rms_tau: float
    mean_i2r: float
    imbal: float
    wheel_x: float
    cond: float
    min_sv: float
    com_margin_x: float
    com_margin_y: float
    support_margin: float
    normal_cv: float
    body_clearance: float
    knee_clearance: float
    roll: float
    pitch: float
    height_err: float


def _configure_actuators(model: mujoco.MjModel) -> None:
    """Configure leg joints as position PD and wheels as zero-velocity motors."""
    for i in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) or ""
        model.actuator_biastype[i] = 1
        model.actuator_gaintype[i] = 0
        model.actuator_forcelimited[i] = 0
        if "wheel" in name:
            model.actuator_gainprm[i, 0] = 2.0
            model.actuator_biasprm[i, 0] = 0.0
            model.actuator_biasprm[i, 1] = 0.0
            model.actuator_biasprm[i, 2] = -2.0
            model.actuator_ctrlrange[i] = [-20.0, 20.0]
        else:
            model.actuator_gainprm[i, 0] = 60.0
            model.actuator_biasprm[i, 0] = 0.0
            model.actuator_biasprm[i, 1] = -60.0
            model.actuator_biasprm[i, 2] = -3.0
            model.actuator_ctrlrange[i] = [-3.14, 3.14]


def _ids(model: mujoco.MjModel):
    act = {}
    qadr = {}
    for leg in LEG_NAMES:
        for jt in (*LEG_JOINTS, WHEEL_JOINT):
            name = f"{leg}_{jt}"
            act[name] = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            qadr[name] = model.jnt_qposadr[jid]
    bodies = {
        "base": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link"),
        **{
            f"{leg}_wheel": mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_BODY, f"{leg}_wheel_Link"
            )
            for leg in LEG_NAMES
        },
        **{
            f"{leg}_knee": mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_BODY, f"{leg}_knee_Link"
            )
            for leg in LEG_NAMES
        },
    }
    return act, qadr, bodies


def _mirrored_ab(leg: str, ab: float) -> float:
    return ab if leg[1] == "l" else -ab


def _set_pose(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    act: dict[str, int],
    qadr: dict[str, int],
    cand: Candidate,
) -> None:
    mujoco.mj_resetData(model, data)
    for leg in LEG_NAMES:
        vals = (_mirrored_ab(leg, cand.ab), cand.hip, cand.knee)
        for jt, val in zip(LEG_JOINTS, vals):
            name = f"{leg}_{jt}"
            data.qpos[qadr[name]] = val
            data.ctrl[act[name]] = val
        data.ctrl[act[f"{leg}_{WHEEL_JOINT}"]] = 0.0
    data.qpos[0:3] = [0.0, 0.0, cand.z_fk]
    data.qpos[3] = 1.0
    data.qpos[4:7] = 0.0
    mujoco.mj_forward(model, data)


def _rpy(data: mujoco.MjData, bodies: dict[str, int]) -> tuple[float, float]:
    rot = data.xmat[bodies["base"]].reshape(3, 3)
    roll = math.atan2(rot[2, 1], rot[2, 2])
    pitch = math.atan2(-rot[2, 0], math.sqrt(rot[2, 1] ** 2 + rot[2, 2] ** 2))
    return roll, pitch


def _robot_com(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    masses = model.body_mass[1:]
    return (data.xipos[1:] * masses[:, None]).sum(axis=0) / masses.sum()


def _support_metrics(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    bodies: dict[str, int],
) -> tuple[float, float, float, float]:
    wheel_xy = np.array([data.xpos[bodies[f"{leg}_wheel"]][:2] for leg in LEG_NAMES])
    com_xy = _robot_com(model, data)[:2]
    min_xy = wheel_xy.min(axis=0)
    max_xy = wheel_xy.max(axis=0)
    margin_low = com_xy - min_xy
    margin_high = max_xy - com_xy
    margin_x = float(min(margin_low[0], margin_high[0]))
    margin_y = float(min(margin_low[1], margin_high[1]))
    support_margin = float(min(margin_x, margin_y))

    normal = []
    for leg in LEG_NAMES:
        bid = bodies[f"{leg}_wheel"]
        fz = 0.0
        for i in range(data.ncon):
            con = data.contact[i]
            b1 = model.geom_bodyid[con.geom1]
            b2 = model.geom_bodyid[con.geom2]
            if b1 == bid or b2 == bid:
                wrench = np.zeros(6)
                mujoco.mj_contactForce(model, data, i, wrench)
                fz += abs(float(wrench[0]))
        normal.append(fz)
    normal = np.asarray(normal, dtype=float)
    if normal.sum() < 1e-6:
        normal_cv = 9.99
    else:
        normal_cv = float(normal.std() / max(1e-6, normal.mean()))
    return margin_x, margin_y, support_margin, normal_cv


def _clearance_metrics(data: mujoco.MjData, bodies: dict[str, int]) -> tuple[float, float]:
    base_z = float(data.xpos[bodies["base"]][2])
    # The collision box in wheelleg.xml is centered at z=0.054 with half-height 0.073.
    body_clearance = base_z + 0.054 - 0.073
    knee_z = min(float(data.xpos[bodies[f"{leg}_knee"]][2]) for leg in LEG_NAMES)
    wheel_z = min(float(data.xpos[bodies[f"{leg}_wheel"]][2]) for leg in LEG_NAMES)
    return body_clearance, knee_z - wheel_z


def _static_cost(r: StaticResult) -> float:
    wheel_over = max(0.0, abs(r.wheel_x) - SOFT_WHEEL_X)
    support_short = max(0.0, MIN_COM_MARGIN - r.support_margin)
    body_short = max(0.0, MIN_UNDERBODY_CLEARANCE - r.body_clearance)
    knee_short = max(0.0, MIN_KNEE_CLEARANCE - r.knee_clearance)
    cond_over = max(0.0, r.cond - MAX_COND)
    tilt = math.hypot(r.roll, r.pitch)

    return (
        6500.0 * r.height_err**2
        + 2.6 * (r.peak_tau / MAX_TORQUE) ** 2
        + 1.1 * (r.rms_tau / MAX_TORQUE) ** 2
        + 1.8 * max(0.0, r.imbal - 1.6) ** 2
        + 0.65 * (abs(r.wheel_x) / SOFT_WHEEL_X) ** 2
        + 9.0 * (wheel_over / max(1e-6, HARD_WHEEL_X - SOFT_WHEEL_X)) ** 2
        + 10.0 * (support_short / MIN_COM_MARGIN) ** 2
        + 2.0 * (r.normal_cv / 0.35) ** 2
        + 30.0 * (cond_over / 1.0) ** 2
        + 4.0 * (body_short / 0.06) ** 2
        + 2.0 * (knee_short / 0.04) ** 2
        + 1.0 * (tilt / 0.05) ** 2
    )


def _analytic_cost(r: dict, z_target: float) -> float:
    wheel_over = max(0.0, abs(r["r_hip_x_mag"]) - SOFT_WHEEL_X)
    cond_over = max(0.0, r["cond"] - MAX_COND)
    return (
        1400.0 * (r["z"] - z_target) ** 2
        + 2.2 * (r["max_tau"] / MAX_TORQUE) ** 2
        + 0.7 * (math.sqrt(r["i2r"] / 3.0) / MAX_TORQUE) ** 2
        + 0.55 * (abs(r["r_hip_x_mag"]) / SOFT_WHEEL_X) ** 2
        + 8.0 * (wheel_over / max(1e-6, HARD_WHEEL_X - SOFT_WHEEL_X)) ** 2
        + 2.0 * (cond_over / 2.0) ** 2
    )


def generate_candidates(
    z_target: float,
    step: float,
    ab_max: float,
    keep: int,
    z_tol: float,
) -> list[Candidate]:
    candidates: list[Candidate] = []
    ab_values = np.arange(0.0, ab_max + 0.5 * step, step)
    hip_values = np.arange(HIP_SCAN[0], HIP_SCAN[1] + 0.5 * step, step)
    knee_values = np.arange(KNEE_SCAN[0], KNEE_SCAN[1] + 0.5 * step, step)
    for ab in ab_values:
        for hip in hip_values:
            if hip < HIP_MIN or hip > HIP_MAX:
                continue
            for knee in knee_values:
                if knee < KNEE_MIN or knee > KNEE_MAX:
                    continue
                r = get_all(float(ab), float(hip), float(knee))
                if abs(r["z"] - z_target) > z_tol:
                    continue
                if r["wz"] >= r["kz"]:
                    continue
                if abs(r["r_hip_x_mag"]) > HARD_WHEEL_X:
                    continue
                if r["max_tau"] > MAX_TORQUE * 1.15:
                    continue
                candidates.append(
                    Candidate(
                        ab=float(ab),
                        hip=float(hip),
                        knee=float(knee),
                        z_fk=float(r["z"]),
                        analytic_cost=_analytic_cost(r, z_target),
                    )
                )
    candidates.sort(key=lambda c: c.analytic_cost)
    return candidates[:keep]


def evaluate_candidate(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    act: dict[str, int],
    qadr: dict[str, int],
    bodies: dict[str, int],
    cand: Candidate,
    z_target: float,
    settle_steps: int,
    avg_steps: int,
) -> StaticResult | None:
    _set_pose(model, data, act, qadr, cand)
    for _ in range(settle_steps):
        mujoco.mj_step(model, data)

    roll, pitch = _rpy(data, bodies)
    if abs(roll) > 0.35 or abs(pitch) > 0.35:
        return None

    tau_buf = []
    for _ in range(avg_steps):
        mujoco.mj_step(model, data)
        tau_buf.append([data.actuator_force[act[name]] for name in LEG_ACTUATORS])
    tau = np.asarray(tau_buf, dtype=float).mean(axis=0)
    tau_abs = np.abs(tau)
    peak = float(tau_abs.max())
    rms = float(np.sqrt(np.mean(tau * tau)))
    mean_i2r = float(np.mean(tau * tau))

    sagittal_abs = []
    for name, value in zip(LEG_ACTUATORS, tau):
        if "hip_pitch" in name or "knee" in name:
            sagittal_abs.append(abs(float(value)))
    sagittal_abs = np.asarray(sagittal_abs, dtype=float)
    imbal = float(sagittal_abs.max() / max(1e-6, sagittal_abs.mean()))

    fk = get_all(cand.ab, cand.hip, cand.knee)
    margin_x, margin_y, support_margin, normal_cv = _support_metrics(model, data, bodies)
    body_clearance, knee_clearance = _clearance_metrics(data, bodies)
    z = float(data.xpos[bodies["base"]][2])

    result = StaticResult(
        z_target=z_target,
        z=z,
        ab=cand.ab,
        hip=cand.hip,
        knee=cand.knee,
        cost=0.0,
        peak_tau=peak,
        rms_tau=rms,
        mean_i2r=mean_i2r,
        imbal=imbal,
        wheel_x=float(fk["r_hip_x_mag"]),
        cond=float(fk["cond"]),
        min_sv=float(fk["min_sv"]),
        com_margin_x=margin_x,
        com_margin_y=margin_y,
        support_margin=support_margin,
        normal_cv=normal_cv,
        body_clearance=body_clearance,
        knee_clearance=knee_clearance,
        roll=roll,
        pitch=pitch,
        height_err=z - z_target,
    )
    result.cost = _static_cost(result)
    return result


def optimize(
    z_targets: list[float],
    step: float,
    ab_max: float,
    keep: int,
    z_tol: float,
    settle_steps: int,
    avg_steps: int,
) -> list[StaticResult]:
    model = mujoco.MjModel.from_xml_path(str(SCENE_XML))
    data = mujoco.MjData(model)
    _configure_actuators(model)
    act, qadr, bodies = _ids(model)

    results: list[StaticResult] = []
    for zt in z_targets:
        candidates = generate_candidates(zt, step=step, ab_max=ab_max, keep=keep, z_tol=z_tol)
        best: StaticResult | None = None
        for cand in candidates:
            result = evaluate_candidate(
                model,
                data,
                act,
                qadr,
                bodies,
                cand,
                zt,
                settle_steps=settle_steps,
                avg_steps=avg_steps,
            )
            if result is None:
                continue
            if best is None or result.cost < best.cost:
                best = result
        if best is None:
            print(f"{zt:.2f}: no valid MuJoCo-static candidate from {len(candidates)} seeds")
            continue
        results.append(best)
        print_result(best)
    return results


def print_header() -> None:
    print("\nMuJoCo static posture optimization")
    print("z_tgt      z    ab    hip   knee | peak   rms  imbal wheelX  cond | comX  comY  clrB  clrK | cost")
    print("-" * 111)


def print_result(r: StaticResult) -> None:
    print(
        f"{r.z_target:5.2f} {r.z:6.3f} {r.ab:5.2f} {r.hip:6.3f} {r.knee:6.3f} | "
        f"{r.peak_tau:5.2f} {r.rms_tau:5.2f} {r.imbal:6.2f} "
        f"{r.wheel_x:6.3f} {r.cond:5.2f} | "
        f"{r.com_margin_x:5.3f} {r.com_margin_y:5.3f} "
        f"{r.body_clearance:5.3f} {r.knee_clearance:5.3f} | "
        f"{r.cost:6.2f}"
    )


def print_tables(results: list[StaticResult]) -> None:
    if not results:
        return
    print("\nCopy-paste tables:")
    print("_H_TARGET = " + repr([round(r.z_target, 3) for r in results]))
    print("_Z_STATIC = " + repr([round(r.z, 3) for r in results]))
    print("_HIP = " + repr([round(r.hip, 3) for r in results]))
    print("_KNEE = " + repr([round(r.knee, 3) for r in results]))
    print("_ABD_LEFT = " + repr([round(r.ab, 3) for r in results]))
    print("_ABD_RIGHT = " + repr([round(-r.ab, 3) for r in results]))

    best = min(results, key=lambda r: r.cost)
    print("\nRecommended default:")
    print(
        f"z={best.z_target:.2f}, ab={best.ab:.3f}, hip={best.hip:.3f}, "
        f"knee={best.knee:.3f}, peak={best.peak_tau:.2f}Nm, "
        f"rms={best.rms_tau:.2f}Nm, support_margin={best.support_margin:.3f}m"
    )
    pose = []
    for leg in LEG_NAMES:
        pose.extend([_mirrored_ab(leg, best.ab), best.hip, best.knee])
    pose.extend([0.0, 0.0, 0.0, 0.0])
    print("default_dof_pos = " + repr([round(v, 3) for v in pose]))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--z-min", type=float, default=0.36)
    parser.add_argument("--z-max", type=float, default=0.45)
    parser.add_argument("--z-step", type=float, default=0.01)
    parser.add_argument("--grid-step", type=float, default=0.01)
    parser.add_argument("--ab-max", type=float, default=0.0)
    parser.add_argument("--keep", type=int, default=45)
    parser.add_argument("--z-tol", type=float, default=0.035)
    parser.add_argument("--settle-steps", type=int, default=350)
    parser.add_argument("--avg-steps", type=int, default=80)
    parser.add_argument("--quick", action="store_true", help="Coarser and faster scan")
    args = parser.parse_args()

    if args.quick:
        args.grid_step = max(args.grid_step, 0.02)
        args.keep = min(args.keep, 20)
        args.z_tol = max(args.z_tol, 0.040)
        args.settle_steps = min(args.settle_steps, 220)
        args.avg_steps = min(args.avg_steps, 40)

    n = int(round((args.z_max - args.z_min) / args.z_step)) + 1
    z_targets = [round(args.z_min + i * args.z_step, 3) for i in range(n)]

    print_header()
    results = optimize(
        z_targets,
        step=args.grid_step,
        ab_max=args.ab_max,
        keep=args.keep,
        z_tol=args.z_tol,
        settle_steps=args.settle_steps,
        avg_steps=args.avg_steps,
    )
    print_tables(results)


if __name__ == "__main__":
    main()
