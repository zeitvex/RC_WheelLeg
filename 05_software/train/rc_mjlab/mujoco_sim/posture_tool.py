"""Fast posture table helper based on wheelleg.xml link offsets.

This tool is useful because it uses the real FL leg offsets from MJCF instead
of the simplified two-link geometry used by posture_optimizer.py. It is still a
static single-leg approximation, so use it to choose candidate crawl/standing
poses, then verify in MuJoCo and on the robot at low speed.
"""

import math

F_PER_LEG = 12.3 * 9.81 / 4.0
KNEE_MIN = -2.65
HIP_MIN = -2.58
HIP_MAX = 2.58
AB_MIN = -0.436
AB_MAX = 0.611
HARD_WHEEL_X_OFFSET = 0.08


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return ((1, 0, 0), (0, c, -s), (0, s, c))


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return ((c, 0, s), (0, 1, 0), (-s, 0, c))


def mv(m, v):
    return [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    ]


def add(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def get_posture(q_ab, q_hip, q_knee):
    """Return static FK/torque metrics for one FL leg.

    Returns:
        base_z, tau_ab, tau_hip, tau_knee, i2r_total, knee_z, wheel_z, wheel_x_from_hip
    """
    # FL offsets from mjcf/wheelleg.xml.
    t_ab = [0.32826, 0.066172, 0.053981]
    t_hip = [0.06389, -0.027344, 0.00010727]
    t_knee = [0.0, 0.1035, -0.25]
    t_wheel = [0.0, 0.014699, -0.20011]

    p = mv(ry(q_knee), t_wheel)
    p = add(p, t_knee)
    p = mv(ry(q_hip), p)
    p = add(p, t_hip)
    p = mv(rx(q_ab), p)
    p = add(p, t_ab)

    base_z = 0.10 - p[2]
    wheel_z = p[2]

    knee_pos = mv(ry(q_hip), t_knee)
    knee_pos = mv(rx(q_ab), knee_pos)
    knee_z = knee_pos[2] + t_ab[2]

    joint_knee = mv(ry(q_hip), add(t_knee, t_hip))
    joint_knee = mv(rx(q_ab), joint_knee)
    joint_knee = add(t_ab, joint_knee)

    joint_hip = mv(rx(q_ab), t_hip)
    joint_hip = add(t_ab, joint_hip)

    r_knee = [p[0] - joint_knee[0], p[1] - joint_knee[1], p[2] - joint_knee[2]]
    r_hip = [p[0] - joint_hip[0], p[1] - joint_hip[1], p[2] - joint_hip[2]]
    r_ab = [p[0] - t_ab[0], p[1] - t_ab[1], p[2] - t_ab[2]]

    tau_ab = r_ab[1] * F_PER_LEG
    tau_hip = -r_hip[0] * F_PER_LEG
    tau_knee = -r_knee[0] * F_PER_LEG
    i2r_total = tau_ab * tau_ab + tau_hip * tau_hip + tau_knee * tau_knee
    wheel_x_from_hip = p[0] - joint_hip[0]
    return base_z, tau_ab, tau_hip, tau_knee, i2r_total, knee_z, wheel_z, wheel_x_from_hip


def score_candidate(z, z_target, tau_ab, tau_hip, tau_knee, wheel_x, x_target=0.0):
    peak = max(abs(tau_ab), abs(tau_hip), abs(tau_knee))
    rms = math.sqrt((tau_ab * tau_ab + tau_hip * tau_hip + tau_knee * tau_knee) / 3.0)
    x_err = wheel_x - x_target
    x_over = max(0.0, abs(wheel_x) - 0.05)
    return (
        3000.0 * (z - z_target) ** 2
        + 2.5 * (peak / 17.0) ** 2
        + (rms / 17.0) ** 2
        + 0.4 * (x_err / 0.05) ** 2
        + 6.0 * (x_over / 0.03) ** 2
    ), peak, rms


def find_best(z_target, ab_range=(0.0, 0.0), step=0.002, x_target=0.0, hard_wheel_x_offset=HARD_WHEEL_X_OFFSET):
    """Find one static posture near target height without violating hard limits."""
    best = None
    ab0, ab1 = ab_range
    n_ab = max(1, int(round((ab1 - ab0) / step)) + 1)
    n_hip = int(round((1.6 - 0.0) / step)) + 1
    n_knee = int(round((-0.4 - KNEE_MIN) / step)) + 1

    for ia in range(n_ab):
        ab = ab0 + ia * step
        if ab < AB_MIN or ab > AB_MAX:
            continue
        for ih in range(n_hip):
            hip = ih * step
            if hip < HIP_MIN or hip > HIP_MAX:
                continue
            for ik in range(n_knee):
                knee = KNEE_MIN + ik * step
                z, ta, th, tk, i2r, kz, wz, wx = get_posture(ab, hip, knee)
                if abs(z - z_target) > 0.0015:
                    continue
                if abs(wx) > hard_wheel_x_offset:
                    continue
                if wz >= kz:
                    continue
                cost, peak, rms = score_candidate(z, z_target, ta, th, tk, wx, x_target=x_target)
                cand = (cost, ab, hip, knee, z, ta, th, tk, peak, rms, i2r, wx)
                if best is None or cand[0] < best[0]:
                    best = cand
    return best


def print_table(
    z_targets,
    name,
    ab_range=(0.0, 0.0),
    step=0.002,
    x_target=0.0,
    hard_wheel_x_offset=HARD_WHEEL_X_OFFSET,
):
    print(f"\n{'=' * 96}")
    print(name)
    print(f"{'=' * 96}")
    print(f"{'z':>6} {'ab':>6} {'hip':>7} {'knee':>7} "
          f"{'tau_ab':>8} {'tau_hip':>8} {'tau_knee':>9} "
          f"{'peak':>8} {'rms':>8} {'x_hip':>8}")
    print("-" * 96)
    for zt in z_targets:
        best = find_best(
            zt,
            ab_range=ab_range,
            step=step,
            x_target=x_target,
            hard_wheel_x_offset=hard_wheel_x_offset,
        )
        if best is None:
            print(f"{zt:>6.3f} no valid config")
            continue
        _, ab, hip, knee, z, ta, th, tk, peak, rms, _, wx = best
        print(f"{z:>6.3f} {ab:>6.3f} {hip:>7.3f} {knee:>7.3f} "
              f"{ta:>8.3f} {th:>8.3f} {tk:>9.3f} "
              f"{peak:>8.3f} {rms:>8.3f} {wx:>8.4f}")


def print_crawl_default():
    best = find_best(0.17, ab_range=(0.0, 0.0), step=0.002)
    if best is None:
        return
    _, ab, hip, knee, z, ta, th, tk, peak, rms, _, wx = best
    print("\nSuggested runtime crawl_default_dof_pos:")
    print(
        f"[{ab:.3f}, {hip:.3f}, {knee:.3f}, "
        f"{ab:.3f}, {hip:.3f}, {knee:.3f}, "
        f"{ab:.3f}, {hip:.3f}, {knee:.3f}, "
        f"{ab:.3f}, {hip:.3f}, {knee:.3f}, "
        "0.0, 0.0, 0.0, 0.0]"
    )
    print(f"# z={z:.3f}, peak={peak:.3f}Nm, rms={rms:.3f}Nm, wheel_x_from_hip={wx:+.4f}m")


if __name__ == "__main__":
    print_table([round(0.36 + 0.01 * i, 2) for i in range(10)],
                "STANDING candidates from MJCF FL geometry", step=0.004)
    print_table([round(0.17 + 0.01 * i, 2) for i in range(9)],
                "CRAWL candidates from MJCF FL geometry", step=0.002)
    print_table([round(0.10 + 0.01 * i, 2) for i in range(6)],
                "LOW CRAWL candidates, fixed FL abduction = +0.2",
                ab_range=(0.2, 0.2), step=0.002, hard_wheel_x_offset=0.45)
    print_crawl_default()
