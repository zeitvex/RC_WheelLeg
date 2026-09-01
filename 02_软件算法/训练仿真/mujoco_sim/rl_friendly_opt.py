"""RlFriendlyPostureOpt — 结合电机发热 + RL友好度约束的站/爬姿优化

RL友好度约束（从实机经验总结，静力学可计算）：
  1. 髋关节力臂 ≥ 0.08m   — 不让髋闲置（动态响应差）
  2. 三电机不均衡 ≤ 1.3x   — 不让单电机先超载
  3. 运动学条件数 κ ≤ 3.5 — 不让有效传动比过高（放大控制噪声）

使用方法：
  uv run python mujoco_sim/rl_friendly_opt.py          # 打印推荐
  uv run python -c "from mujoco_sim.rl_friendly_opt import get_all; print(get_all(0, 0.8, -1.22))"

基于 mjcf/wheelleg.xml 的 FL 腿运动学。
"""
import math
import numpy as np

# ---------------------------------------------------------------------------
# 运动学常数（来自 wheelleg.xml FL 腿）
# ---------------------------------------------------------------------------
T_AB = [0.32826, 0.066172, 0.053981]
T_HIP = [0.06389, -0.027344, 0.00010727]
T_KNEE = [0.0, 0.1035, -0.25]
T_WHEEL = [0.0, 0.014699, -0.20011]
F_PER_LEG = 12.3 * 9.81 / 4.0
KNEE_MIN, KNEE_MAX = -2.65, 2.65
HIP_MIN, HIP_MAX = -2.58, 2.58

# ---------------------------------------------------------------------------
# 工具函数
# ---------------------------------------------------------------------------
def rx(a):
    c = math.cos(a); s = math.sin(a)
    return ((1, 0, 0), (0, c, -s), (0, s, c))

def ry(a):
    c = math.cos(a); s = math.sin(a)
    return ((c, 0, s), (0, 1, 0), (-s, 0, c))

def mv(m, v):
    return [m[0][0]*v[0] + m[0][1]*v[1] + m[0][2]*v[2],
            m[1][0]*v[0] + m[1][1]*v[1] + m[1][2]*v[2],
            m[2][0]*v[0] + m[2][1]*v[1] + m[2][2]*v[2]]

def add(a, b): return [a[0]+b[0], a[1]+b[1], a[2]+b[2]]

# ---------------------------------------------------------------------------
# 主函数
# ---------------------------------------------------------------------------
def get_all(q_ab, q_hip, q_knee):
    """FK + 力矩 + 运动学指标。

    返回 dict:
      z, ab, hip, knee, tau_ab, tau_hip, tau_knee,
      max_tau, i2r, imbal, cond, r_hip_x_mag, calf_deg
    """
    # --- FK ---
    p = mv(ry(q_knee), T_WHEEL)
    p = add(p, T_KNEE)
    p = mv(ry(q_hip), p)
    p = add(p, T_HIP)
    p = mv(rx(q_ab), p)
    wh = add(p, T_AB)
    base_z = 0.10 - wh[2]
    wheel_z = wh[2]

    # 膝位置（用于 wheel-below-knee）
    pk = mv(ry(q_hip), T_KNEE)
    pk = mv(rx(q_ab), pk)
    kz = pk[2] + T_AB[2]

    # 关节位置
    jk = mv(ry(q_hip), add(T_KNEE, T_HIP))
    jk = mv(rx(q_ab), jk)
    jk = add(T_AB, jk)
    jh = mv(rx(q_ab), T_HIP)
    jh = add(T_AB, jh)

    # 力矩
    rk = [wh[0]-jk[0], wh[1]-jk[1], wh[2]-jk[2]]
    rh = [wh[0]-jh[0], wh[1]-jh[1], wh[2]-jh[2]]
    ra = [wh[0]-T_AB[0], wh[1]-T_AB[1], wh[2]-T_AB[2]]
    tau_ab = ra[1] * F_PER_LEG
    tau_hip = -rh[0] * F_PER_LEG
    tau_knee = -rk[0] * F_PER_LEG

    # --- Jacobian（有限差分） ---
    eps = 1e-6
    def foot_rel_hip(h, k):
        fp = mv(ry(k), T_WHEEL)
        fp = add(fp, T_KNEE)
        fp = mv(ry(h), fp)
        return [fp[0] + T_AB[0] - jh[0], fp[2] + T_AB[2] - jh[2]]
    fp0 = foot_rel_hip(q_hip, q_knee)
    fph = foot_rel_hip(q_hip + eps, q_knee)
    fpk = foot_rel_hip(q_hip, q_knee + eps)
    J = np.array([
        [(fph[0]-fp0[0])/eps, (fpk[0]-fp0[0])/eps],
        [(fph[1]-fp0[1])/eps, (fpk[1]-fp0[1])/eps],
    ])
    s = np.linalg.svd(J, compute_uv=False)
    cond = s[0] / s[-1] if s[-1] > 1e-10 else 999.0
    min_sv = s[-1]

    # --- 小腿角度（相对铅垂线） ---
    calf_x = (-0.20011) * math.sin(q_knee)
    calf_z = (-0.20011) * math.cos(q_knee)
    cv_x = calf_x * math.cos(q_hip) + calf_z * math.sin(q_hip)
    cv_z = -calf_x * math.sin(q_hip) + calf_z * math.cos(q_hip)
    calf_deg = math.degrees(math.atan2(cv_x, -cv_z))

    abs_taus = [abs(tau_ab), abs(tau_hip), abs(tau_knee)]
    return {
        'z': base_z, 'ab': q_ab, 'hip': q_hip, 'knee': q_knee,
        'tau_ab': tau_ab, 'tau_hip': tau_hip, 'tau_knee': tau_knee,
        'max_tau': max(abs_taus),
        'i2r': tau_ab**2 + tau_hip**2 + tau_knee**2,
        'imbal': max(abs_taus) / max(1e-10, min(abs_taus)),
        'cond': cond, 'min_sv': min_sv,
        'r_hip_x_mag': abs(rh[0]),
        'calf_deg': calf_deg,
        'kz': kz, 'wz': wheel_z,
    }


def rl_cost(r):
    """RL友好度综合成本（越小越好）。

    约束来源：
      c1 — 瓶颈电机发热           τ²/τ_max²      主目标
      c2 — 电机不均衡 > 1.3x      (imbal-1.3)²    单电机先超载
      c3 — 髋力臂 < 8cm           (0.08 - r_hip)  髋闲置→动态响应差
      c4 — 有效传动比 κ > 3.5     (κ - 3.5)       高刚度→冲击传递大
      c5 — 腿的被动刚度 > 1.3x    (stiff-1.3)     刚度比→冲击吸收（新！）
    """
    c1 = (r['max_tau'] / 17.0) ** 2
    c2 = max(0.0, (r['imbal'] - 1.3) / 1.0) ** 2
    c3 = max(0.0, (0.08 - r['r_hip_x_mag'])) / 0.08
    c4 = max(0.0, (r['cond'] - 3.5)) / 5.0
    # stiffness ratio normalized to z=0.40 (σ_min≈0.115)
    stiff = (0.115 / r['min_sv']) ** 2
    c5 = max(0.0, (stiff - 1.3)) / 3.0
    return 100.0*c1 + 50.0*c2 + 80.0*c3 + 30.0*c4 + 40.0*c5


def sweep_z(z_targets, name, ab_max=0.44, tol=0.004):
    """遍历 z 扫描最优姿态。"""
    print(f"\n{'='*100}")
    print(f" {name}")
    print(f"{'='*100}")
    print(f"{'z_tgt':>6} {'z':>6} {'ab':>5} {'hip':>6} {'knee':>6}  | "
          f"{'maxτ':>6} {'imbal':>6} {'r_hip':>6} {'κ':>5}  | "
          f"{'c1热':>6} {'c2均':>6} {'c3髋':>6} {'c4奇':>6} {'cost':>7}")
    print("-"*100)
    results = []
    for zt in z_targets:
        best_cost = float('inf')
        best_r = None
        for ab in [round(i*0.02, 2) for i in range(int(ab_max/0.02)+1)]:
            for h in [round(i*0.01, 2) for i in range(260)]:
                for kn in [round(-2.65+i*0.01, 2) for i in range(256)]:
                    if h + kn > -0.1: continue
                    r = get_all(ab, h, kn)
                    if abs(r['z'] - zt) > tol: continue
                    if r['wz'] >= r['kz']: continue
                    if r['tau_ab'] < 0 or r['tau_hip'] < 0: continue
                    cost = rl_cost(r)
                    if cost < best_cost:
                        best_cost = cost; best_r = r
        if best_r:
            r = best_r
            c1s = 100.0*(r['max_tau']/17.0)**2
            c2s = 50.0*max(0.0,(r['imbal']-1.3))**2
            c3s = 80.0*max(0.0,(0.08-r['r_hip_x_mag']))/0.08
            c4s = 30.0*max(0.0,(r['cond']-3.5))/5.0
            print(f"{zt:>6.2f} {r['z']:>6.3f} {r['ab']:>5.2f} {r['hip']:>6.2f} {r['knee']:>6.2f}  | "
                  f"{r['max_tau']:>6.3f} {r['imbal']:>6.1f}x {r['r_hip_x_mag']:>6.3f} {r['cond']:>5.1f}  | "
                  f"{c1s:>6.1f} {c2s:>6.1f} {c3s:>6.1f} {c4s:>6.1f} {best_cost:>7.1f}")
            results.append((zt, r))
        else:
            print(f"{zt:>6.2f}  —  no valid")
    return results


if __name__ == "__main__":
    stand = sweep_z([round(0.36+0.01*i, 2) for i in range(10)],
                    "STANDING — heat + RL constraints (ab free)", 0.44)
    crawl = sweep_z([round(0.10+0.01*i, 2) for i in range(9)],
                    "CRAWL — heat + RL constraints (ab_max=0.25)", 0.25)

    print(f"\n{'='*100}")
    print(" RECOMMENDATION")
    print(f"{'='*100}")
    if stand:
        zt, r = min(stand, key=lambda x: rl_cost(x[1]))
        print(f"  Standing: z={r['z']:.3f}  ab={r['ab']:.2f}  hip={r['hip']:.2f}  knee={r['knee']:.2f}")
        print(f"    maxτ={r['max_tau']:.3f}  imbal={r['imbal']:.1f}x  κ={r['cond']:.1f}  r_hip={r['r_hip_x_mag']:.3f}m")
    if crawl:
        zt, r = min(crawl, key=lambda x: rl_cost(x[1]))
        print(f"  Crawl:    z={r['z']:.3f}  ab={r['ab']:.2f}  hip={r['hip']:.2f}  knee={r['knee']:.2f}")
        print(f"    maxτ={r['max_tau']:.3f}  imbal={r['imbal']:.1f}x  κ={r['cond']:.1f}  r_hip={r['r_hip_x_mag']:.3f}m")
