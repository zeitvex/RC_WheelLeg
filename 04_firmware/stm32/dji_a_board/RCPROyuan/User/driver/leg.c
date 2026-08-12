/**
 * @file  leg.c
 * @brief 腿部运动学 — 基于硬件实测标定参数
 *
 * 物理模型:
 *   二连杆串联 + 同步带1:1 + 40度足端偏置
 *
 *   正运动学 (电机角 → 链节角 → 足端):
 *     θ_hip        = -(motor_thigh + thigh_zero)
 *     θ_shank_link = -(motor_shank + motor_thigh) + shank_coupling_c
 *     φ            = θ_shank_link - foot_offset  (等效足端方向)
 *     x = L1*sin(θ_hip) + L2*sin(φ)
 *     z = L1*cos(θ_hip) + L2*cos(φ)
 *
 *   逆运动学 (足端 → 电机角):
 *     标准二连杆IK求 θ_hip 和 φ
 *     θ_shank_link = φ + foot_offset
 *     motor_thigh = -(θ_hip + thigh_zero)
 *     motor_shank = -motor_thigh - θ_shank_link + shank_coupling_c
 *
 *   IK 选择"后倾解" (θ_hip = ψ - β):
 *     大腿略向后倾, 膝关节在前方弯曲
 *     这是该四足的自然站立构型
 */

#include "leg.h"
#include <math.h>

/* ============================================================
 *  逆运动学 — 直接输出无符号电机角
 *
 *  X: 足端前向偏移 (mm), 前向为正
 *  Z: 足端向下距离 (mm), 向下为正
 *
 *  使用"后倾解": θ_hip = ψ - β
 *  验证: 趴下(motors≈0), 垂直(thigh=-1.0, shank=2.76)
 * ============================================================ */
void Inverse_Calculation(float X, float Z,
                         float *motor_thigh, float *motor_shank,
                         const RobotGeometry *geom)
{
    float L1   = geom->L1;
    float L2   = geom->L2;
    float foff = geom->foot_offset_rad;
    float tzero = geom->thigh_zero;
    float sc = geom->shank_coupling_c;
    float splay = geom->splay_angle_rad;

    /* 补偿15度外八倾角: 实际腿在倾斜平面内, 达到垂直高度Z需要的计算长度会变长 */
    float Z_leg = Z / cosf(splay);

    float L = sqrtf(X * X + Z_leg * Z_leg);

    /* 防止超出工作空间 */
    float L_max = L1 + L2 - 1.0f;
    float L_min = fabsf(L1 - L2) + 1.0f;
    if (L > L_max) L = L_max;
    if (L < L_min) L = L_min;

    /* --- 标准二连杆 IK --- */

    /* 膝关节内角 (L1和L2之间的三角形内角) */
    float cos_knee_int = (L1 * L1 + L2 * L2 - L * L) / (2.0f * L1 * L2);
    if (cos_knee_int >  1.0f) cos_knee_int =  1.0f;
    if (cos_knee_int < -1.0f) cos_knee_int = -1.0f;

    /* 髋关节处三角形内角 */
    float cos_hip_int = (L * L + L1 * L1 - L2 * L2) / (2.0f * L * L1);
    if (cos_hip_int >  1.0f) cos_hip_int =  1.0f;
    if (cos_hip_int < -1.0f) cos_hip_int = -1.0f;
    float beta = acosf(cos_hip_int);

    /* 髋到足连线与竖直方向的夹角 */
    float psi = atan2f(X, Z_leg);

    /* "后倾解": θ_hip = ψ - β
     * 大腿向后倾斜, 膝关节在前方 — 匹配该四足的自然构型 */
    float theta_hip = psi - beta;

    /* 等效足端方向角 φ (第二连杆在IK空间中的绝对角) */
    float sin_phi = (X - L1 * sinf(theta_hip)) / L2;
    float cos_phi = (Z_leg - L1 * cosf(theta_hip)) / L2;
    float phi = atan2f(sin_phi, cos_phi);

    /* 小腿链节角 = φ + 足端偏置 */
    float theta_shank_link = phi + foff;

    /* --- 转换为电机角 --- 
     * 恢复物理耦合: 之前因为趴下高度的问题误以为解耦, 但实际上用户的 PRONE_Z 是一个较小的非完全趴下的高度
     * 电机实际上是带有跟随耦合的机制, 小腿绝对角度受大腿牵连!
     * θ_shank_link = -(ms + mt) + sc
     */
    float mt = -(theta_hip + tzero);
    float ms = -mt - theta_shank_link + sc;

    *motor_thigh = mt;
    *motor_shank = ms;
}

/* ============================================================
 *  摆线轨迹生成
 * ============================================================ */
void Gen_Cycloid_Trajectory(float phase, float swing_ratio,
                             float Xs, float Xe, float Zs, float h,
                             float *x, float *z)
{
    phase = fmodf(phase, 1.0f);
    if (phase < 0.0f) phase += 1.0f;

    if (phase < swing_ratio)
    {
        float t = phase / swing_ratio;
        /* 标准摆线: 起步和落地时速度和加速度均为0，减小冲击 */
        *x = Xs + (Xe - Xs) * (t - 1.0f / (2.0f * PI) * sinf(2.0f * PI * t));
        *z = Zs - h * (1.0f - cosf(2.0f * PI * t)) / 2.0f;
    }
    else
    {
        /* 支撑相，地面上匀速滑行/静止 */
        float t = (phase - swing_ratio) / (1.0f - swing_ratio);
        *x = Xe + (Xs - Xe) * t;
        *z = Zs;
    }
}

/* ============================================================
 *  三次贝塞尔曲线轨迹生成
 *  比摆线具有更高的提腿速度，灵活性更强
 * ============================================================ */
void Gen_Bezier_Trajectory(float phase, float swing_ratio,
                           float Xs, float Xe, float Zs, float h,
                           float *x, float *z)
{
    phase = fmodf(phase, 1.0f);
    if (phase < 0.0f) phase += 1.0f;

    if (phase < swing_ratio)
    {
        float tau = phase / swing_ratio;
        float tau2 = tau * tau;
        float tau3 = tau2 * tau;
        float one_minus_tau = 1.0f - tau;
        float one_minus_tau2 = one_minus_tau * one_minus_tau;
        float one_minus_tau3 = one_minus_tau2 * one_minus_tau;

        float step_length = Xe - Xs;

        /* 贝塞尔多项式系数 */
        float b = 3.0f * one_minus_tau2 * tau;
        float c = 3.0f * one_minus_tau * tau2;
        float d = tau3;

        /* X方向控制点: 匀速推进 P1=1/3, P2=2/3
         * 若要前扫/后扫不对称，可调整这两个除数参数 */
        *x = Xs + b * (step_length / 3.0f) + 
                  c * (2.0f * step_length / 3.0f) + 
                  d * step_length;

        /* Z方向控制点: 
         * 如果设为h/2，最高点只有 0.375*h。
         * 为了让它刚好在 tau=0.5 时抬高 h，控制点需设为 4/3 * h */
        float p_z = h * 4.0f / 3.0f; 
        *z = Zs - (b * p_z + c * p_z);
    }
    else
    {
        /* 支撑相，地面上匀速滑行/静止 */
        float t = (phase - swing_ratio) / (1.0f - swing_ratio);
        *x = Xe + (Xs - Xe) * t;
        *z = Zs;
    }
}

/* ============================================================
 *  正运动学 — 电机无符号角 → 足端位置 (mm)
 *
 *  从电机角还原链节角, 再算足端位置
 * ============================================================ */
void FK_LegPosition(float mt, float ms,
                    const RobotGeometry *geom,
                    float *x_out, float *z_out)
{
    float L1   = geom->L1;
    float L2   = geom->L2;
    float foff = geom->foot_offset_rad;
    float tzero = geom->thigh_zero;
    float sc = geom->shank_coupling_c;
    float splay = geom->splay_angle_rad;

    /* 电机角 → 链节角 */
    float theta_hip        = -(mt + tzero);
    float theta_shank_link = -(ms + mt) + sc;

    /* 等效足端方向 */
    float phi = theta_shank_link - foff;

    *x_out = L1 * sinf(theta_hip) + L2 * sinf(phi);
    
    /* 腿平面内的Z长度，再投射回垂直身体距离 */
    float Z_leg = L1 * cosf(theta_hip) + L2 * cosf(phi);
    *z_out = Z_leg * cosf(splay);
}

/* ============================================================
 *  足端速度 — Jacobian × 电机角速度
 *
 *  设 q = [mt, ms]^T  (无符号电机角)
 *  θ_hip = -(mt + C1)        → dθ_hip/dmt = -1,  dθ_hip/dms = 0
 *  θ_sl  = -(ms + mt) + C2   → dθ_sl/dmt  = -1,  dθ_sl/dms  = -1
 *  φ = θ_sl - foff           → dφ/dmt     = -1,  dφ/dms     = -1
 *
 *  x = L1*sin(θh) + L2*sin(φ)
 *  dx/dmt = L1*cos(θh)*(-1) + L2*cos(φ)*(-1) = -(L1*cos(θh) + L2*cos(φ))
 *  dx/dms = L2*cos(φ)*(-1)                    = -L2*cos(φ)
 *
 *  z = L1*cos(θh) + L2*cos(φ)
 *  dz/dmt = -L1*sin(θh)*(-1) - L2*sin(φ)*(-1) = L1*sin(θh) + L2*sin(φ)
 *  dz/dms = -L2*sin(φ)*(-1)                    = L2*sin(φ)
 * ============================================================ */
void FK_LegVelocity(float mt, float ms,
                    float vel_mt, float vel_ms,
                    const RobotGeometry *geom,
                    float *xdot, float *zdot)
{
    float L1   = geom->L1;
    float L2   = geom->L2;
    float foff = geom->foot_offset_rad;
    float tzero = geom->thigh_zero;
    float sc = geom->shank_coupling_c;
    float splay = geom->splay_angle_rad;

    float theta_hip = -(mt + tzero);
    float theta_sl  = -(ms + mt) + sc;
    float phi       = theta_sl - foff;

    float ch = cosf(theta_hip);
    float sh = sinf(theta_hip);
    float cp = cosf(phi);
    float sp = sinf(phi);

    /* Jacobian J = [dx/dmt, dx/dms; dz/dmt, dz/dms] */
    float Jx_mt = -(L1 * ch + L2 * cp);
    float Jx_ms = -(L2 * cp);
    
    float cos_splay = cosf(splay);
    float Jz_mt =  (L1 * sh + L2 * sp) * cos_splay;
    float Jz_ms =  (L2 * sp) * cos_splay;

    *xdot = Jx_mt * vel_mt + Jx_ms * vel_ms;
    *zdot = Jz_mt * vel_mt + Jz_ms * vel_ms;
}
