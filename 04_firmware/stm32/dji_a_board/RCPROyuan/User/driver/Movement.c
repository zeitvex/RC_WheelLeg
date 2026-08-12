/**
 * @file  Movement.c
 * @brief 四足运动控制 — 直接电机角映射版
 *
 * 改动:
 *   - IK 直接输出无符号电机角, 不再经过 PostureConfig
 *   - FK/Jacobian 基于实测标定参数 (foot_offset, thigh_zero, shank_coupling_c)
 *   - use_full_vmc=0 时暂时关闭全状态VMC, 先验证IK
 */

#include "Movement.h"
#include <math.h>

/* ============================================================
 *  编码器偏置 (从 lingzu_task.c 同步)
 * ============================================================ */
static float vmc_enc_offset[LEG_COUNT][2] = {{0},{0},{0},{0}};
static uint8_t vmc_offsets_valid = 0;

void VMC_UpdateEncoderOffsets(const float offsets[8])
{
    vmc_enc_offset[LEG_FL][0] = offsets[0];
    vmc_enc_offset[LEG_FL][1] = offsets[1];
    vmc_enc_offset[LEG_FR][0] = offsets[2];
    vmc_enc_offset[LEG_FR][1] = offsets[3];
    vmc_enc_offset[LEG_RL][0] = offsets[4];
    vmc_enc_offset[LEG_RL][1] = offsets[5];
    vmc_enc_offset[LEG_RR][0] = offsets[6];
    vmc_enc_offset[LEG_RR][1] = offsets[7];
    vmc_offsets_valid = 1;
}

/* ============================================================
 *  底层
 * ============================================================ */
void Posture(int motor_id, float position, float speed,
             float kp, float kd, float torque)
{
    rs02_set_target_rad((uint8_t)motor_id, position, speed, kp, kd, torque);
}

void Leg_Thigh_SetTarget(leg_index_e leg, float q, float speed,
                          float kp, float kd, float torque)
{
    if (leg >= LEG_COUNT) return;
    Posture(LEG_MOTOR_MAP[leg][0], q, speed, kp, kd, torque);
}

void Leg_Shank_SetTarget(leg_index_e leg, float q, float speed,
                          float kp, float kd, float torque)
{
    if (leg >= LEG_COUNT) return;
    Posture(LEG_MOTOR_MAP[leg][1], q, speed, kp, kd, torque);
}

void Leg_All_SetTarget(leg_index_e leg, float q1, float q2, float speed,
                        float kp, float kd, float torque)
{
    Leg_Thigh_SetTarget(leg, q1, speed, kp, kd, torque);
    Leg_Shank_SetTarget(leg, q2, speed, kp, kd, torque);
}

/* ============================================================
 *  工具
 * ============================================================ */
static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ============================================================
 *  VMC 力矩 — 旧版简化 (开环, 轨迹目标位置)
 * ============================================================ */
static void vmc_legacy_torque(float mt_unsigned, float ms_unsigned,
                               float x_ref_mm, float z_ref_mm,
                               const RobotGeometry *geom,
                               const VmcConfig *cfg,
                               float *tau1, float *tau2)
{
    *tau1 = 0.0f; *tau2 = 0.0f;
    if (!cfg->enable) return;

    /* FK 计算轨迹足端位置 */
    float x_traj, z_traj;
    FK_LegPosition(mt_unsigned, ms_unsigned, geom, &x_traj, &z_traj);

    float fx = -cfg->kx * (x_traj - x_ref_mm) * 0.001f;
    float fz =  cfg->kz * (z_ref_mm - z_traj) * 0.001f;

    /* Jacobian 转置 (基于实际角度) */
    float foff = geom->foot_offset_rad;
    float theta_hip = -(mt_unsigned + geom->thigh_zero);
    float theta_sl  = -(ms_unsigned + mt_unsigned) + geom->shank_coupling_c;
    float phi = theta_sl - foff;
    float cos_splay = cosf(geom->splay_angle_rad);

    float L1m = geom->L1 * 0.001f;
    float L2m = geom->L2 * 0.001f;

    /* dx/dmt, dz/dmt 等 (见 leg.c 注释) */
    float Jx_mt = -(L1m * cosf(theta_hip) + L2m * cosf(phi));
    float Jx_ms = -(L2m * cosf(phi));
    float Jz_mt =  (L1m * sinf(theta_hip) + L2m * sinf(phi)) * cos_splay;
    float Jz_ms =  (L2m * sinf(phi)) * cos_splay;

    /* J^T * F → τ */
    *tau1 = clampf(Jx_mt * fx + Jz_mt * fz, -cfg->torque_limit, cfg->torque_limit);
    *tau2 = clampf(Jx_ms * fx + Jz_ms * fz, -cfg->torque_limit, cfg->torque_limit);
}

/* ============================================================
 *  VMC 力矩 — 全状态闭环 (编码器反馈)
 * ============================================================ */
static void vmc_full_torque(leg_index_e leg,
                             float x_ref_mm, float z_ref_mm,
                             const RobotGeometry *geom,
                             const VmcFullConfig *cfg,
                             float *tau1, float *tau2)
{
    *tau1 = 0.0f; *tau2 = 0.0f;
    if (!cfg->enable || !vmc_offsets_valid) return;

    float sign = SIDE_SIGNS[leg];
    int tid = LEG_MOTOR_MAP[leg][0];
    int sid = LEG_MOTOR_MAP[leg][1];

    /* 读取编码器, 去偏置, 去镜像 → 无符号电机角 */
    float mt_raw = rs02_get_position_rad((uint8_t)tid);
    float ms_raw = rs02_get_position_rad((uint8_t)sid);
    float mt = (mt_raw - vmc_enc_offset[leg][0]) / sign;
    float ms = (ms_raw - vmc_enc_offset[leg][1]) / sign;

    /* FK: 实际足端位置 */
    float x_act, z_act;
    FK_LegPosition(mt, ms, geom, &x_act, &z_act);

    /* 速度 */
    float vt_raw = rs02_get_velocity_rad((uint8_t)tid);
    float vs_raw = rs02_get_velocity_rad((uint8_t)sid);
    float vt = vt_raw / sign;
    float vs = vs_raw / sign;

    float xdot, zdot;
    FK_LegVelocity(mt, ms, vt, vs, geom, &xdot, &zdot);

    /* 虚拟力 */
    float fx = -cfg->kx * (x_act - x_ref_mm) * 0.001f - cfg->bx * xdot * 0.001f;
    float fz = -cfg->kz * (z_act - z_ref_mm) * 0.001f - cfg->bz * zdot * 0.001f + cfg->gravity_ff;

    /* Jacobian (基于实际角度) */
    float foff = geom->foot_offset_rad;
    float theta_hip = -(mt + geom->thigh_zero);
    float theta_sl  = -(ms + mt) + geom->shank_coupling_c;
    float phi = theta_sl - foff;
    float cos_splay = cosf(geom->splay_angle_rad);

    float L1m = geom->L1 * 0.001f;
    float L2m = geom->L2 * 0.001f;

    float Jx_mt = -(L1m * cosf(theta_hip) + L2m * cosf(phi));
    float Jx_ms = -(L2m * cosf(phi));
    float Jz_mt =  (L1m * sinf(theta_hip) + L2m * sinf(phi)) * cos_splay;
    float Jz_ms =  (L2m * sinf(phi)) * cos_splay;

    *tau1 = clampf(Jx_mt * fx + Jz_mt * fz, -cfg->torque_limit, cfg->torque_limit);
    *tau2 = clampf(Jx_ms * fx + Jz_ms * fz, -cfg->torque_limit, cfg->torque_limit);
}

/* ============================================================
 *  静态姿态
 * ============================================================ */
void Quad_Hold(const RobotGeometry *geom, float base_z, float kp, float kd)
{
    float ratio = (base_z - PRONE_Z) / (STAND_Z - PRONE_Z + 0.1f);
    if (ratio < 0.0f) ratio = 0.0f;
    if (ratio > 1.0f) ratio = 1.0f;

    for (int i = 0; i < LEG_COUNT; i++)
    {
        float target_x = LEG_STANCE_X_OFFSET[i] * ratio;
        float mt, ms;
        Inverse_Calculation(target_x, base_z, &mt, &ms, geom);

        float q1 = mt * SIDE_SIGNS[i];
        float q2 = ms * SIDE_SIGNS[i];
        Leg_All_SetTarget((leg_index_e)i, q1, q2, 0.0f, kp, kd, 0.0f);
    }
}

void Quad_Prone(const RobotGeometry *geom, float base_z, float kp, float kd)
{
    Quad_Hold(geom, base_z, kp, kd);
}

void Quad_Stand(const RobotGeometry *geom, float base_z, float kp, float kd)
{
    Quad_Hold(geom, base_z, kp, kd);
}

/* ============================================================
 *  Trot 步态 — 核心
 * ============================================================ */
void Quadruped_Trot(float t, const GaitParams *params,
                    const RobotGeometry *geom, const GaitProfile *profile)
{
    float period     = params->period;
    float start_z    = params->start_z;
    float step_len   = params->step_length;
    float step_h     = fabsf(params->step_height);
    float turn_rate  = params->turn_rate;
    float swing_ratio = params->duty_cycle;
    if (swing_ratio < 0.1f || swing_ratio > 0.9f) swing_ratio = 0.5f;

    float left_step  = step_len * (1.0f + turn_rate);
    float right_step = step_len * (1.0f - turn_rate);
    if (fabsf(step_len) < 0.01f) {
        /* 当基础步长为0时，使用 turn_rate 直接作为纯自旋步长差 (mm) */
        left_step = turn_rate;
        right_step = -turn_rate;
    }

    float ratio = (start_z - PRONE_Z) / (STAND_Z - PRONE_Z + 0.1f);
    if (ratio < 0.0f) ratio = 0.0f;
    if (ratio > 1.0f) ratio = 1.0f;

    for (int i = 0; i < LEG_COUNT; i++)
    {
        leg_index_e leg = (leg_index_e)i;
        float leg_step = (i == LEG_FL || i == LEG_RL) ? left_step : right_step;
        float half = leg_step / 2.0f;

        float current_phase = (t / period) + TROT_PHASES[i];
        float norm_phase = fmodf(current_phase, 1.0f);
        if (norm_phase < 0.0f) norm_phase += 1.0f;

        float x_traj, z_traj;
        Gen_Bezier_Trajectory(current_phase, swing_ratio,
                              -half, half, start_z, step_h,
                              &x_traj, &z_traj);
                               
        /* 增加 X 轴静步态偏置 */
        x_traj += LEG_STANCE_X_OFFSET[i] * ratio;

        int is_swing = (norm_phase < swing_ratio);

        /* IK: 直接得到无符号电机角 */
        float mt, ms;
        Inverse_Calculation(x_traj, z_traj, &mt, &ms, geom);

        /* 左右镜像 */
        float target_q1 = mt * SIDE_SIGNS[i];
        float target_q2 = ms * SIDE_SIGNS[i];

        /* PD 参数 */
        const ControlGains *gains = is_swing
                                    ? &profile->swing_gains
                                    : &profile->stance_gains;

        /* VMC 力矩 */
        float tau1 = 0.0f, tau2 = 0.0f;
        if (profile->use_full_vmc && vmc_offsets_valid) {
            const VmcFullConfig *vcfg = is_swing
                                        ? &profile->vmc_swing
                                        : &profile->vmc_stance;
            vmc_full_torque(leg, x_traj, z_traj, geom, vcfg, &tau1, &tau2);
        } else {
            vmc_legacy_torque(mt, ms, x_traj, z_traj, geom,
                              &profile->vmc, &tau1, &tau2);
        }

        float tlimit = profile->use_full_vmc
                       ? (is_swing ? profile->vmc_swing.torque_limit
                                   : profile->vmc_stance.torque_limit)
                       : profile->vmc.torque_limit;

        float tq1 = clampf(gains->torque + tau1, -tlimit, tlimit);
        float tq2 = clampf(gains->torque + tau2, -tlimit, tlimit);

        Posture(LEG_MOTOR_MAP[i][0], target_q1, 0.0f,
                gains->kp_thigh, gains->kd, tq1);
        Posture(LEG_MOTOR_MAP[i][1], target_q2, 0.0f,
                gains->kp_shank, gains->kd, tq2);
    }
}

/* ============================================================
 *  便捷接口
 * ============================================================ */
void Quadruped_Forward(float t, const GaitParams *params, const RobotGeometry *geom)
{
    Quadruped_Trot(t, params, geom, &PROFILE_TROT_FORWARD);
}

void Quadruped_Backward(float t, const GaitParams *params, const RobotGeometry *geom)
{
    GaitParams p = *params;
    p.step_length = -50.0f; /* 必须为负数才能向后运动 */
    p.turn_rate   = 0.0f;
    Quadruped_Trot(t, &p, geom, &PROFILE_TROT_WALK);
}

void Quadruped_InPlace(float t, const GaitParams *params, const RobotGeometry *geom)
{
    GaitParams p = *params;
    p.step_length = 0.0f;
    p.turn_rate   = 0.0f;
    Quadruped_Trot(t, &p, geom, &PROFILE_TROT_INPLACE);
}

void Quadruped_SpinLeft(float t, const GaitParams *params, const RobotGeometry *geom)
{
    GaitParams p = *params;
    p.step_length = 0.0f;
    p.turn_rate   = -100.0f; /* 左转：左侧倒退，右侧前进 */
    Quadruped_Trot(t, &p, geom, &PROFILE_TROT_INPLACE);
}

void Quadruped_SpinRight(float t, const GaitParams *params, const RobotGeometry *geom)
{
    GaitParams p = *params;
    p.step_length = 0.0f;
    p.turn_rate   = 100.0f;  /* 右转：左侧前进，右侧倒退 */
    Quadruped_Trot(t, &p, geom, &PROFILE_TROT_INPLACE);
}

/* ============================================================
 *  爬行步态
 * ============================================================ */
void Quadruped_Crawl(float t, const GaitParams *params, const RobotGeometry *geom)
{
    float period    = params->period;
    float start_z   = params->start_z;
    float step_len  = params->step_length;
    float step_h    = fabsf(params->step_height);
    float swing_ratio = params->duty_cycle;
    if (swing_ratio < 0.1f || swing_ratio > 0.9f) swing_ratio = 0.5f;

    float ratio = (start_z - PRONE_Z) / (STAND_Z - PRONE_Z + 0.1f);
    if (ratio < 0.0f) ratio = 0.0f;
    if (ratio > 1.0f) ratio = 1.0f;

    for (int i = 0; i < LEG_COUNT; i++)
    {
        float current_phase = (t / period) + CRAWL_PHASES[i];
        float norm_phase = fmodf(current_phase, 1.0f);
        if (norm_phase < 0.0f) norm_phase += 1.0f;

        float half = step_len / 2.0f;
        float x_traj, z_traj;
        Gen_Bezier_Trajectory(current_phase, swing_ratio,
                              -half, half, start_z, step_h,
                              &x_traj, &z_traj);
                               
        x_traj += LEG_STANCE_X_OFFSET[i] * ratio;

        const ControlGains *gains = (norm_phase < swing_ratio)
                                    ? &CRAWL_SWING_GAINS
                                    : &CRAWL_STANCE_GAINS;

        float mt, ms;
        Inverse_Calculation(x_traj, z_traj, &mt, &ms, geom);

        Posture(LEG_MOTOR_MAP[i][0], mt * SIDE_SIGNS[i], 0.0f,
                gains->kp_thigh, gains->kd, gains->torque);
        Posture(LEG_MOTOR_MAP[i][1], ms * SIDE_SIGNS[i], 0.0f,
                gains->kp_shank, gains->kd, gains->torque);
    }
}
