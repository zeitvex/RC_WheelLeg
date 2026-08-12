#include "gait_config.h"

/* ============================================================
 *  默认几何参数 (实测硬件)
 * ============================================================ */
const RobotGeometry DEFAULT_GEOMETRY = {
    .L1              = 250.0f,
    .L2              = 290.0f,
    .foot_offset_rad = FOOT_OFFSET_RAD,
    .thigh_zero      = THIGH_ZERO_RAD,
    .shank_coupling_c = SHANK_COUPLING_C,
    .splay_angle_rad = LEG_SPLAY_ANGLE_RAD
};

/* ===========================FOOT_OFFSET_RAD=================================
 *  步态配置集 — 前进 Trot
 * ============================================================ */
const GaitProfile PROFILE_TROT_FORWARD = {
    .swing_gains = {
        .kp_thigh = 220.0f,
        .kp_shank = 210.0f,
        .kd       = 4.0f,
        .torque   = 5.5f
    },
    .stance_gains = {
        .kp_thigh = 220.0f,
        .kp_shank = 210.0f,
        .kd       = 4.0f,
        .torque   = 10.0f
    },
    .vmc = {
        .enable       = 1,
        .kx           = 35.0f,
        .kz           = 55.0f,
        .torque_limit = 8.0f
    },
    .vmc_stance = {
        .enable       = 1,
        .kx           = 50.0f,
        .kz           = 300.0f,
        .bx           = 8.0f,
        .bz           = 15.0f,
        .gravity_ff   = 0.0f,
        .torque_limit = 12.0f
    },
    .vmc_swing = {
        .enable       = 1,
        .kx           = 0.0f,
        .kz           = 0.0f,
        .bx           = 2.0f,
        .bz           = 2.0f,
        .gravity_ff   = 0.0f,
        .torque_limit = 4.0f
    },
    .use_full_vmc = 0   /* 先关闭, 验证IK正确后再开 */
};

/* ============================================================
 *  步态配置集 — 通用行走
 * ============================================================ */
const GaitProfile PROFILE_TROT_WALK = {
    .swing_gains = {
        .kp_thigh = 220.0f,
        .kp_shank = 210.0f,
        .kd       = 3.0f,
        .torque   = 5.0f
    },
    .stance_gains = {
        .kp_thigh = 220.0f,
        .kp_shank = 210.0f,
        .kd       = 3.0f,
        .torque   = 5.0f
    },
    .vmc = {
        .enable       = 1,
        .kx           = 35.0f,
        .kz           = 55.0f,
        .torque_limit = 8.0f
    },
    .vmc_stance = {
        .enable       = 1,
        .kx           = 50.0f,
        .kz           = 300.0f,
        .bx           = 8.0f,
        .bz           = 15.0f,
        .gravity_ff   = 0.0f,
        .torque_limit = 10.0f
    },
    .vmc_swing = {
        .enable       = 1,
        .kx           = 0.0f,
        .kz           = 0.0f,
        .bx           = 2.0f,
        .bz           = 2.0f,
        .gravity_ff   = 0.0f,
        .torque_limit = 4.0f
    },
    .use_full_vmc = 0
};

/* ============================================================
 *  步态配置集 — 原地踏步/转向
 * ============================================================ */
const GaitProfile PROFILE_TROT_INPLACE = {
    .swing_gains = {
        .kp_thigh = 120.0f,
        .kp_shank = 130.0f,
        .kd       = 15.0f,
        .torque   = 5.0f
    },
    .stance_gains = {
        .kp_thigh = 140.0f,
        .kp_shank = 150.0f,
        .kd       = 15.0f,
        .torque   = 7.0f
    },
    .vmc = {
        .enable       = 1,
        .kx           = 35.0f,
        .kz           = 55.0f,
        .torque_limit = 8.0f
    },
    .vmc_stance = {
        .enable       = 1,
        .kx           = 60.0f,
        .kz           = 300.0f,
        .bx           = 10.0f,
        .bz           = 15.0f,
        .gravity_ff   = 0.0f,
        .torque_limit = 12.0f
    },
    .vmc_swing = {
        .enable       = 1,
        .kx           = 0.0f,
        .kz           = 0.0f,
        .bx           = 2.0f,
        .bz           = 2.0f,
        .gravity_ff   = 0.0f,
        .torque_limit = 4.0f
    },
    .use_full_vmc = 0
};

/* ============================================================
 *  爬行模式 PD 参数
 * ============================================================ */
const ControlGains CRAWL_SWING_GAINS = {
    .kp_thigh = 45.0f,
    .kp_shank = 45.0f,
    .kd       = 8.0f,
    .torque   = 2.0f
};

const ControlGains CRAWL_STANCE_GAINS = {
    .kp_thigh = 40.0f,
    .kp_shank = 40.0f,
    .kd       = 8.0f,
    .torque   = 6.0f
};

/* 腿部站立初始的 X 轴偏置，可用于改善因为重心靠后导致后腿受力过大的问题 */
// 前腿稍微往前一点 ，后腿根据需求"靠后一点"给出明显的负数偏置 
const float LEG_STANCE_X_OFFSET[LEG_COUNT] = {
    10.0f,    /* FL */
    10.0f,    /* FR */
    -65.0f,   /* RL: 减小负数偏置，以前是*/
    -65.0f    /* RR */
};
