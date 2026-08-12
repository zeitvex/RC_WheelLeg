#ifndef __GAIT_CONFIG_H__
#define __GAIT_CONFIG_H__

#include <math.h>
#include <stdint.h>

#ifndef PI
#define PI 3.1415926535f
#endif
#define DEG2RAD(x) ((x) * PI / 180.0f)

/* ============================================================
 *  腿部索引 & 镜像
 * ============================================================ */
typedef enum {
    LEG_FL = 0,
    LEG_FR = 1,
    LEG_RL = 2,
    LEG_RR = 3,
    LEG_COUNT = 4
} leg_index_e;

/* 左右镜像: FL/RL = -1, FR/RR = +1 */
static const float SIDE_SIGNS[LEG_COUNT] = {-1.0f, 1.0f, -1.0f, 1.0f};

/* Trot 对角腿同相 */
static const float TROT_PHASES[LEG_COUNT] = {0.0f, 0.5f, 0.5f, 0.0f};

/* Crawl 相位: 依次迈腿 (四拍步态) */
static const float CRAWL_PHASES[LEG_COUNT] = {0.0f, 0.5f, 0.75f, 0.25f};

/* 电机映射: [leg][0]=大腿, [leg][1]=小腿 */
static const int LEG_MOTOR_MAP[LEG_COUNT][2] = {
    {1, 2}, /* FL: CAN1 ID1, ID2 */
    {3, 4}, /* FR: CAN1 ID3, ID4 */
    {5, 6}, /* RL: CAN2 ID1, ID2 */
    {7, 8}  /* RR: CAN2 ID3, ID4 */
};

/* ============================================================
 *  机器人几何参数 (硬件实测)
 *
 *  坐标系: 原点=髋关节, X前向正, Z向下正
 *  电机零位: 趴下时所有电机读数=0
 *
 *  硬件标定数据 (右侧无符号):
 *    大腿垂直时 motor = -1.0 rad
 *    小腿链节垂直时 motor = +2.76 rad
 *    足端安装偏置 = 40° (小腿前倾40°足端才垂直)
 *
 *  同步带 1:1 耦合模型:
 *    θ_hip       = -(motor_thigh + THIGH_ZERO)
 *    θ_shank_link = -(motor_shank + motor_thigh) + SHANK_COUPLING_C
 *    θ_foot_eff  = θ_shank_link - FOOT_OFFSET
 *
 *  IK → 电机角 (无符号):
 *    motor_thigh = -(θ_hip + THIGH_ZERO)
 *    motor_shank = -motor_thigh - θ_shank_link + SHANK_COUPLING_C
 * ============================================================ */

/* 硬件标定常量 */
#define THIGH_ZERO_RAD      0.95f            /* 大腿垂直时电机绝对角 (rad) */
#define SHANK_VERTICAL_RAD  2.76f           /* 小腿链节垂直时电机角 (rad) */
#define FOOT_OFFSET_RAD     DEG2RAD(40.0f)  /* 足端安装偏置 40° */

/* 同步带耦合常量: C = 1.76 */
#define SHANK_COUPLING_C    1.76f

/* 外八补偿角度 */
#define LEG_SPLAY_ANGLE_RAD DEG2RAD(15.0f)

typedef struct {
    float L1;                /* 大腿长 (mm) */
    float L2;                /* 小腿到足端球心 (mm) */
    float foot_offset_rad;   /* 足端偏置角 (rad) */
    float thigh_zero;        /* 大腿垂直时电机角 */
    float shank_coupling_c;  /* 耦合常量 */
    float splay_angle_rad;   /* 外八倾斜角 */
} RobotGeometry;

/* ============================================================
 *  PD 增益 + 力矩前馈
 * ============================================================ */
typedef struct {
    float kp_thigh;
    float kp_shank;
    float kd;
    float torque;
} ControlGains;

/* ============================================================
 *  VMC 全状态参数
 * ============================================================ */
typedef struct {
    uint8_t enable;
    float kx;
    float kz;
    float torque_limit;
} VmcConfig;

typedef struct {
    uint8_t enable;
    float kx;
    float kz;
    float bx;
    float bz;
    float gravity_ff;
    float torque_limit;
} VmcFullConfig;

/* ============================================================
 *  步态轨迹参数 (运行时可调)
 * ============================================================ */
typedef struct {
    float step_length;
    float step_height;
    float period;
    float start_z;
    float duty_cycle;
    float turn_rate;
} GaitParams;

/* ============================================================
 *  完整步态配置集
 *  注: PostureConfig 和 GaitScaling 已废除
 *      IK 直接输出电机角, 无需间接映射
 * ============================================================ */
typedef struct {
    ControlGains  swing_gains;
    ControlGains  stance_gains;
    VmcConfig     vmc;
    VmcFullConfig vmc_stance;
    VmcFullConfig vmc_swing;
    uint8_t       use_full_vmc;
} GaitProfile;

/* ============================================================
 *  预定义配置
 * ============================================================ */
extern const RobotGeometry DEFAULT_GEOMETRY;
extern const GaitProfile PROFILE_TROT_FORWARD;
extern const GaitProfile PROFILE_TROT_WALK;
extern const GaitProfile PROFILE_TROT_INPLACE;
extern const ControlGains CRAWL_SWING_GAINS;
extern const ControlGains CRAWL_STANCE_GAINS;

/* 腿部站立初始的 X 轴偏置，可用于改善因为重心靠后导致后腿受力过大的问题 */
extern const float LEG_STANCE_X_OFFSET[LEG_COUNT];

/* 高度常量 (髋关节到足端的距离 mm) */
#define PRONE_Z  150.0f
#define STAND_Z  380.0f    

#define ROBOT_MASS_KG      5.0f
#define GRAVITY_FF_PER_LEG (ROBOT_MASS_KG * 9.81f / 4.0f)

#endif /* __GAIT_CONFIG_H__ */
