#include "lingzu_task.h"
#include "lingzu_motor.h"
#include "cmsis_os.h"
#include "rm_hal_lib.h" // for BEEP macros if needed
#include <math.h>
#include "Movement.h"
#include "leg.h"

#define TO_ZERO_TIME 10000.0f
#define PI 3.1415926535f

extern Can_Bus_Data_Struct CAN_1;
extern Can_Bus_Data_Struct CAN_2;

/** 
电机编号
前   小腿 大腿         大腿 小腿
left  2   1------------3   4 right -- can 1
           |         |
           |         |
           |         |
           |         |
后    2   1------------3   4       -- can 2
                                        */

static const int leg_motor_map[4][2] = {
	{1, 2}, // FL
	{3, 4}, // FR
	{5, 6}, // RL
	{7, 8}  // RR
};

// 调用其中一个电机并设置角度或速度
void Posture(int a,float position,float speed,float kp,float kd,float torque )
{
	rs02_set_target_rad((uint8_t)a, position, speed, kp, kd, torque); // User/driver/lingzu_motor.c
}

typedef struct
{
	uint8_t enable;
	float kx;
	float kz;
	float torque_limit;
} vmc_cfg_t;

static vmc_cfg_t g_vmc_cfg = {1, 35.0f, 55.0f, 8.0f};

void Movement_VMC_SetEnable(uint8_t enable)
{
	g_vmc_cfg.enable = (enable != 0U) ? 1U : 0U;
}

void Movement_VMC_SetGains(float kx, float kz)
{
	g_vmc_cfg.kx = (kx < 0.0f) ? 0.0f : kx;
	g_vmc_cfg.kz = (kz < 0.0f) ? 0.0f : kz;
}

void Movement_VMC_SetTorqueLimit(float limit)
{
	g_vmc_cfg.torque_limit = (limit < 0.5f) ? 0.5f : limit;
}

static float vmc_clamp(float v, float min_v, float max_v)
{
	if (v < min_v) return min_v;
	if (v > max_v) return max_v;
	return v;
}

static void vmc_planar_jt_torque(float q1, float q2, float x_m, float z_m, float z_ref_m, float l1_m, float l2_m, float *tau1, float *tau2)
{
	if (g_vmc_cfg.enable == 0U)
	{
		*tau1 = 0.0f;
		*tau2 = 0.0f;
		return;
	}

	float fx = -g_vmc_cfg.kx * x_m;
	float fz = g_vmc_cfg.kz * (z_ref_m - z_m);

	float c1 = cosf(q1);
	float s1 = sinf(q1);
	float c12 = cosf(q1 + q2);
	float s12 = sinf(q1 + q2);

	float dx_dq1 = -(l1_m * c1 + l2_m * c12);
	float dx_dq2 = -(l2_m * c12);
	float dz_dq1 = -(l1_m * s1 + l2_m * s12);
	float dz_dq2 = -(l2_m * s12);

	*tau1 = vmc_clamp(dx_dq1 * fx + dz_dq1 * fz, -g_vmc_cfg.torque_limit, g_vmc_cfg.torque_limit);
	*tau2 = vmc_clamp(dx_dq2 * fx + dz_dq2 * fz, -g_vmc_cfg.torque_limit, g_vmc_cfg.torque_limit);
}

// 摆线轨迹生成函数
// phase: 当前相位 [0, 1]
// swing_ratio: 摆动相占比 (例如 0.5 表示 50% 时间摆动，50% 时间支撑)
// Xs, Xe: X轴起点和终点
// Zs: Z轴基础高度
// h: 抬腿高度 (正值)
void Gen_Cycloid_Trajectory(float phase, float swing_ratio, float Xs, float Xe, float Zs, float h, float *x, float *z)
{
    // 归一化相位
    phase = fmodf(phase, 1.0f);
    if(phase < 0) phase += 1.0f;

    if (phase < swing_ratio) // 摆动相 (Swing Phase)
    {
        // 映射 phase 到 [0, 1]
        float t = phase / swing_ratio;
        
        // X轴: 从 Xs 到 Xe 的摆线运动
        // x(t) = Xs + (Xe - Xs) * (t - 1/(2pi)*sin(2pi*t))
        *x = Xs + (Xe - Xs) * (t - 1.0f/(2.0f*PI) * sinf(2.0f*PI*t));
        
        // Z轴: 抬腿 (Zs -> Zs-h -> Zs)
        *z = Zs - h * sinf(PI * t); 
    }
    else // 支撑相 (Stance Phase)
    {
        // 映射 phase 到 [0, 1]
        float t = (phase - swing_ratio) / (1.0f - swing_ratio);
        
        // X轴: 从 Xe 回到 Xs 的匀速直线运动 (推动身体前进)
        *x = Xe + (Xs - Xe) * t;
        
        // Z轴: 保持基础高度
        *z = Zs;
    }
}

// 原地踏步专属摆线轨迹生成函数 (仅Z轴上下运动，无X轴位移)
void Gen_Cycloid_Trajectory_InPlace(float phase, float swing_ratio, float Zs, float h, float *x, float *z)
{
    // 归一化相位
    phase = fmodf(phase, 1.0f);
    if(phase < 0) phase += 1.0f;

    // X轴始终保持为0 (无前后位移)
    *x = 0.0f;

    if (phase < swing_ratio) // 摆动相 (Swing Phase)
    {
        // 映射 phase 到 [0, 1]
        float t = phase / swing_ratio;
        
        // Z轴: 抬腿 (Zs -> Zs-h -> Zs)
        *z = Zs - h * sinf(PI * t); 
    }
    else // 支撑相 (Stance Phase)
    {
        // Z轴: 保持基础高度
        *z = Zs;
    }
}

/**
 * @brief  四足前进函数 (Quadruped Forward Walk) - 基于站立姿态重写
 * @param  t: 当前时间
 * @param  base_params: 基础轨迹参数 (包含步长、步高、周期等)
 * @param  geom: 机器人几何参数
 * @note   该函数以站立姿态为基准，当步长为0时执行后退逻辑
 */
/**
 * @brief  四足前进函数 (Quadruped Forward Walk)
 * @param  t: 当前时间
 * @param  base_params: 基础轨迹参数 (包含步长、步高、周期等)
 * @param  geom: 机器人几何参数
 * @note   直接复用 Quadruped_Walk 的逻辑，确保前进/后退姿态完全一致
 *         当 step_length > 0 时前进，当 step_length = 0 时原地踏步
 */
void Quadruped_Forward_Walk(float t, TrajectoryParams *base_params, RobotGeometry *geom)
{
	TrajectoryParams p_local = *base_params;
    p_local.step_length = -80.0f; // 强制步长为0//-10后退//10也后退---///目前-50到-80之间可能出现最稳定的前进，目前试了-80能够前进
    p_local.turn_rate = -0.14f;   // 强制不转向
    // 直接调用 Quadruped_Walk，确保参数和姿态完全一致
	p_local.step_height = -fabsf(p_local.step_height); 
	p_local.duty_cycle = 0.25f;
    Quadruped_Walk(t, &p_local, geom);
}


/// 
void Quadruped_Walk(float t, TrajectoryParams *base_params, RobotGeometry *geom)
{
    float period = base_params->period;
    // 调大Kp/Kd以获得更好的刚性
    float kp = 120.0f; 
    float kd = 15.0f; 
    float torque = 7.0f; //力矩
    float speed = 0.0f; // 前馈速度暂设为0，主要靠位置伺服
    
    // Trot步态相位: 对角腿同相
    float phases[4] = {0.0f, 0.5f, 0.5f, 0.0f};
    
    // 基础参数
    float base_step_len = base_params->step_length;
    float step_h = base_params->step_height; // 确保为正值
    float start_z = base_params->start_z;
    float turn_rate = base_params->turn_rate; // -1.0(左转) ~ 1.0(右转)
    
    // 摆动相占比 (参考 leg_task.c 的 arr 参数)
    // 0.5 为标准 Trot，小于 0.5 (如 0.4) 可增加双脚着地时间，提高稳定性
    float swing_ratio = base_params->duty_cycle; 
    if (swing_ratio < 0.1f || swing_ratio > 0.9f) swing_ratio = 0.5f; // 安全默认值
    
    // 计算左右侧步长 (差速转向)
    float left_step_len = base_step_len * (1.0f + turn_rate);
    float right_step_len = base_step_len * (1.0f - turn_rate);

    // 左右腿镜像系数：Left(FL, RL)=1.0, Right(FR, RR)=1.0 (根据用户反馈取消反转)
    // 用户反馈：右前腿(FR)正常(CW)，左前腿(FL)反了(CCW)，需要 FL 单独反转
    // FL(Index 0): -1.0f (需要反转)
    // FR(Index 1): 1.0f  (改为正转，用户反馈顺时针(CW)是错的，应为逆时针(CCW))
    // RL(Index 2): -1.0f (推测与FL一致)
    // RR(Index 3): 1.0f  (推测与FR一致)
    float side_signs[4] = {-1.0f, 1.0f, -1.0f, 1.0f};
    /*******************需要基于站立姿态开始计算步幅,所以数据需要与站立姿态保持一致***************/
    // 与 Quad_Stand 对齐的大腿/小腿基准转动参数(决定站立姿态)
    float thigh_scale = 1.1f;                      // 大腿转动因子(站立基准)
    float hip_offset  = 10.0f * PI / 180.0f;       // 臀部关节偏移角(站立基准)
    // 新增：前腿大腿角度微调 (同步站立姿态)
    float front_thigh_trim = -10.0f * PI / 180.0f; //-25  -15

    float shank_scale = 0.6f;                      // 小腿转动因子(站立基准)
    float knee_offset = 85.0f * PI / 180.0f;       // 膝关节偏移角(站立基准)
    // 新增：前腿小腿角度微调 (同步站立姿态)
    float front_shank_trim = -20.0f * PI / 180.0f; 

    // 额外的步态摆动缩放系数(在站立基准姿态附近放大相对摆动角度)
    float gait_thigh_scale = 1.0f;                 // >1 增大大腿摆动    1.5  2.5
    float gait_shank_scale = 1.5f;                 // >1 增大小腿摆动    2.0  2.0
    // 新增：后腿小腿摆动缩放因子 (仅在前进时生效)
    float hind_shank_scale = 2.0f;                 // 后腿小腿基础值1.0 倍

    // 计算几何空间下的“站立中性姿态”关节角(不含左右镜像)
    float stand_q1, stand_q2;
    Inverse_Calculation(0, start_z, &stand_q1, &stand_q2, geom); // User/driver/leg.c

    // 将站立几何角映射为关节空间基准角(包含与Quad_Stand一致的偏移和缩放)
    // 注意: neutral_q1_geom 需在循环中根据前后腿分别计算
    // float neutral_q2_geom = (stand_q2 + knee_offset) * shank_scale; // 移至循环内

    for (int i = 0; i < 4; i++)
    {
        // 针对前腿 (FL=0, FR=1) 叠加额外的微调角度
        float current_hip_offset = hip_offset;
        float current_knee_offset = knee_offset;
        if (i < 2) {
            current_hip_offset += front_thigh_trim;
            current_knee_offset += front_shank_trim;
        }

        // 计算当前腿的站立中性姿态 (作为步态摆动的基准)
        float neutral_q1_geom = (stand_q1 + current_hip_offset) * thigh_scale;
        float neutral_q2_geom = (stand_q2 + current_knee_offset) * shank_scale;

        float current_phase = (t / period) + phases[i];
        
        // 归一化相位用于判断支撑/摆动状态
        float norm_phase = fmodf(current_phase, 1.0f);
        if(norm_phase < 0) norm_phase += 1.0f;

        float x, z; 
        
        // 确定当前腿的步长
        float leg_step_len;
        if (i == 0 || i == 2) // 左侧腿 (FL, RL)
            leg_step_len = left_step_len;
        else // 右侧腿 (FR, RR)
            leg_step_len = right_step_len;

        // 摆动起点(后) 和 终点(前)
        float x_back = -leg_step_len / 2.0f;
        float x_front = leg_step_len / 2.0f;

        // 生成摆线轨迹 (支持 swing_ratio)
        Gen_Cycloid_Trajectory(current_phase, swing_ratio, x_back, x_front, start_z, step_h, &x, &z);
        
        // --------------- 动态参数调整 (Dynamic Parameter Scheduling) ---------------------
        // 解决身子下沉导致的蹭地问题
        float dynamic_kp_thigh, dynamic_kp_shank, dynamic_torque;
        
        if (norm_phase < swing_ratio) 
        {
            // [摆动相 Swing Phase] (腿在空中)
            // 任务：必须精准抬腿，防止蹭地。
            // 策略：低力矩(不需要抗重力)，高Kp(强制跟踪轨迹，把腿缩上去)
            dynamic_torque = 5.0f;
            dynamic_kp_thigh = 120.0f;
            dynamic_kp_shank = 130.0f;
        }
        else 
        {
            // [支撑相 Stance Phase] (脚踩地)
            // 任务：支撑机身重量。
            // 策略：高力矩(抗重力)，适中Kp(吸收冲击)
            dynamic_torque = 5.0f;
            dynamic_kp_thigh = 120.0f;
            dynamic_kp_shank = 130.0f;
        }
        // -----------------------------------------------

        // 逆运动学解算 (得到几何正角度)
        float q1, q2;
        Inverse_Calculation(x, z, &q1, &q2, geom); // User/driver/leg.c

        // 1) 先应用与 Quad_Stand 相同的偏移和缩放, 得到当前步态时刻的几何关节角
        float q1_hip       = q1 + current_hip_offset;
        float q2_with_knee = q2 + current_knee_offset;

        float gait_q1_geom = q1_hip       * thigh_scale;
        float gait_q2_geom = q2_with_knee * shank_scale;

        // 2) 计算相对于“站立中性姿态”的关节增量(全部在几何空间)
        float delta_q1_geom = gait_q1_geom - neutral_q1_geom;
        float delta_q2_geom = gait_q2_geom - neutral_q2_geom;

        // 3) 在中性姿态基础上叠加放大后的摆动增量
        // float current_shank_scale = gait_shank_scale;  <-- Remove this line
        float current_shank_swing_scale = gait_shank_scale; // Rename to avoid confusion
        
        // 仅在前进时 (步长为负) 且针对后腿 (i=2:RL, i=3:RR) 调整小腿摆动幅度
        // 注意 i 的顺序: 0=FL, 1=FR, 2=RL, 3=RR
        if (base_step_len < -0.01f && i >= 2) 
        {
            current_shank_swing_scale *= hind_shank_scale;
        }

        float final_q1_geom = neutral_q1_geom + delta_q1_geom * gait_thigh_scale;
        float final_q2_geom = neutral_q2_geom + delta_q2_geom * current_shank_swing_scale;

        // 4) 最后一步应用左右镜像符号, 映射到实际电机角度
        float target_q1 = final_q1_geom * side_signs[i];
        float target_q2 = final_q2_geom * side_signs[i];

        float tau_vmc_q1 = 0.0f;
        float tau_vmc_q2 = 0.0f;
        vmc_planar_jt_torque(q1, q2, x * 0.001f, z * 0.001f, start_z * 0.001f, geom->L1 * 0.001f, geom->L2 * 0.001f, &tau_vmc_q1, &tau_vmc_q2);

        float torque_q1 = vmc_clamp(dynamic_torque + tau_vmc_q1, -g_vmc_cfg.torque_limit, g_vmc_cfg.torque_limit);
        float torque_q2 = vmc_clamp(dynamic_torque + tau_vmc_q2, -g_vmc_cfg.torque_limit, g_vmc_cfg.torque_limit);

        // 下发控制 (使用动态参数)
        Posture(leg_motor_map[i][0], target_q1, speed, dynamic_kp_thigh, kd, torque_q1);
        Posture(leg_motor_map[i][1], target_q2, speed, dynamic_kp_shank, kd, torque_q2);
    }
}
///  
/// 
///  实际作用：原地踏步
/// 
void Quadruped_InPlace(float t, TrajectoryParams *base_params, RobotGeometry *geom)
{
    Quadruped_PivotTurn_Core(t, base_params, geom, 0.0f);
}

// 输入: t(float), base_params(TrajectoryParams*), geom(RobotGeometry*)
// 输出: 无
// 作用: 后退 (复用行走逻辑但步长为0)
void Quadruped_backward(float t, TrajectoryParams *base_params, RobotGeometry *geom)
{
    TrajectoryParams p_local = *base_params;
    p_local.step_length = 25.0f; // 强制步长为0
    p_local.turn_rate = 0.0f;   // 强制不转向
    
    //SwC上=后退
    // SwC中 = 空出 (恢复为步长0的原地踏步)
    // p_local.step_length = 100.0f; 
    
    Quadruped_Walk(t, &p_local, geom);
}

// 输入: t(float), base_params(TrajectoryParams*), geom(RobotGeometry*)
// 输出: 无
// 作用: 左转步态 (右侧步长 > 左侧步长)
void Quadruped_Turn_Left(float t, TrajectoryParams *base_params, RobotGeometry *geom)
{
    TrajectoryParams p_local = *base_params;
    // 设定左转转向率
    // turn_rate = -0.3f 表示左侧步长 = base * (1-0.3) = 0.7, 右侧步长 = base * (1+0.3) = 1.3
    // 右侧步长 > 左侧步长 -> 产生向左的力矩
    p_local.turn_rate = -0.4f; // 调整此值改变转向半径，负值表示左转
    Quadruped_Walk(t, &p_local, geom);
}

// 输入: t(float), base_params(TrajectoryParams*), geom(RobotGeometry*)
// 输出: 无
// 作用: 右转步态 (左侧步长 > 右侧步长)
void Quadruped_Turn_Right(float t, TrajectoryParams *base_params, RobotGeometry *geom)
{
    TrajectoryParams p_local = *base_params;
    // 设定右转转向率
    // turn_rate = 0.3f 表示左侧步长 = base * (1+0.3) = 1.3, 右侧步长 = base * (1-0.3) = 0.7
    // 左侧步长 > 右侧步长 -> 产生向右的力矩
    p_local.turn_rate = 0.4f; // 调整此值改变转向半径，正值表示右转
    Quadruped_Walk(t, &p_local, geom);
}

/**
 * 输入: t(float), base_params(TrajectoryParams*), geom(RobotGeometry*), turn_dir(float)
 * 输出: 无
 * 作用: 原地差速转向核心步态
 *   turn_dir > 0 -> 左转: 左侧腿步幅小(内侧)，右侧腿步幅大(外侧)
 *   turn_dir < 0 -> 右转: 右侧腿步幅小(内侧)，左侧腿步幅大(外侧)
 *
 * 原理: 四条腿全部参与Trot步态，通过差速控制左右侧步幅比例产生旋转力矩。
 *   内侧腿步幅小(可接近0)起到相对支撑作用，外侧腿步幅大驱动机身旋转。
 *   避免了单侧完全静止导致的前后晃动问题。
 *
 * 参数说明:
 *   outer_ratio: 外侧腿步幅比例 (相对于 step_length), 建议 0.8~1.0
 *   inner_ratio: 内侧腿步幅比例 (相对于 step_length), 建议 0.0~0.2
 *   两者差值越大转向越急，inner_ratio=0时内侧腿原地踏步
 */
static void Quadruped_PivotTurn_Core(float t, TrajectoryParams *base_params, RobotGeometry *geom, float turn_dir)
{
    float period    = base_params->period;
    float kd        = 15.0f;
    float speed     = 0.0f;
    float start_z   = base_params->start_z;

    float step_h = fabsf(base_params->step_height);
    if (step_h < 5.0f) step_h = 20.0f;

    // 基础步幅 (以外侧腿为基准)
    float base_step = fabsf(base_params->step_length);
    if (base_step < 5.0f) base_step = 60.0f;

    float swing_ratio = base_params->duty_cycle;
    if (swing_ratio < 0.1f || swing_ratio > 0.9f) swing_ratio = 0.5f;

    // 差速比例:
    //   外侧腿步幅 = base_step * outer_ratio  (迈大步，产生推力)
    //   内侧腿步幅 = base_step * inner_ratio   (迈小步，近似支撑但不完全静止)
    float outer_ratio = 1.15f;   // 外侧腿步幅比例
    float inner_ratio = 0.15f;  // 内侧腿步幅比例 (约为外侧的15%，几乎原地踏步)
    //此处的两个参数只与原地踏步相关
    float left_ratio  = 1.0f;   // 左腿步幅比例
    float right_ratio = 1.0f;   // 右腿步幅比例
    // 补偿右转偏大：右转时外侧腿(左腿)步幅缩小约15%（单纯的硬改，其主要问题为底层问题，但为了简单直接使用了经验补偿方式）
    if (turn_dir < 0.0f) {
        outer_ratio = 1.0f;
        inner_ratio = 0.15f;
    }

    //这是一个原地踏步的想法（ai说如果转向半径还是太大，可以将 inner_ratio 调小到 0.05f 甚至 0.0f（完全原地踏步）。）
    if(turn_dir == 0.0f) {
            inner_ratio = 0.05f;
            outer_ratio = 1.0f;
//            left_ratio  = 0.38f;   // 左腿略小，抵消推力不对称
//            right_ratio = 0.42f;   // 右腿略大
    }



    // Trot 标准相位 (FL=0, FR=0.5, RL=0.5, RR=0)
    float phases[4] = {0.0f, 0.5f, 0.5f, 0.0f};

    // 左右腿镜像符号 (与 Quadruped_Walk 保持一致)
    float side_signs[4] = {-1.0f, 1.0f, -1.0f, 1.0f};

    // ---- 与 Quadruped_Walk 保持一致的关节参数 ----
    float thigh_scale      = 1.1f;
    float hip_offset       = 10.0f  * PI / 180.0f;
    float front_thigh_trim = -10.0f * PI / 180.0f;
    float shank_scale      = 0.6f;
    float knee_offset      = 85.0f  * PI / 180.0f;
    float front_shank_trim = -20.0f * PI / 180.0f;
    float gait_thigh_scale = 1.0f;
    float gait_shank_scale = 1.5f;


    // 计算站立中性关节角
    float stand_q1, stand_q2;
    Inverse_Calculation(0, start_z, &stand_q1, &stand_q2, geom);

    for (int i = 0; i < 4; i++)
    {
        // 判断当前腿是内侧(同转向侧)还是外侧(驱动侧)
        // turn_dir > 0 (左转): 左腿(i==0,2)为内侧，右腿(i==1,3)为外侧
        // turn_dir < 0 (右转): 右腿(i==1,3)为内侧，左腿(i==0,2)为外侧 
        int is_left_leg = (i == 0 || i == 2);
        int is_inner;   // 1=内侧腿(小步幅), 0=外侧腿(大步幅)
        if (turn_dir > 0.0f)
            is_inner = is_left_leg;   // 左转: 左腿内侧
        else if(turn_dir < 0.0f)
            is_inner = !is_left_leg;  // 右转: 右腿内侧
        else 
            is_inner = 0.05f;    // turn_dir==0: 所有腿都是外侧，步幅一致，真正的原地踏步，因为这里默认turn_dir=0.0f
        
            // 根据内外侧分配步幅
        float leg_step = base_step * (is_inner ? inner_ratio : outer_ratio);
        ///此处为为了原地踏步而单独加的一部分运算，因为在运行原地踏步的时候又微微右转的迹象
//        if (turn_dir == 0.0f) {
//            leg_step = base_step * (is_inner ? left_ratio : right_ratio);
//        }

        float half_step = leg_step / 2.0f;
        float x_back  = -half_step;
        float x_front =  half_step;

        float current_hip_offset  = hip_offset;
        float current_knee_offset = knee_offset;
        if (i < 2) {
            current_hip_offset  += front_thigh_trim;
            current_knee_offset += front_shank_trim;
        }

        float current_phase = (t / period) + phases[i];
        float norm_phase = fmodf(current_phase, 1.0f);
        if (norm_phase < 0) norm_phase += 1.0f;

        float x, z;
        Gen_Cycloid_Trajectory(current_phase, swing_ratio, x_back, x_front, start_z, step_h, &x, &z);

        // 动态参数: 外侧腿支撑相提高力矩推进，内侧腿参数适中
        float dynamic_kp_thigh, dynamic_kp_shank, dynamic_torque;
        if (norm_phase < swing_ratio)
        {
            // 摆动相: 精准抬腿
            dynamic_torque   = 5.0f;
            dynamic_kp_thigh = 120.0f;
            dynamic_kp_shank = 130.0f;
        }
        else
        {
            // 支撑相: 外侧腿高力矩推进，内侧腿适中保持稳定
            dynamic_torque   = is_inner ? 5.0f : 7.0f;
            dynamic_kp_thigh = is_inner ? 120.0f : 140.0f;
            dynamic_kp_shank = is_inner ? 130.0f : 150.0f;
        }

        // 逆运动学解算
        float q1, q2;
        Inverse_Calculation(x, z, &q1, &q2, geom);

        // 与 Quadruped_Walk 相同的角度映射 (中性姿态 + 增量放大)
        float neutral_q1 = (stand_q1 + current_hip_offset)  * thigh_scale;
        float neutral_q2 = (stand_q2 + current_knee_offset) * shank_scale;

        float gait_q1 = (q1 + current_hip_offset)  * thigh_scale;
        float gait_q2 = (q2 + current_knee_offset) * shank_scale;

        float final_q1 = neutral_q1 + (gait_q1 - neutral_q1) * gait_thigh_scale;
        float final_q2 = neutral_q2 + (gait_q2 - neutral_q2) * gait_shank_scale;

        float target_q1 = final_q1 * side_signs[i];
        float target_q2 = final_q2 * side_signs[i];

        float tau_vmc_q1 = 0.0f, tau_vmc_q2 = 0.0f;
        vmc_planar_jt_torque(q1, q2, x * 0.001f, z * 0.001f, start_z * 0.001f,
                             geom->L1 * 0.001f, geom->L2 * 0.001f, &tau_vmc_q1, &tau_vmc_q2);

        float torque_q1 = vmc_clamp(dynamic_torque + tau_vmc_q1, -g_vmc_cfg.torque_limit, g_vmc_cfg.torque_limit);
        float torque_q2 = vmc_clamp(dynamic_torque + tau_vmc_q2, -g_vmc_cfg.torque_limit, g_vmc_cfg.torque_limit);

        Posture(leg_motor_map[i][0], target_q1, speed, dynamic_kp_thigh, kd, torque_q1);
        Posture(leg_motor_map[i][1], target_q2, speed, dynamic_kp_shank, kd, torque_q2);
    }
}

/**
 * 输入: t(float), base_params(TrajectoryParams*), geom(RobotGeometry*)
 * 输出: 无
 * 作用: 原地左转 - 左腿小步幅(内侧), 右腿大步幅(外侧), 差速产生向左旋转力矩
 */
void Quadruped_Spin_Left_InPlace(float t, TrajectoryParams *base_params, RobotGeometry *geom)
{
    Quadruped_PivotTurn_Core(t, base_params, geom, 1.0f);
}

/**
 * 输入: t(float), base_params(TrajectoryParams*), geom(RobotGeometry*)
 * 输出: 无
 * 作用: 原地右转 - 右腿小步幅(内侧), 左腿大步幅(外侧), 差速产生向右旋转力矩
 */
void Quadruped_Spin_Right_InPlace(float t, TrajectoryParams *base_params, RobotGeometry *geom)
{
    Quadruped_PivotTurn_Core(t, base_params, geom, -1.0f);
}

// 输入: leg(leg_index_e), q_thigh(float), speed(float), kp(float), kd(float), torque(float)
// 输出: 无
// 作用: 设置指定腿大腿关节的目标角度及控制参数
void Leg_Thigh_SetTarget(leg_index_e leg, float q_thigh, float speed, float kp, float kd, float torque)
{
	if (leg > LEG_RR)
	{
		return;
	}
	Posture(leg_motor_map[leg][0], q_thigh, speed, kp, kd, torque);
}

// 输入: leg(leg_index_e), q_shank(float), speed(float), kp(float), kd(float), torque(float)
// 输出: 无
// 作用: 设置指定腿小腿关节的目标角度及控制参数
void Leg_Shank_SetTarget(leg_index_e leg, float q_shank, float speed, float kp, float kd, float torque)
{
	if (leg < LEG_FL || leg > LEG_RR)
	{
		return;
	}
	Posture(leg_motor_map[leg][1], q_shank, speed, kp, kd, torque);
}

// 输入: leg(leg_index_e), q_thigh(float), q_shank(float), speed(float), kp(float), kd(float), torque(float)
// 输出: 无
// 作用: 同时设置指定腿大腿和小腿的目标角度及控制参数
void Leg_All_SetTarget(leg_index_e leg, float q_thigh, float q_shank, float speed, float kp, float kd, float torque)
{
	Leg_Thigh_SetTarget(leg, q_thigh, speed, kp, kd, torque);
	Leg_Shank_SetTarget(leg, q_shank, speed, kp, kd, torque);
}

// 输入: geom(RobotGeometry*), base_z(float), kp(float), kd(float)
// 输出: 无
// 作用: 根据给定高度使四条腿进入指定站立姿态
void Quad_Stand(RobotGeometry *geom, float base_z, float kp, float kd)
{
	float q1, q2;
	float side_signs[4] = {-1.0f, 1.0f, -1.0f, 1.0f};
	Inverse_Calculation(0, base_z, &q1, &q2, geom); // User/driver/leg.c
    // 站立时大腿转动因子，>1 更弯，<1 更直
    float thigh_scale = 1.2f; 
    //臀部关节偏移角
    float hip_offset = 10.0f * PI / 180.0f; 
    // 新增：前腿大腿角度微调 (正值使前腿大腿更靠后/抬头，负值靠前/低头)
    float front_thigh_trim = -15.0f * PI / 180.0f;

    // 小腿转动因子 (根据实际测试调整)
	float shank_scale = 0.6f;  //0.6f
    // 新增：膝关节偏移角 (单位: rad)，让小腿比几何解更“伸直 / 向下”
    float knee_offset = 66.0f * PI / 180.0f; 

	for (int i = 0; i < 4; i++)
	{
        // 针对前腿 (FL=0, FR=1) 叠加额外的微调角度
        float current_hip_offset = hip_offset;
        if (i < 2) {
            current_hip_offset += front_thigh_trim;
        }

        float q1_hip = q1 + current_hip_offset;
        // 臀部角度
        float target_q1 = q1_hip * thigh_scale * side_signs[i] ;
		// 对 q2 加上偏移，再缩放、再乘左右腿符号
        float q2_with_offset = q2 + knee_offset;   // 或 q2 - knee_offset，根据实际方向来选
        float target_q2 = q2_with_offset * shank_scale * side_signs[i];

		Leg_All_SetTarget((leg_index_e)i, target_q1, target_q2, 0.0f, kp, kd, 0.0f);
	}
}

// 输入: geom(RobotGeometry*), base_z(float), kp(float), kd(float)
// 输出: 无
// 作用: 根据给定高度使四条腿进入趴下姿态
void Quad_Prone(RobotGeometry *geom, float base_z, float kp, float kd)
{
	float q1, q2;
	float side_signs[4] = {-1.0f, 1.0f, -1.0f, 1.0f};
	Inverse_Calculation(0, base_z, &q1, &q2, geom); // User/driver/leg.c
	for (int i = 0; i < 4; i++)
	{
		float target_q1 = q1 * side_signs[i];
		float target_q2 = q2 * side_signs[i];
		Leg_All_SetTarget((leg_index_e)i, target_q1, target_q2, 0.0f, kp, kd, 0.0f);
	}
}

// 输入: t(float), params(TrajectoryParams*), geom(RobotGeometry*)
// 输出: 无
// 作用: 根据步态参数执行一次四足步态更新
void Quad_Gait_Step(float t, TrajectoryParams *params, RobotGeometry *geom)
{
	Quadruped_Walk(t, params, geom); // User/driver/Movement.c
}

/**
 * @brief 趴着行走 (爬行步态)
 * @param t 当前时间 (秒)
 * @param base_params 步态参数 (步长、高度等)
 * @param geom 机器人几何参数
 */
void Quadruped_Crawl(float t, TrajectoryParams *base_params, RobotGeometry *geom)
{
    float period = base_params->period;
    float speed = 0.0f;
    
    // Trot步态相位: 对角腿同相
    float phases[4] = {0.0f, 0.5f, 0.5f, 0.0f};
    
    // 基础参数
    float base_step_len = base_params->step_length;
    float step_h = fabsf(base_params->step_height); 
    float start_z = base_params->start_z; // 这里通常传入较小的 Z 值实现“趴着”
    
    float swing_ratio = base_params->duty_cycle; 
    if (swing_ratio < 0.1f || swing_ratio > 0.9f) swing_ratio = 0.5f;

    // 左右腿镜像系数
    float side_signs[4] = {-1.0f, 1.0f, -1.0f, 1.0f};

    // --- 【修正跑偏】 ---
    // 现象：向左转，且严重“跛脚”：左后(RL)和右前(FR)不离地，只有左前(FL)和右后(RR)离地
    // 原因：
    // 1. "对角线失衡"：FL/RR 这一组抬得高，说明这组的对角线（FR/RL）在支撑时把身体顶得太高了，或者 FL/RR 本身收缩得太厉害。
    // 2. "不离地"：FR/RL 这一组在摆动相时，没有收缩够，或者身体被 FL/RR 压得太低。
    // 3. 根本原因：之前的 left_support_boost 是针对左侧所有腿加的，导致 FL 和 RL 表现一致，但现在发现是 FL/RR 和 FR/RL 这两组对角线表现不一致。
    
    // 解决策略：不再按左右分，而是按对角线组分。
    // Group A (FL+RR): 抬得太高 -> 减小摆动相收缩 (extra_q2_swing)
    // Group B (FR+RL): 不离地 -> 增大摆动相收缩，或者增大 Group A 的支撑高度
    
    // 鉴于目前是 FR+RL 不离地，我们需要让它们缩得更多
    float swing_retract_A = 10.0f * PI / 180.0f; // FL+RR 正常收缩
    float swing_retract_B = 30.0f * PI / 180.0f; // FR+RL 强力收缩 (让它离地)

    // 支撑力度：既然左转，说明右侧推力大，或者左侧阻力大(拖地)
    // FR+RL 不离地就是最大的阻力来源。解决了不离地，应该就能解决左转。
    float left_support_boost = 15.0f * PI / 180.0f; 
    float right_support_boost = 5.0f * PI / 180.0f; 

    // 恢复步长平衡
    float left_step_scale = 1.0f; 
    float right_step_scale = 1.0f; 

    // --- 【动作 2 & 4: 对角支撑与迈步】 ---

    // --- 【修改点 1: 基础姿态偏移】 ---
    // 趴着行走时，可能需要不同于站立的偏移量
    float hip_offset  = 0.0f * PI / 180.0f;  // 趴着时大腿可能不需要额外偏移
    float knee_offset = 0.0f * PI / 180.0f;  // 趴着时小腿偏移

    for (int i = 0; i < 4; i++)
    {
        float current_phase = (t / period) + phases[i];
        float norm_phase = fmodf(current_phase, 1.0f);
        if(norm_phase < 0) norm_phase += 1.0f;

        float x, z; 
        
        // 应用步长修正
        float leg_step_len;
        float turn_rate = base_params->turn_rate;
        if (i == 0 || i == 2) { 
            leg_step_len = base_step_len * left_step_scale * (1.0f + turn_rate); // 左侧
        } else { 
            leg_step_len = base_step_len * right_step_scale * (1.0f - turn_rate); // 右侧
        }

        // 摆动起点和终点
        float x_back = -leg_step_len / 2.0f;
        float x_front = leg_step_len / 2.0f;

        // 生成轨迹
        Gen_Cycloid_Trajectory(current_phase, swing_ratio, x_back, x_front, start_z, step_h, &x, &z);
        
        // --- 【修改点 2: 动态 PID 参数】 ---
        float dynamic_kp = 30.0f;   // 趴着重心低，Kp可以适当减小
        float dynamic_kd = 8.0f;
        float dynamic_torque = 3.0f; // 趴着负载轻，力矩可减小
        
        if (norm_phase < swing_ratio) {
            dynamic_kp = 40.0f; // 摆动相增加刚度确保抬腿
        }

        // 逆运动学解算
        float q1, q2;
        Inverse_Calculation(x, z, &q1, &q2, geom);

        // --- 【修改点 3: 角度映射与修正】 ---
        // 在这里加入您对腿部当前角度的具体更改逻辑
        // 例如：增加一个随时间变化的微调，或者根据腿的索引 i 加入特定偏移
        float q1_final = q1 + hip_offset;
        
        // 使用支撑补偿
        float current_knee_offset = knee_offset;
        if (i == 0 || i == 2) current_knee_offset += left_support_boost;
        else current_knee_offset += right_support_boost;
        
        float q2_final = q2 + current_knee_offset;

        // 示例：给大腿增加一个额外的正弦摆动（仅作演示）
        // q1_final += 0.1f * sinf(2.0f * PI * current_phase);

        float target_q1 = q1_final * side_signs[i];
        float target_q2 = q2_final * side_signs[i];

        // 下发控制
        Posture(leg_motor_map[i][0], target_q1, speed, dynamic_kp, dynamic_kd, dynamic_torque);
        Posture(leg_motor_map[i][1], target_q2, speed, dynamic_kp, dynamic_kd, dynamic_torque);
    }
}

/**
 * @brief 爬行准备姿态 (Static Crawl Ready Posture)
 * @param geom 机器人几何参数
 * @param base_z 基础高度
 * @param kp 刚度
 * @param kd 阻尼
 * 
 * 作用：
 * 当 SwC 切到中间但未推摇杆时，使机器人进入“低重心、大腿上抬”的姿态。
 * 这与 Quadruped_Sequential_Crawl 中的初始状态保持一致，避免突变。
 */
void Quad_Prone_Crawl_Ready(RobotGeometry *geom, float base_z, float kp, float kd)
{
    float q1, q2;
    float side_signs[4] = {-1.0f, 1.0f, -1.0f, 1.0f};
    
    // 与 Quadruped_Sequential_Crawl 保持一致的偏移量
    // 负值表示大腿向上抬起，降低重心
    float prone_hip_offset = +30.0f * PI / 180.0f; 
    float prone_claf_offset = 0.0f * PI / 180.0f;
    // 计算基础逆解
    Inverse_Calculation(0, base_z, &q1, &q2, geom); 
    
    // 叠加偏移
    float q1_final = q1 + prone_hip_offset;
    float q2_final = q2 + prone_claf_offset; // 小腿保持默认，或者也可以微调

    for (int i = 0; i < 4; i++)
    {
        float target_q1 = q1_final * side_signs[i];
        float target_q2 = q2_final * side_signs[i];
        
        // 使用静态保持的力矩 (通常为0或很小，依靠位置闭环)
        // 但为了支撑效果，可以给一点点前馈，或者只靠 Kp
        float static_torque = 0.0f; 

        Posture(leg_motor_map[i][0], target_q1, 0.0f, kp, kd, static_torque);
        Posture(leg_motor_map[i][1], target_q2, 0.0f, kp, kd, static_torque);
    }
}
////q1控制大腿  q2控制小腿
/**
 * @brief 分阶段爬行步态 (Sequential Crawl)
 * @param t 当前时间 (秒)
 * @param base_params 步态参数 (步长、高度等)
 * @param geom 机器人几何参数
 * 
 * 动作分解：
 * 1. 降低重心：所有大腿向上抬起 (大腿关节角调整)，使机身贴近地面。
 * 2. 对角支撑 (FL+RR)：左前和右后小腿向下伸展，将身体撑起，离开地面。
 * 3. 迈步推进 (FR+RL)：右前和左后大腿摆动向前 (迈步)，同时小腿收缩 (避免拖地)。
 * 4. 交替支撑：FR+RL 小腿放下支撑，FL+RR 小腿收缩并大腿摆动。
 * 
 * 实现思路：
 * 使用 phases 数组控制对角相位，结合周期 t 将动作分解为支撑相和摆动相。
 * 在此基础上，叠加显式的“大腿上抬”和“小腿下伸”的偏置逻辑。
 */
void Quadruped_Sequential_Crawl(float t, TrajectoryParams *base_params, RobotGeometry *geom)
{
    float period = base_params->period;
    float speed = 0.0f;
    
    // Trot相位: 对角腿同相
    // Group A: FL(0) & RR(3) -> Phase 0.0
    // Group B: FR(1) & RL(2) -> Phase 0.5
    float phases[4] = {0.0f, 0.5f, 0.5f, 0.0f};
    
    // 基础参数
    float base_step_len = base_params->step_length;
    float step_h = fabsf(base_params->step_height); 
    float start_z = base_params->start_z; 
    
    // 设定摆动相占比 (例如 0.5 表示一半时间支撑，一半时间迈步)
    float swing_ratio = base_params->duty_cycle; 
    if (swing_ratio < 0.1f || swing_ratio > 0.9f) swing_ratio = 0.5f;

    // 左右腿镜像系数
    float side_signs[4] = {-1.0f, 1.0f, -1.0f, 1.0f};

    // --- 【动作 1: 降低重心 (大腿上抬)】 ---
    // 通过给大腿关节一个固定的负偏移 (假设负是向上抬)，让机身趴低。
    // 或者直接在 Inverse_Calculation 中使用较小的 Z 值 (start_z)。
    // 这里我们使用 start_z 来控制重心高度，并在大腿角度上叠加一个微调。
    // 假设正常站立 hip_offset 是 10度，这里我们减小它或者设为负值让大腿更平。
    float prone_hip_offset = +30.0f * PI / 180.0f; // 与 Quad_Prone_Crawl_Ready 保持一致

    // --- 【修正跑偏】 ---
    // 现象：还是左转
    // 原因：之前逻辑反了。左转说明右侧跑得比左侧快。
    // 我们刚才设了 左0.8 / 右1.1，这正是让右边跑得更快，所以加剧了左转。
    
    // 解决策略：反过来！
    // 要修正左转，必须让机器人产生向右的力矩。
    // 向右力矩 = 左侧加速 (迈大步) + 右侧减速 (迈小步)
    
    // 支撑力度：保持现状 (20/20)，因为离地情况良好
    float left_support_boost = 20.0f * PI / 180.0f; 
    float right_support_boost = 20.0f * PI / 180.0f; 

    // 调整步长平衡：
    // 左侧加大 -> 1.1
    // 右侧减小 -> 0.9
    float left_step_scale = 1.3f;  
    float right_step_scale = 0.7f; 

    // --- 【动作 2 & 4: 对角支撑与迈步】 ---
    // 通过周期函数控制小腿的伸缩。
    // 支撑腿：小腿伸展 (向下顶地) -> 增加 knee_offset 或 Z 轴向下
    // 迈步腿：小腿收缩 (向上收起) -> 减小 knee_offset 或 Z 轴向上
    
    // 小腿伸缩幅度 (弧度)
 

    for (int i = 0; i < 4; i++)
    {
        float current_phase = (t / period) + phases[i];
        float norm_phase = fmodf(current_phase, 1.0f);
        if(norm_phase < 0) norm_phase += 1.0f;

        float x, z; 
        
        // 应用步长修正
        float leg_step_len;
        if (i == 0 || i == 2) { 
            leg_step_len = base_step_len * left_step_scale; // 左侧
        } else { 
            leg_step_len = base_step_len * right_step_scale; // 右侧
        }

        // 摆动起点和终点
        float x_back = -leg_step_len / 2.0f;
        float x_front = leg_step_len / 2.0f;

        // 生成基础轨迹 (摆线)
        // 这已经包含了“迈步”时的抬腿动作 (Z轴变化) 和 前进动作 (X轴变化)
        Gen_Cycloid_Trajectory(current_phase, swing_ratio, x_back, x_front, start_z, step_h, &x, &z);
        
        // --- 叠加自定义的“拆分动作”逻辑 ---
        
        // 动态调整参数
        float dynamic_kp = 35.0f;   
        float dynamic_kd = 8.0f;
        float dynamic_torque = 5.0f; 
        
        // 额外的关节角度修正量
        float extra_q1 = 0.0f; // 大腿修正
        float extra_q2 = 0.0f; // 小腿修正

        // float swing_retract_common = 35.0f * PI / 180.0f; // Unused

        if (norm_phase < swing_ratio) 
        {
            // [摆动相 / 迈步相] (对应 Group B 当 Group A 支撑时)
            
            // 默认强力收缩 (35度)
            float retract = 35.0f * PI / 180.0f;
            
            // 针对 FL (左前) 抬太高的问题，减小它的收缩
            if (i == 0) retract = 15.0f * PI / 180.0f; 
            
            extra_q2 -= retract;
            
            // --- 调整大腿摆动 ---
            // 如果您希望不改变大腿的摆动，只需将 extra_q1 设为 0
            // 或者如果您想保留一点抬腿效果但减小幅度，可以减小这个值
            extra_q1 -= 0.0f * PI / 180.0f; 

            dynamic_kp = 45.0f; 
            dynamic_torque = 2.0f; 
        }
        else 
        {
            // [支撑相] (对应 Group A)
            // 动作：小腿向下伸展，将身体撑起
            // 动作：大腿向后划 (由 Gen_Cycloid_Trajectory 的 X 变化处理)
            
            // 在这里显式地让小腿“向下顶”
            // 可以在计算出的 q2 基础上，增加一个正偏移
            if (i == 0 || i == 2) {
                // 左侧腿 (FL, RL)
                float boost = left_support_boost;
                // 如果是前腿 (FL=0)，减小支撑力度，防止起伏过大
                if (i == 0) boost *= 0.7f; 
                extra_q2 += boost;
            } else {
                // 右侧腿 (FR, RR)
                float boost = right_support_boost;
                // 如果是前腿 (FR=1)，减小支撑力度
                if (i == 1) boost *= 0.7f;
                extra_q2 += boost;
            }

            dynamic_kp = 40.0f; // 支撑刚度
            dynamic_torque = 6.0f; // 支撑力矩大
        }

        // 逆运动学解算
        float q1, q2;
        Inverse_Calculation(x, z, &q1, &q2, geom);

        // 应用修正
        // 1. 全局降低重心 (所有大腿上抬)
        float q1_final = q1 + prone_hip_offset;
        
        // 2. 叠加迈步/支撑带来的额外修正
        q1_final += extra_q1;
        float q2_final = q2 + extra_q2; // 恢复正常的逆解叠加逻辑

        float target_q1 = q1_final * side_signs[i];
        float target_q2 = q2_final * side_signs[i];

        // 下发控制
        Posture(leg_motor_map[i][0], target_q1, speed, dynamic_kp, dynamic_kd, dynamic_torque);
        Posture(leg_motor_map[i][1], target_q2, speed, dynamic_kp, dynamic_kd, dynamic_torque);
    }
}











