#ifndef __MOVEMENT_H__
#define __MOVEMENT_H__
#include "lingzu_task.h"
#include "lingzu_motor.h"
#include "cmsis_os.h"
#include "rm_hal_lib.h" // for BEEP macros if needed
#include <math.h>
#include "leg.h"

extern Can_Bus_Data_Struct CAN_1;
extern Can_Bus_Data_Struct CAN_2;

typedef enum
{
	LEG_FL = 0,
	LEG_FR = 1,
	LEG_RL = 2,
	LEG_RR = 3
} leg_index_e;

void Posture(int a,float position,float speed,float kp,float kd,float torque );
void Movement_VMC_SetEnable(uint8_t enable);
void Movement_VMC_SetGains(float kx, float kz);
void Movement_VMC_SetTorqueLimit(float limit);
void Quadruped_Forward_Walk(float t, TrajectoryParams *base_params, RobotGeometry *geom);
void Quadruped_Walk(float t, TrajectoryParams *base_params, RobotGeometry *geom);
void Quadruped_InPlace(float t, TrajectoryParams *base_params, RobotGeometry *geom);
void Quadruped_backward(float t, TrajectoryParams *base_params, RobotGeometry *geom);
void Quadruped_Turn_Left(float t, TrajectoryParams *base_params, RobotGeometry *geom);
void Quadruped_Turn_Right(float t, TrajectoryParams *base_params, RobotGeometry *geom);
static void Quadruped_PivotTurn_Core(float t, TrajectoryParams *base_params, RobotGeometry *geom, float turn_dir);
/**
 * 输入: t(float), base_params(TrajectoryParams*), geom(RobotGeometry*)
 * 输出: 无
 * 作用: 以站立姿态参数为基准执行原地左旋步态
 */
void Quadruped_Spin_Left_InPlace(float t, TrajectoryParams *base_params, RobotGeometry *geom);

/**
 * 输入: t(float), base_params(TrajectoryParams*), geom(RobotGeometry*)
 * 输出: 无
 * 作用: 以站立姿态参数为基准执行原地右旋步态
 */
void Quadruped_Spin_Right_InPlace(float t, TrajectoryParams *base_params, RobotGeometry *geom);

/**
 * @brief 趴着行走 (爬行步态)
 * @param t 当前时间 (秒)
 * @param base_params 步态参数 (步长、高度等)
 * @param geom 机器人几何参数
 */
void Quadruped_Crawl(float t, TrajectoryParams *base_params, RobotGeometry *geom);

/**
 * @brief 分阶段爬行步态 (Sequential Crawl)
 * @param t 当前时间 (秒)
 * @param base_params 步态参数 (步长、高度等)
 * @param geom 机器人几何参数
 */
void Quadruped_Sequential_Crawl(float t, TrajectoryParams *base_params, RobotGeometry *geom);

/**
 * @brief 爬行准备姿态 (Static Crawl Ready Posture)
 * @param geom 机器人几何参数
 * @param base_z 基础高度
 * @param kp 刚度
 * @param kd 阻尼
 */
void Quad_Prone_Crawl_Ready(RobotGeometry *geom, float base_z, float kp, float kd);

/**
 * 输入: leg(leg_index_e), q_thigh(float), speed(float), kp(float), kd(float), torque(float)
 * 输出: 无
 * 作用: 设置指定腿大腿关节的目标角度及控制参数
 */
void Leg_Thigh_SetTarget(leg_index_e leg, float q_thigh, float speed, float kp, float kd, float torque);

/**
 * 输入: leg(leg_index_e), q_shank(float), speed(float), kp(float), kd(float), torque(float)
 * 输出: 无
 * 作用: 设置指定腿小腿关节的目标角度及控制参数
 */
void Leg_Shank_SetTarget(leg_index_e leg, float q_shank, float speed, float kp, float kd, float torque);

/**
 * 输入: leg(leg_index_e), q_thigh(float), q_shank(float), speed(float), kp(float), kd(float), torque(float)
 * 输出: 无
 * 作用: 同时设置指定腿大腿和小腿的目标角度及控制参数
 */
void Leg_All_SetTarget(leg_index_e leg, float q_thigh, float q_shank, float speed, float kp, float kd, float torque);

/**
 * 输入: geom(RobotGeometry*), base_z(float), kp(float), kd(float)
 * 输出: 无
 * 作用: 根据给定高度使四条腿进入指定站立姿态
 */
void Quad_Stand(RobotGeometry *geom, float base_z, float kp, float kd);

/**
 * 输入: geom(RobotGeometry*), base_z(float), kp(float), kd(float)
 * 输出: 无
 * 作用: 根据给定高度使四条腿进入趴下姿态
 */
void Quad_Prone(RobotGeometry *geom, float base_z, float kp, float kd);

/**
 * 输入: t(float), params(TrajectoryParams*), geom(RobotGeometry*)
 * 输出: 无
 * 作用: 根据步态参数执行一次四足步态更新
 */
void Quad_Gait_Step(float t, TrajectoryParams *params, RobotGeometry *geom);

#endif

