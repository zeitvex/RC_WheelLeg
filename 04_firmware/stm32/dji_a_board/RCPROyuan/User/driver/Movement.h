#ifndef __MOVEMENT_H__
#define __MOVEMENT_H__

#include "gait_config.h"
#include "leg.h"
#include "lingzu_motor.h"

/* ============================================================
 *  底层: 单电机指令
 * ============================================================ */
void Posture(int motor_id, float position, float speed, float kp, float kd, float torque);

/* ============================================================
 *  单腿控制
 * ============================================================ */
void Leg_Thigh_SetTarget(leg_index_e leg, float q, float speed, float kp, float kd, float torque);
void Leg_Shank_SetTarget(leg_index_e leg, float q, float speed, float kp, float kd, float torque);
void Leg_All_SetTarget(leg_index_e leg, float q1, float q2, float speed, float kp, float kd, float torque);

/* ============================================================
 *  VMC 编码器偏置同步
 * ============================================================ */
void VMC_UpdateEncoderOffsets(const float offsets[8]);

/* ============================================================
 *  静态姿态
 * ============================================================ */
void Quad_Hold(const RobotGeometry *geom, float base_z, float kp, float kd);
void Quad_Prone(const RobotGeometry *geom, float base_z, float kp, float kd);
void Quad_Stand(const RobotGeometry *geom, float base_z, float kp, float kd);

/* ============================================================
 *  Trot 步态
 * ============================================================ */
void Quadruped_Trot(float t, const GaitParams *params,
                    const RobotGeometry *geom, const GaitProfile *profile);

void Quadruped_Forward(float t, const GaitParams *params, const RobotGeometry *geom);
void Quadruped_Backward(float t, const GaitParams *params, const RobotGeometry *geom);
void Quadruped_InPlace(float t, const GaitParams *params, const RobotGeometry *geom);
void Quadruped_SpinLeft(float t, const GaitParams *params, const RobotGeometry *geom);
void Quadruped_SpinRight(float t, const GaitParams *params, const RobotGeometry *geom);

/* ============================================================
 *  爬行步态
 * ============================================================ */
void Quadruped_Crawl(float t, const GaitParams *params, const RobotGeometry *geom);

#endif /* __MOVEMENT_H__ */
