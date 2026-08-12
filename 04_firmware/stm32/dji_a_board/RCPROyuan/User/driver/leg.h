#ifndef __LEG_H__
#define __LEG_H__

#include <math.h>
#include "gait_config.h"

/* ============================================================
 *  逆运动学 — 直接输出无符号电机角
 *
 *  输入: X (前后偏移 mm, 前向正), Z (高度 mm, 向下正)
 *  输出: motor_thigh (大腿无符号电机角 rad)
 *        motor_shank (小腿无符号电机角 rad)
 *
 *  调用方在外部乘 SIDE_SIGNS[leg] 做左右镜像
 *
 *  内部使用:
 *    θ_hip = ψ - β  (后倾解, 适合膝前弯四足)
 *    motor_thigh = -(θ_hip + thigh_zero)
 *    motor_shank = -motor_thigh - θ_shank + shank_coupling_c
 * ============================================================ */
void Inverse_Calculation(float X, float Z,
                         float *motor_thigh, float *motor_shank,
                         const RobotGeometry *geom);

/* ============================================================
 *  摆线轨迹生成 
 * ============================================================ */
void Gen_Cycloid_Trajectory(float phase, float swing_ratio,
                            float Xs, float Xe, float Zs, float h,
                            float *x, float *z);

/* ============================================================
 *  三次贝塞尔曲线轨迹生成
 * ============================================================ */
void Gen_Bezier_Trajectory(float phase, float swing_ratio,
                           float Xs, float Xe, float Zs, float h,
                           float *x, float *z);

/* ============================================================
 *  正运动学 — 电机无符号角 → 足端位置 (mm)
 *  注意: 输入是无符号电机角 (已去掉SIDE_SIGNS)
 * ============================================================ */
void FK_LegPosition(float motor_thigh_unsigned, float motor_shank_unsigned,
                    const RobotGeometry *geom,
                    float *x_out, float *z_out);

/* ============================================================
 *  足端速度 — Jacobian × 电机角速度
 *  输入: 无符号电机角 + 无符号角速度
 * ============================================================ */
void FK_LegVelocity(float motor_thigh_unsigned, float motor_shank_unsigned,
                    float vel_thigh_unsigned, float vel_shank_unsigned,
                    const RobotGeometry *geom,
                    float *xdot, float *zdot);

#endif /* __LEG_H__ */
