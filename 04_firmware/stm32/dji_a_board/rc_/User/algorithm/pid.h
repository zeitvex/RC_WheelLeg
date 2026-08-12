/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

/**
  *********************** (C) COPYRIGHT 2018 DJI **********************
  * @update
  * @history
  * Version     Date              Author           Modification
  * V1.0.0      January-15-2018   ric.luo
  * @verbatim
  *********************** (C) COPYRIGHT 2018 DJI **********************
  */ 
 


  
#ifndef __pid_H__
#define __pid_H__

#include "stm32f4xx_hal.h"

enum
{
  LAST  = 0,
  NOW   = 1,
};

/**
  * @brief     PID 结构体
  */
typedef struct
{
  /* p、i、d参数 */
  float p;
  float i;
  float d;

  /* 目标值、反馈值、误差值 */
  float set;
  float get;
  float err[2];

  /* p、i、d各项计算出的输出 */
  float pout; 
  float iout; 
  float dout; 

  /* pid公式计算出的总输出 */
  float out;

  /* pid最大输出限制  */
  uint32_t max_output;
  
  /* pid积分输出项限幅 */
  uint32_t integral_limit;
 
} pid_t_robomaster;

/**
  * @brief     PID 初始化函数
  * @param[in] pid: PID 结构体
  * @param[in] max_out: 最大输出
  * @param[in] intergral_limit: 积分限幅
  * @param[in] kp/ki/kd: 具体 PID 参数
  */
void pid_init(pid_t_robomaster *pid, uint32_t max_out, uint32_t intergral_limit, float kp, float ki, float kd);

/**
  * @brief     PID 参数复位函数
  * @param[in] pid: PID 结构体
  * @param[in] kp/ki/kd: 具体 PID 参数
  */
void pid_reset(pid_t_robomaster *pid, float kp, float ki, float kd);

/**
  * @brief     PID 计算函数，使用位置式 PID 计算
  * @param[in] pid: PID 结构体
  * @param[in] get: 反馈数据
  * @param[in] set: 目标数据
  * @retval    PID 计算输出
  */
float pid_calc(pid_t_robomaster *pid, float get, float set);


//云台电机 PID 结构体定义
extern pid_t_robomaster pid_pit;
extern pid_t_robomaster pid_yaw;
extern pid_t_robomaster pid_pit_speed;
extern pid_t_robomaster pid_yaw_speed;
//底盘电机 PID 结构体定义
extern pid_t_robomaster pid_wheel_spd[4];
extern pid_t_robomaster pid_chassis_angle;
    //拨弹电机 PID 结构体定义
extern pid_t_robomaster pid_trigger;
extern pid_t_robomaster pid_trigger_speed;
    
extern pid_t_robomaster pid_test_moto;
#endif
