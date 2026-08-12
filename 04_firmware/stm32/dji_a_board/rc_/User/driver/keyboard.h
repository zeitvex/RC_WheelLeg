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
 


#ifndef __KB_H__
#define __KB_H__

#include "stm32f4xx_hal.h"

/**********************************************************************************
 * bit      :15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
 * keyboard : V    C    X	  Z    G    F    R   E   Q  CTRL  SHIFT  D   A   S   W
 **********************************************************************************/

/**
  * @brief     底盘运动速度快慢模式
  */
typedef enum 
{
  NORMAL_MODE = 0,    //正常模式
  FAST_MODE,          //快速模式
  SLOW_MODE,          //慢速模式
} kb_move_e;

/**
  * @brief     鼠标按键状态类型枚举
  */
typedef enum
{
  KEY_RELEASE = 0,    //没有按键按下
  KEY_WAIT_EFFECTIVE, //等待按键按下有效，防抖
  KEY_PRESS_ONCE,     //按键按下一次的状态
  KEY_PRESS_DOWN,     //按键已经被按下
  KEY_PRESS_LONG,     //按键长按状态
} kb_state_e;

/**
  * @brief     键盘鼠标数据结构体
  */
typedef struct
{
  /* 键盘模式使能标志 */
  uint8_t kb_enable;
  
  /* 鼠标键盘控制模式下的底盘移动速度目标值 */
  float vx;          //底盘前进后退目标速度
  float vy;          //底盘左右平移目标速度
  float vw;          //底盘旋转速度
  float max_spd;     //运动最大速度
  
  /* 左右按键状态 */
  kb_state_e lk_sta; //左侧按键状态
  kb_state_e rk_sta; //右侧按键状态
  
  uint16_t lk_cnt;
  uint16_t rk_cnt;

  /* 运动模式，键盘控制底盘运动快慢 */
  kb_move_e move_mode;

} km_control_t;

extern km_control_t km;

/**
  * @brief     PC 键盘鼠标数据处理函数
  */
void pc_kb_hook(void);

#endif
