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
 



#ifndef __CALIBRATE_H__
#define __CALIBRATE_H__

#include "stm32f4xx_hal.h"

#define CALIED_FLAG 0x55

/**
  * @brief     云台校准数据结构体
  */
typedef __packed struct
{
  int16_t yaw_offset;
  int16_t pit_offset;
  uint8_t cali_cmd;
  uint8_t calied_flag;
} gimbal_cali_t;
/**
  * @brief     全局校准数据结构体
  */
typedef __packed struct
{
  uint8_t       saved_flag;       //校准数据保存标志
  uint32_t      firmware_version; //固件版本号
  gimbal_cali_t gimbal_cali_data; //云台校准数据
} global_cali_t;

/**
  * @brief     从用户 flash 中读取校准数据
  */
void read_cali_data(void);
/**
  * @brief     云台校准数据保存函数
  */
void gimbal_cali_hook(void);

/* 全局校准数据结构体 */
extern global_cali_t glb_cali_data;

#endif
