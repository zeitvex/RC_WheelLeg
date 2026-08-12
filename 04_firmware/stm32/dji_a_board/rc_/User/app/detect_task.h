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
 



#include "stm32f4xx_hal.h"

#ifndef __DETECT_TASK_H__
#define __DETECT_TASK_H__

typedef enum 
{
  DEVICE_NORMAL = 0,
  CHASSIS_M1_OFFLINE,
  CHASSIS_M2_OFFLINE,
  CHASSIS_M3_OFFLINE,
  CHASSIS_M4_OFFLINE,
  REMOTE_CTRL_OFFLINE,
  GIMBAL_YAW_OFFLINE,
  GIMBAL_PIT_OFFLINE,
  TRIGGER_MOTO_OFFLINE,
  ERROR_LIST_LENGTH,
} err_id_e;

typedef __packed struct
{
  volatile uint32_t last_time;
  volatile uint32_t err_exist : 1;   //1 = err_exist, 0 = everything ok
  volatile uint32_t enable : 1; 
  volatile uint32_t warn_pri : 6;    //priority
  volatile uint32_t delta_time : 16; //time interval last
  volatile uint32_t set_timeout : 16;
} offline_dev_t;

typedef __packed struct
{
  volatile offline_dev_t *err_now;
  volatile offline_dev_t  err_list[ERROR_LIST_LENGTH];
  err_id_e err_id;
} glb_err_type_t;

void global_err_detector_init(void);
void err_detector_hook(int err_id);
void detect_task(const void* argu);

void module_offline_callback(void);

extern glb_err_type_t glb_err;

#endif
