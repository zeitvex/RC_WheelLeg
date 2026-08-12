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
 


#ifndef __START_UP_H__
#define __START_UP_H__

#include "rm_hal_lib.h"
#include "cmsis_os.h"
#include "lingzu_task.h"
#include "uart_task.h"
/**
  * @brief     最多支持 5 个任务函数的配置和开启
  * @usage     首先开启 USER_TASKx 的宏定义，然后在 USER_TASKx 后添加需要开启的任务函数名
  */
#define USER_TASK1 lingzu_task
//#define USER_TASK2 gimbal_task
#define USER_TASK3 study_task
#define USER_TASK4 uart_task
//#define USER_TASK5

/**
  * @brief     在任务函数执行前运行，可以用来初始化任务中用到的 IO 端口，
  *            配置、开启外界硬件设备，注册硬件设备的接收回调函数
  */
void init_setup(void);

/**
  * @brief     开启任务相关函数，在这里定义不需要改动
  */
void sys_start_task(void);

#endif
