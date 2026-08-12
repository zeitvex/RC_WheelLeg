#ifndef __EXECUTE_TASK_H__
#define __EXECUTE_TASK_H__

#include "stm32f4xx_hal.h"

void execute_task(const void* argu);

void test_moto_init(void);

void right_moto_init(void);
void left_moto_init(void);

void moto_control(void);
void set_test_motor_current_right(int16_t test_moto_current[]);
void set_test_motor_current_left(int16_t test_moto_current[]);
#endif

