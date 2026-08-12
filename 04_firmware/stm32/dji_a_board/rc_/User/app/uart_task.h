#ifndef __UART_TASK_H__
#define __UART_TASK_H__

#include "FreeRTOS.h"
#include "queue.h"

extern QueueHandle_t led_control_queue;

void uart_task(const void* argu);

#endif
