#ifndef _TASK_CONTROL_H
#define _TASK_CONTROL_H

#include <stm32f4xx.h>
#include <driver_control.h>
#include <FreeRTOS.h>
#include <task.h>

#define CONTROL_TASK_CYCLE 5


void control_task(void *pvParameters);
static void master_to_slave(u8 * data, u8 len);

#endif 
