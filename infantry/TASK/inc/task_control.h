#ifndef _TASK_CONTROL_H
#define _TASK_CONTROL_H

#include <stm32f4xx.h>
#include <FreeRTOS.h>
#include <task.h>

#define DIAL_CONTROL_ID 0X1FF
#define CONTROL_TASK_CYCLE 1


void control_task(void *pvParameters);

#endif 
