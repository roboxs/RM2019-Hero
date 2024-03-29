#ifndef _TASK_SYSINIT_H
#define _TASK_SYSINIT_H


/*底层文件*/
#include <stm32f4xx.h>
#include <stm32f4xx_it.h>
#include <FreeRTOS.h>
#include <task.h>


/*硬件初始化*/
#include <bsp_gpio.h>
#include <bsp_uart.h>
#include <bsp_dma.h>
#include <bsp_nvic.h>
#include <bsp_can.h>
#include <bsp_tim.h>
#include <delay.h>

/*驱动*/
#include <driver_imu.h>
#include <driver_cloudmotor.h>
#include <driver_control.h>
#include <driver_fricmotor.h>
#include <driver_dial.h>
/*DMP*/
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
/*任务*/
#include <task_led.h>
#include <task_control.h>
#include <task_imu.h>
#include <task_chassis.h>
#include <task_gimbal.h>
#include <task_vision.h>

void system_init(void);

#endif

