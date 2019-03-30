#ifndef _TASK_SYSINIT_H
#define _TASK_SYSINIT_H


/*�ײ��ļ�*/
#include <stm32f4xx.h>
#include <stm32f4xx_it.h>
#include <FreeRTOS.h>
#include <task.h>


/*Ӳ����ʼ��*/
#include <bsp_gpio.h>
#include <bsp_uart.h>
#include <bsp_dma.h>
#include <bsp_nvic.h>
#include <bsp_can.h>
#include <bsp_tim.h>
#include <delay.h>

/*����*/
#include <driver_imu.h>
#include <driver_cloudmotor.h>
#include <driver_control.h>
#include <driver_fricmotor.h>
/*DMP*/
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
/*����*/
#include <task_led.h>
#include <task_control.h>
#include <task_imu.h>
#include <task_chassis.h>
#include <task_gimbal.h>

void system_init(void);

#endif
