#ifndef _TASK_IMU_H
#define _TASK_IMU_H

#include <stm32f4xx.h>


typedef struct
{
	float pitch;
	float yaw;
	float roll;
	
	short gx;
	short gy;
	short gz;
}Gyroscope_t;




extern Gyroscope_t g_cloud_gyroscope;
extern Gyroscope_t g_chassis_gyroscope;

void imu_task(void *pvParameters);

#endif

