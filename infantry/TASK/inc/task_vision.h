#ifndef _TASK_VISION_H
#define _TASK_VISION_H
#include <FreeRTOS.h>
#include <task.h>
#include <stm32f4xx.h>

#define DN_REG_ID 0xA5 //帧头


typedef enum
{
	VISION_SMALL,
	VISION_BIG,
	VISION_CLOSE,
}VisionState_e;


typedef struct
{
	float get_target_angle_yaw;
	float last_get_target_angle_yaw;
	float get_target_angle_pitch;
	float last_get_target_angle_pitch;
	
	float send_motor_angle_yaw;
	float send_motor_angle_pitch;
	
}VisionData_t;


void vision_task(void *pvParameters);
void vision_receive_data(void);
void vision_send_data(u8 cmd , float *yaw, float *pitch);

extern TaskHandle_t xHandleTaskPCParse;
extern VisionData_t g_vision_data;
extern VisionState_e vision_state;
#endif
