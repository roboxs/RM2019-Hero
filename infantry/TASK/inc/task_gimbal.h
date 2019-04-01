#ifndef __TASK_GIMBAL_H
#define __TASK_GIMBAL_H

#include <stm32f4xx.h>
#include <driver_control.h>

//#define GIMBAL_DEBUG_YAW

#define GIMBAL_CONTROL_ID 0X1FF
#define GIMBAL_TASK_CYCLE 10

#define GIMBAL_YAW_MID_ANGLE  86.5f
#define GIMBAL_YAW_MAX_ANGLE  48.0f	 //右
#define GIMBAL_YAW_MIN_ANGLE  130.5f //左

#define GIMBAL_PITCH_MID_ANGLE 150.0f
#define GIMBAL_PITCH_MIN_ANGLE 127.8f
#define GIMBAL_PITCH_MAX_ANGLE 175.0f

/*YAW-过程参数*/
#define YAW_OUTER_P  -80
#define YAW_OUTER_I  -0.1
#define YAW_OUTER_D  0
#define YAW_INNER_P  3
#define YAW_INNER_I  0.1
#define YAW_INNER_D  0
#define YAW_OUTER_MAXOUT				 400
#define YAW_OUTER_INTEGRATION_LIMIT		 100
#define YAW_INNER_MAXOUT				 30000
#define YAW_INNER_INTEGRATION_LIMIT		 30000
/*YAW-初始参数*/
#define YAW_OUTER_P_INIT -150
#define YAW_OUTER_I_INIT -3
#define YAW_OUTER_D_INIT -200
#define YAW_OUTER_MAXOUT_INIT				 3000
#define YAW_OUTER_INTEGRATION_LIMIT_INIT	 3000

/*PITCH-过程参数*/
#define PITCH_OUTER_P  120
#define PITCH_OUTER_I  1
#define PITCH_OUTER_D  200
#define PITCH_INNER_P  1
#define PITCH_INNER_I  0.1
#define PITCH_INNER_D  0
#define PITCH_OUTER_MAXOUT				 3000
#define PITCH_OUTER_INTEGRATION_LIMIT	 3000
#define PITCH_INNER_MAXOUT				 5000
#define PITCH_INNER_INTEGRATION_LIMIT	 5000
/*PITCH-初始参数*/
#define PITCH_OUTER_P_INIT  150
#define PITCH_OUTER_I_INIT  3
#define PITCH_OUTER_D_INIT  400
#define PITCH_OUTER_MAXOUT_INIT				 2000
#define PITCH_OUTER_INTEGRATION_LIMIT_INIT	 2000

typedef struct
{
	float target;
	float angle_measure;
	float speed_measuer;
	int16_t give_cuttent;
}GimbalMotor_t;

typedef struct
{
	pid_t yaw_outer_pid;
	pid_t yaw_inner_pid;
	pid_t pitch_outer_pid;
	pid_t pitch_inner_pid;
	GimbalMotor_t motor_yaw;
	GimbalMotor_t motor_pitch;
	//float gimbal_speed;
	
}GimbalMove_t;




void gimbal_task(void *pvParameters);
static void gimbal_init(GimbalMove_t *gimbal_control_init);
static void gimbal_pid_calculate(GimbalMove_t * gimbal_calculate);
static void control_gimbal_motor(int16_t yaw, int16_t pitch, int16_t dial1, int16_t dial2);
static void gimbal_update(GimbalMove_t *gimbal_update);
static void gimbal_test_yaw(void);


extern GimbalMove_t g_gimbal_control;

#endif
