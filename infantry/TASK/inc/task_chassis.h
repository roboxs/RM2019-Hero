#ifndef _TASK_CHASSIS_H
#define _TASK_CHASSIS_H
#include <stm32f4xx.h>
#include <driver_control.h>

#define CHASSIS_CONTROL_ID 0x200
#define CHASSIS_TASK_CYCLE 2 //底盘任务周期2ms
#define CHASSIS_MAX_SPEED 3000

/*底盘移动PID初始化*/
#define CHASSIS_PID_KP 4
#define CHASSIS_PID_KI 30
#define CHASSIS_PID_KD 0
#define CHASSIS_MAX_PID_OUTER 6000
#define CHASSIS_OUTER_INTEGRATION_LIMIT 8000

#define CHASSIS_VX_KP 1.5
#define CHASSIS_VX_KI 0.1
#define CHASSIS_VX_KD 0
#define CHASSIS_VX_MAX_PID_OUTER 3000
#define CHASSIS_VX_OUTER_INTEGRATION_LIMIT 5000

#define CHASSIS_VY_KP 1.5
#define CHASSIS_VY_KI 0.1
#define CHASSIS_VY_KD 0
#define CHASSIS_VY_MAX_PID_OUTER 3000
#define CHASSIS_VY_OUTER_INTEGRATION_LIMIT 5000

/*底盘旋转PID初始化*/
#define CHASSIS_VW_KP 40
#define CHASSIS_VW_KI 0.1
#define CHASSIS_VW_KD 0
#define CHASSIS_VW_MAX_PID_OUTER 100
#define CHASSIS_VW_OUTER_INTEGRATION_LIMIT 400


typedef struct
{
	float target;
	float measure;
	float give_current;
}ChassisMotor_t;


typedef struct
{
	u8 chassis_mode;				//底盘模式
	ChassisMotor_t chassis_motor[4];//底盘电机
	pid_t motor_speed_pid[4];		//底盘速度PID
	pid_t chassis_vx_pid;
	pid_t chassis_vy_pid;
	pid_t chassis_vw_pid;
	
	float vx_measure;
	float vy_measure;
	float vw_measure;
	float vx_target;
	float vy_target;
	float vw_target;
	
}ChassisMove_t;

enum
{
	CHASSIS_RC_MODE = 0x01,
	CHASSIS_KEYBOARD_MODE=0x02,
};

enum
{
	CHASSIS_NO_FOLLOW_GIMBAL=1, //底盘不跟随云台
	CHASSIS_FOLLOW_GIMBAL=2,//底盘跟随云台
	CHASSIS_ROCK=3,//云台固定,底盘摇摆
};

void chassis_task(void *pvParameters);
static void abs_limit(float *object, float abs_max);
static void chassis_init(ChassisMove_t *chassis_move);
static void chassis_set_target(ChassisMove_t * chassis_move);
static void mecanum_calculate(float vx,float vy,float vw, short *speed);
static void chassis_updata_data(ChassisMove_t *chassis_updata);
static void chassis_pid_calculate(ChassisMove_t *chassis_move);
static void control_chassis_motor(int16_t moto1,int16_t moto2,int16_t moto3,int16_t moto4);


extern ChassisMove_t g_chassis_move;
#endif

