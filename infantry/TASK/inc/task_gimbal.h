#ifndef __TASK_GIMBAL_H
#define __TASK_GIMBAL_H

#include <stm32f4xx.h>
#include <driver_control.h>

//#define GIMBAL_DEBUG_YAW
//#define GIMBAL_DEBUG_PITCH

#define GIMBAL_CONTROL_ID 0X1FF//CAN指令
#define GIMBAL_TASK_CYCLE 1 //云台任务更新周期

#define RC_DEADBAND 50  //遥控的输入死区

#define GIMBAL_YAW_MID_ANGLE  266.0f
#define GIMBAL_YAW_MAX_ANGLE  230.0f //右
#define GIMBAL_YAW_MIN_ANGLE  300.0f //左

#define GIMBAL_YAW_MID_ANGLE_GYRO  0.0f
#define GIMBAL_YAW_MAX_ANGLE_GYRO  -360.0f //右
#define GIMBAL_YAW_MIN_ANGLE_GYRO  360.0f //左 

#define GIMBAL_PITCH_MID_ANGLE 121.5f
#define GIMBAL_PITCH_MIN_ANGLE 97.0f//上
#define GIMBAL_PITCH_MAX_ANGLE 137.7f//下

#define YAW_RC_SEN 0.0001f		//yaw轴遥控的灵敏度
#define PITCH_RC_SEN -0.00005f 	//pitch轴遥控的灵敏度

#define YAW_MOUSE_SEN 0.0015f 	//yaw轴鼠标的灵敏度
#define PITCH_MOUSE_SEN 0.0007f //pitch轴鼠标的灵敏度


/*YAW-过程参数*/
#define YAW_OUTER_P  150
#define YAW_OUTER_I  0
#define YAW_OUTER_D  0
#define YAW_INNER_P  6
#define YAW_INNER_I  0.08
#define YAW_INNER_D  10
#define YAW_OUTER_MAXOUT				 400
#define YAW_OUTER_INTEGRATION_LIMIT		 100
#define YAW_INNER_MAXOUT				 4000
#define YAW_INNER_INTEGRATION_LIMIT		 3000
/*YAW-初始参数*/
#define YAW_OUTER_P_INIT -500
#define YAW_OUTER_I_INIT -0
#define YAW_OUTER_D_INIT -12000
#define YAW_OUTER_MAXOUT_INIT				 5000
#define YAW_OUTER_INTEGRATION_LIMIT_INIT	 3000

/*PITCH-过程参数*/
#define PITCH_OUTER_P  -180
#define PITCH_OUTER_I  0
#define PITCH_OUTER_D  0
#define PITCH_INNER_P  5
#define PITCH_INNER_I  0.08
#define PITCH_INNER_D  3
#define PITCH_OUTER_MAXOUT				 4000
#define PITCH_OUTER_INTEGRATION_LIMIT	 3000
#define PITCH_INNER_MAXOUT				 5000
#define PITCH_INNER_INTEGRATION_LIMIT	 5000
/*PITCH-初始参数*/
#define PITCH_OUTER_P_INIT  -300
#define PITCH_OUTER_I_INIT  -0.8
#define PITCH_OUTER_D_INIT  -3000
#define PITCH_OUTER_MAXOUT_INIT				 5000
#define PITCH_OUTER_INTEGRATION_LIMIT_INIT	 2000

//云台当前的模式
typedef enum 
{
	GIMBAL_MOTOR_GYROSCOPE,    //电机陀螺仪角度控制
    GIMBAL_MOTOR_ENCONDE, //电机编码值角度控制
}GimbalMode_e;


//云台当前的状态
typedef enum 
{
	GIMBAL_INIT,//初始化状态
}GimbalState_e;


typedef struct
{
	float target;
	float angle_measure;
	float last_angle_measure;
	float speed_measure;
	int16_t give_current;
}GimbalMotor_t;

typedef struct
{
	int8_t over_zero_flag;	//过零处理标志位
	int8_t over_zero_flag_rel;	//反向过零处理标志位
	
	pid_t yaw_outer_pid;
	pid_t yaw_inner_pid;
	pid_t pitch_outer_pid;
	pid_t pitch_inner_pid;
	GimbalMode_e mode;
	GimbalMotor_t motor_yaw;
	GimbalMotor_t motor_pitch;
	
	int8_t init_finish_flag;//初始化完成标志位
	uint32_t init_time;//初始化时钟
	uint32_t run_time;//运行时钟
}GimbalMove_t;




void gimbal_task(void *pvParameters);
static void gimbal_init(GimbalMove_t *gimbal_control_init);
static void gimbal_set_control(GimbalMove_t *gimbal_set);
static void gimbal_set_target(float *add_yaw, float*add_pitch);
static void gimbal_pid_calculate(GimbalMove_t * gimbal_calculate);
static void control_gimbal_motor(int16_t yaw, int16_t pitch, int16_t dial1, int16_t dial2);
static void gimbal_update(GimbalMove_t *gimbal_update);
static void gimbal_motor_angle_limit(GimbalMove_t *motor_limit,float yaw_add, float pitch_add);
static void gimbal_motor_angle_limit_gyro(GimbalMove_t *motor_limit,float yaw_add, float pitch_add);


static void gimbal_test_yaw(GimbalMove_t * gimbal_calculate);
static void gimbal_test_pitch(GimbalMove_t * gimbal_calculate);


extern GimbalMove_t g_gimbal_control;

#endif
