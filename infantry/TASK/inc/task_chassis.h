#ifndef _TASK_CHASSIS_H
#define _TASK_CHASSIS_H
#include <stm32f4xx.h>
#include <driver_control.h>

#define CHASSIS_CONTROL_ID 0x200
#define CHASSIS_TASK_CYCLE 2 //������������2ms
#define CHASSIS_MAX_SPEED 3000//�����˶�������ٶ�

#define CHASSIS_VX_MAX_SPEED 2500//X���������ٶ�
#define CHASSIS_VY_MAX_SPEED 2200//Y��������ٶ�
#define CHASSIS_RC_SEN 3//����ң�ص�������
#define CHASSIS_KEY_SEN 0.7//������������
#define CHASSIS_VW_RC_SEN 2//ң����תʱ��������

/*�����ƶ�PID��ʼ��*/
#define CHASSIS_PID_KP 25
#define CHASSIS_PID_KI 3
#define CHASSIS_PID_KD 15
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

/*������תPID��ʼ��*/
#define CHASSIS_ANGLE_KP 200
#define CHASSIS_ANGLE_KI 0
#define CHASSIS_ANGLE_KD 50
#define CHASSIS_ANGLE_MAX_PID_OUTER 2500
#define CHASSIS_ANGLE_OUTER_INTEGRATION_LIMIT 5000


typedef enum
{
	CHASSIS_NO_MOVE,//���̾�ֹ״̬
	CHASSIS_FOLLOW_GIMBAL,//���̸�����̨ģʽ
	CHASSIS_FOLLOW_CHASSIS,//�������
	CHASSIS_NO_FOLLOW,//���̲�����Ƕ�ֵ
}ChassisMode_e;

typedef struct
{
	float target;
	float measure;
	float give_current;
}ChassisMotor_t;


typedef struct
{
	ChassisMode_e mode;
	ChassisMotor_t chassis_motor[4];//���̵��
	pid_t motor_speed_pid[4];		//�����ٶ�PID
	pid_t chassis_angle_pid;
	
	float vx_measure;
	float vy_measure;
	float vw_measure;
	float vx_target;
	float vy_target;
	float vw_target;
	
}ChassisMove_t;

//enum
//{
//	CHASSIS_NO_FOLLOW_GIMBAL=1, //���̲�������̨
//	CHASSIS_FOLLOW_GIMBAL=2,//���̸�����̨
//	CHASSIS_ROCK=3,//��̨�̶�,����ҡ��
//};

void chassis_task(void *pvParameters);
static void abs_limit(float *object, float abs_max);
static void chassis_init(ChassisMove_t *chassis_move);
static void chassis_set_mode(ChassisMove_t * chassis);
static void chassis_set_target(ChassisMove_t * chassis_move);
static void chassis_control_set(float *vx, float *vy, float *vw, ChassisMove_t * chassis);
static void chassis_rc_to_target(float *vx, float *vy);
static void mecanum_calculate(float vx,float vy,float vw, short *speed);
static void chassis_updata_data(ChassisMove_t *chassis_updata);
static void chassis_pid_calculate(ChassisMove_t *chassis_move);
static void control_chassis_motor(int16_t moto1,int16_t moto2,int16_t moto3,int16_t moto4);


extern ChassisMove_t g_chassis_move;
#endif

