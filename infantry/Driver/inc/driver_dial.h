#ifndef _DRIVER_DIAL_H
#define _DRIVER_DIAL_H

#include <stm32f4xx.h>
#include <driver_control.h>
#include <sys.h>

#define DIAL_2006_SPEED 2000
#define DIAL_3510_SPEED 1500
#define PUSH_SPEED_MIN 0
#define SUPER_DIAL_SPEED_MIN 0
//将2006编码器的值转化成角度值 (10/8191)
#define MOTOR_ECD_TO_DEG 0.00122085215480405322915394945672f
//#define MOTOR_ECD_TO_DEG 0.00231319355647083769734432528642f  //3508  (360/8191/19)

/*拨盘电机-2006*/
#define DIAL_2006_OUTER_P  120
#define DIAL_2006_OUTER_I  0
#define DIAL_2006_OUTER_D  0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
#define DIAL_2006_INNER_P  800
#define DIAL_2006_INNER_I  0.5
#define DIAL_2006_INNER_D  0
#define DIAL_2006_OUTER_MAXOUT				 1000
#define DIAL_2006_OUTER_INTEGRATION_LIMIT	 400
#define DIAL_2006_INNER_MAXOUT				 15000
#define DIAL_2006_INNER_INTEGRATION_LIMIT	 5000

/*拨盘电机-3510*/
#define DIAL_3510_OUTER_P  8
#define DIAL_3510_OUTER_I  0
#define DIAL_3510_OUTER_D  0
#define DIAL_3510_INNER_P  3
#define DIAL_3510_INNER_I  0.1
#define DIAL_3510_INNER_D  0
#define DIAL_3510_OUTER_MAXOUT				 10000
#define DIAL_3510_OUTER_INTEGRATION_LIMIT	 5000
#define DIAL_3510_INNER_MAXOUT				 5000
#define DIAL_3510_INNER_INTEGRATION_LIMIT	 2000

/*大弹丸-直流电机*/
#define PUSH_MOTOR_EN 			PIout(5)	//W
#define PUSH_MOTOR_IN1			PAout(2)	//U
#define PUSH_MOTOR_IN2			PAout(3)	//V

/*大弹丸-拨盘电机*/
#define DIAL_MOTOR_EN 			PIout(2)	//Z
#define DIAL_MOTOR_IN1			PIout(7)	//Y
#define DIAL_MOTOR_IN2			PIout(6)	//X

enum
{
	SIGNLE_SHOOT =1,
	CONTINUOUS_SHOOT =2,
	NOT_SHOOT=3,
};

typedef struct
{
	int8_t single_ready_flag;//单发发射准备标志位
	int8_t over_zero_flag;	//过零处理标志位
	float give_current;
	float last_angle_target;//上次设置的角度值
	
	pid_t moto_speed_pid;
	pid_t moto_angle_pid;
}DialMotor_t;

void dial_init(void);
void singel_shoot(void);
void continuous_shoot(void);
void move_push_motor(int16_t moto);
void move_super_dial_motor(int16_t moto);

extern DialMotor_t g_dial_2006_motor;
extern DialMotor_t g_dial_2006_motor_assist;

#endif

