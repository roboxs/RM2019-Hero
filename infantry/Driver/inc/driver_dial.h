#ifndef _DRIVER_DIAL_H
#define _DRIVER_DIAL_H

#include <stm32f4xx.h>
#include <driver_control.h>

#define DIAL_2006_SPEED 2000
#define DIAL_3510_SPEED 1500
//将2006编码器的值转化成角度值 (10/8191)
#define MOTOR_ECD_TO_DEG 0.00122085215480405322915394945672f

/*拨盘电机-2006*/
#define DIAL_2006_OUTER_P  8
#define DIAL_2006_OUTER_I  0
#define DIAL_2006_OUTER_D  0
#define DIAL_2006_INNER_P  800
#define DIAL_2006_INNER_I  0.5
#define DIAL_2006_INNER_D  0
#define DIAL_2006_OUTER_MAXOUT				 10000
#define DIAL_2006_OUTER_INTEGRATION_LIMIT	 5000
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

enum
{
	SIGNLE_SHOOT =1,
	CONTINUOUS_SHOOT =2,
	NOT_SHOOT=3,
};

typedef struct
{
	int8_t singel_ready_flag;//单发发射准备标志位
	float current_round;//当前编码器的圈数
	float cuttent_ecd;//当前编码器的值
	float give_current;
	
	pid_t moto_speed_pid;
	pid_t moto_angle_pid;
}DialMotor_t;


void dial_pid_calculate(void);
void dial_init(void);

extern DialMotor_t g_dial_2006_motor;

#endif

