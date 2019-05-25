#ifndef _DRIVER_SHOOT_H
#define _DRIVER_SHOOT_H

#include <stm32f4xx.h>
#include <task_gimbal.h>

#define SHOOT_CONTROL_ID 0X200

#define FRICMOTOR_PERIOD  0.0001f
#define FRICMOTOR_MAX_SPEED 1600
#define FRICMOTOR_MIN_SPEED 1000

#define SUPER_FRICMOTOR_PERIOD  0.0001f
#define SUPER_FRICMOTOR_MAX_SPEED 1800
#define SUPER_FRICMOTOR_MIN_SPEED 1000

#define CONTINUOUS_SHOOT_TIME 400//连续射击所要求的时间
#define RC_SHOOT_TIME 2000		//遥控连续射击所要求的时间
#define UP_ADD_TIME 80 //高速加速时间

#define BLOCK_TIME 700 //电机堵转时间
#define REVERSE_TIEM 500//电机堵转之后反转时间
#define BLOCK_SPEED -500//电机堵转之后速度

#define BLOCK_TIME_CNS 1000 //连发堵转时间

typedef enum
{
	SHOOT_STOP = 0,
	SHOOT_READY ,
	SHOOT_FINISH,
	SHOOT_SGL,//单发射击模式
	SHOOT_CUS,//连续射击模式
}ShootMode_e;

typedef enum
{
	BULLET_SMALL = 0,
	BULLET_BIG,
}BulletMode_e;


typedef struct
{
	unsigned char fric; //摩擦轮当前的状态
	ShootMode_e mode ;//射击模式
	BulletMode_e mode_blt;//弹丸模式
	unsigned char state;//射击状态
	unsigned char angle_flag;//角度设置标志
	unsigned char block_flag;//堵转标志位
	
	unsigned char press_l;
    unsigned char press_r;
    unsigned char last_press_l;
    unsigned char last_press_r;
    uint16_t press_l_time;
    uint16_t press_r_time;
	
	unsigned char rc_s1;
	unsigned char last_rc_s1;
	uint16_t rc_s1_time;
	
	unsigned char key_press_Q;
	unsigned char last_key_press_Q;
	
	uint32_t run_time;//当前运行的系统时间
	uint32_t cmd_time;//给定角度时系统时间
	uint32_t block_time;//计算堵转时间
}ShootControl_t;

void shoot_init(void);
void shoot_control(void);
static void shoot_data_update(void);
static void shoot_state_update(void);
static void control_shoot_motor(int16_t dial);


extern ShootControl_t shoot;

#endif
