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

#define CONTINUOUS_SHOOT_TIME 400//���������Ҫ���ʱ��
#define RC_SHOOT_TIME 2000		//ң�����������Ҫ���ʱ��
#define UP_ADD_TIME 80 //���ټ���ʱ��

#define BLOCK_TIME 700 //�����תʱ��
#define REVERSE_TIEM 500//�����ת֮��תʱ��
#define BLOCK_SPEED -500//�����ת֮���ٶ�

#define BLOCK_TIME_CNS 1000 //������תʱ��

typedef enum
{
	SHOOT_STOP = 0,
	SHOOT_READY ,
	SHOOT_FINISH,
	SHOOT_SGL,//�������ģʽ
	SHOOT_CUS,//�������ģʽ
}ShootMode_e;

typedef enum
{
	BULLET_SMALL = 0,
	BULLET_BIG,
}BulletMode_e;


typedef struct
{
	unsigned char fric; //Ħ���ֵ�ǰ��״̬
	ShootMode_e mode ;//���ģʽ
	BulletMode_e mode_blt;//����ģʽ
	unsigned char state;//���״̬
	unsigned char angle_flag;//�Ƕ����ñ�־
	unsigned char block_flag;//��ת��־λ
	
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
	
	uint32_t run_time;//��ǰ���е�ϵͳʱ��
	uint32_t cmd_time;//�����Ƕ�ʱϵͳʱ��
	uint32_t block_time;//�����תʱ��
}ShootControl_t;

void shoot_init(void);
void shoot_control(void);
static void shoot_data_update(void);
static void shoot_state_update(void);
static void control_shoot_motor(int16_t dial);


extern ShootControl_t shoot;

#endif
