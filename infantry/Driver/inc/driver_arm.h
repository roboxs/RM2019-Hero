#ifndef _DRIVER_CHASSIS_H
#define _DRIVER_CHASSIS_H

#include <stm32f4xx.h>
#include <driver_control.h>

#define CONTROLCHASSIS		0x200
#define MAX_SPEED_CHASSIS 	900

enum
{
	LF=0,
	RF=1,
	LB=2,
	RB=3,
};


typedef struct
{
	int16_t target;
	int16_t measure;
}WheelMotor_s;


typedef struct
{
	float target_vx;
	float target_vy;
	float target_vw;
	WheelMotor_s chassis_motor_lf;
	WheelMotor_s chassis_motor_rf;
	WheelMotor_s chassis_motor_lb;
	WheelMotor_s chassis_motor_rb;
}ChassisStucture_s;


extern ChassisStucture_s g_chassis;


void set_chassis_speed(float xspeed ,float yspeed,float yaw);
static void mecanum_calculate(float vx,float vy,float vw, short *speed);
void chassis_movement(void);
void control_chassis_motor(int16_t chassis_motor_1,int16_t chassis_motor_2,int16_t chassis_motor_3,int16_t chassis_motor_4);
void test_chassis_motor(void);

#endif 
