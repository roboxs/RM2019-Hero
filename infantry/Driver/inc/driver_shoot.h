#ifndef _DRIVER_SHOOT_H
#define _DRIVER_SHOOT_H

#include <stm32f4xx.h>

//typedef struct
//{
//	float ecd;
//}ShootControl_t;

#define FRICMOTOR_PERIOD 0.001f
#define FRICMOTOR_MAX_SPEED FRICMOTOR_LOW_SPEED
#define FRICMOTOR_MIN_SPEED FRICMOTOR_OFF_SPEED

void shoot_init(void);
void shoot_control(void);

#endif
