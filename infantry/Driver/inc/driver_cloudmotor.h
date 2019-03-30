#ifndef _DRIVER_CLOUDMOTOR_H
#define _DRIVER_CLOUDMOTOR_H

#include <stm32f4xx.h>


#define COULDCONTROLID 0x1FF

#define YAWCENTER      323
#define PITCHCENTER    75


void could_motor_init(void);
//void control_could_motor(int16_t yaw_iq, int16_t pitch_iq);

#endif


