#ifndef _DRIVER_FRICMOTOR_H
#define _DRIVER_FRICMOTOR_H

#include <stm32f4xx.h>

#define FRICMOTOR_OFF_SPEED 1000
#define FRICMOTOR_LOW_SPEED 1400
#define FRICMOTOR_HIGH_SPEED 1600

#define SUPER_FRICMOTOR_OFF_SPEED 1000
#define SUPER_FRICMOTOR_LOW_SPEED 1300
#define SUPER_FRICMOTOR_HIGH_SPEED 1370


typedef struct
{
	float period;
	float max;
	float min;
	float in;
	float out;
	float target;
}RampStr_t;//Ð±ÆÂ½á¹¹Ìå

void fricmotor_off(void);
void super_fricmotor_off(void);
void fricmotor_on(uint16_t moto1, uint16_t moto2);
void super_fricmotor_on(uint16_t moto1, uint16_t moto2);
void ramp_init(RampStr_t *ramp_init, float max, float min, float period);
void ramp_calculate(RampStr_t * ramp_cal, float in);

#endif
