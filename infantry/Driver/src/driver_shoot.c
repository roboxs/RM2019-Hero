#include <driver_shoot.h>
#include <driver_fricmotor.h>
#include <driver_dial.h>

RampStr_t fric_ramp1,fric_ramp2;

int count_t=0;
void shoot_init(void)
{
	ramp_init(&fric_ramp1, FRICMOTOR_MAX_SPEED, FRICMOTOR_MIN_SPEED, FRICMOTOR_PERIOD);
	ramp_init(&fric_ramp2, FRICMOTOR_MAX_SPEED, FRICMOTOR_MIN_SPEED, FRICMOTOR_PERIOD);
}

void shoot_control(void)
{
	dial_pid_calculate();
	
	ramp_calculate(& fric_ramp1, FRICMOTOR_LOW_SPEED);
	if(fric_ramp1.out >= fric_ramp1.max)
	{
		ramp_calculate(& fric_ramp2, FRICMOTOR_LOW_SPEED);
	}
	
	fricmotor_on(fric_ramp1.out , fric_ramp2.out);
	
}
