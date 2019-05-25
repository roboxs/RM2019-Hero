#include <driver_fricmotor.h>
#include <driver_control.h>
#include <task_led.h>


void fricmotor_off(void)
{
	TIM_SetCompare1(TIM5,FRICMOTOR_OFF_SPEED);
	TIM_SetCompare2(TIM5,FRICMOTOR_OFF_SPEED);
}

void super_fricmotor_off(void)
{
	TIM_SetCompare3(TIM5,FRICMOTOR_OFF_SPEED);
	TIM_SetCompare4(TIM5,FRICMOTOR_OFF_SPEED);
}

void fricmotor_on(uint16_t moto1, uint16_t moto2)
{
	TIM_SetCompare1(TIM5,moto1);
	TIM_SetCompare2(TIM5,moto2);
}

void super_fricmotor_on(uint16_t moto1, uint16_t moto2)
{
	TIM_SetCompare3(TIM5,moto1);
	TIM_SetCompare4(TIM5,moto2);
}

//Ð±ÆÂ³õÊ¼»¯
void ramp_init(RampStr_t *ramp_init, float max, float min, float period)
{
	ramp_init->max = max;
	ramp_init->min = min;
	ramp_init->period = period;
	ramp_init->in = 0;
	ramp_init->out =min;
}



//Ð±ÆÂ¼ÆËã
void ramp_calculate(RampStr_t * ramp_cal, float in)
{
	ramp_cal->in = in;
	ramp_cal->out += ramp_cal->in * ramp_cal->period;
	
	if(ramp_cal->out >= ramp_cal->in) 
		ramp_cal->out =ramp_cal->in;
	
	if(ramp_cal->out > ramp_cal->max)
	{
		ramp_cal->out =ramp_cal->max;
	}
	else if(ramp_cal->out < ramp_cal->min)
	{
		ramp_cal->out = ramp_cal->min;
	}
}

