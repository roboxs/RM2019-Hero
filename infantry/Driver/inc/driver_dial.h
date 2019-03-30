#ifndef _DRIVER_DIAL_H
#define _DRIVER_DIAL_H

#include <stm32f4xx.h>

#define DIAL_2006_SPEED 2000
#define DIAL_3510_SPEED 1500

enum
{
	SIGNLE_SHOOT =1,
	CONTINUOUS_SHOOT =2,
	NOT_SHOOT=3,
};

void dial_pid_calculate(void);


#endif

