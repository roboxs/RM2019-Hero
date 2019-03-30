#include <delay.h>
#include <task_control.h>
#include <task_led.h>
#include <driver_control.h>
#include <driver_fricmotor.h>
#include <driver_dbus.h>

void control_task(void *pvParameters)
{
	while(1)
	{
		
		vTaskDelay(CONTROL_TASK_CYCLE);
	}
}





