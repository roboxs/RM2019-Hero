#include <task_led.h>
#include <delay.h>



void led0_task(void *pvParameters)
{
	u32 last_wake_time = xTaskGetTickCount();
	
	  while(1)
    {
        LED1=!LED1;
        vTaskDelayUntil(&last_wake_time , 500);
    }
}

void led1_task(void *pvParameters)
{
    while(1)
    {
        LED1=!LED1;
        delay_ms(10);
    }
}
