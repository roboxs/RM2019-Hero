#include <task_sysinit.h>





void system_init(void)
{
	delay_init(180);
	BSP_GPIO_Init();
	BSP_UART_Init();
	BSP_DMA_Init();
	BSP_CAN_Init();
	BSP_TIM_Init();
	BSP_NVIC_Init();
	
//	pid_init_all();
	//24v 输出 依次上电
    for (uint8_t i = 0; i < 3 ; i++)
    {
        GPIO_SetBits(GPIOH, GPIO_Pin_3 << i);
        delay_us(700);
    }
	
}




