#include <task_sysinit.h>





void system_init(void)
{
	delay_init(180);
	BSP_GPIO_Init();
	BSP_UART_Init();
	BSP_DMA_Init();
	BSP_CAN_Init();
	BSP_TIM_Init();
//	BSP_IMU_Init();
	BSP_NVIC_Init();
	
	pid_init_all();

//	while(mpu_dmp_init());
	POWER_CLAMP_CTRL = 0;
	POWER_PUSH_CTRL  = 0;
	
}




