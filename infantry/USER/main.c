
#include "task_sysinit.h"


void start_task(void *pvParameters);


TaskHandle_t g_start_task_handler;



int main(void)
{	
	system_init();
	delay_ms(100);
	//�������ȼ� ��ֵԽ�� ���ȼ�Խ��
	xTaskCreate(start_task, 		"start_task"	, 512 , NULL, 2, &g_start_task_handler);

	vTaskStartScheduler();//�����������
	
	while(1);
		
}


void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //�����ٽ���
    
	
//	xTaskCreate(led0_task,	  "led0_task",		128,	 NULL, 1, NULL);
	
	xTaskCreate(imu_task, 	  "imu_task", 		256, 	 NULL, 2, NULL);
	
//	xTaskCreate(control_task, "control_task", 	256, 	 NULL, 2, NULL);
	
//	xTaskCreate(chassis_task, "chassis_task", 	256, 	 NULL, 3, NULL);
	
	xTaskCreate(gimbal_task, "gimbal_task", 	256, 	 NULL, 3, NULL);
	
    vTaskDelete(g_start_task_handler);
    taskEXIT_CRITICAL();            
}



