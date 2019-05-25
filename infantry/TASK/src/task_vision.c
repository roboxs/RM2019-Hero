#include <FreeRTOS.h>
#include <task.h>
#include <task_vision.h>
#include <bsp_dma.h>
#include <driver_crc.h>
#include <user_math.h>
#include <task_imu.h>
#include <task_gimbal.h>

TaskHandle_t xHandleTaskPCParse;
VisionData_t g_vision_data;
HexToFloat_t vision_memory;
HexToFloat_t vision_send_memory;
VisionState_e vision_state;

void vision_task(void *pvParameters)
{

	while(1)
	{
		vision_receive_data();
		vTaskDelay(5);
	}
}

void vision_receive_data(void)
{
	//更新数据
	g_vision_data.last_get_target_angle_yaw = g_vision_data.get_target_angle_yaw;
	g_vision_data.last_get_target_angle_pitch = g_vision_data.get_target_angle_pitch;
	if(g_dma_minipc_receice_buff[0] == DN_REG_ID && g_dma_minipc_receice_buff[1] == 0X01 &&
		verify_crc8_check_sum(g_dma_minipc_receice_buff,11))
	{
		;
	}
	else if(g_dma_minipc_receice_buff[0] == DN_REG_ID && g_dma_minipc_receice_buff[1] == 0X02&&
		verify_crc8_check_sum(g_dma_minipc_receice_buff,11))
	{
		;
	}
	else if(g_dma_minipc_receice_buff[0] == DN_REG_ID && g_dma_minipc_receice_buff[1] == 0X03&&
		verify_crc8_check_sum(g_dma_minipc_receice_buff,11))
	{
		//yaw轴数据处理
		//g_vision_data.last_get_target_angle_yaw = g_vision_data.get_target_angle_yaw;
		vision_memory.hex_data[3] = g_dma_minipc_receice_buff[5];
		vision_memory.hex_data[2] = g_dma_minipc_receice_buff[4];
		vision_memory.hex_data[1] = g_dma_minipc_receice_buff[3];
		vision_memory.hex_data[0] = g_dma_minipc_receice_buff[2];
		g_vision_data.get_target_angle_yaw = vision_memory.float_data;
		//pitch轴数据处理
		vision_memory.hex_data[3] = g_dma_minipc_receice_buff[9];
		vision_memory.hex_data[2] = g_dma_minipc_receice_buff[8];
		vision_memory.hex_data[1] = g_dma_minipc_receice_buff[7];
		vision_memory.hex_data[0] = g_dma_minipc_receice_buff[6];
		g_vision_data.get_target_angle_pitch = vision_memory.float_data;
	}
	
}

void vision_send_data(u8 cmd , float *yaw, float *pitch)
{
	u8 dwLength;
	if(cmd==0x01)   //发送目标装甲大小信息
	{
		dwLength=4;
		g_dma_minipc_send_buff[0]=0XA6;
		g_dma_minipc_send_buff[1]=cmd;
		
		if(vision_state == VISION_SMALL)
		{
			g_dma_minipc_send_buff[2]=0x01;
		}
		else if(vision_state==VISION_BIG)
		{
			g_dma_minipc_send_buff[2]=0x02;
		}
		else if(vision_state==VISION_CLOSE)
		{
			g_dma_minipc_send_buff[2]=0x05;
		}
		
		append_crc8_check_sum(g_dma_minipc_send_buff,2+1+1);
	}

	if(cmd==0X03)   //发送云台角度数据
	{
		dwLength=11;
		g_dma_minipc_send_buff[0]=0XA6;
		g_dma_minipc_send_buff[1]=cmd;
		
		vision_send_memory.float_data= *yaw;
		g_dma_minipc_send_buff[2] = vision_send_memory.hex_data[0];
		g_dma_minipc_send_buff[3] = vision_send_memory.hex_data[1];
		g_dma_minipc_send_buff[4] = vision_send_memory.hex_data[2];
		g_dma_minipc_send_buff[5] = vision_send_memory.hex_data[3];
		
        vision_send_memory.float_data= *pitch;
		g_dma_minipc_send_buff[6]= vision_send_memory.hex_data[0];
		g_dma_minipc_send_buff[7]= vision_send_memory.hex_data[1];
		g_dma_minipc_send_buff[8]= vision_send_memory.hex_data[2];
		g_dma_minipc_send_buff[9]= vision_send_memory.hex_data[3];
		
		
		append_crc8_check_sum(g_dma_minipc_send_buff,2+4+4+1);
	}
	DMA_Cmd(DMA1_Stream0, DISABLE);
	while (DMA_GetCmdStatus(DMA1_Stream0)); 
	DMA_ClearFlag(DMA1_Stream0,DMA_FLAG_TCIF0|DMA_FLAG_HTIF0|DMA_FLAG_TEIF0|DMA_FLAG_DMEIF0|DMA_FLAG_FEIF0);                                                                                                 
	DMA_ClearITPendingBit(DMA1_Stream0,DMA_IT_TEIF0|DMA_IT_HTIF0|DMA_IT_TCIF0|DMA_IT_DMEIF0|DMA_IT_FEIF0);
	DMA_SetCurrDataCounter(DMA1_Stream0,dwLength);
	DMA_Cmd(DMA1_Stream0, ENABLE);
}

