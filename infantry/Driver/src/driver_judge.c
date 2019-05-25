#include <driver_judge.h>
#include <driver_crc.h>
#include <bsp_dma.h>
#include <user_math.h>
#include <stm32f4xx_it.h>

/** 
	* @brief 裁判系统数据解析
  * 
  */
ext_game_robot_state_t state_data;


HexToFloat_t judge_memory;


void data_judge(void)
{
	u16 i=0;
	for(i=0;i<USART6_RXCnt;i++)
	{
		/*机器人当前状态数据*/
		if(g_dma_judge_receive_buff[i]==DN_REG_ID && (g_dma_judge_receive_buff[i+6]<<8 | g_dma_judge_receive_buff[i+5])==0x201&&
			(verify_crc16_check_sum(&g_dma_judge_receive_buff[i],JUDGE_ROBOT_STATE)==1))//机器人状态
		{
			state_data.robot_id=g_dma_judge_receive_buff[i+7];
		}
	}
	USART6_RXCnt = 0;
}

