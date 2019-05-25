#ifndef _BSP_DMA_H
#define _BSP_DMA_H

#include <stm32f4xx.h>


extern const u16  DMA_JUDGE_RECEIVE_BUF_SIZE 	;//接受缓存区数据的长度
extern const u16  DMA_JUDGE_SEND_BUF_SIZE 		;//发送缓存区数据长度
extern const u16 DMA_MINIPC_SEND_BUFF_SIZE ;
extern const u16 DMA_MINIPC_REVE_BUFF_SIZE ;
extern u8 g_dma_judge_receive_buff[];//接受缓存区
extern u8 g_dma_judge_send_buff[];
extern u8 g_dma_minipc_send_buff[];
extern u8 g_dma_minipc_receice_buff[];

void BSP_DMA_Init(void);

#endif

