#ifndef _BSP_DMA_H
#define _BSP_DMA_H

#include <stm32f4xx.h>


extern const u16  RECEIVE_BUF_SIZE 	;//���ܻ��������ݵĳ���
extern const u16  SEND_BUF_SIZE 		;//���ͻ��������ݳ���
extern u8 g_receive_buff[];//���ܻ�����
extern u8 g_send_buff[]   ;

void BSP_DMA_Init(void);

#endif

