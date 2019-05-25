#include <bsp_dma.h>
#include <driver_dbus.h>

const u16  DMA_JUDGE_RECEIVE_BUF_SIZE = 1000	;//���ܻ��������ݵĳ���
const u16  DMA_JUDGE_SEND_BUF_SIZE 	= 1000	;//���ͻ��������ݳ���
const u16 DMA_MINIPC_SEND_BUFF_SIZE = 30;
const u16 DMA_MINIPC_REVE_BUFF_SIZE = 30;

u8 g_dma_judge_receive_buff[DMA_JUDGE_RECEIVE_BUF_SIZE]={0};//���ܻ�����
u8 g_dma_judge_send_buff[DMA_JUDGE_SEND_BUF_SIZE]={0};

u8 g_dma_minipc_send_buff[DMA_MINIPC_SEND_BUFF_SIZE]={0};//�Ӿ����ͻ���
u8 g_dma_minipc_receice_buff[DMA_MINIPC_REVE_BUFF_SIZE]={0};//�Ӿ����ܻ���



void BSP_DMA_Init(void)
{
	DMA_InitTypeDef DMA_Structure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
	
	
											/*USART6->DMA��������*/
	DMA_DeInit(DMA2_Stream7);
	while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE){}//�ȴ�DMA������ 
	
	DMA_Structure.DMA_Channel 					= DMA_Channel_5;  //ͨ��ѡ��
	DMA_Structure.DMA_PeripheralBaseAddr 		= (u32)&USART6->DR;//DMA�����ַ
	DMA_Structure.DMA_Memory0BaseAddr 			= (u32)g_dma_judge_send_buff;//DMA �洢��0��ַ
	DMA_Structure.DMA_DIR 						= DMA_DIR_MemoryToPeripheral;//�洢��������ģʽ
	DMA_Structure.DMA_BufferSize 				= DMA_JUDGE_SEND_BUF_SIZE;//���ݴ����� 
	DMA_Structure.DMA_PeripheralInc 			= DMA_PeripheralInc_Disable;//���������ģʽ
	DMA_Structure.DMA_MemoryInc 				= DMA_MemoryInc_Enable;//�洢������ģʽ
	DMA_Structure.DMA_PeripheralDataSize 		= DMA_PeripheralDataSize_Byte;//�������ݳ���:8λ
	DMA_Structure.DMA_MemoryDataSize 			= DMA_MemoryDataSize_Byte;//�洢�����ݳ���:8λ
	DMA_Structure.DMA_Mode 						= DMA_Mode_Normal;// ʹ����ͨģʽ 
	DMA_Structure.DMA_Priority 					= DMA_Priority_Medium;//�е����ȼ�
	DMA_Structure.DMA_FIFOMode 					= DMA_FIFOMode_Disable;         
	DMA_Structure.DMA_FIFOThreshold 			= DMA_FIFOThreshold_Full;
	DMA_Structure.DMA_MemoryBurst 				= DMA_MemoryBurst_Single;//�洢��ͻ�����δ���
	DMA_Structure.DMA_PeripheralBurst 			= DMA_PeripheralBurst_Single;//����ͻ�����δ���
	DMA_Init(DMA2_Stream7, &DMA_Structure);
	DMA_Cmd(DMA2_Stream7,ENABLE);	
		
											/*USART6->DMA��������*/
	DMA_DeInit(DMA2_Stream2);
	while (DMA_GetCmdStatus(DMA2_Stream2) != DISABLE){}//�ȴ�DMA������ 
		
	DMA_Structure.DMA_BufferSize				= DMA_JUDGE_RECEIVE_BUF_SIZE;
	DMA_Structure.DMA_Channel					= DMA_Channel_5;
	DMA_Structure.DMA_DIR						= DMA_DIR_PeripheralToMemory;//���赽�洢��Ԫ
	DMA_Structure.DMA_FIFOMode					= DMA_FIFOMode_Disable;
	DMA_Structure.DMA_FIFOThreshold				= DMA_FIFOThreshold_Full;
	DMA_Structure.DMA_Memory0BaseAddr			= (u32)g_dma_judge_receive_buff;
	DMA_Structure.DMA_MemoryBurst				= DMA_MemoryBurst_Single;//���浥Ԫͻ�����δ���
	DMA_Structure.DMA_MemoryDataSize			= DMA_MemoryDataSize_Byte;//8λ
	DMA_Structure.DMA_MemoryInc					= DMA_MemoryInc_Enable;//�洢��Ԫ��������
	DMA_Structure.DMA_Mode						= DMA_Mode_Normal;
	DMA_Structure.DMA_PeripheralBaseAddr		= (u32)&USART6->DR;
	DMA_Structure.DMA_PeripheralBurst			= DMA_PeripheralBurst_Single;
	DMA_Structure.DMA_PeripheralDataSize		= DMA_PeripheralDataSize_Byte;//8λ
	DMA_Structure.DMA_PeripheralInc				= DMA_PeripheralInc_Disable;//�����ַ�����ر�
	DMA_Structure.DMA_Priority					= DMA_Priority_Medium;//DMA�ж����ȼ�
	DMA_Init(DMA2_Stream2,&DMA_Structure);//ʹ��DMA2��������2 ��USART6_RX����
	DMA_Cmd(DMA2_Stream2,ENABLE);	
	
		
											/*DBUS->DMA��������*/
	DMA_DeInit(DMA2_Stream5);
	DMA_Structure.DMA_Channel 					= DMA_Channel_4;
	DMA_Structure.DMA_PeripheralBaseAddr 		= (uint32_t)&(USART1->DR);
	DMA_Structure.DMA_Memory0BaseAddr 			= (uint32_t)g_sbus_rx_buffer;
	DMA_Structure.DMA_DIR 						= DMA_DIR_PeripheralToMemory;
	DMA_Structure.DMA_BufferSize 				= 18;
	DMA_Structure.DMA_PeripheralInc 			= DMA_PeripheralInc_Disable;
	DMA_Structure.DMA_MemoryInc 				= DMA_MemoryInc_Enable;
	DMA_Structure.DMA_PeripheralDataSize 		= DMA_PeripheralDataSize_Byte;
	DMA_Structure.DMA_MemoryDataSize 			= DMA_MemoryDataSize_Byte;
	DMA_Structure.DMA_Mode 						= DMA_Mode_Circular;//DMAѭ��ģʽ
	DMA_Structure.DMA_Priority 					= DMA_Priority_VeryHigh;
	DMA_Structure.DMA_FIFOMode 					= DMA_FIFOMode_Disable;
	DMA_Structure.DMA_FIFOThreshold 			= DMA_FIFOThreshold_1QuarterFull;
	DMA_Structure.DMA_MemoryBurst 				= DMA_Mode_Normal;
	DMA_Structure.DMA_PeripheralBurst 			= DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream5,&DMA_Structure);
	DMA_Cmd(DMA2_Stream5,ENABLE);
	
								/*�Ӿ�MINIPC��������*/
    DMA_DeInit(DMA1_Stream0); 
    while (DMA_GetCmdStatus(DMA1_Stream0));
    DMA_Structure.DMA_Channel                 = DMA_Channel_5;
    DMA_Structure.DMA_PeripheralBaseAddr      = (u32)&UART8->DR;
    DMA_Structure.DMA_Memory0BaseAddr         = (u32)g_dma_minipc_send_buff;
    DMA_Structure.DMA_DIR                     = DMA_DIR_MemoryToPeripheral;
    DMA_Structure.DMA_BufferSize              = DMA_MINIPC_SEND_BUFF_SIZE;
    DMA_Structure.DMA_PeripheralInc           = DMA_PeripheralInc_Disable;
    DMA_Structure.DMA_MemoryInc               = DMA_MemoryInc_Enable;
    DMA_Structure.DMA_PeripheralDataSize      = DMA_PeripheralDataSize_Byte;
    DMA_Structure.DMA_MemoryDataSize          = DMA_MemoryDataSize_Byte;
    DMA_Structure.DMA_Mode                    = DMA_Mode_Normal;
    DMA_Structure.DMA_Priority                = DMA_Priority_Medium;
    DMA_Structure.DMA_FIFOMode                = DMA_FIFOMode_Disable;
    DMA_Structure.DMA_FIFOThreshold           = DMA_FIFOThreshold_Full;
    DMA_Structure.DMA_MemoryBurst             = DMA_MemoryBurst_Single;
    DMA_Structure.DMA_PeripheralBurst         = DMA_PeripheralBurst_Single; 
    DMA_Init(DMA1_Stream0,&DMA_Structure);
  
								/*�Ӿ�MINIPC��������*/
    DMA_DeInit(DMA1_Stream6);       
    while (DMA_GetCmdStatus(DMA1_Stream6));
    DMA_Structure.DMA_BufferSize              = DMA_MINIPC_REVE_BUFF_SIZE;
    DMA_Structure.DMA_Channel                 = DMA_Channel_5;
    DMA_Structure.DMA_DIR                     = DMA_DIR_PeripheralToMemory ;
    DMA_Structure.DMA_FIFOMode                = DMA_FIFOMode_Disable;
    DMA_Structure.DMA_FIFOThreshold           = DMA_FIFOThreshold_Full;
    DMA_Structure.DMA_Memory0BaseAddr         = (u32)g_dma_minipc_receice_buff;
    DMA_Structure.DMA_MemoryBurst             = DMA_MemoryBurst_Single;
    DMA_Structure.DMA_MemoryDataSize          = DMA_MemoryDataSize_Byte;
    DMA_Structure.DMA_MemoryInc               = DMA_MemoryInc_Enable;
    DMA_Structure.DMA_Mode                    = DMA_Mode_Normal;
    DMA_Structure.DMA_PeripheralBaseAddr      = (u32)&(UART8->DR);
    DMA_Structure.DMA_PeripheralBurst         = DMA_PeripheralBurst_Single;
    DMA_Structure.DMA_PeripheralDataSize      = DMA_PeripheralDataSize_Byte;
    DMA_Structure.DMA_PeripheralInc           = DMA_PeripheralInc_Disable;
    DMA_Structure.DMA_Priority                = DMA_Priority_Medium;
    DMA_Init(DMA1_Stream6,&DMA_Structure);
    DMA_Cmd(DMA1_Stream6, ENABLE);
	
}
