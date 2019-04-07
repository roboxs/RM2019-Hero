#include <bsp_gpio.h>

void BSP_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH,ENABLE);
	
	
	
	//LED0,1,2(PG1,2,3)
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType	= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin		= GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_100MHz;
	GPIO_Init(GPIOG,&GPIO_InitStructure);
	GPIO_SetBits(GPIOG,GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6);//关闭LED灯	
	
	//USART1(DBUS)
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_7 ;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType 	= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART1);
	
	
	//USART6(裁判系统)
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_AF;//复用模式
	GPIO_InitStructure.GPIO_OType	= GPIO_OType_PP;//推挽模式
	GPIO_InitStructure.GPIO_Pin		= GPIO_Pin_14|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_UP;//上拉
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_Init(GPIOG,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource9,GPIO_AF_USART6);//PG9和PG14复用为USART6
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource14,GPIO_AF_USART6);
	

	//IIC(MPU6500+IST8310)
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_7 | GPIO_Pin_9 |GPIO_Pin_8|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType 	= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOF , GPIO_Pin_8);//AD0接地
	GPIO_SetBits(GPIOF , GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_9);//SCL和SDA置高
	
	
	//CAN1
	GPIO_InitStructure.GPIO_Pin		= GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType	= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource0,GPIO_AF_CAN1);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource1,GPIO_AF_CAN1);
	
	//CAN2
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType	= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin		= GPIO_Pin_12|GPIO_Pin_13;
	GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_100MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource12,GPIO_AF_CAN2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_CAN2);
	
	//  TIM5_CH1/CH2(摩擦轮电机)
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType	= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin		= GPIO_Pin_11|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed	= GPIO_High_Speed;
	GPIO_Init(GPIOH,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOH,GPIO_PinSource11,GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOH,GPIO_PinSource10,GPIO_AF_TIM5);
	
	//可控电压源
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType	= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin		= GPIO_Pin_2|GPIO_Pin_4;
	GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_100MHz;
	GPIO_Init(GPIOH,&GPIO_InitStructure);
	GPIO_SetBits(GPIOH,GPIO_Pin_2|GPIO_Pin_4);//关闭LED灯
	
//	//直流电机编码器 TIM2_OC1/OC2
//	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_AF;
//	GPIO_InitStructure.GPIO_OType	= GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_Pin		= GPIO_Pin_0|GPIO_Pin_1;
//	GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_NOPULL;
//	GPIO_InitStructure.GPIO_Speed	= GPIO_High_Speed;
//	GPIO_Init(GPIOA,&GPIO_InitStructure);
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM2);
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM2);
//	
//	//直流电机编码器 TIM8_OC1/OC2
//	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_AF;
//	GPIO_InitStructure.GPIO_OType	= GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_Pin		= GPIO_Pin_5|GPIO_Pin_6;
//	GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_NOPULL;
//	GPIO_InitStructure.GPIO_Speed	= GPIO_High_Speed;
//	GPIO_Init(GPIOI,&GPIO_InitStructure);
//	GPIO_PinAFConfig(GPIOI,GPIO_PinSource5,GPIO_AF_TIM8);
//	GPIO_PinAFConfig(GPIOI,GPIO_PinSource6,GPIO_AF_TIM8);
}
