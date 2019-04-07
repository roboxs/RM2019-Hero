#include <bsp_tim.h>
#include <delay.h>


void BSP_TIM_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
//	TIM_ICInitTypeDef TIM_ICInitStructure;
	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);//Ƶ��Ϊ180M/2=90M
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);//Ƶ��Ϊ180M/4=45M
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);//Ƶ��Ϊ180M/4=45M
	
	//   TIM5_OC1/OC2 Ħ���ֵ��
	TIM_TimeBaseInitStructure.TIM_ClockDivision				=0;
	TIM_TimeBaseInitStructure.TIM_CounterMode				=TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period					=20000-1;/*  1M/2000=500HZ */
	TIM_TimeBaseInitStructure.TIM_Prescaler					=90-1;/*   45M/45=1M(1us)	*/
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStructure);
	
	TIM_OCInitStructure.TIM_OCMode			= TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity		= TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState		= TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse			= 1000;
	TIM_OC1Init(TIM5,&TIM_OCInitStructure);
	TIM_OC2Init(TIM5,&TIM_OCInitStructure);
	
	TIM_OC1PreloadConfig(TIM5 , TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM5 , TIM_OCPreload_Enable);
	TIM_Cmd(TIM5,ENABLE);
	
//	//TIM2_CH1/CH2 ֱ���������,ʹ�ñ�����ģʽ
//	TIM_TimeBaseInitStructure.TIM_ClockDivision				=0;
//	TIM_TimeBaseInitStructure.TIM_CounterMode				=TIM_CounterMode_Up;
//	TIM_TimeBaseInitStructure.TIM_Period					=0;
//	TIM_TimeBaseInitStructure.TIM_Prescaler					=0xFFFF;
//	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
//	//ʹ�ñ�����ģʽ3 ����������ͨ��һ�����
//	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
//	TIM_ICStructInit(&TIM_ICInitStructure);
//	//0x1010,N=5,�����˲�Ƶ���Ƕ�ʱ��Ƶ�ʵ�1/16
//	TIM_ICInitStructure.TIM_ICFilter = 10;
//	TIM_ICInit(TIM2, &TIM_ICInitStructure);
//	//���TIM�ĸ��±�־λ
//	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
//	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
//	TIM_SetCounter(TIM2,0);
//	TIM_Cmd(TIM2, ENABLE); 
//	
//	//TIM8_CH1/CH2 ֱ���������,ʹ�ñ�����ģʽ
//	TIM_TimeBaseInitStructure.TIM_ClockDivision				=0;
//	TIM_TimeBaseInitStructure.TIM_CounterMode				=TIM_CounterMode_Up;
//	TIM_TimeBaseInitStructure.TIM_Period					=0;
//	TIM_TimeBaseInitStructure.TIM_Prescaler					=0xFFFF;
//	TIM_TimeBaseInit(TIM8,&TIM_TimeBaseInitStructure);
//	//ʹ�ñ�����ģʽ3 ����������ͨ��һ�����
//	TIM_EncoderInterfaceConfig(TIM8, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
//	TIM_ICStructInit(&TIM_ICInitStructure);
//	//0x1010,N=5,�����˲�Ƶ���Ƕ�ʱ��Ƶ�ʵ�1/16
//	TIM_ICInitStructure.TIM_ICFilter = 10;
//	TIM_ICInit(TIM8, &TIM_ICInitStructure);
//	//���TIM�ĸ��±�־λ
//	TIM_ClearFlag(TIM8, TIM_FLAG_Update);
//	TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE);
//	TIM_SetCounter(TIM8,0);
//	TIM_Cmd(TIM8, ENABLE); 
}
