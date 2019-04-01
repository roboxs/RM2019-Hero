#include <bsp_tim.h>
#include <delay.h>


void BSP_TIM_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);//频率为180M/4=45M
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);//频率为180M/4=45M
	
	//   TIM5_OC1/OC2 摩擦轮电机
	TIM_TimeBaseInitStructure.TIM_ClockDivision				=0;
	TIM_TimeBaseInitStructure.TIM_CounterMode				=TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period					=2000-1;/*  1M/2000=500HZ */
	TIM_TimeBaseInitStructure.TIM_Prescaler					=45-1;/*   45M/45=1M(1us)	*/
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
	
	//TIM2_CH1/CH2 直流拨弹电机
	TIM_TimeBaseInitStructure.TIM_ClockDivision				=0;
	TIM_TimeBaseInitStructure.TIM_CounterMode				=TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period					=2000-1;/*  1M/2000=500HZ */
	TIM_TimeBaseInitStructure.TIM_Prescaler					=45-1;/*   45M/45=1M(1us)	*/
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
	
	TIM_OCInitStructure.TIM_OCMode			= TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity		= TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState		= TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse			= 1000;
	TIM_OC1Init(TIM2,&TIM_OCInitStructure);
	TIM_OC2Init(TIM2,&TIM_OCInitStructure);
	
}
