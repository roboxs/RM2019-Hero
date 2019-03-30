#ifndef _DRIVER_IIC_H
#define _DRIVER_IIC_H

#include<stm32f4xx.h>
#include<sys.h>


//IO��������
#define SDA_IN()  {GPIOF->MODER&=~(3<<(9*2));GPIOF->MODER|=0<<9*2;}	//PF9����ģʽ
#define SDA_OUT() {GPIOF->MODER&=~(3<<(9*2));GPIOF->MODER|=1<<9*2;} //PF9���ģʽ
//IO��������	 
#define IIC_SCL    PFout(7) //SCL
#define IIC_SDA    PFout(9) //SDA	 
#define READ_SDA   PFin(9)  //����SDA 


void IIC_Init(void);			 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�

#endif 
