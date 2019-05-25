#include <delay.h>
#include <task_control.h>
#include <task_led.h>
#include <driver_control.h>
#include <driver_dbus.h>
#include <driver_dial.h>
#include <user_math.h>

uint8_t g_can1_tx_message[8];

void control_task(void *pvParameters)
{
	while(1)
	{
		g_can1_tx_message[0] = RC_Ctl.rc.s1;
		g_can1_tx_message[1] = RC_Ctl.rc.s2;
		g_can1_tx_message[2] = RC_Ctl.mouse.press_l;
		g_can1_tx_message[3] = RC_Ctl.mouse.press_r;
		g_can1_tx_message[4] = RC_Ctl.key.v;
		g_can1_tx_message[5] = (RC_Ctl.key.v >> 8);
		//���ӻ�ͨ��
		master_to_slave(g_can1_tx_message,6);
		vTaskDelay(CONTROL_TASK_CYCLE);
	}
}

/****
	*@note: �����ʹӻ�ͨ��,��Ҫ���ڴ���ң�ص�����
	*/

static void master_to_slave(u8 * data, u8 len)
{
	CanTxMsg can1_txmessage;
	can1_txmessage.StdId = 0x66;
	can1_txmessage.IDE = CAN_Id_Standard;	//��׼��ʶ��
	can1_txmessage.RTR = CAN_RTR_Data;//����֡��ʽ
	can1_txmessage.DLC = 0x08; 		  //�������ݳ���
	
	for(unsigned char i=0; i<len ; i++)
	{
		can1_txmessage.Data[i] = data[i];
	}
	
	CAN_Transmit(CAN1,&can1_txmessage);
}




