/** 
	* @brief ����ϵͳ���ݶ���
  * 
  */
	
#ifndef _JUDGE_H
#define _JUDGE_H

#include <string.h>
#include <stdio.h>
#include <stm32f4xx.h>


#define UP_REG_ID    0xA0  //
#define DN_REG_ID    0xA5  //֡��ʼ�ֽ�
#define HEADER_LEN   sizeof(frame_header_t)
#define CMD_LEN      2    //�����ֽ���
#define CRC_LEN      2    //CRC16У���ֽ���
#define PROTOCAL_FRAME_MAX_SIZE 200//�������ݵ���󳤶�



//typedef __packed struct
//{
//uint16_t chassis_volt;
//uint16_t chassis_current;
//float chassis_power;
//uint16_t chassis_power_buffer;
//uint16_t shooter_heat0;
//uint16_t shooter_heat1;
//} ext_power_heat_data_t;

		/*����ϵͳ������״̬����*/
typedef __packed struct
{
uint8_t robot_id;
uint8_t robot_level;
uint16_t remain_HP;
uint16_t max_HP;
uint16_t shooter_heat0_cooling_rate;
uint16_t shooter_heat0_cooling_limit;
uint16_t shooter_heat1_cooling_rate;
uint16_t shooter_heat1_cooling_limit;
uint8_t mains_power_gimbal_output : 1;
uint8_t mains_power_chassis_output : 1;
uint8_t mains_power_shooter_output : 1;
} ext_game_robot_state_t;


#define JUDGE_ROBOT_STATE 15




void data_judge(void);


#endif

