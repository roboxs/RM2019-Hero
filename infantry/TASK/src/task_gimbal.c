#include <task_gimbal.h>
#include <FreeRTOS.h>
#include <task.h>
#include <driver_dbus.h>
#include <task_imu.h>
#include <task_led.h>
#include <driver_dial.h>
#include <driver_shoot.h>
#include <driver_imu.h>
#include <inv_mpu.h>
#include "mpu6500_reg.h"


GimbalMove_t g_gimbal_control;
float gkp=-50,gki=-0.1,gkd=-0;
float mid_target =86.5f;
//portTickType system_current_time = 0;

#ifdef GIMBAL_DEBUG_YAW
float yaw_test_kp=5, yaw_test_ki=0.1, yaw_test_kd=0;
#endif

void gimbal_task(void *pvParameters)
{
	//等待电机反转
	vTaskDelay(900);
	//云台的初始化
	gimbal_init(&g_gimbal_control);
	shoot_init();
	while(1)
	{
		//system_current_time = xTaskGetTickCount();
		//云台电机数值更新
		gimbal_update(&g_gimbal_control);
#ifdef GIMBAL_DEBUG_YAW
		//云台电机PID计算
		gimbal_test_yaw(&g_gimbal_control);
#else
		gimbal_pid_calculate(&g_gimbal_control);
#endif
		//拨盘PID计算
		shoot_control();
		 
#ifdef GIMBAL_DEBUG_YAW
		//云台电机赋电流值
		control_gimbal_motor(g_gimbal_control.motor_yaw.give_current, 0,
							 0, 0);
#else 
		control_gimbal_motor(g_gimbal_control.motor_yaw.give_current, g_gimbal_control.motor_pitch.give_current, 
								0, g_dial_2006_motor.give_current);
#endif
									

		//修改PID参数
//		pid_reset(&(g_gimbal_control.pitch_outer_pid),gkp,gki,gkd);
		
		LED3=!LED3;
		vTaskDelay(GIMBAL_TASK_CYCLE);
	}
}


static void gimbal_init(GimbalMove_t *gimbal_control_init)
{
	/*yaw外环初始化*/
	pid_struct_init( &(gimbal_control_init->yaw_outer_pid), POSITION_PID, ANGLE_LOOP, YAW_OUTER_MAXOUT_INIT, YAW_OUTER_INTEGRATION_LIMIT_INIT, 
						YAW_OUTER_P_INIT, YAW_OUTER_I_INIT, YAW_OUTER_D_INIT);
	/*yaw内环初始化*/
	pid_struct_init( &(gimbal_control_init->yaw_inner_pid), POSITION_PID, ANGLE_LOOP, YAW_INNER_MAXOUT, YAW_INNER_INTEGRATION_LIMIT, 
						YAW_INNER_P, YAW_INNER_I, YAW_INNER_D);
	/*pitch外环初始化*/	
	pid_struct_init( &(gimbal_control_init->pitch_outer_pid), POSITION_PID, ANGLE_LOOP, PITCH_OUTER_MAXOUT_INIT, PITCH_OUTER_INTEGRATION_LIMIT_INIT, 
						PITCH_OUTER_P_INIT, PITCH_OUTER_I_INIT, PITCH_OUTER_D_INIT);
	/*pitch内环初始化*/
	pid_struct_init( &(gimbal_control_init->pitch_inner_pid), POSITION_PID, ANGLE_LOOP, PITCH_INNER_MAXOUT, PITCH_INNER_INTEGRATION_LIMIT, 
						PITCH_INNER_P, PITCH_INNER_I, PITCH_INNER_D);
	
	//初始化完成之后切换PID参数
//	pid_reset(&(g_gimbal_control.pitch_outer_pid), PITCH_OUTER_P, PITCH_OUTER_I, PITCH_OUTER_D);
//	pid_reset(&(g_gimbal_control.yaw_outer_pid), YAW_OUTER_P, YAW_OUTER_I, YAW_OUTER_D);
//	g_gimbal_control.yaw_outer_pid.feedback_loop = DOUBLE_LOOP;
//	g_gimbal_control.yaw_inner_pid.feedback_loop = DOUBLE_LOOP;
}

static void gimbal_pid_calculate(GimbalMove_t * gimbal_calculate)
{
	//设定yaw的目标角度
	//gimbal_calculate->yaw_outer_pid.target = mid_target;
	//设定pitch的目标角度
	switch(RC_Ctl.rc.s1)//左开关
	{
		case 1:
			break;
		case 3:
			switch(RC_Ctl.rc.s2)
			{
				case 1:
					break;
				case 3:
					//PITCH遥控控制角度
					if(RC_Ctl.rc.ch1 >= 0)
					{
						gimbal_calculate->pitch_outer_pid.target = (GIMBAL_PITCH_MIN_ANGLE - GIMBAL_PITCH_MID_ANGLE)/660*RC_Ctl.rc.ch1 + GIMBAL_PITCH_MID_ANGLE;
					}
					else if(RC_Ctl.rc.ch1 < 0)
					{	
						gimbal_calculate->pitch_outer_pid.target = (GIMBAL_PITCH_MID_ANGLE - GIMBAL_PITCH_MAX_ANGLE)/660*RC_Ctl.rc.ch1 + GIMBAL_PITCH_MID_ANGLE;
					}
					
					//YAW遥控控制角度
					if(RC_Ctl.rc.ch0 >= 0)
					{
						gimbal_calculate->yaw_outer_pid.target = (GIMBAL_YAW_MAX_ANGLE - GIMBAL_YAW_MID_ANGLE)/660*RC_Ctl.rc.ch0 + GIMBAL_YAW_MID_ANGLE;
					}
					else if(RC_Ctl.rc.ch0 < 0)
					{
						gimbal_calculate->yaw_outer_pid.target = (GIMBAL_YAW_MID_ANGLE - GIMBAL_YAW_MIN_ANGLE)/660*RC_Ctl.rc.ch0 + GIMBAL_YAW_MID_ANGLE;
					}
					break;
				case 2:
					break;
			}
			break;
		case 2:break;
	}
		gimbal_calculate->yaw_outer_pid.target = GIMBAL_YAW_MID_ANGLE;
		gimbal_calculate->pitch_outer_pid.target = GIMBAL_PITCH_MID_ANGLE;

	
	//云台PID计算
	pid_calculate( &(gimbal_calculate->yaw_inner_pid),  &(gimbal_calculate->yaw_outer_pid));
	pid_calculate( &(gimbal_calculate->pitch_inner_pid), &(gimbal_calculate->pitch_outer_pid));
	
	//赋云台电机的电流值
	gimbal_calculate->motor_yaw.give_current = (int16_t) gimbal_calculate->yaw_outer_pid.pos_out;
	gimbal_calculate->motor_pitch.give_current = (int16_t) gimbal_calculate->pitch_outer_pid.pos_out;

}
static void gimbal_update(GimbalMove_t *gimbal_update)
{
	//在can中断中更新速度和角度
	gimbal_update->yaw_outer_pid.measure = gimbal_update-> motor_yaw.angle_measure;
	gimbal_update->yaw_inner_pid.measure = gimbal_update-> motor_yaw.speed_measuer;
	gimbal_update->pitch_outer_pid.measure = gimbal_update->motor_pitch.angle_measure;
	gimbal_update->pitch_inner_pid.measure = gimbal_update->motor_pitch.speed_measuer;
}


#ifdef GIMBAL_DEBUG_YAW
	static void gimbal_test_yaw(GimbalMove_t * gimbal_calculate)
	{
		static int test_count=0;
		gimbal_calculate->yaw_inner_pid.feedback_loop = SPEED_LOOP;
		pid_reset(&g_gimbal_control.yaw_inner_pid, yaw_test_kp, yaw_test_ki, yaw_test_kd);
		
		switch(test_count)
		{
			case 0:
				gimbal_calculate->yaw_inner_pid.target = 1000;
				break;
			case 1000:
				gimbal_calculate->yaw_inner_pid.target = 0;
				break;
			case 1500:
				gimbal_calculate->yaw_inner_pid.target = -1000;
				break;
			case 2500:
				gimbal_calculate->yaw_inner_pid.target = 0;
				break;
		}
		test_count++;
		pid_calculate( &(gimbal_calculate->yaw_inner_pid),  &(gimbal_calculate->yaw_outer_pid));
		gimbal_calculate->motor_yaw.give_current = gimbal_calculate->yaw_inner_pid.pos_out;
		if(test_count ==3000) test_count =0;		
		
	}
#endif

#ifdef GIMBAL_DEBUG_PITCH
	static void gimbal_test_pitch(void)
	{
		
	}
	
#endif






static void control_gimbal_motor(int16_t yaw, int16_t pitch, int16_t dial1, int16_t dial2)
{
	CanTxMsg CAN1_CouldMotorStr;
	
	CAN1_CouldMotorStr.StdId=GIMBAL_CONTROL_ID;
	CAN1_CouldMotorStr.IDE=CAN_Id_Standard;
	CAN1_CouldMotorStr.RTR=CAN_RTR_Data;	 
	CAN1_CouldMotorStr.DLC=0x08;
	
	CAN1_CouldMotorStr.Data[0]= yaw >> 8;
	CAN1_CouldMotorStr.Data[1]= yaw;
	CAN1_CouldMotorStr.Data[2]= pitch >> 8;
	CAN1_CouldMotorStr.Data[3]= pitch;
	CAN1_CouldMotorStr.Data[4]= dial1 >> 8;
	CAN1_CouldMotorStr.Data[5]= dial1;
	CAN1_CouldMotorStr.Data[6]= dial2 >> 8;
	CAN1_CouldMotorStr.Data[7]= dial2;
	
	CAN_Transmit(CAN1,&CAN1_CouldMotorStr);
}
