/*拨盘电机用3510 和 2006*/

#include <driver_dial.h> 
#include <driver_control.h>
#include <driver_dbus.h>

DialMotor_t g_dial_2006_motor;

static void singel_shoot(void);

void dial_init(void)
{
	
/*dial-2006-outer*/		pid_struct_init(&g_dial_2006_motor.moto_angle_pid, POSITION_PID, SPEED_LOOP, DIAL_2006_OUTER_MAXOUT, DIAL_2006_OUTER_INTEGRATION_LIMIT,
					 DIAL_2006_OUTER_P, DIAL_2006_OUTER_I, DIAL_2006_OUTER_D);
/*dial-2006-inner*/		pid_struct_init(&g_dial_2006_motor.moto_speed_pid, POSITION_PID, SPEED_LOOP, DIAL_2006_INNER_MAXOUT, DIAL_2006_INNER_INTEGRATION_LIMIT,
					 DIAL_2006_INNER_P, DIAL_2006_INNER_I ,DIAL_2006_INNER_D);
	
///*dial-3510-outer*/		pid_struct_init(&g_dial_3510_pid_outer, POSITION_PID, SPEED_LOOP, DIAL_3510_OUTER_MAXOUT, DIAL_3510_OUTER_INTEGRATION_LIMIT,
//					 DIAL_3510_OUTER_P, DIAL_3510_OUTER_I, DIAL_3510_OUTER_D);
///*dial-3510-inner*/		pid_struct_init(&g_dial_3510_pid_inner, POSITION_PID, SPEED_LOOP, DIAL_3510_INNER_MAXOUT, DIAL_3510_INNER_INTEGRATION_LIMIT,
//					 DIAL_3510_INNER_P, DIAL_3510_INNER_I, DIAL_3510_INNER_D);	
}


void dial_pid_calculate(void)
{
//	switch(RC_Ctl.rc.s1)
//	{
//		case 1://小弹丸模式
//			switch(RC_Ctl.rc.s2)
//			{
//				case SIGNLE_SHOOT:
//					break;
//				case CONTINUOUS_SHOOT:
//					g_dial_2006_pid_inner.target =DIAL_2006_SPEED;
//					break;
//				case NOT_SHOOT:
//					g_dial_2006_pid_inner.target =0;
//					break;
//			}
//			break;
//		case 2://大弹丸模式
//			switch(RC_Ctl.rc.s2)
//			{
//				case SIGNLE_SHOOT:
//					break;
//				case CONTINUOUS_SHOOT:
//					g_dial_3510_pid_inner.target =DIAL_3510_SPEED;
//					break;
//				case NOT_SHOOT:
//					g_dial_3510_pid_inner.target=0;
//					break;
//			}
//			break;
//		case 3:
//			break;
//	}
	switch(RC_Ctl.rc.s1)
	{
		//单发模式,用角度环
		case 1:
			singel_shoot();
			break;
		//发射复位
		case 3:
			g_dial_2006_motor.give_current = 0;
			g_dial_2006_motor.singel_ready_flag = 1;
			break;
		//连发模式,用速度环
		case 2:
			//设置成速度环
			g_dial_2006_motor.moto_speed_pid.feedback_loop = SPEED_LOOP;
			g_dial_2006_motor.moto_angle_pid.feedback_loop = SPEED_LOOP;
			//设置目标速度
			g_dial_2006_motor.moto_speed_pid.target = DIAL_2006_SPEED;
			//速度反馈更新在CAN中断中
			//给电机赋电流值
			g_dial_2006_motor.give_current = g_dial_2006_motor.moto_speed_pid.pos_out;
	}
	pid_calculate(&(g_dial_2006_motor.moto_speed_pid),&(g_dial_2006_motor.moto_angle_pid));
}

static void singel_shoot()
{
	//设置成角度环
	g_dial_2006_motor.moto_speed_pid.feedback_loop = ANGLE_LOOP;
	g_dial_2006_motor.moto_angle_pid.feedback_loop = ANGLE_LOOP;
	//计算当前转动轴的角度测量值
	g_dial_2006_motor.moto_angle_pid.measure = (g_dial_2006_motor.current_round * 8191 + g_dial_2006_motor.cuttent_ecd) * MOTOR_ECD_TO_DEG;
	//设置目标角度,在当前的角度上增加60°
	if(g_dial_2006_motor.singel_ready_flag == 1)
	{
		g_dial_2006_motor.moto_angle_pid.target = g_dial_2006_motor.moto_angle_pid.measure + 60 ;
		g_dial_2006_motor.singel_ready_flag = 0;
	}
	//赋电流值
	g_dial_2006_motor.give_current = g_dial_2006_motor.moto_angle_pid.pos_out; 
}

