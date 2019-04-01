/*���̵����3510 �� 2006*/

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
//		case 1://С����ģʽ
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
//		case 2://����ģʽ
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
		//����ģʽ,�ýǶȻ�
		case 1:
			singel_shoot();
			break;
		//���临λ
		case 3:
			g_dial_2006_motor.give_current = 0;
			g_dial_2006_motor.singel_ready_flag = 1;
			break;
		//����ģʽ,���ٶȻ�
		case 2:
			//���ó��ٶȻ�
			g_dial_2006_motor.moto_speed_pid.feedback_loop = SPEED_LOOP;
			g_dial_2006_motor.moto_angle_pid.feedback_loop = SPEED_LOOP;
			//����Ŀ���ٶ�
			g_dial_2006_motor.moto_speed_pid.target = DIAL_2006_SPEED;
			//�ٶȷ���������CAN�ж���
			//�����������ֵ
			g_dial_2006_motor.give_current = g_dial_2006_motor.moto_speed_pid.pos_out;
	}
	pid_calculate(&(g_dial_2006_motor.moto_speed_pid),&(g_dial_2006_motor.moto_angle_pid));
}

static void singel_shoot()
{
	//���óɽǶȻ�
	g_dial_2006_motor.moto_speed_pid.feedback_loop = ANGLE_LOOP;
	g_dial_2006_motor.moto_angle_pid.feedback_loop = ANGLE_LOOP;
	//���㵱ǰת����ĽǶȲ���ֵ
	g_dial_2006_motor.moto_angle_pid.measure = (g_dial_2006_motor.current_round * 8191 + g_dial_2006_motor.cuttent_ecd) * MOTOR_ECD_TO_DEG;
	//����Ŀ��Ƕ�,�ڵ�ǰ�ĽǶ�������60��
	if(g_dial_2006_motor.singel_ready_flag == 1)
	{
		g_dial_2006_motor.moto_angle_pid.target = g_dial_2006_motor.moto_angle_pid.measure + 60 ;
		g_dial_2006_motor.singel_ready_flag = 0;
	}
	//������ֵ
	g_dial_2006_motor.give_current = g_dial_2006_motor.moto_angle_pid.pos_out; 
}

