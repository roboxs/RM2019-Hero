/*���̵����3510 �� 2006*/

#include <driver_dial.h> 
#include <driver_control.h>
#include <driver_dbus.h>
#include <user_math.h>

DialMotor_t g_dial_2006_motor;

//float p_out = 50,i_out,d_out;



static void singel_shoot(void);

void dial_init(void)
{
	
/*dial-2006-outer*/		pid_struct_init(&g_dial_2006_motor.moto_angle_pid, POSITION_PID, SPEED_LOOP, DIAL_2006_OUTER_MAXOUT, DIAL_2006_OUTER_INTEGRATION_LIMIT,
					 DIAL_2006_OUTER_P, DIAL_2006_OUTER_I, DIAL_2006_OUTER_D);
/*dial-2006-inner*/		pid_struct_init(&g_dial_2006_motor.moto_speed_pid, POSITION_PID, SPEED_LOOP, DIAL_2006_INNER_MAXOUT, DIAL_2006_INNER_INTEGRATION_LIMIT,
					 DIAL_2006_INNER_P, DIAL_2006_INNER_I ,DIAL_2006_INNER_D);
}


void dial_pid_calculate(void)
{
	switch(RC_Ctl.rc.s1)
	{
		//����ģʽ,��˫��
		case 1:
			singel_shoot();
			break;
		//���临λ
		case 3:
			g_dial_2006_motor.give_current = 0;
			g_dial_2006_motor.single_ready_flag = 1;
			//�ϴνǶ��趨ֵ
			g_dial_2006_motor.last_angle_target = g_dial_2006_motor.moto_angle_pid.measure;
			break;
		//����ģʽ,���ٶȻ�
		case 2:
			//���ó��ٶȻ�
			g_dial_2006_motor.moto_speed_pid.feedback_loop = SPEED_LOOP;
			g_dial_2006_motor.moto_angle_pid.feedback_loop = SPEED_LOOP;
			//�޸�PID����
			pid_reset(&(g_dial_2006_motor.moto_speed_pid), 5,0.5,0);
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
	g_dial_2006_motor.moto_speed_pid.feedback_loop = DOUBLE_LOOP;
	g_dial_2006_motor.moto_angle_pid.feedback_loop = DOUBLE_LOOP;
	//�޸�PID����
	pid_reset(&(g_dial_2006_motor.moto_speed_pid), 5,0.5,0);
	//pid_reset(&(g_dial_2006_motor.moto_angle_pid), p_out,i_out,d_out);
	if(g_dial_2006_motor.single_ready_flag == 1)
	{	
		g_dial_2006_motor.single_ready_flag = 0;
		//����Ŀ��Ƕ�,���ϴ��趨�ĽǶ�������60��
		g_dial_2006_motor.moto_angle_pid.target = g_dial_2006_motor.last_angle_target + 60 ;
		//��Ҫ���㴦��
		if(g_dial_2006_motor.moto_angle_pid.target >360) 
			g_dial_2006_motor.over_zero_flag =1;
	}
		//2006����Ƕȹ��㴦��
		if(g_dial_2006_motor.over_zero_flag == 1)
		{
			if(ABS(g_dial_2006_motor.moto_angle_pid.measure) < 5) 
			{
				g_dial_2006_motor.moto_angle_pid.target = g_dial_2006_motor.moto_angle_pid.target - 360;
				g_dial_2006_motor.over_zero_flag =0;
			}
		}
	
	//������ֵ
	g_dial_2006_motor.give_current = g_dial_2006_motor.moto_speed_pid.pos_out; 
}

