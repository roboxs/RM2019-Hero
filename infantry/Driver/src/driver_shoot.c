#include <driver_shoot.h>
#include <driver_fricmotor.h>
#include <driver_dial.h>
#include <driver_dbus.h>
//#include <FreeRTOS.h>
//#include <task.h>

static const RC_Ctl_t *shoot_rc; //ң����ָ��,���ڴ���ң����������
ShootControl_t shoot;

RampStr_t fric_ramp1,fric_ramp2;
RampStr_t super_fric_ramp1,super_fric_ramp2;

void shoot_init(void)
{
	//���̵����ʼ��
	dial_init();
	ramp_init(&fric_ramp1, FRICMOTOR_MAX_SPEED, FRICMOTOR_MIN_SPEED, FRICMOTOR_PERIOD);
	ramp_init(&fric_ramp2, FRICMOTOR_MAX_SPEED, FRICMOTOR_MIN_SPEED, FRICMOTOR_PERIOD);
	ramp_init(&super_fric_ramp1, SUPER_FRICMOTOR_MAX_SPEED, SUPER_FRICMOTOR_MIN_SPEED, SUPER_FRICMOTOR_PERIOD);
	ramp_init(&super_fric_ramp2, SUPER_FRICMOTOR_MAX_SPEED, SUPER_FRICMOTOR_MIN_SPEED, SUPER_FRICMOTOR_PERIOD);
	//���ң��ָ���ʼ��
	shoot_rc = get_remote_control_point();
	//�������ýǶȱ�־
	shoot.angle_flag = 1;
}

void shoot_control(void)
{
	//���ݸ���
	shoot_data_update();
	//ģʽ����
	shoot_state_update();
	//С�����²�����2006
	if(shoot.mode == SHOOT_SGL)//�������ģʽ
	{
		singel_shoot();
	}
	else if(shoot.mode == SHOOT_CUS)//����ģʽ
	{
		continuous_shoot();
	}
	
	//ǰ��2006Ŀ��ֵ�Ĵ���
	if(shoot.mode != SHOOT_READY)
	{
		if(g_dial_2006_motor.moto_speed_pid.feedback_loop == SPEED_LOOP)
		{
			g_dial_2006_motor_assist.moto_speed_pid.feedback_loop = SPEED_LOOP;
			g_dial_2006_motor_assist.moto_speed_pid.target = g_dial_2006_motor.moto_speed_pid.target;
		}
		else if(g_dial_2006_motor.moto_speed_pid.feedback_loop == DOUBLE_LOOP)
		{
			g_dial_2006_motor_assist.moto_speed_pid.feedback_loop = DOUBLE_LOOP;
			g_dial_2006_motor_assist.moto_angle_pid.target = g_dial_2006_motor.moto_angle_pid.target;
		}
	}
	
	//Pid����
	pid_calculate(&(g_dial_2006_motor.moto_speed_pid),&(g_dial_2006_motor.moto_angle_pid));
	pid_calculate(&(g_dial_2006_motor_assist.moto_speed_pid),&(g_dial_2006_motor_assist.moto_angle_pid));
	
	if(shoot.mode == SHOOT_READY)
	{
		g_dial_2006_motor.last_angle_target = g_dial_2006_motor.moto_angle_pid.measure;
		shoot.angle_flag = 1;
		g_dial_2006_motor.moto_speed_pid.pos_out = 0;
		g_dial_2006_motor_assist.moto_speed_pid.pos_out = 0;
	}
	
	if(shoot.mode ==SHOOT_STOP)
	{
		//�رղ��̵��
		g_dial_2006_motor.moto_speed_pid.pos_out = 0;
		g_dial_2006_motor_assist.moto_speed_pid.pos_out = 0;
		//�ر�СĦ����
		fric_ramp1.out = FRICMOTOR_OFF_SPEED;
		fric_ramp2.out = FRICMOTOR_OFF_SPEED;
		//�رմ�Ħ����
		super_fric_ramp1.out = SUPER_FRICMOTOR_OFF_SPEED;
		super_fric_ramp2.out = SUPER_FRICMOTOR_OFF_SPEED;
		fricmotor_off();
		super_fricmotor_off();
	}
	else 
	{
		static uint16_t up_time = 0;
		//�ж��Ҽ��Ƿ���,����Ϊ�������,����Ϊ�������
		if(shoot.press_r)
		{
			up_time = UP_ADD_TIME;
		}
		
		if(up_time > 0)
		{
			fric_ramp1.target = FRICMOTOR_HIGH_SPEED;
			fric_ramp2.target = FRICMOTOR_HIGH_SPEED;
			super_fric_ramp1.target = SUPER_FRICMOTOR_HIGH_SPEED;
			super_fric_ramp2.target = SUPER_FRICMOTOR_HIGH_SPEED;
			up_time--;
		}
		else 
		{
			fric_ramp1.target = FRICMOTOR_LOW_SPEED;
			fric_ramp2.target = FRICMOTOR_LOW_SPEED;
			super_fric_ramp1.target = SUPER_FRICMOTOR_LOW_SPEED;
			super_fric_ramp2.target = SUPER_FRICMOTOR_LOW_SPEED;
		}
		
		if(shoot.mode_blt == BULLET_SMALL)
		{
			//СĦ����ת��
			ramp_calculate(& fric_ramp1, fric_ramp1.target);
			
			if(fric_ramp1.out == fric_ramp1.in || fric_ramp1.out == fric_ramp1.max)
			{
				ramp_calculate(& fric_ramp2,fric_ramp2.target);
			}
			//��Ħ���־�ֹ
			super_fric_ramp1.out = SUPER_FRICMOTOR_OFF_SPEED;
			super_fric_ramp2.out = SUPER_FRICMOTOR_OFF_SPEED;
		}
		
		else if(shoot.mode_blt == BULLET_BIG)
		{
			//��Ħ����ת��
			ramp_calculate(& super_fric_ramp1, super_fric_ramp1.target);
			
			if(super_fric_ramp1.out == super_fric_ramp1.in || super_fric_ramp1.out == super_fric_ramp1.max)
			{
				ramp_calculate(& super_fric_ramp2, super_fric_ramp2.target);
			}
			//СĦ���־�ֹ
			fric_ramp1.out = FRICMOTOR_OFF_SPEED;
			fric_ramp2.out = FRICMOTOR_OFF_SPEED;
		}
		//���pwm
		fricmotor_on(fric_ramp1.out , fric_ramp2.out);
		super_fricmotor_on(super_fric_ramp1.out, super_fric_ramp2.out);
	}
	
	//������ֵ
	g_dial_2006_motor.give_current = g_dial_2006_motor.moto_speed_pid.pos_out; 				//����2006
	g_dial_2006_motor_assist.give_current = g_dial_2006_motor_assist.moto_speed_pid.pos_out;//ǰ��2006
	if(shoot.mode_blt == BULLET_BIG)
	{
		g_dial_2006_motor.give_current = 0;
		g_dial_2006_motor_assist.give_current = 0;
	}
	//ǰ��2006
	control_shoot_motor(g_dial_2006_motor_assist.give_current = 0);
}

//������ݸ���
static void shoot_data_update(void)
{
	//������ݸ���
	shoot.last_press_l = shoot.press_l;
	shoot.last_press_r = shoot.press_r;
	shoot.press_l = shoot_rc->mouse.press_l;
	shoot.press_r = shoot_rc->mouse.press_r;
	//ң�����ݸ���
	shoot.last_rc_s1 = shoot.rc_s1;
	shoot.rc_s1 = shoot_rc->rc.s1;
	//Q�����ݸ���
	shoot.last_key_press_Q = shoot.key_press_Q;
	if(shoot_rc->key.v & KEY_PRESSED_OFFSET_Q) shoot.key_press_Q = 1;
	else shoot.key_press_Q =0;
	
	//��갴��ʱ�����
	if(shoot.press_l == SET)
	{
		if(shoot.press_l_time < CONTINUOUS_SHOOT_TIME)
		{
			shoot.press_l_time ++;
		}
	}else {
		shoot.press_l_time = 0;
	}
	
	if(shoot.press_r == SET)
	{
		if(shoot.press_r_time < CONTINUOUS_SHOOT_TIME)
		{
			shoot.press_r_time ++;
		}
	}else{
		shoot.press_r_time = 0;
	}
	//ң���µ�ʱ�����
	if(shoot.rc_s1 == 2)
	{
		if(shoot.rc_s1_time < RC_SHOOT_TIME)
		{
			shoot.rc_s1_time++;
		}
	}else{
		shoot.rc_s1_time = 0;
	}
}

//���״̬����
static void shoot_state_update(void)
{
	if(shoot.mode == SHOOT_READY && shoot.mode_blt == BULLET_SMALL)
	{
		//������µ��һ�λ���s1�����²�����һ���ӵ�
		if((shoot.press_l && shoot.last_press_l == 0) || (shoot.rc_s1 == 2 && shoot.last_rc_s1 !=2))
		{
			shoot.mode = SHOOT_SGL;//�������ģʽ
		}
		//���һֱ���µ������s1���س����²����������ӵ�
		if((shoot.press_l_time == CONTINUOUS_SHOOT_TIME) || (shoot.rc_s1_time == RC_SHOOT_TIME))
		{
			shoot.mode = SHOOT_CUS;//�������ģʽ
		}
	}
	//ң�����м�,����E�����Կ���Ħ����
	if((shoot_rc->key.v & KEY_PRESSED_OFFSET_E) && (shoot_rc->rc.s1 == 3) && (shoot.mode == SHOOT_STOP))
	{
		shoot.mode = SHOOT_READY;
	}
	else if(shoot.rc_s1 == 1 && shoot.last_rc_s1 ==3)//���Խ׶�ʹ��,�ϲ�����Ħ����
	{
		if(shoot.mode == SHOOT_STOP)
		{
			shoot.mode = SHOOT_READY;
		}
		else if(shoot.mode ==SHOOT_READY)
		{
			shoot.mode = SHOOT_STOP;
		}
	}
	//ң�����м�,����F�����Թر�Ħ����
	else if((shoot_rc->key.v & KEY_PRESSED_OFFSET_F) && (shoot_rc->rc.s1 == 3))
	{
		shoot.mode = SHOOT_STOP;
	}
	//�����ֹͣʱ,���ܹ���Q�л���С����
	if((shoot.mode == SHOOT_STOP) && (shoot.key_press_Q && shoot.last_key_press_Q == 0))
	{
		shoot.mode_blt = BULLET_SMALL;
	}
	else if((shoot.mode == SHOOT_STOP) && (shoot_rc->key.v == (KEY_PRESSED_OFFSET_Q | KEY_PRESSED_OFFSET_CTRL)))
	{
		shoot.mode_blt = BULLET_BIG;
	}
}

static void control_shoot_motor(int16_t dial)
{
	CanTxMsg CAN2_ShootMotorStr;
	
	CAN2_ShootMotorStr.StdId=SHOOT_CONTROL_ID;
	CAN2_ShootMotorStr.IDE=CAN_Id_Standard;
	CAN2_ShootMotorStr.RTR=CAN_RTR_Data;	 
	CAN2_ShootMotorStr.DLC=0x08;
	CAN2_ShootMotorStr.Data[0]= (dial >> 8);
	CAN2_ShootMotorStr.Data[1]= dial;
	CAN2_ShootMotorStr.Data[2]= 0;
	CAN2_ShootMotorStr.Data[3]= 0;
	CAN2_ShootMotorStr.Data[4]= 0;
	CAN2_ShootMotorStr.Data[5]= 0;
	CAN2_ShootMotorStr.Data[6]= 0;
	CAN2_ShootMotorStr.Data[7]= 0;
	
	CAN_Transmit(CAN2,&CAN2_ShootMotorStr);
}
