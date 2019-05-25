#include <FreeRTOS.h>
#include <task.h>
#include <task_chassis.h>
#include <task_gimbal.h>
#include <driver_dbus.h>
#include <task_imu.h>
#include <user_math.h>

ChassisMove_t g_chassis_move;


float chassis_kp=200,chassis_ki=0,chassis_kd=50;
float dead = 0;
float err_dead = 0;
void chassis_task(void *pvParameters)
{
//	unsigned char i;
	//�ȴ������ת
	vTaskDelay(900);
	//���̳�ʼ��
	chassis_init(&g_chassis_move);
	while(1)
	{
		/*�������ݸ���*/
		chassis_updata_data(&g_chassis_move);
		/*���̵�ǰģʽ����*/
		chassis_set_mode(&g_chassis_move);
		/*����Ŀ��ֵ�趨*/
		if(g_gimbal_control.init_finish_flag == SET) chassis_set_target(&g_chassis_move);//��̨��ʼ�����֮����ܿ���
		/*����PID����*/
		chassis_pid_calculate(&g_chassis_move);
		/*Ӣ�� ��ǰ-���-��ǰ-�Һ� ID 1-2-3-4*/
		control_chassis_motor(g_chassis_move.chassis_motor[0].give_current,g_chassis_move.chassis_motor[1].give_current,
							  g_chassis_move.chassis_motor[2].give_current,g_chassis_move.chassis_motor[3].give_current);
		
		
//		for(i=0; i<4; i++)
//		{
//			pid_reset(&(g_chassis_move.motor_speed_pid[i]),chassis_kp,chassis_ki,chassis_kd);
//		}
		pid_reset(&(g_chassis_move.chassis_angle_pid),chassis_kp,chassis_ki,chassis_kd);
		g_chassis_move.chassis_angle_pid.deadband = err_dead;
		vTaskDelay(CHASSIS_TASK_CYCLE);
	}
}

static void chassis_set_mode(ChassisMove_t * chassis)
{
	
	if(RC_Ctl.rc.s2 == 3)//�����е�ʱ
	{
		chassis->mode = CHASSIS_NO_FOLLOW;
	}
	else if(RC_Ctl.rc.s2 == 1)//�����ϵ�ʱ
	{
		chassis->mode = CHASSIS_FOLLOW_GIMBAL;
	}
	else if(RC_Ctl.rc.s2 == 2)//�����µ�ʱ
	{
		chassis->mode = CHASSIS_NO_MOVE;
	}
	else //ң�عر�ʱ
	{
		chassis->mode = CHASSIS_NO_MOVE;
	}
}

static void chassis_set_target(ChassisMove_t * chassis_move)
{
	float vx_set=0, vy_set = 0,vw_set =0 ;
	
	chassis_control_set(&vx_set,&vy_set,&vw_set,chassis_move);
	if(chassis_move->mode == CHASSIS_NO_MOVE)//��ֹ״̬
	{
		chassis_move->vx_target = vx_set;
		chassis_move->vy_target = vy_set;
		chassis_move->vw_target = vw_set;
	}
	else if(chassis_move->mode == CHASSIS_NO_FOLLOW)//��������ת
	{
		chassis_move->vx_target = vx_set;
		chassis_move->vy_target = vy_set;
		chassis_move->vw_target = vw_set;//���ٶ�
	}
	else if(chassis_move->mode == CHASSIS_FOLLOW_GIMBAL)//������̨��ת
	{
		//yaw�᷽��,������������ڵ���ĽǶ�������ٶ�
		chassis_move->vx_target = vx_set;
		chassis_move->vy_target = vy_set;
		chassis_move->vw_target = vw_set;
	}

}
static void chassis_control_set(float *vx, float *vy, float *vw, ChassisMove_t * chassis)
{
	if(chassis->mode == CHASSIS_NO_MOVE)//Ŀ��ֵΪ0
	{
		*vx =0;
		*vy =0;
		*vw =0;
	}
	else if(chassis->mode == CHASSIS_NO_FOLLOW)//���̲�����Ƕ�
	{
		chassis_rc_to_target(vx,vy);
		*vw = CHASSIS_VW_RC_SEN * RC_Ctl.rc.ch0;
	}
	else if(chassis->mode== CHASSIS_FOLLOW_GIMBAL)
	{
		chassis_rc_to_target(vx,vy);
		pid_calculate(&chassis->chassis_angle_pid,NULL);
		*vw = add_dead_limit(&chassis->chassis_angle_pid.pos_out , dead);
	}
}

//��ң����ת����Ŀ��ֵ
static void chassis_rc_to_target(float *vx, float *vy)
{
	float vx_set,vy_set;
	float key_target_x,key_target_y;
	
	vx_set = RC_Ctl.rc.ch3 * CHASSIS_RC_SEN;
	vy_set = RC_Ctl.rc.ch2 * CHASSIS_RC_SEN;
	
	if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_SHIFT)//����shift����ǰ��
	{
		key_target_x = CHASSIS_VX_MAX_SPEED;
		key_target_y = CHASSIS_VY_MAX_SPEED;
	}
	else {
		key_target_x = CHASSIS_VX_MAX_SPEED * CHASSIS_KEY_SEN;
		key_target_y = CHASSIS_VY_MAX_SPEED * CHASSIS_KEY_SEN;
	}
	
	
	if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_W)
	{
		vx_set = key_target_x;
	}
	else if (RC_Ctl.key.v & KEY_PRESSED_OFFSET_S)
	{
		vx_set = -key_target_x;
	}
	
	if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_D)
	{
		vy_set = key_target_y;
	}
	else if (RC_Ctl.key.v & KEY_PRESSED_OFFSET_A)
	{
		vy_set = -key_target_y;
	}
	*vx = vx_set;
	*vy = vy_set;
}

static void chassis_init(ChassisMove_t *chassis_move)
{
	unsigned char i;
	//����PID��ʼ��
	for(i=0; i<4; i++)
	{
		pid_struct_init(&chassis_move->motor_speed_pid[i], POSITION_PID, SPEED_LOOP, CHASSIS_MAX_PID_OUTER, CHASSIS_OUTER_INTEGRATION_LIMIT,
						CHASSIS_PID_KP, CHASSIS_PID_KI, CHASSIS_PID_KD);
//		chassis_move->motor_speed_pid[i].deadband = 5;
	}
	
	//��ʼ��������תPID
	pid_struct_init(&chassis_move->chassis_angle_pid, POSITION_PID, SPEED_LOOP, CHASSIS_ANGLE_MAX_PID_OUTER, CHASSIS_ANGLE_OUTER_INTEGRATION_LIMIT, 
					CHASSIS_ANGLE_KP, CHASSIS_ANGLE_KI, CHASSIS_ANGLE_KD);
	//��̨yaw�����Ŀ��ֵ
	chassis_move->chassis_angle_pid.target = GIMBAL_YAW_MID_ANGLE;
}

static void chassis_updata_data(ChassisMove_t *chassis_update)
{
	unsigned char i =0;
	//��������ٶȸ�����CAN�ж���
	for(i=0; i<4; i++)
	{
		chassis_update->motor_speed_pid[i].measure	= chassis_update->chassis_motor[i].measure;
	}
	//��̨����ĽǶȸ��´�������
	chassis_update->vx_measure = 0.25f * (chassis_update->chassis_motor[0].measure + chassis_update->chassis_motor[1].measure + chassis_update->chassis_motor[2].measure + chassis_update->chassis_motor[3].measure);
	chassis_update->vy_measure = 0.25f * (chassis_update->chassis_motor[0].measure - chassis_update->chassis_motor[1].measure - chassis_update->chassis_motor[2].measure + chassis_update->chassis_motor[3].measure);
	chassis_update->vw_measure = 0.25f * (chassis_update->chassis_motor[0].measure + chassis_update->chassis_motor[1].measure - chassis_update->chassis_motor[2].measure - chassis_update->chassis_motor[3].measure);
}

static void chassis_pid_calculate(ChassisMove_t *chassis_move)
{
	unsigned char i;
	short wheel_speed[4];
	
	mecanum_calculate(chassis_move->vx_target, chassis_move->vy_target, chassis_move->vw_target,wheel_speed);
	//����ÿ�����ӵ�Ŀ��ֵ
	chassis_move->motor_speed_pid[0].target = wheel_speed[0];
	chassis_move->motor_speed_pid[1].target = wheel_speed[1];
	chassis_move->motor_speed_pid[2].target = -wheel_speed[2];
	chassis_move->motor_speed_pid[3].target = -wheel_speed[3];
	
	//����ÿ�����ӵ��ٶ�ֵ
	for(i=0; i<4; i++)
	{
		pid_calculate(&chassis_move->motor_speed_pid[i],NULL);
	}
	
	//��ÿ�����ӵĵ���ֵ
	for(i=0; i<4; i++)
	{
		chassis_move->chassis_motor[i].give_current = chassis_move->motor_speed_pid[i].pos_out;
	}
}


 /****
    *@brief �޷���������
    *@param[in] object   ��Ҫ�޷�����
    *@param[in] abs_max	 �޷�ֵ
    */
static void abs_limit(float *object, float abs_max)
{
    if(*object > abs_max)  *object =  abs_max;
    if(*object < -abs_max) *object = -abs_max;
}

static void mecanum_calculate(float vx,float vy,float vw, short *speed)
{
	u8 index;
	float buffer[4]={0},speed_max = 0 ,param;
	
	buffer[0] = vx + vy + vw;
	buffer[1] = vx - vy + vw;
	buffer[2] = vx - vy - vw;
	buffer[3] = vx + vy - vw;
	
	 for(index = 0, speed_max = 0; index < 4; index++)
    {
        if((buffer[index] > 0 ? buffer[index] : -buffer[index]) > speed_max)
        {
            speed_max = (buffer[index] > 0 ? buffer[index] : -buffer[index]);
        }
    }
    if(CHASSIS_MAX_SPEED < speed_max)
    {
        param = (float)CHASSIS_MAX_SPEED / speed_max;
        speed[0] = buffer[0] * param;
        speed[1] = buffer[1] * param;
        speed[2] = buffer[2] * param;
        speed[3] = buffer[3] * param; 
    }
    else
    {
        speed[0] = buffer[0];
        speed[1] = buffer[1];
        speed[2] = buffer[2];
        speed[3] = buffer[3];
    }
}

static void control_chassis_motor(int16_t moto1,int16_t moto2,int16_t moto3,int16_t moto4)
{
	CanTxMsg CAN1_ChassisMotorStr;
	
	CAN1_ChassisMotorStr.StdId=CHASSIS_CONTROL_ID;
	CAN1_ChassisMotorStr.IDE=CAN_Id_Standard;
	CAN1_ChassisMotorStr.RTR=CAN_RTR_Data;	 
	CAN1_ChassisMotorStr.DLC=0x08;
	
	CAN1_ChassisMotorStr.Data[0]=(u8)(moto1 >> 8);
	CAN1_ChassisMotorStr.Data[1]=(u8)moto1;
	CAN1_ChassisMotorStr.Data[2]=(u8)(moto2 >> 8);
	CAN1_ChassisMotorStr.Data[3]=(u8)moto2;
	CAN1_ChassisMotorStr.Data[4]=(u8)(moto3 >> 8);
	CAN1_ChassisMotorStr.Data[5]=(u8)moto3;
	CAN1_ChassisMotorStr.Data[6]=(u8)(moto4 >> 8);
	CAN1_ChassisMotorStr.Data[7]=(u8)moto4;
	
	CAN_Transmit(CAN1,&CAN1_ChassisMotorStr);
}
