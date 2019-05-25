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
	//等待电机反转
	vTaskDelay(900);
	//底盘初始化
	chassis_init(&g_chassis_move);
	while(1)
	{
		/*底盘数据更新*/
		chassis_updata_data(&g_chassis_move);
		/*底盘当前模式设置*/
		chassis_set_mode(&g_chassis_move);
		/*底盘目标值设定*/
		if(g_gimbal_control.init_finish_flag == SET) chassis_set_target(&g_chassis_move);//云台初始化完成之后才能控制
		/*底盘PID更新*/
		chassis_pid_calculate(&g_chassis_move);
		/*英雄 左前-左后-右前-右后 ID 1-2-3-4*/
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
	
	if(RC_Ctl.rc.s2 == 3)//处于中档时
	{
		chassis->mode = CHASSIS_NO_FOLLOW;
	}
	else if(RC_Ctl.rc.s2 == 1)//处于上档时
	{
		chassis->mode = CHASSIS_FOLLOW_GIMBAL;
	}
	else if(RC_Ctl.rc.s2 == 2)//处于下档时
	{
		chassis->mode = CHASSIS_NO_MOVE;
	}
	else //遥控关闭时
	{
		chassis->mode = CHASSIS_NO_MOVE;
	}
}

static void chassis_set_target(ChassisMove_t * chassis_move)
{
	float vx_set=0, vy_set = 0,vw_set =0 ;
	
	chassis_control_set(&vx_set,&vy_set,&vw_set,chassis_move);
	if(chassis_move->mode == CHASSIS_NO_MOVE)//静止状态
	{
		chassis_move->vx_target = vx_set;
		chassis_move->vy_target = vy_set;
		chassis_move->vw_target = vw_set;
	}
	else if(chassis_move->mode == CHASSIS_NO_FOLLOW)//不跟随旋转
	{
		chassis_move->vx_target = vx_set;
		chassis_move->vy_target = vy_set;
		chassis_move->vw_target = vw_set;//角速度
	}
	else if(chassis_move->mode == CHASSIS_FOLLOW_GIMBAL)//跟随云台旋转
	{
		//yaw轴方向,用陀螺仪相对于电机的角度算出角速度
		chassis_move->vx_target = vx_set;
		chassis_move->vy_target = vy_set;
		chassis_move->vw_target = vw_set;
	}

}
static void chassis_control_set(float *vx, float *vy, float *vw, ChassisMove_t * chassis)
{
	if(chassis->mode == CHASSIS_NO_MOVE)//目标值为0
	{
		*vx =0;
		*vy =0;
		*vw =0;
	}
	else if(chassis->mode == CHASSIS_NO_FOLLOW)//底盘不跟随角度
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

//将遥控量转化成目标值
static void chassis_rc_to_target(float *vx, float *vy)
{
	float vx_set,vy_set;
	float key_target_x,key_target_y;
	
	vx_set = RC_Ctl.rc.ch3 * CHASSIS_RC_SEN;
	vy_set = RC_Ctl.rc.ch2 * CHASSIS_RC_SEN;
	
	if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_SHIFT)//按下shift加速前进
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
	//底盘PID初始化
	for(i=0; i<4; i++)
	{
		pid_struct_init(&chassis_move->motor_speed_pid[i], POSITION_PID, SPEED_LOOP, CHASSIS_MAX_PID_OUTER, CHASSIS_OUTER_INTEGRATION_LIMIT,
						CHASSIS_PID_KP, CHASSIS_PID_KI, CHASSIS_PID_KD);
//		chassis_move->motor_speed_pid[i].deadband = 5;
	}
	
	//初始化底盘旋转PID
	pid_struct_init(&chassis_move->chassis_angle_pid, POSITION_PID, SPEED_LOOP, CHASSIS_ANGLE_MAX_PID_OUTER, CHASSIS_ANGLE_OUTER_INTEGRATION_LIMIT, 
					CHASSIS_ANGLE_KP, CHASSIS_ANGLE_KI, CHASSIS_ANGLE_KD);
	//云台yaw电机的目标值
	chassis_move->chassis_angle_pid.target = GIMBAL_YAW_MID_ANGLE;
}

static void chassis_updata_data(ChassisMove_t *chassis_update)
{
	unsigned char i =0;
	//单个电机速度更新在CAN中断中
	for(i=0; i<4; i++)
	{
		chassis_update->motor_speed_pid[i].measure	= chassis_update->chassis_motor[i].measure;
	}
	//云台电机的角度更新传给底盘
	chassis_update->vx_measure = 0.25f * (chassis_update->chassis_motor[0].measure + chassis_update->chassis_motor[1].measure + chassis_update->chassis_motor[2].measure + chassis_update->chassis_motor[3].measure);
	chassis_update->vy_measure = 0.25f * (chassis_update->chassis_motor[0].measure - chassis_update->chassis_motor[1].measure - chassis_update->chassis_motor[2].measure + chassis_update->chassis_motor[3].measure);
	chassis_update->vw_measure = 0.25f * (chassis_update->chassis_motor[0].measure + chassis_update->chassis_motor[1].measure - chassis_update->chassis_motor[2].measure - chassis_update->chassis_motor[3].measure);
}

static void chassis_pid_calculate(ChassisMove_t *chassis_move)
{
	unsigned char i;
	short wheel_speed[4];
	
	mecanum_calculate(chassis_move->vx_target, chassis_move->vy_target, chassis_move->vw_target,wheel_speed);
	//设置每个轮子的目标值
	chassis_move->motor_speed_pid[0].target = wheel_speed[0];
	chassis_move->motor_speed_pid[1].target = wheel_speed[1];
	chassis_move->motor_speed_pid[2].target = -wheel_speed[2];
	chassis_move->motor_speed_pid[3].target = -wheel_speed[3];
	
	//计算每个轮子的速度值
	for(i=0; i<4; i++)
	{
		pid_calculate(&chassis_move->motor_speed_pid[i],NULL);
	}
	
	//赋每个轮子的电流值
	for(i=0; i<4; i++)
	{
		chassis_move->chassis_motor[i].give_current = chassis_move->motor_speed_pid[i].pos_out;
	}
}


 /****
    *@brief 限幅函数声明
    *@param[in] object   需要限幅对象
    *@param[in] abs_max	 限幅值
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
