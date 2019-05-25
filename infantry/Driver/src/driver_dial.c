/*拨盘电机用3510 和 2006*/

#include <driver_dial.h> 
#include <driver_control.h>
#include <driver_dbus.h>
#include <user_math.h>
#include <driver_shoot.h>
#include <FreeRTOS.h>
#include <task.h>


DialMotor_t g_dial_2006_motor;
DialMotor_t g_dial_2006_motor_assist;



void dial_init(void)
{
	
/*dial-2006-outer*/		pid_struct_init(&g_dial_2006_motor.moto_angle_pid, POSITION_PID, DOUBLE_LOOP, DIAL_2006_OUTER_MAXOUT, DIAL_2006_OUTER_INTEGRATION_LIMIT,
					 DIAL_2006_OUTER_P, DIAL_2006_OUTER_I, DIAL_2006_OUTER_D);
/*dial-2006-inner*/		pid_struct_init(&g_dial_2006_motor.moto_speed_pid, POSITION_PID, DOUBLE_LOOP, DIAL_2006_INNER_MAXOUT, DIAL_2006_INNER_INTEGRATION_LIMIT,
					 DIAL_2006_INNER_P, DIAL_2006_INNER_I ,DIAL_2006_INNER_D);
//后面2006电机
/*dial-2006-outer*/		pid_struct_init(&g_dial_2006_motor_assist.moto_angle_pid, POSITION_PID, DOUBLE_LOOP, DIAL_2006_OUTER_MAXOUT, DIAL_2006_OUTER_INTEGRATION_LIMIT,
					 DIAL_2006_OUTER_P, DIAL_2006_OUTER_I, DIAL_2006_OUTER_D);
/*dial-2006-inner*/		pid_struct_init(&g_dial_2006_motor_assist.moto_speed_pid, POSITION_PID, DOUBLE_LOOP, DIAL_2006_INNER_MAXOUT, DIAL_2006_INNER_INTEGRATION_LIMIT,
					 DIAL_2006_INNER_P, DIAL_2006_INNER_I ,DIAL_2006_INNER_D);
}


void singel_shoot(void)
{

		//设置成双环控制
		g_dial_2006_motor.moto_speed_pid.feedback_loop = DOUBLE_LOOP;
		g_dial_2006_motor.moto_angle_pid.feedback_loop = DOUBLE_LOOP;
		//修改PID参数
		pid_reset(&(g_dial_2006_motor.moto_speed_pid), 8,4,3);
		pid_reset(&(g_dial_2006_motor_assist.moto_speed_pid), 8,4,3);
	
		if(shoot.angle_flag == 1)
		{
			//设置目标角度,在上次设定的角度上增加60°
			g_dial_2006_motor.moto_angle_pid.target = g_dial_2006_motor.last_angle_target + 60 ;
			//获取给定角度时系统的时间
			shoot.cmd_time = xTaskGetTickCount();
			//需要过零处理
			if(g_dial_2006_motor.moto_angle_pid.target >360) 
				g_dial_2006_motor.over_zero_flag =1;
			shoot.angle_flag = 0;
		}
	
		//2006电机角度过零处理
		if(g_dial_2006_motor.over_zero_flag == 1)
		{
			if(ABS(g_dial_2006_motor.moto_angle_pid.measure) < 5) 
			{
				g_dial_2006_motor.moto_angle_pid.target = g_dial_2006_motor.moto_angle_pid.target - 360;
				g_dial_2006_motor.over_zero_flag =0;
			}
		}
		
		//单发完成之后设置为设置准备模式
		if(ABS(g_dial_2006_motor.moto_angle_pid.target - g_dial_2006_motor.moto_angle_pid.measure) < 0.05f)
		{
			shoot.mode = SHOOT_READY;
		}
		else  //如果发生堵转
		{
			shoot.run_time = xTaskGetTickCount();
			if(shoot.run_time - shoot.cmd_time > BLOCK_TIME && shoot.run_time - shoot.cmd_time < BLOCK_TIME + REVERSE_TIEM)
			{
				g_dial_2006_motor.moto_speed_pid.feedback_loop = SPEED_LOOP;
				g_dial_2006_motor.moto_speed_pid.target = BLOCK_SPEED;
			}
		}
}

void  continuous_shoot(void)
{
		//设置成速度环
		g_dial_2006_motor.moto_speed_pid.feedback_loop = SPEED_LOOP;
		g_dial_2006_motor.moto_angle_pid.feedback_loop = SPEED_LOOP;
		//修改PID参数
		pid_reset(&(g_dial_2006_motor.moto_speed_pid), 800,0.5,0);	
		pid_reset(&(g_dial_2006_motor_assist.moto_speed_pid), 800,0.5,0);	
		//设置目标速度
		g_dial_2006_motor.moto_speed_pid.target = DIAL_2006_SPEED;
	//速度反馈更新在CAN中断中
		if((shoot.press_l == 0 && shoot.last_press_l == 1) || (shoot.rc_s1 == 3 && shoot.last_rc_s1 == 2)) shoot.mode = SHOOT_READY;
		
		shoot.run_time = xTaskGetTickCount();
		//从此处开始堵转,并开始计算堵转时间
		if(ABS(g_dial_2006_motor.moto_speed_pid.measure) < 500) 
		{
			shoot.block_time ++;
			if(shoot.block_time == BLOCK_TIME_CNS)
			{
				shoot.block_flag = 1;//产生堵转
				shoot.cmd_time = xTaskGetTickCount();
			}
		}
		else 
		{
			shoot.block_time = 0;
		}
		if(shoot.block_flag == 1)
		{
			g_dial_2006_motor.moto_speed_pid.target = BLOCK_SPEED;
			if(shoot.run_time - shoot.cmd_time > REVERSE_TIEM) 
			{
				shoot.block_flag = 0;
				shoot.block_time = 0;
			}
		}
}

/****
	*@note :最小限制幅度
	*/
static void min_limit( int16_t *object, int16_t limit)
{
	if((*object) < limit ) *object = limit;
}



/****
	*@note :大弹丸推动电机速度函数
	*@param[in] : 左电机速度 右电机速度
	*/
void move_push_motor(int16_t moto)
{
	if((moto>0))
	{
		PUSH_MOTOR_IN1 =1;
		PUSH_MOTOR_IN2 =0;
		moto=moto;
	}
	else if( (moto<0))
	{
		PUSH_MOTOR_IN1 =0;
		PUSH_MOTOR_IN2 =1;
		moto=-moto;
	}
	else if( moto==0 )
	{
		TIM_SetCompare1(TIM8,moto);
		return ;
	}
	//速度最小限幅
	min_limit(&moto,PUSH_SPEED_MIN);
	TIM_SetCompare1(TIM8,moto);
}

/****
	*@note :大弹丸推动电机速度函数
	*@param[in] : 左电机速度 右电机速度
	*/
void move_super_dial_motor(int16_t moto)
{
	if((moto>0))
	{
		DIAL_MOTOR_IN1 =1;
		DIAL_MOTOR_IN2 =0;
		moto=moto;
	}
	else if( (moto<0))
	{
		DIAL_MOTOR_IN1 =0;
		DIAL_MOTOR_IN2 =1;
		moto=-moto;
	}
	else if( moto==0 )
	{
		TIM_SetCompare4(TIM8,moto);
		return ;
	}
	//速度最小限幅
	min_limit(&moto,SUPER_DIAL_SPEED_MIN);
	TIM_SetCompare4(TIM8,moto);
}

