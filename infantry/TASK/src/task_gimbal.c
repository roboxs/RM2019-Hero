#include <task_gimbal.h>
#include <FreeRTOS.h>
#include <task.h>
#include <driver_dbus.h>
#include <task_imu.h>
#include <task_led.h>
#include <driver_dial.h>
#include <driver_shoot.h>
#include <task_chassis.h>
#include <user_math.h>
#include <task_vision.h>


GimbalMove_t g_gimbal_control;
float gkp=6,gki=0.08,gkd=0;
float i_angle =8.0;
float mid_target =261.5f;
float deadhand = 0.5f;


void gimbal_task(void *pvParameters)
{
	//等待电机反转
	vTaskDelay(10000);
	//云台的初始化
	gimbal_init(&g_gimbal_control);
	shoot_init();
	//完成初始化的时间
	g_gimbal_control.init_time = xTaskGetTickCount();
	while(1)
	{
		//云台电机数值更新
		gimbal_update(&g_gimbal_control);
		/*云台控制量设置*/
		if(g_gimbal_control.init_finish_flag == SET) gimbal_set_control(&g_gimbal_control); //云台初始化完成之后才能用鼠标键盘控制
		//云台进行pid计算
		gimbal_pid_calculate(&g_gimbal_control);
		//初始化完成之后 拨盘PID计算
		 if(g_gimbal_control.init_finish_flag == SET)
		 {			 
			shoot_control();
//			 g_gimbal_control.yaw_outer_pid.target = mid_target;
		 }
		//赋电流值
		control_gimbal_motor(g_gimbal_control.motor_yaw.give_current,g_gimbal_control.motor_pitch.give_current,
								g_dial_2006_motor.give_current, 0);
							
		//修改PID参数
//		pid_reset(&(g_gimbal_control.yaw_inner_pid),gkp,gki,gkd);
//		g_gimbal_control.yaw_outer_pid.integral.err_min = i_angle;
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
	/*积分分离误差角度*/
	gimbal_control_init->pitch_outer_pid.integral.err_min = 5.0;
	/*设置当前云台的模式*/
	gimbal_control_init->mode = GIMBAL_MOTOR_ENCONDE;
	/*设置云台初始的角度*/
	gimbal_control_init->yaw_outer_pid.target = GIMBAL_YAW_MID_ANGLE;
	gimbal_control_init->pitch_outer_pid.target = GIMBAL_PITCH_MID_ANGLE;
}

static void gimbal_set_control(GimbalMove_t *gimbal_set)
{
	float yaw_add=0,pitch_add=0;//两个轴增加的角度
	
	
	//设置当前云台角度的控制
	gimbal_set_target(&yaw_add, &pitch_add);
	//如果当前的反馈值是陀螺仪角度值
	if(gimbal_set->mode == GIMBAL_MOTOR_GYROSCOPE)
	{
		gimbal_motor_angle_limit_gyro(gimbal_set,yaw_add,pitch_add);
	}
	//如果当前的反馈值是电机的角度
	if(gimbal_set->mode == GIMBAL_MOTOR_ENCONDE)
	{
		//云台最大角度的限制
		gimbal_motor_angle_limit(gimbal_set, yaw_add, pitch_add);
	}
	/*视觉模式*/
	if(RC_Ctl.key.v & KEY_PRESSED_OFFSET_R)
	{
//		if(g_vision_data.get_target_angle_yaw == g_vision_data.last_get_target_angle_yaw)//任务视觉没有发送数据
//		{
//			gimbal_set->yaw_outer_pid.target = g_cloud_gyroscope.real_yaw;
//		}
//		else
//		{
//			gimbal_set->yaw_outer_pid.target = g_vision_data.get_target_angle_yaw;
//		}
		gimbal_set->yaw_outer_pid.target = g_vision_data.get_target_angle_yaw;
		gimbal_set->pitch_outer_pid.target = g_vision_data.get_target_angle_pitch;
		vision_state = VISION_SMALL;
	}
	else
	{
		vision_state = VISION_CLOSE;
	}
}

static void gimbal_set_target(float *add_yaw, float *add_pitch)
{
	static float rc_yaw_add, rc_pitch_add;
//	static int16_t yaw_channel = 0, pitch_channel = 0;
	
	//遥控输入限制
//	yaw_channel = add_rc_dead_limit(& RC_Ctl.rc.ch0, RC_DEADBAND);
//	pitch_channel = add_rc_dead_limit(& RC_Ctl.rc.ch1, RC_DEADBAND/5);
	
	rc_yaw_add = RC_Ctl.mouse.x * YAW_MOUSE_SEN;
	rc_pitch_add =RC_Ctl.mouse.y * PITCH_MOUSE_SEN;
		
	*add_yaw = rc_yaw_add;
	*add_pitch = rc_pitch_add;
}

static void gimbal_pid_calculate(GimbalMove_t * gimbal_calculate)
{
	//当前云台系统运行的时间
	gimbal_calculate->run_time = xTaskGetTickCount();
	//云台初始化完成
//	if( (gimbal_calculate->init_finish_flag == RESET) &&(gimbal_calculate->run_time - gimbal_calculate->init_time > 5000) 
//		 && ABS(gimbal_calculate->yaw_outer_pid.target - gimbal_calculate->yaw_outer_pid.measure) <1.5f
//		 && ABS(gimbal_calculate->pitch_outer_pid.target - gimbal_calculate->pitch_outer_pid.measure) <0.1f) 
		gimbal_calculate->init_finish_flag = SET;
	
	//当云台初始化完成后
	if((g_gimbal_control.yaw_inner_pid.feedback_loop == ANGLE_LOOP) && gimbal_calculate->init_finish_flag == SET )
	{
		//云台控制切换为双环控制
		g_gimbal_control.yaw_inner_pid.feedback_loop = DOUBLE_LOOP;
		g_gimbal_control.yaw_outer_pid.feedback_loop = DOUBLE_LOOP;
		g_gimbal_control.pitch_inner_pid.feedback_loop = DOUBLE_LOOP;
		g_gimbal_control.pitch_outer_pid.feedback_loop = DOUBLE_LOOP;
		//外环的参数修改
		pid_reset(&(g_gimbal_control.pitch_outer_pid), PITCH_OUTER_P, PITCH_OUTER_I, PITCH_OUTER_D);
		pid_reset(&(g_gimbal_control.yaw_outer_pid), YAW_OUTER_P, YAW_OUTER_I, YAW_OUTER_D);
		//变量清空
		g_gimbal_control.yaw_outer_pid.pout = 0;
		g_gimbal_control.yaw_outer_pid.iout = 0;
		g_gimbal_control.yaw_outer_pid.dout = 0;
		g_gimbal_control.yaw_inner_pid.pout = 0;
		g_gimbal_control.yaw_inner_pid.iout = 0;
		g_gimbal_control.yaw_inner_pid.dout = 0;
		g_gimbal_control.pitch_outer_pid.pout = 0;
		g_gimbal_control.pitch_outer_pid.iout = 0;
		g_gimbal_control.pitch_outer_pid.dout = 0;
		g_gimbal_control.pitch_inner_pid.pout = 0;
		g_gimbal_control.pitch_inner_pid.iout = 0;
		g_gimbal_control.pitch_inner_pid.dout = 0;
		//积分分离清空
		g_gimbal_control.yaw_outer_pid.integral.err_min = 0;
		g_gimbal_control.pitch_outer_pid.integral.err_min = 0;
		//初始化完成之后重新设置云台的初始值
		g_gimbal_control.yaw_outer_pid.target = g_cloud_gyroscope.real_yaw;
		//设置云台控制的模式
		g_gimbal_control.mode = GIMBAL_MOTOR_GYROSCOPE;
	}
	
	//云台PID计算
	pid_calculate( &(gimbal_calculate->yaw_inner_pid),  &(gimbal_calculate->yaw_outer_pid));
	pid_calculate( &(gimbal_calculate->pitch_inner_pid), &(gimbal_calculate->pitch_outer_pid));
	
	//赋云台电机的电流值
	if(gimbal_calculate->init_finish_flag == SET)
	{
		gimbal_calculate->motor_yaw.give_current = (int16_t) gimbal_calculate->yaw_inner_pid.pos_out;
		gimbal_calculate->motor_pitch.give_current = (int16_t) gimbal_calculate->pitch_inner_pid.pos_out;
	}
	else if(gimbal_calculate->init_finish_flag == RESET)
	{
		gimbal_calculate->motor_yaw.give_current = (int16_t) gimbal_calculate->yaw_outer_pid.pos_out;
		gimbal_calculate->motor_pitch.give_current = (int16_t) gimbal_calculate->pitch_outer_pid.pos_out;
	}
}
static void gimbal_update(GimbalMove_t *gimbal_update)
{
	//在can中断中更新速度和角度
	gimbal_update->yaw_outer_pid.measure = gimbal_update-> motor_yaw.angle_measure;
	gimbal_update->yaw_inner_pid.measure = gimbal_update-> motor_yaw.speed_measure;
	gimbal_update->pitch_outer_pid.measure = gimbal_update->motor_pitch.angle_measure;
	gimbal_update->pitch_inner_pid.measure = gimbal_update->motor_pitch.speed_measure;
}

static void gimbal_motor_angle_limit(GimbalMove_t *motor_limit,float yaw_add, float pitch_add)
{
	motor_limit->yaw_outer_pid.target +=yaw_add;
	motor_limit->pitch_outer_pid.target += pitch_add;
	if(motor_limit->yaw_outer_pid.target > GIMBAL_YAW_MIN_ANGLE)
	{
		motor_limit->yaw_outer_pid.target = GIMBAL_YAW_MIN_ANGLE;
	}
	else if(motor_limit->yaw_outer_pid.target < GIMBAL_YAW_MAX_ANGLE)
	{
		motor_limit->yaw_outer_pid.target = GIMBAL_YAW_MAX_ANGLE;
	}
	
	if(motor_limit->pitch_outer_pid.target > GIMBAL_PITCH_MAX_ANGLE)
	{
		motor_limit->pitch_outer_pid.target = GIMBAL_PITCH_MAX_ANGLE;
	}
	else if(motor_limit->pitch_outer_pid.target < GIMBAL_PITCH_MIN_ANGLE)
	{
		motor_limit->pitch_outer_pid.target = GIMBAL_PITCH_MIN_ANGLE;
	}
	
}

static void gimbal_motor_angle_limit_gyro(GimbalMove_t *motor_limit,float yaw_add, float pitch_add)
{
	//对yaw轴输入角度进行限制
//	if((g_chassis_move.chassis_angle_pid.measure - GIMBAL_YAW_MID_ANGLE > 20) && (yaw_add < 0)) //在最左边,只累积正的角度值
//	{
//		yaw_add = 0;
//	}
//	else if((g_chassis_move.chassis_angle_pid.measure - GIMBAL_YAW_MID_ANGLE < -20) && (yaw_add > 0))//在最右边,只累积负的角度值
//	{
//		yaw_add = 0;
//	}
	
		/*                        */
	if(!RC_Ctl.mouse.x&& ABS(motor_limit->yaw_inner_pid.err[NOW]) < 2.0f) 
	{
		GPIO_ResetBits(GPIOG, GPIO_Pin_5);     //关闭
		motor_limit->yaw_outer_pid.target = g_cloud_gyroscope.real_yaw;
	}
	else
	{
		GPIO_SetBits(GPIOG, GPIO_Pin_5);     //关闭
		motor_limit->yaw_outer_pid.target += yaw_add;
	}
	motor_limit->yaw_outer_pid.target += yaw_add;
	motor_limit->pitch_outer_pid.target += pitch_add;
	
	
	//角度大于360需要过零处理
	if(motor_limit->yaw_outer_pid.target > 360) motor_limit->over_zero_flag = 1;
	//角度小于-360需要反向过零处理
	else if(motor_limit->yaw_outer_pid.target < -360) motor_limit->over_zero_flag_rel = 1;
	
	if(motor_limit->over_zero_flag == 1)
	{
		if(ABS(motor_limit->yaw_outer_pid.measure) < 5)
		{
			motor_limit->yaw_outer_pid.target = motor_limit->yaw_outer_pid.target - 360;
			motor_limit->over_zero_flag = 0;
		}
	}
	else if(motor_limit->over_zero_flag_rel == 1)
	{
		if(ABS(motor_limit->yaw_outer_pid.measure) < 5)
		{
			motor_limit->yaw_outer_pid.target = motor_limit->yaw_outer_pid.target + 360;
			motor_limit->over_zero_flag_rel = 0;
		}
	}
	//目标值重新置零
	if(ABS(motor_limit->yaw_outer_pid.target) == 360) motor_limit->yaw_outer_pid.target = 0;
	//pitch角度限制
	if(motor_limit->pitch_outer_pid.target > GIMBAL_PITCH_MAX_ANGLE)
	{
		motor_limit->pitch_outer_pid.target = GIMBAL_PITCH_MAX_ANGLE;
	}
	else if(motor_limit->pitch_outer_pid.target < GIMBAL_PITCH_MIN_ANGLE)
	{
		motor_limit->pitch_outer_pid.target = GIMBAL_PITCH_MIN_ANGLE;
	}
	
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
	static void gimbal_test_pitch(GimbalMove_t * gimbal_calculate)
	{
		static int test_count=0;
		gimbal_calculate->pitch_inner_pid.feedback_loop = SPEED_LOOP;
		pid_reset(&g_gimbal_control.pitch_inner_pid, pitch_test_kp, pitch_test_ki, pitch_test_kd);
		
		switch(test_count)
		{
			case 0:
				gimbal_calculate->pitch_inner_pid.target = 1000;
				break;
			case 1000:
				gimbal_calculate->pitch_inner_pid.target = 0;
				break;
			case 1500:
				gimbal_calculate->pitch_inner_pid.target = -1000;
				break;
			case 2500:
				gimbal_calculate->pitch_inner_pid.target = 0;
				break;
		}
		test_count++;
		pid_calculate( &(gimbal_calculate->pitch_inner_pid),  &(gimbal_calculate->pitch_outer_pid));
		gimbal_calculate->motor_pitch.give_current = gimbal_calculate->pitch_inner_pid.pos_out;
		if(test_count ==3000) test_count =0;
	}
	
#endif






static void control_gimbal_motor(int16_t yaw, int16_t pitch, int16_t dial1, int16_t dial2)
{
	CanTxMsg CAN1_CouldMotorStr;
	
	CAN1_CouldMotorStr.StdId=GIMBAL_CONTROL_ID;
	CAN1_CouldMotorStr.IDE=CAN_Id_Standard;
	CAN1_CouldMotorStr.RTR=CAN_RTR_Data;	 
	CAN1_CouldMotorStr.DLC=0x08;
	CAN1_CouldMotorStr.Data[0]= (yaw >> 8);
	CAN1_CouldMotorStr.Data[1]= yaw;
	CAN1_CouldMotorStr.Data[2]= (pitch >> 8);
	CAN1_CouldMotorStr.Data[3]= pitch;
	CAN1_CouldMotorStr.Data[4]= (dial1 >> 8);
	CAN1_CouldMotorStr.Data[5]= dial1;
	CAN1_CouldMotorStr.Data[6]= (dial2 >> 8);
	CAN1_CouldMotorStr.Data[7]= dial2;
	
	CAN_Transmit(CAN1,&CAN1_CouldMotorStr);
}
