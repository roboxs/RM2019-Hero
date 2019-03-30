#include <driver_cloudmotor.h>
#include <delay.h>
#include <driver_control.h>
#include <task_gimbal.h>

//void could_motor_init(void)
//{
//	int i;
//	float yaw_speed , pitch_speed;
//	g_gimbal_control.pitch_outer_pid.target=YAWCENTER;
//	g_pitch_pid_outer.target=PITCHCENTER;
//	
//	for(i=0;i<400;i++)
//	{
//		yaw_speed = pid_calculate(&g_yaw_pid_inner , &g_yaw_pid_outer);
//		pitch_speed = pid_calculate(&g_pitch_pid_inner , &g_pitch_pid_outer);
//		delay_ms(5);
//	}
//}


