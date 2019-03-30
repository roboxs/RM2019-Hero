/*拨盘电机用3510 和 2006*/

#include <driver_dial.h> 
#include <driver_control.h>
#include <driver_dbus.h>


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
		case 1:
			break;		
		case 3:
			g_dial_2006_pid_inner.target =DIAL_2006_SPEED;
			break;
		case 2:
			g_dial_2006_pid_inner.target =0;
	}
	
	pid_calculate(&g_dial_2006_pid_inner,&g_dial_2006_pid_outer);
	pid_calculate(&g_dial_3510_pid_inner,&g_dial_3510_pid_outer);
}

void singel_shoot()
{
	
}

