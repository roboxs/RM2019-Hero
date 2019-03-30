#include <driver_chassis.h>
#include <driver_dbus.h>


#define ABS(x)	((x>0) ? (x) : (-x))


ChassisStucture_s g_chassis;



void set_chassis_speed(float xspeed ,float yspeed, float yaw)
{
	xspeed = (xspeed >  MAX_SPEED_CHASSIS)?(MAX_SPEED_CHASSIS) : (xspeed);
	xspeed = (xspeed < -MAX_SPEED_CHASSIS)?(-MAX_SPEED_CHASSIS) : (xspeed);
	yspeed = (yspeed >  MAX_SPEED_CHASSIS)?(MAX_SPEED_CHASSIS) : (yspeed);
	yspeed = (yspeed < -MAX_SPEED_CHASSIS)?(-MAX_SPEED_CHASSIS) : (yspeed);
	g_chassis.target_vx = xspeed;
	g_chassis.target_vy = yspeed;
	g_chassis.target_vw = yaw;
}

static void mecanum_calculate(float vx,float vy,float vw, short *speed)
{
	u8 i, index;
	float buffer[4]={0},speed_max = 0 ,param;
	
	buffer[LF] = vx + vy + vw;
	buffer[RF] = vx - vy - vw;
	buffer[LB] = vx - vy + vw;
	buffer[RB] = vx + vy - vw;
	
	
	 for(index = 0, speed_max = 0; index < 4; index++)
    {
        if((buffer[index] > 0 ? buffer[index] : -buffer[index]) > speed_max)
        {
            speed_max = (buffer[index] > 0 ? buffer[index] : -buffer[index]);
        }
    }
    if(MAX_SPEED_CHASSIS < speed_max)
    {
        param = (float)MAX_SPEED_CHASSIS / speed_max;
        speed[LF] = buffer[LF] * param;
        speed[RF] = buffer[RF] * param;
        speed[LB] = buffer[LB] * param;
        speed[RB] = buffer[RB] * param; 
    }
    else
    {
        speed[LF] = buffer[LF];
        speed[RF] = buffer[RF];
        speed[LB] = buffer[LB];
        speed[RB] = buffer[RB];
    }
}


void chassis_movement(void)
{
	short wheel_speed[4];
	mecanum_calculate(g_chassis.target_vx, g_chassis.target_vy, g_chassis.target_vw, wheel_speed);
	

	g_chassis.chassis_motor_lf.target = (wheel_speed[LF]*2);
	g_chassis.chassis_motor_rf.target = wheel_speed[RF]*2;
	g_chassis.chassis_motor_lb.target = (wheel_speed[LB]*2);
	g_chassis.chassis_motor_rb.target = wheel_speed[RB]*2;
	
	g_wheel_lf_pid_inner.target = g_chassis.chassis_motor_lf.target;
	g_wheel_rf_pid_inner.target = -g_chassis.chassis_motor_rf.target;
	g_wheel_lb_pid_inner.target = g_chassis.chassis_motor_lb.target;
	g_wheel_rb_pid_inner.target = -g_chassis.chassis_motor_rb.target;
	
	wheel_speed[LF]=pid_calculate(&g_wheel_lf_pid_inner, &g_wheel_lf_pid_outer);
	wheel_speed[RF]=pid_calculate(&g_wheel_rf_pid_inner, &g_wheel_rf_pid_outer);
	wheel_speed[LB]=pid_calculate(&g_wheel_lb_pid_inner, &g_wheel_lb_pid_outer);
	wheel_speed[RB]=pid_calculate(&g_wheel_rb_pid_inner, &g_wheel_rb_pid_outer);
	
	control_chassis_motor(wheel_speed[LF], wheel_speed[RF], wheel_speed[LB], wheel_speed[RB]);
}

void control_chassis_motor(int16_t chassis_motor_1,int16_t chassis_motor_2,int16_t chassis_motor_3,int16_t chassis_motor_4)
{
	CanTxMsg CAN1_ChassisMotorStr;
	
	CAN1_ChassisMotorStr.StdId=CONTROLCHASSIS;
	CAN1_ChassisMotorStr.IDE=CAN_Id_Standard;
	CAN1_ChassisMotorStr.RTR=CAN_RTR_Data;	 
	CAN1_ChassisMotorStr.DLC=0x08;
	
	CAN1_ChassisMotorStr.Data[0]=(u8)(chassis_motor_1 >> 8);
	CAN1_ChassisMotorStr.Data[1]=(u8)chassis_motor_1;
	CAN1_ChassisMotorStr.Data[2]=(u8)(chassis_motor_2 >> 8);
	CAN1_ChassisMotorStr.Data[3]=(u8)chassis_motor_2;
	CAN1_ChassisMotorStr.Data[4]=(u8)(chassis_motor_3 >> 8);
	CAN1_ChassisMotorStr.Data[5]=(u8)chassis_motor_3;
	CAN1_ChassisMotorStr.Data[6]=(u8)(chassis_motor_4 >> 8);
	CAN1_ChassisMotorStr.Data[7]=(u8)chassis_motor_4;
	
	CAN_Transmit(CAN1,&CAN1_ChassisMotorStr);
}

void test_chassis_motor(void)
{
	CanTxMsg CAN1_ChassisMotorStr;
	
	CAN1_ChassisMotorStr.StdId=CONTROLCHASSIS;
	CAN1_ChassisMotorStr.IDE=CAN_Id_Standard;
	CAN1_ChassisMotorStr.RTR=CAN_RTR_Data;	 
	CAN1_ChassisMotorStr.DLC=0x08;
	
	CAN1_ChassisMotorStr.Data[1]=0x01;
	CAN1_ChassisMotorStr.Data[0]=0xf4;
	
	CAN1_ChassisMotorStr.Data[3]=0x05;
	CAN1_ChassisMotorStr.Data[2]=0xDC;
	
	CAN1_ChassisMotorStr.Data[5]=0x05;
	CAN1_ChassisMotorStr.Data[4]=0xDC;
	
	CAN1_ChassisMotorStr.Data[7]=0x05;
	CAN1_ChassisMotorStr.Data[6]=0xDC;
	
	CAN_Transmit(CAN1,&CAN1_ChassisMotorStr);
}


