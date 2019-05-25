#ifndef  _DRIVER_DBUS_H
#define	 _DRIVER_DBUS_H

#include<stm32f4xx.h>

/* ----------------------- 电脑键盘定义-------------------------------- */
#define KEY_PRESSED_OFFSET_W ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B ((uint16_t)1 << 15)

/* ----------------------- DJI遥控结构体 ------------------------------------- */
typedef struct
{
		struct
		{
				short ch0;
				short ch1;
				short ch2;
				short ch3;
				uint8_t s1;
				uint8_t s2;
		}rc;
		
		struct
		{
				int16_t x;
				int16_t y;
				int16_t z;
				uint8_t press_l;
				uint8_t press_r;
		}mouse;
		
		struct
		{
				uint16_t v;
		}key;
}RC_Ctl_t;

const RC_Ctl_t *get_remote_control_point(void);

extern volatile unsigned char g_sbus_rx_buffer[];
extern RC_Ctl_t RC_Ctl;

#endif


