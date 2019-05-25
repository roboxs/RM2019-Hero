/**
 * 
 * @brief 串口1接收遥控的数据结构
 *
 */

#include <driver_dbus.h>

volatile unsigned char g_sbus_rx_buffer[25];//遥控接受到数据
RC_Ctl_t RC_Ctl;


//返回遥控器指针
const RC_Ctl_t *get_remote_control_point(void)
{
    return &RC_Ctl;
}
