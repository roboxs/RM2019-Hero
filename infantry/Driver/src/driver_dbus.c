/**
 * 
 * @brief ����1����ң�ص����ݽṹ
 *
 */

#include <driver_dbus.h>

volatile unsigned char g_sbus_rx_buffer[25];//ң�ؽ��ܵ�����
RC_Ctl_t RC_Ctl;


//����ң����ָ��
const RC_Ctl_t *get_remote_control_point(void)
{
    return &RC_Ctl;
}
