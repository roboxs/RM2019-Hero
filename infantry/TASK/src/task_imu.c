#include <task_imu.h>
#include <task_led.h>
#include <driver_imu.h>
#include <inv_mpu.h>
#include <driver_filter.h>

#define GYRO_SEN 0.00106526443603169529841533860381f

Gyroscope_t g_cloud_gyroscope;
Gyroscope_t g_chassis_gyroscope;
Filter_t g_chebyshev_filter;	//�б�ѩ���˲���

void imu_task(void *pvParameters)
{
	float pitch, roll, yaw;
	short raw_gx,raw_gy,raw_gz;
	
	//���Եȴ������������ʼ��λ��
	//vTaskDelay(100);
	BSP_IMU_Init();
	//dmp��ʼ�� ����֮��MPU��ʼ�����
	LED1=0;
	while(mpu_dmp_init());
	LED1=1;
	
	
	while(1)
	{
		LED2=~LED2;
		//��ȡŷ����
		mpu_dmp_get_data(&pitch, &roll, &yaw);
		//��ȡ�����ǽ��ٶ�ԭʼ����
		MPU_Get_Gyroscope(&raw_gx, &raw_gy, &raw_gz);
		//
		g_cloud_gyroscope.pitch = pitch;
		g_cloud_gyroscope.roll  = roll;
		g_cloud_gyroscope.yaw   = yaw;
		//��ԭ���ݽ����˲�����
		g_chebyshev_filter.raw_value=-raw_gx;
		Chebyshev50HzLPF(&g_chebyshev_filter);
		//��̨x����ٶȻ�ȡ
		g_cloud_gyroscope.gx=g_chebyshev_filter.filtered_value;
		g_cloud_gyroscope.gy=-raw_gy;
		g_cloud_gyroscope.gz=-raw_gz;

		
		g_chassis_gyroscope.pitch = pitch;
		g_chassis_gyroscope.roll  = roll;
		g_chassis_gyroscope.yaw   = yaw;
		
		g_chassis_gyroscope.gx=raw_gx;
		g_chassis_gyroscope.gy=raw_gy;
		g_chassis_gyroscope.gz=raw_gz;
		vTaskDelay(1);
	}
}

