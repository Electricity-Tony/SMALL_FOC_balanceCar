/**
 * @file app_imu.h
 * @brief imu相关配置头文件
 * @author Asn (921576434@qq.com)
 * @date 2019.11
 * @version 2.5
 * @copyright Copyright (c) RM2020电控
 * @par 日志:
 *		v1.0 移植mpu9250库\n
 *		v1.1 更改mahony算法的PI控制参数\n
 *		v1.2 发现之前陀螺仪硬件滤波参数有错误，已修正\n
 *		v2.0 现已加入磁力计，并优化了算法，加快了初始化收敛速度\n
 *		v2.1 正式更名app_filter为app_math，并把limit和invsqrt移进去\n
 *		v2.2 优化PI控制器参数，并加入误差预处理，当加速度模量过大或过小都舍弃误差\n
 *		v2.3 增加了soft角度解算条件，避免一开始收敛过程中累积圈数，增加了条件编译，解决不用磁力计的警告\n
 *		v2.4 更改了加速度计滤波参数\n
 *		v2.5 打开了加速度解算区间（原区间0.75g~1.25g）\n
 */
#ifndef __APP_IMU_H
#define __APP_IMU_H

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#include "stm32f1xx_hal.h"

#define ZERO_SAMPLE_NUM 100 // 零点采样的数据量
#define APP_MATH_ABS(x) ((x) > 0 ? (x) : -(x))
	typedef struct
	{
		struct
		{
			float Gyro[3];
			int16_t Data[3][ZERO_SAMPLE_NUM]; // 采样零点数据
			float Sum[3];					  // 采样值之和
			uint16_t Cnt[3];				  // 零点计数
			float Mag[3];
		} offset;
		struct
		{
			int16_t Accel[3];
			int16_t Gyro[3];
			int16_t lastGyro[3];
			int16_t Mag[3];
			int16_t MPU_Temp;
		} original; // 原始的
		struct
		{
			float Accel[3];
			float Gyro[3];
			float Mag[3];
		} kalman;
		struct
		{
			float Accel[3];
			float Gyro[3];
			float Mag[3];
		} LPF;
		struct
		{
			float Accel[3];
			float Gyro[3];
			float Mag[3];
			float MPU_Temp;
		} unitized; // 单位化的
		struct
		{
			float Roll;
			float Pitch;
			float Yaw;
		} soft;
		struct
		{
			float Roll;
			float Pitch;
			float Yaw;
		} integral;
		float Angle_Rate[3];
		float Roll;
		float Pitch;
		float Yaw;
		uint8_t reset;
		uint8_t ready;				  // 建议独立创建一个监控任务，以低频刷新ready为0，实时监测9250是否工作
		uint8_t isThisTimeInvalid[3]; // 若本次静态零点采集失败，则置一
	} MPU_DEF;

	extern MPU_DEF app_imu_data;

	uint8_t app_imu_Init(void);
	void app_imu_So3thread(void);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif