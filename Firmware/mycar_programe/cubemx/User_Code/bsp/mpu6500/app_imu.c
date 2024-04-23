/**
 * @file app_imu.c
 * @brief imu相关配置头文件
 * @author Asn (921576434@qq.com)
 * @date 2020.02.26
 * @version 1.0
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
 *		v2.6 把磁力计获取速率变慢\n
 *		v2.7 把加速度计滤波换成了kalman滤波
 *		v2.8 之前的温度计算方式有误，已修正
 *		v2.9 把四元数PI参数根据加速度计硬件滤波频率做了调整
 */
#include "app_imu.h"
#include "app_math.h"
#include "bsp_mpu6500.h"
// #include "bsp_ak8975.h"

#define USE_LPF // 使用低通滤波
#ifdef USE_MAG
#define MAG_OFFSET
#endif
#define USE_OFFSET	   // 使用零点校正
#define DYNAMIC_OFFSET // 使用动态校正零点，这个可以使用，imu还是主要受上面的影响

// 需调整的变量,调完可以使用static
#ifdef USE_MAG					 // 这个宏定义在bsp_ak8975文件定义，目前不可用
float so3_comp_params_Kp = 2.0f; // 四元数校正PI参数
float so3_comp_params_Ki = 0.05f;
float so3_comp_params_mKp = 1.0f;
#else
float so3_comp_params_Kp = 3.0f; // 四元数校正PI参数
float so3_comp_params_Ki = 0.05f;
#endif
uint16_t Zero_Threshold[3] = {50, 50, 50}; // 用于零点校正，判断数据是否为静止数据------------------------> 100足够大了，最好看一下原始数据，看看够不够（其实有点大）<--------------------建议更改，提升自检要求
float Dynamic_Zero_Thre = 4.0f;				  // 动态校正的零点阈值
float Offset_Coeff[3] = {1.0f, 1.0f, 1.0f};	  // 对量程进行校正，角速度校正系数           ！！！！！！在此标准化量程，即转一个直角，显示90度的效果！！！！！！！！！！！！！！
float manualOffsetGyro[3] = {0, 0, 0};		  // 手动添加校正值，解决零点误差在+-1之间的问题，因为原始数据是int16_t类型，这个参数能解决有规律的漂移,追求完美的可以试试

static int16_t Flash_Val[4]; //[0]记录Flash写入的次数，[1-3]为陀螺仪零点值
static float app_imu_OffsetVal[3];
/* 功能性函数宏定义 */
#define MICROS() (1000 * HAL_GetTick()) // 计时，单位us,除1000000是秒

/* 变量宏定义 */
#define SAMPLE_FREQUENCY 1000	  // 采样频率
#define GYRO_CUT_OFF_FREQUENCY 40 // 截止频率，这里要根据情况和需求更改  !!!!!!!!!!!!!!往届有把截止频率改的特别小的（10左右），底盘实测收敛很慢，会造成超调，故改大。过大的话过滤高频噪声的能力差
#define ACCE_CUT_OFF_FREQUENCY 20
#define MAG_CUT_OFF_FREQUENCY 20
#define SELF_TEST_T 5						  // 自检时间，单位秒
#define G 9.80665f							  // 重力加速度
#define TORADIAN 0.0174533f					  // 转换为弧度制，四元数的姿态解算需要使用 π/180
#define TOANGLE 57.2957795f					  // 最后解算出来的弧度制转换为角度
#define ACC_RESOLUTION (16.0f * G / 32768.0f) // 加速度计分辨率 m/s^2/LSb
#define GYRO_RESOLUTION (2000.0f / 32768.0f)  // 陀螺仪分辨率   dps/LSb
#define MAG_RESOLUTION (0.3f * 0.001f)		  // 磁力计分辨率   uT/LSb
/* 结构体 */
MPU_DEF app_imu_data;
kalman_filter AccFilter[3];
LPF2 Gyro_LPF[3];

// 方向余弦矩阵
static float att_matrix[3][3]; // 必须由姿态解算算出该矩阵

/**
 * @brief imu零漂计算初始化
 * @return uint8_t
 */
uint8_t app_imu_Init(void)
{
	static uint16_t unstable_num;
	app_imu_data.reset = 1;
#ifdef USE_LPF
	for (uint8_t i = 0; i < 3; i++)
	{
		app_math_LPF2pSetCutoffFreq(&Gyro_LPF[i], SAMPLE_FREQUENCY, GYRO_CUT_OFF_FREQUENCY);
	}
#endif
	static uint8_t flag = 1; // 采样标志位
	static float tick;		 // 用于超时计数
	while (flag)
	{
		tick = MICROS();
		for (uint16_t num = 0; num < ZERO_SAMPLE_NUM; num++)
		{ /*零点采样*/
			app_imu_data.original.Gyro[0] = bsp_mpu6500_ReadReg(MPU6500_GYRO_XOUT_H) << 8 | bsp_mpu6500_ReadReg(MPU6500_GYRO_XOUT_L);
			app_imu_data.original.Gyro[1] = bsp_mpu6500_ReadReg(MPU6500_GYRO_YOUT_H) << 8 | bsp_mpu6500_ReadReg(MPU6500_GYRO_YOUT_L);
			app_imu_data.original.Gyro[2] = bsp_mpu6500_ReadReg(MPU6500_GYRO_ZOUT_H) << 8 | bsp_mpu6500_ReadReg(MPU6500_GYRO_ZOUT_L);
			while (APP_MATH_ABS(app_imu_data.original.Gyro[0]) > Zero_Threshold[0] || APP_MATH_ABS(app_imu_data.original.Gyro[1]) > Zero_Threshold[1] || APP_MATH_ABS(app_imu_data.original.Gyro[2]) > Zero_Threshold[2])
			{
				app_imu_data.original.Gyro[0] = bsp_mpu6500_ReadReg(MPU6500_GYRO_XOUT_H) << 8 | bsp_mpu6500_ReadReg(MPU6500_GYRO_XOUT_L);
				app_imu_data.original.Gyro[1] = bsp_mpu6500_ReadReg(MPU6500_GYRO_YOUT_H) << 8 | bsp_mpu6500_ReadReg(MPU6500_GYRO_YOUT_L);
				app_imu_data.original.Gyro[2] = bsp_mpu6500_ReadReg(MPU6500_GYRO_ZOUT_H) << 8 | bsp_mpu6500_ReadReg(MPU6500_GYRO_ZOUT_L);
				unstable_num++;
				if ((MICROS() - tick) / 1000000.0f > SELF_TEST_T)
				{
					for (uint8_t j = 0; j < 3; j++)
					{
						app_imu_data.offset.Gyro[j] = Flash_Val[j + 1] / 300.0f;
						app_imu_data.isThisTimeInvalid[j] = 1;
						app_imu_data.offset.Cnt[j] = 0; // 清除计数
					}
					return 0; // 5s内初始化不成功就使用Flash内存的历史零点值
				}
			}
			for (uint8_t k = 0; k < 3; k++)
			{
				app_imu_data.offset.Data[k][app_imu_data.offset.Cnt[k]] = app_imu_data.original.Gyro[k]; //<零点采样值
				app_imu_data.offset.Sum[k] += app_imu_data.offset.Data[k][app_imu_data.offset.Cnt[k]];	 //<零点采样和
				app_imu_data.offset.Cnt[k]++;															 // 采样计数
			}
		}
		if (unstable_num > 300)
		{ /*采样数据无效*/
			unstable_num = 0;
			for (uint8_t i = 0; i < 3; i++)
			{
				app_imu_data.offset.Sum[i] = 0;
				app_imu_data.offset.Cnt[i] = 0;
			}
		}
		else
		{
			for (uint8_t i = 0; i < 3; i++)
			{ /*采样数据有效*/
				app_imu_OffsetVal[i] = app_imu_data.offset.Gyro[i] = app_imu_data.offset.Sum[i] / ZERO_SAMPLE_NUM;
				Flash_Val[i + 1] = (int16_t)app_imu_data.offset.Gyro[i] * 300; // 更新flash的值，×300的目的是让数值更准一点，毕竟将float转成了int16_t存起来的
				app_imu_data.offset.Cnt[i] = 0;
			}
			Flash_Val[0]++;
			flag = 0;
		}
	}
	return 1;
}

/**
 * @brief 读取原始数据和单位换算
 */
static void MPU_Read_Raw(void)
{
	static uint8_t dynamicFlag[3];
#ifdef USE_MAG
	static uint8_t mag_read_cnt = 0;
	static uint8_t akm_data[6];
#endif
	static uint8_t mpu_data_buf[14];

	/* 读取加速度计&陀螺仪 */
	bsp_mpu6500_ReadRegs(MPU6500_ACCEL_XOUT_H, mpu_data_buf, 14);
	app_imu_data.original.Accel[0] = (mpu_data_buf[0] << 8 | mpu_data_buf[1]);
	app_imu_data.original.Accel[1] = (mpu_data_buf[2] << 8 | mpu_data_buf[3]);
	app_imu_data.original.Accel[2] = (mpu_data_buf[4] << 8 | mpu_data_buf[5]);
	app_imu_data.original.MPU_Temp = (mpu_data_buf[6] << 8 | mpu_data_buf[7]);
	app_imu_data.unitized.MPU_Temp = app_imu_data.original.MPU_Temp / 326.8f + 25.f;
	app_imu_data.original.Gyro[0] = (mpu_data_buf[8] << 8 | mpu_data_buf[9]);
	app_imu_data.original.Gyro[1] = (mpu_data_buf[10] << 8 | mpu_data_buf[11]);
	app_imu_data.original.Gyro[2] = (mpu_data_buf[12] << 8 | mpu_data_buf[13]);
#ifdef USE_MAG
	/* 读取磁力计 */
	if (mag_read_cnt == 5)
	{
		bsp_ak8975_ReadRegs(AK8975_HXL_REG, akm_data, 6);
		bsp_ak8975_Trig();
		bsp_ak8975_ReadRegs(AK8975_ASAX_REG, bsp_ak8975_Asa, 3);
		bsp_ak8975_Trig();
		// AK8963_ASA[i++] = (s16)((data - 128.0f) / 256.0f + 1.0f) ;	调节校准的公式
		for (uint8_t i = 0; i < 3; i++)
			app_imu_data.original.Mag[i] = (akm_data[i * 2 + 1] << 8 | akm_data[i * 2]);
		mag_read_cnt = 0;
	}
	mag_read_cnt++;
#endif
	for (uint8_t i = 0; i < 3; i++)
	{
		/* 加速度计进行卡尔曼滤波，陀螺仪二阶低通滤波 */
#ifdef USE_LPF
		app_imu_data.kalman.Accel[i] = app_math_Kalman(&AccFilter[i], (float)app_imu_data.original.Accel[i]);
		app_imu_data.LPF.Gyro[i] = app_math_LPF2pApply(&Gyro_LPF[i], (float)app_imu_data.original.Gyro[i]);
		/* 取角速度 */
		app_imu_data.Angle_Rate[i] = (float)(app_imu_data.LPF.Gyro[i] - app_imu_data.offset.Gyro[i] + manualOffsetGyro[i]) * Offset_Coeff[i]; // 16位量程，只对原始数据进行补偿和滤波处理,需要根据9250改
		/* 单位化 */
#ifdef USE_OFFSET
		app_imu_data.unitized.Gyro[i] = app_imu_data.Angle_Rate[i] * GYRO_RESOLUTION * TORADIAN; // rad/s
#else
		app_imu_data.unitized.Gyro[i] = (float)app_imu_data.LPF.Gyro[i] * Offset_Coeff[i] * GYRO_RESOLUTION * TORADIAN; // rad/s
#endif
		app_imu_data.unitized.Accel[i] = (float)app_imu_data.kalman.Accel[i] * ACC_RESOLUTION;
#endif
#ifdef DYNAMIC_OFFSET
		/*静止时，更新零点*/
		if (APP_MATH_ABS(app_imu_data.Angle_Rate[i]) < Dynamic_Zero_Thre && APP_MATH_ABS(app_imu_data.offset.Gyro[i] - app_imu_data.original.lastGyro[i]) < 3)
		{
			dynamicFlag[i]++;
			if (dynamicFlag[i] >= 200)
				dynamicFlag[i] = 200; // 限位
		}
		else
			dynamicFlag[i] = 0;
		if (dynamicFlag[i] >= 50)
		{																																							  // 大于50个周期开始更新零点
			app_imu_data.offset.Sum[i] -= app_imu_data.offset.Data[i][app_imu_data.offset.Cnt[i]];																	  //< 清除旧数据
			app_imu_data.offset.Data[i][app_imu_data.offset.Cnt[i]] = app_imu_data.original.Gyro[i];																  //< 更新数据
			app_imu_data.offset.Sum[i] += app_imu_data.offset.Data[i][app_imu_data.offset.Cnt[i]];																	  //< 加入新数据
			if (app_imu_data.isThisTimeInvalid[i] == 0)																												  /* 启动时采样成功 */
				app_imu_data.offset.Gyro[i] = app_math_Limit(app_imu_data.offset.Sum[i] / ZERO_SAMPLE_NUM, app_imu_OffsetVal[i] + 0.5f, app_imu_OffsetVal[i] - 0.5f); //< 更新零点
			app_imu_data.offset.Cnt[i]++;
			if (app_imu_data.offset.Cnt[i] == ZERO_SAMPLE_NUM)
			{
				app_imu_data.offset.Cnt[i] = 0;
				app_imu_data.isThisTimeInvalid[i] = 0; /* 若启动时没有初始化成功，等待动态采样成功，启用动态校正值 */
			}
			app_imu_data.Angle_Rate[i] = 0;
		}
#endif
		app_imu_data.original.lastGyro[i] = app_imu_data.original.Gyro[i];
	}
}

/***************************四元数解算部分*********************************/
/* 无需调整的全局变量 */
static float dq0 = 0.0f, dq1 = 0.0f, dq2 = 0.0f, dq3 = 0.0f; /** quaternion of sensor frame relative to auxiliary frame */
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	 /** quaternion of sensor frame relative to auxiliary frame */
static float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
static float gyro_bias[3];
static uint8_t bFilterInit;
//! Using accelerometer, sense the gravity vector.
//! Using magnetometer, sense yaw.

/**
 * @brief 迭代初始化
 */
static void NonlinearSO3AHRSinit(float ax, float ay, float az, float mx,
								 float my, float mz) // 其实这个函数是没什么用的，他本身就是利用加速度计和磁力计算出偏航角，再算出四元数，但是一般不用磁力计，懒得注释进宏定义了
{
	float initialRoll, initialPitch;
	float cosRoll, sinRoll, cosPitch, sinPitch;
	float magX, magY;
	float initialHdg, cosHeading, sinHeading;

	initialRoll = 3.1415f + atan2(-ay, -az);
	initialPitch = -atan2(ax, -az);

	cosRoll = cosf(initialRoll);
	sinRoll = sinf(initialRoll);
	cosPitch = cosf(initialPitch);
	sinPitch = sinf(initialPitch);
	if (mx == 0 || mz == 0)
	{
		mx = 1;
		mz = 1;
	}
	magX = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;
	magY = my * cosRoll - mz * sinRoll;

	initialHdg = atan2f(-magY, magX);

	cosRoll = cosf(initialRoll * 0.5f);
	sinRoll = sinf(initialRoll * 0.5f);

	cosPitch = cosf(initialPitch * 0.5f);
	sinPitch = sinf(initialPitch * 0.5f);

	cosHeading = cosf(initialHdg * 0.5f);
	sinHeading = sinf(initialHdg * 0.5f);

	q0 = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
	q1 = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
	q2 = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
	q3 = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;

	// auxillary variables to reduce number of repeated operations, for 1st pass
	q0q0 = q0 * q0;
	q0q1 = q0 * q1;
	q0q2 = q0 * q2;
	q0q3 = q0 * q3;
	q1q1 = q1 * q1;
	q1q2 = q1 * q2;
	q1q3 = q1 * q3;
	q2q2 = q2 * q2;
	q2q3 = q2 * q3;
	q3q3 = q3 * q3;
}
/**
 * @brief Mahony算法
 */
float kp_use = 0.0f, ki_use = 0.0f, mkp_use = 0.0f;
static void NonlinearSO3AHRSupdate(float gx, float gy, float gz, float ax,
								   float ay, float az, float mx, float my, float mz, float twoKp, float twoKi, float twomKp,
								   float dt)
{
	float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
	float recipNorm;
#ifdef USE_MAG
	static uint8_t reset_cnt = 0;
#endif
	// Make filter converge to initial solution faster
	if (bFilterInit == 0)
	{
		NonlinearSO3AHRSinit(ax, ay, az, mx, my, mz);
		bFilterInit = 1;
	}
	if (!((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)))
	{
		float hx, hy, hz, bx, bz;
		float halfwx, halfwy, halfwz;

		recipNorm = app_math_InvSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;
		// Reference direction of Earth's magnetic field
		hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
		hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
		hz = 2.0f * mx * (q1q3 - q0q2) + 2.0f * my * (q2q3 + q0q1) + 2.0f * mz * (0.5f - q1q1 - q2q2);
		bx = sqrt(hx * hx + hy * hy);
		bz = hz;
		// Estimated direction of magnetic field
		halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
		halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
		halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex += mkp_use * (my * halfwz - mz * halfwy);
		halfey += mkp_use * (mz * halfwx - mx * halfwz);
		halfez += mkp_use * (mx * halfwy - my * halfwx);
	}

	// 增加一个条件：  加速度的模量与G相差不远时。 0.75*G < normAcc < 1.25*G
	//  Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
	{
		float halfvx, halfvy, halfvz;

		recipNorm = app_math_InvSqrt(ax * ax + ay * ay + az * az);
		/*//新陀螺仪目前没看到需要用到这个的地方*/
		// if((1/(0.75f*9.8f))>recipNorm>(1/(1.25f*9.8f))){
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;
		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex += ay * halfvz - az * halfvy;
		halfey += az * halfvx - ax * halfvz;
		halfez += ax * halfvy - ay * halfvx;
	}
	//}
	// Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
	if (halfex != 0.0f && halfey != 0.0f && halfez != 0.0f)
	{
		// Compute and apply integral feedback if enabled
		if (twoKi > 0.0f)
		{
			gyro_bias[0] += ki_use * halfex * dt; // integral error scaled by Ki
			gyro_bias[1] += ki_use * halfey * dt;
			gyro_bias[2] += ki_use * halfez * dt;
			// apply integral feedback
			gx += gyro_bias[0];
			gy += gyro_bias[1];
			gz += gyro_bias[2];
		}
		else
		{
			gyro_bias[0] = 0.0f; // prevent integral windup
			gyro_bias[1] = 0.0f;
			gyro_bias[2] = 0.0f;
		}
		// Apply proportional feedback
		gx += kp_use * halfex;
		gy += kp_use * halfey;
		gz += kp_use * halfez;
	}
#ifdef USE_MAG
	if (app_imu_data.reset == 1)
	{
		kp_use = 50.0f;
		ki_use = 0.0f;
		mkp_use = 5.0f;
		if (APP_MATH_ABS(halfex) + APP_MATH_ABS(halfey) + APP_MATH_ABS(halfez) < 0.5f)
		{
			reset_cnt++;
			if (reset_cnt > 20)
			{
				reset_cnt = 0;
				app_imu_data.reset = 0;
			}
		}
		else
		{
			reset_cnt = 0;
		}
	}
	else
	{
		kp_use = twoKp;
		ki_use = twoKi;
		mkp_use = twomKp;
	}
#else
	if (app_imu_data.reset == 1)
	{
		kp_use = 40.0f;
		ki_use = 0.0f;
		if (APP_MATH_ABS(halfex) + APP_MATH_ABS(halfey) + APP_MATH_ABS(halfez) < 0.001f)
		{
			app_imu_data.reset = 0;
		}
	}
	else
	{
		kp_use = twoKp;
		ki_use = twoKi;
	}
#endif
	// Time derivative of quaternion. q_dot = 0.5*q\otimes omega.
	//! q_k = q_{k-1} + dt*\dot{q}
	//! \dot{q} = 0.5*q \otimes P(\omega)
	dq0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	dq1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	dq2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	dq3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	q0 += dt * dq0;
	q1 += dt * dq1;
	q2 += dt * dq2;
	q3 += dt * dq3;

	// Normalise quaternion
	recipNorm = app_math_InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	// Auxiliary variables to avoid repeated arithmetic
	q0q0 = q0 * q0;
	q0q1 = q0 * q1;
	q0q2 = q0 * q2;
	q0q3 = q0 * q3;
	q1q1 = q1 * q1;
	q1q2 = q1 * q2;
	q1q3 = q1 * q3;
	q2q2 = q2 * q2;
	q2q3 = q2 * q3;
	q3q3 = q3 * q3;
}

/**
 * @brief 软角度
 */
static float Soft_Angle(float angle, uint8_t whichAngle)
{
	static int16_t angleCircle[3];
	static float LastAngle[3];
	if ((angle - LastAngle[whichAngle]) > 180)
		angleCircle[whichAngle]--;
	else if ((angle - LastAngle[whichAngle]) < -180)
		angleCircle[whichAngle]++;
	LastAngle[whichAngle] = angle;
	return angleCircle[whichAngle] * 360 + angle;
}
/**
 * @brief 姿态解算
 */

uint32_t tPrev, tNow;
void app_imu_So3thread(void)
{
	float euler[3] = {0, 0, 0}; // rad
	static float dt;
	/* 计算两次解算时间间隔 */
	tNow = MICROS();
	//    float dt = (tPrev > 0) ? (tNow - tPrev) / 1000000.0f : 0;
	dt = (tPrev > 0) ? (tNow - tPrev) / 1000000.0f : 0;
	tPrev = tNow;
	//    if(dt == 0)  return;    // 第一次是0也没关系，反正是积分
	/* 读取数据(已经滤波、校正、单位化) */
	MPU_Read_Raw();

/* 四元数姿态融合 */
#ifdef USE_MAG
	NonlinearSO3AHRSupdate(app_imu_data.unitized.Gyro[0], app_imu_data.unitized.Gyro[1], app_imu_data.unitized.Gyro[2],
						   app_imu_data.unitized.Accel[0], app_imu_data.unitized.Accel[1], app_imu_data.unitized.Accel[2],
						   app_imu_data.unitized.Mag[0], app_imu_data.unitized.Mag[1], app_imu_data.unitized.Mag[2], so3_comp_params_Kp, so3_comp_params_Ki, so3_comp_params_mKp, dt);
#else
	NonlinearSO3AHRSupdate(app_imu_data.unitized.Gyro[0], app_imu_data.unitized.Gyro[1], app_imu_data.unitized.Gyro[2],
						   app_imu_data.unitized.Accel[0], app_imu_data.unitized.Accel[1], app_imu_data.unitized.Accel[2],
						   0, 0, 0, so3_comp_params_Kp, so3_comp_params_Ki, 0, dt);
#endif
	/* 转换成方向余弦矩阵 */
	// Convert q->R, This R converts inertial frame to body frame.
	att_matrix[0][0] = q0q0 + q1q1 - q2q2 - q3q3; // 11
	att_matrix[0][1] = 2.f * (q1q2 + q0q3);		  // 12
	att_matrix[0][2] = 2.f * (q1q3 - q0q2);		  // 13
	att_matrix[1][0] = 2.f * (q1q2 - q0q3);		  // 21
	att_matrix[1][1] = q0q0 - q1q1 + q2q2 - q3q3; // 22
	att_matrix[1][2] = 2.f * (q2q3 + q0q1);		  // 23
	att_matrix[2][0] = 2.f * (q1q3 + q0q2);		  // 31
	att_matrix[2][1] = 2.f * (q2q3 - q0q1);		  // 32
	att_matrix[2][2] = q0q0 - q1q1 - q2q2 + q3q3; // 33
	/* 转换成欧拉角 */
	euler[0] = atan2f(att_matrix[1][2], att_matrix[2][2]); //! Roll
	euler[1] = -asinf(att_matrix[0][2]);				   //! Pitch
	euler[2] = atan2f(att_matrix[0][1], att_matrix[0][0]); // ！Yaw
	/* 得出姿态角 */
	app_imu_data.Roll = euler[0] * TOANGLE; // 绝对角
	app_imu_data.Pitch = euler[1] * TOANGLE;
	app_imu_data.Yaw = -euler[2] * TOANGLE;
	if (app_imu_data.reset == 0) // 避免初始化时收敛过程影响soft的累积圈数
	{
		app_imu_data.soft.Roll = Soft_Angle(app_imu_data.Roll, 0); // 绝对路程角
		app_imu_data.soft.Pitch = Soft_Angle(app_imu_data.Pitch, 1);
		app_imu_data.soft.Yaw = Soft_Angle(app_imu_data.Yaw, 2);
	}
	app_imu_data.integral.Roll += app_imu_data.Angle_Rate[0] * GYRO_RESOLUTION * dt; // 相对积分角
	app_imu_data.integral.Pitch += app_imu_data.Angle_Rate[1] * GYRO_RESOLUTION * dt;
	app_imu_data.integral.Yaw += app_imu_data.Angle_Rate[2] * GYRO_RESOLUTION * dt;
	if (APP_MATH_ABS(app_imu_data.original.Gyro[0]) <= 1 && APP_MATH_ABS(app_imu_data.original.Gyro[1]) <= 1 && APP_MATH_ABS(app_imu_data.original.Gyro[2]) <= 1 &&
		APP_MATH_ABS(app_imu_data.original.Accel[0] <= 1) && APP_MATH_ABS(app_imu_data.original.Accel[1] <= 1) && APP_MATH_ABS(app_imu_data.original.Accel[2] <= 1))
		app_imu_data.ready = 0;
	else
		app_imu_data.ready = 1;
}
