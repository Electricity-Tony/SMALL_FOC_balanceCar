/**
 * @file bsp_mpu6500.c
 * @brief 陀螺仪MPU6500
 * @author Tony_Wang
 * @version 1.0
 * @date 2023-6-21
 * @copyright
 * @par 日志:
 *   V1.0 基本Cube配置与基本设置
 * 	 V1.1 通过此前的 icm6500 姿态解算移植，实现角度输出
 *
 */

#include "bsp_mpu6500.h"
#include "spi.h"
// 需要自己转接的3个宏定义
#define mpu6500_SPI hspi1
#define ICM_Disable() HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_SET)
#define ICM_Enable() HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_RESET)

/**
 * @brief 陀螺仪写函数
 */
static void bsp_mpu6500_WriteReg(uint8_t regAddr, uint8_t data)
{
	regAddr &= 0x7f; // 首位0位写
	ICM_Enable();
	// HAL_Delay(1);
	HAL_SPI_Transmit(&mpu6500_SPI, &regAddr, 1, 100);
	HAL_SPI_Transmit(&mpu6500_SPI, &data, 1, 100);
	// HAL_Delay(1);
	ICM_Disable();
}
/**
 * @brief 陀螺仪单次读取函数
 */
uint8_t bsp_mpu6500_ReadReg(uint8_t regAddr)
{
	uint8_t temp;
	bsp_mpu6500_ReadRegs(regAddr, &temp, 1);
	return temp;
}
/**
 * @brief 陀螺仪读取函数
 */
void bsp_mpu6500_ReadRegs(uint8_t regAddr, uint8_t *pBuff, uint8_t length)
{
	regAddr |= 0x80;
	ICM_Enable();
	// HAL_Delay(1);
	HAL_SPI_Transmit(&mpu6500_SPI, &regAddr, 1, 100);
	HAL_SPI_Receive(&mpu6500_SPI, pBuff, length, 100);
	ICM_Disable();
	// HAL_Delay(1);
}

/**
 * @brief mpu6500初始化
 * @retval HAL_OK  成功
 * @retval 其他 异常退出
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef bsp_mpu6500_Init(void)
{
	static uint8_t ID;
	bsp_mpu6500_WriteReg(MPU6500_PWR_MGMT_1, 0x80); // 清除内部寄存器
	HAL_Delay(100);
	bsp_mpu6500_WriteReg(MPU6500_PWR_MGMT_1, 0x01); // 配置时钟源
	HAL_Delay(100);

	HAL_Delay(1000);
	ID = bsp_mpu6500_ReadReg(MPU6500_WHO_AM_I); // WHO AM I!!!
	if (ID == MPU6500_ID)
	// if(ID != 0x00)
	{

		// bsp_mpu6500_WriteReg(MPU6500_SIGNAL_PATH_RESET,0x03);
		HAL_Delay(10);
		bsp_mpu6500_WriteReg(MPU6500_USER_CTRL, 0x01); // 复位各寄存器
		HAL_Delay(10);
		bsp_mpu6500_WriteReg(MPU6500_PWR_MGMT_2, 0x00); // 开启陀螺仪和加速度计
		HAL_Delay(10);
		// bsp_mpu6500_WriteReg(MPU6500_SMPLRT_DIV,0);						//不分频
		HAL_Delay(10);
		bsp_mpu6500_WriteReg(MPU6500_CONFIG, MPU6500_G_CFG_20HZ); // 低通滤波配置
		HAL_Delay(10);
		bsp_mpu6500_WriteReg(MPU6500_GYRO_CONFIG, MPU6500_GYRO_2000dps); // 陀螺仪量程+-2000dps
		HAL_Delay(10);
		bsp_mpu6500_WriteReg(MPU6500_ACCEL_CONFIG, MPU6500_ACCEL_8g); // 加速度计量程+-16g
		HAL_Delay(10);
		bsp_mpu6500_WriteReg(MPU6500_ACCEL_CONFIG_2, MPU6500_A_CFG_20HZ); // 加速度计滤波配置
		HAL_Delay(10);
		bsp_mpu6500_WriteReg(MPU6500_FIFO_EN, 0x00); // 关闭FIFO
		HAL_Delay(10);
		bsp_mpu6500_WriteReg(MPU6500_USER_CTRL, 0x20); // 开启陀螺仪

		return HAL_OK;
	}
	else
		return HAL_ERROR;
}
