/**
 * @file bsp_mpu6500.c
 * @brief 陀螺仪MPU6500
 * @author Tony_Wang
 * @version 1.0
 * @date 2023-6-21
 * @copyright
 * @par 日志:
 *   V1.0 基本Cube配置与基本设置
 *
 */


#ifndef __BSP_MPU6500_H
#define __BSP_MPU6500_H
#include "stm32f1xx_hal.h"


/*此六个寄存器中的值表示在制造测试过程中产生的自测试输出。此值用于检查最终用户执行的后续自测试输出*/
#define MPU6500_SELF_TEST_XG        0x00
#define MPU6500_SELF_TEST_YG        0x01
#define MPU6500_SELF_TEST_ZG        0x02
#define MPU6500_SELF_TEST_XA        0x0D
#define MPU6500_SELF_TEST_YA        0x0E
#define MPU6500_SELF_TEST_ZA        0x0F

/*此六个寄存器用于消除陀螺仪输出中的直流偏置。在进入传感器寄存器之前，将此寄存器中的值添加到陀螺仪传感器值中。*/
#define MPU6500_XG_OFFSET_H         0x13
#define MPU6500_XG_OFFSET_L         0x14
#define MPU6500_YG_OFFSET_H         0x15
#define MPU6500_YG_OFFSET_L         0x16
#define MPU6500_ZG_OFFSET_H         0x17
#define MPU6500_ZG_OFFSET_L         0x18

/*除以内部采样率see register CONFIG生成控制传感器数据输出速率的采样率，FIFO采样率.此寄存器只有在FCHOICE=2‘b11FCHOICE_B寄存器位为2’b00和0<DLPF_CFG<7时才有效
采样率=内部采样率/1+SMPLRT_DIV,内部采样率=1 kHz*/
#define MPU6500_SMPLRT_DIV          0x19

/*四个配置寄存器说明如下文所示*/
#define MPU6500_CONFIG              0x1A
#define MPU6500_GYRO_CONFIG         0x1B
#define MPU6500_ACCEL_CONFIG        0x1C
#define MPU6500_ACCEL_CONFIG_2      0x1D

/*低功率加速度计ODR控制寄存器*/
#define MPU6500_LP_ACCEL_ODR        0x1E

/*此寄存器保存x/y/z中断返回值*/
#define MPU6500_MOT_THR             0x1F

/*FIFO使能寄存器。若置1，则将对应数据以采样频率写入FIFO*/
#define MPU6500_FIFO_EN             0x23

/*IIC主设备控制器，见下文*/
#define MPU6500_I2C_MST_CTRL        0x24

/*IIC从设备相关寄存器*/
#define MPU6500_I2C_SLV0_ADDR       0x25
#define MPU6500_I2C_SLV0_REG        0x26
#define MPU6500_I2C_SLV0_CTRL       0x27
#define MPU6500_I2C_SLV1_ADDR       0x28
#define MPU6500_I2C_SLV1_REG        0x29
#define MPU6500_I2C_SLV1_CTRL       0x2A
#define MPU6500_I2C_SLV2_ADDR       0x2B
#define MPU6500_I2C_SLV2_REG        0x2C
#define MPU6500_I2C_SLV2_CTRL       0x2D
#define MPU6500_I2C_SLV3_ADDR       0x2E
#define MPU6500_I2C_SLV3_REG        0x2F
#define MPU6500_I2C_SLV3_CTRL       0x30
#define MPU6500_I2C_SLV4_ADDR       0x31
#define MPU6500_I2C_SLV4_REG        0x32
#define MPU6500_I2C_SLV4_DO         0x33
#define MPU6500_I2C_SLV4_CTRL       0x34
#define MPU6500_I2C_SLV4_DI         0x35

/*IIC  主设备状态寄存器*/
#define MPU6500_I2C_MST_STATUS      0x36
/*三个中断相关寄存器*/
#define MPU6500_INT_PIN_CFG         0x37
#define MPU6500_INT_ENABLE          0x38
#define MPU6500_INT_STATUS          0x3A

/*这14个寄存器存储加速度、陀螺仪、温度的原始数据*/
#define MPU6500_ACCEL_XOUT_H        0x3B
#define MPU6500_ACCEL_XOUT_L        0x3C
#define MPU6500_ACCEL_YOUT_H        0x3D
#define MPU6500_ACCEL_YOUT_L        0x3E
#define MPU6500_ACCEL_ZOUT_H        0x3F
#define MPU6500_ACCEL_ZOUT_L        0x40
#define MPU6500_TEMP_OUT_H          0x41
#define MPU6500_TEMP_OUT_L          0x42
#define MPU6500_GYRO_XOUT_H         0x43
#define MPU6500_GYRO_XOUT_L         0x44
#define MPU6500_GYRO_YOUT_H         0x45
#define MPU6500_GYRO_YOUT_L         0x46
#define MPU6500_GYRO_ZOUT_H         0x47
#define MPU6500_GYRO_ZOUT_L         0x48

/*这24个寄存器存储IIC从设备（0、1、2和3通过辅助IIC接口，从外部传感器读取的数据
从机设备4读取的数据存放在I2C_SLV4_DI中（寄存器53）*/
#define MPU6500_EXT_SENS_DATA_00    0x49
#define MPU6500_EXT_SENS_DATA_01    0x4A
#define MPU6500_EXT_SENS_DATA_02    0x4B
#define MPU6500_EXT_SENS_DATA_03    0x4C
#define MPU6500_EXT_SENS_DATA_04    0x4D
#define MPU6500_EXT_SENS_DATA_05    0x4E
#define MPU6500_EXT_SENS_DATA_06    0x4F
#define MPU6500_EXT_SENS_DATA_07    0x50
#define MPU6500_EXT_SENS_DATA_08    0x51
#define MPU6500_EXT_SENS_DATA_09    0x52
#define MPU6500_EXT_SENS_DATA_10    0x53
#define MPU6500_EXT_SENS_DATA_11    0x54
#define MPU6500_EXT_SENS_DATA_12    0x55
#define MPU6500_EXT_SENS_DATA_13    0x56
#define MPU6500_EXT_SENS_DATA_14    0x57
#define MPU6500_EXT_SENS_DATA_15    0x58
#define MPU6500_EXT_SENS_DATA_16    0x59
#define MPU6500_EXT_SENS_DATA_17    0x5A
#define MPU6500_EXT_SENS_DATA_18    0x5B
#define MPU6500_EXT_SENS_DATA_19    0x5C
#define MPU6500_EXT_SENS_DATA_20    0x5D
#define MPU6500_EXT_SENS_DATA_21    0x5E
#define MPU6500_EXT_SENS_DATA_22    0x5F
#define MPU6500_EXT_SENS_DATA_23    0x60

/*IIC从设备数据输出寄存器*/
#define MPU6500_I2C_SLV0_DO         0x63
#define MPU6500_I2C_SLV1_DO         0x64
#define MPU6500_I2C_SLV2_DO         0x65
#define MPU6500_I2C_SLV3_DO         0x66

#define MPU6500_I2C_MST_DELAY_CTRL  0x67
#define MPU6500_SIGNAL_PATH_RESET   0x68
#define MPU6500_MOT_DETECT_CTRL     0x69
#define MPU6500_USER_CTRL           0x6A

/*电源管理寄存器，用于配置MPU6500时钟源，控制传感器失能等*/
#define MPU6500_PWR_MGMT_1          0x6B
#define MPU6500_PWR_MGMT_2          0x6C

/*记录写入到FIFO的字节数*/
#define MPU6500_FIFO_COUNTH         0x72
#define MPU6500_FIFO_COUNTL         0x73

/*用于从FIFO缓冲区读写数据*/
#define MPU6500_FIFO_R_W            0x74

/*存储一个8位数据，用于验证设备的标示*/
#define MPU6500_WHO_AM_I            0x75	// mpu6500 id = 0x70

/*此六个寄存器用于消除加速度计输出中的直流偏置。在进入传感器寄存器之前，将此寄存器中的值添加到加速度计传感器值中。*/
#define MPU6500_XA_OFFSET_H         0x77
#define MPU6500_XA_OFFSET_L         0x78
#define MPU6500_YA_OFFSET_H         0x7A
#define MPU6500_YA_OFFSET_L         0x7B
#define MPU6500_ZA_OFFSET_H         0x7D
#define MPU6500_ZA_OFFSET_L         0x7E
	
#define MPU6500_ID					0x70	
#define MPU_IIC_ADDR				0x68
////////////////////////////////////////////
/* 移植的部分配置寄存器值 */

// 陀螺仪滤波器配置
#define MPU6500_G_CFG_184HZ           1
#define MPU6500_G_CFG_92HZ            2
#define MPU6500_G_CFG_41HZ            3
#define MPU6500_G_CFG_20HZ            4
#define MPU6500_G_CFG_10HZ            5
#define MPU6500_G_CFG_5HZ             6
#define MPU6500_G_CFG_3600HZ          7

// 加速度计滤波器配置
#define MPU6500_A_CFG_184HZ           1
#define MPU6500_A_CFG_92HZ            2
#define MPU6500_A_CFG_41HZ            3
#define MPU6500_A_CFG_20HZ            4
#define MPU6500_A_CFG_10HZ            5
#define MPU6500_A_CFG_5HZ             6
#define MPU6500_A_CFG_460HZ           7

// 陀螺仪量程配置
#define MPU6500_GYRO_250dps         0x00
#define MPU6500_GYRO_500dps         0x08
#define MPU6500_GYRO_1000dps        0x10
#define MPU6500_GYRO_2000dps        0x18

// 加速度计量程配置
#define MPU6500_ACCEL_2g            0x00
#define MPU6500_ACCEL_4g            0x08
#define MPU6500_ACCEL_8g            0x10
#define MPU6500_ACCEL_16g           0x11



//////////////////////////////////////////////

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

uint8_t bsp_mpu6500_ReadReg(uint8_t regAddr);//提供给姿态解算的读数据函数
void bsp_mpu6500_ReadRegs(uint8_t regAddr,uint8_t *pBuff,uint8_t length);//标准SPI读函数
HAL_StatusTypeDef bsp_mpu6500_Init(void);//mpu6500初始化

// void bsp_mpu6500_WriteReg(uint8_t regAddr, uint8_t data);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif
