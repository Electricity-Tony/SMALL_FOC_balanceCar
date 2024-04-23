
#ifndef __APP_MOTOR_HPP
#define __APP_MOTOR_HPP
#include "stm32f1xx.h"
#include "main.h"

/* my include */
#include "Can/bsp_can.h"
#include "foc/bsp_foc.hpp"

/* my define */
/* 电调发送功能字 */
enum ESC_TX_fun
{
    SAMPLE_DATA = 0xF1, // 采样数据回发
    PID_DATA = 0xE1,    // PID参数回发
    MOTOR_DATA = 0xE2,  // 电机参数回发
};

/* 电调接收功能字 */
enum ESC_RX_fun
{
    /* 功能指令 */
    CAL = 0x11,       // 电机校准命令
    SET_PID = 0x12,   // 设置PID参数命令
    SHOW_PID = 0x13,  // 查询PID参数命令
    SAVE_PID = 0x14,  // 保存PID参数命令
    SAFE_MODE = 0x15, // 电机下电进入安全模式
    RUN_MODE = 0x16,  // 电机进入正常工作模式
    SET_MOTOR = 0x17, // 设置电机编码器方向和极对数

    /* 控制指令 */
    CONTROL = 0x21 // 控制电机命令
};

/* pid 回发第一位 环确定 */
enum ESC_TX_pid_1
{
    CURRENT = 0x01,
    SPEED = 0x02,
    POSITION = 0x03
};

/* pid 回发第二位 参数确定 */
enum ESC_TX_pid_2
{
    P = 0x01,
    I = 0x02,
    D = 0x03,
    IMAX = 0x04,
    PIDMAX = 0x05
};

/* char int float 共用地址转换 */
typedef union
{
    float f;
    uint16_t i[2];
    uint8_t c[4];
} inline_Struct;

/* 标准发送函数 */
HAL_StatusTypeDef app_motor_can_Sendmessage(CAN_HandleTypeDef *hcan, int16_t StdId, uint8_t *Can_Send_Data);

/* 传感器回发函数 */
void app_motor_can_Basic_postback(foc *motor);

/* 命令执行函数 */
void app_motor_can_CAL(foc *motor);
void app_motor_can_CAL_ZeroElectricalAngle(foc *motor);
void app_motor_can_PostBackpid(foc *motor);
void app_motor_can_SETpid(foc *motor, uint8_t *CAN_RX_data);
void app_motor_can_SAVEpid(foc *motor);
void app_motor_can_Ctrl(foc *motor, uint8_t *CAN_RX_data);
void app_motor_can_SAFEmode(foc *motor);
void app_motor_can_RUNmode(foc *motor);
void app_motor_can_SETMotor(foc *motor, uint8_t *CAN_RX_data);
void app_motor_can_ShowMotor(foc *motor);

/* 命令跳转函数 */
uint8_t app_motor_can_toCAL(uint8_t *CAN_RX_data);
uint8_t app_motor_can_toPostBackpid(uint8_t *CAN_RX_data);
uint8_t app_motor_can_toSETpid(uint8_t *CAN_RX_data);
uint8_t app_motor_can_toSAVEpid(uint8_t *CAN_RX_data);
uint8_t app_motor_can_toCtrl(uint8_t *CAN_RX_data);
uint8_t app_motor_can_toSAFEmode(uint8_t *CAN_RX_data);
uint8_t app_motor_can_toRUNmode(uint8_t *CAN_RX_data);
uint8_t app_motor_can_toSETMotor(uint8_t *CAN_RX_data);

/* 命令解析函数 */
void app_motor_can_parse(uint8_t *CAN_RX_data);

/* flash 相关函数 */
void app_motor_flash_pidget(foc *motor);
void app_motor_flash_pidsave(foc *motor);

/* canid 相关函数 */
void app_motor_key_canidset(void);
#endif