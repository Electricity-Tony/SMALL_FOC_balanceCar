
#ifndef __APP_TXMOTOR_HPP
#define __APP_TXMOTOR_HPP
#include "stm32f1xx.h"
#include "main.h"

/* my include */
#include "Can/bsp_can.h"
#include "dep.hpp"

/* my define */
/* 控制器接收功能字 */
enum CON_RX_fun
{
    SAMPLE_DATA = 0xF1, // 采样数据回发
    PID_DATA = 0xE1,    // PID参数回发
    MOTOR_DATA = 0xE2,  // 电机参数回发
};

/* 控制器发送功能字 */
enum CON_TX_fun
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
enum PID_RING
{
    CURRENT = 0x01,
    SPEED = 0x02,
    POSITION = 0x03
};

/* pid 回发第二位 参数确定 */
enum PID_PARAMETER
{
    P = 0x01,
    I = 0x02,
    D = 0x03,
    IMAX = 0x04,
    PIDMAX = 0x05
};

/* 运行模式 */
typedef enum
{
    openloop = 0x00,    // 无编码器开环运行
    currentMode = 0x01, // 电流(力矩)运行
    speedMode = 0x02,   // 速度运行
    angleMode = 0x03,   // 角度运行
} run_mode;

/* char int float 共用地址转换 */
typedef union
{
    float f;
    uint16_t i[2];
    uint8_t c[4];
} inline_Struct;

/* ************************** can_motor 构造函数 ************************* */

class can_motor
{
    /* 基本pid构造集合 */
    typedef struct
    {
        float P;
        float I;
        float D;
        float IMax;
        float PIDMax;
    } PIDtypedef;

private:
    /* data */
public:
    CAN_HandleTypeDef *motor_hcan; // 挂载的 can 总线
    uint32_t can_id;               // can_motor 的canid号

    /* 传感器数据 */
    float true_angle;
    float true_speed;
    float true_current;

    /* 电机参数 */
    int8_t dir;                  // 电角度方向
    uint8_t pole_pairs;          // 极对数
    float zero_electrical_angle; // 零电角度方向

    /* 三环pid 参数 */
    PIDtypedef PID_CURRENT;
    PIDtypedef PID_IN;
    PIDtypedef PID_OUT;

    can_motor(/* args */){};
    can_motor(CAN_HandleTypeDef *motor_hcan, int16_t can_id);

    /* 命令发送 */
    void command_CAL(void);
    void command_CAL_zeroElectricalAngle(void);
    void command_SETpid(PID_RING PID_RING, PID_PARAMETER PID_PARAMETER, float value);
    void command_Savepid(void);
    void command_PostBackpid(void);
    void command_SafeMode(void);
    void command_RunMode(void);
    void command_SetMotor(uint8_t parameter_id, int8_t data);

    /* 标准控制 */
    void ctrl(run_mode mode, float value);

    /* 接受数据解析函数 */
    uint8_t extract_pid(uint8_t *CAN_RX_data);
    uint8_t extract_SampleData(uint8_t *CAN_RX_data);
    uint8_t extract_MmotorData(uint8_t *CAN_RX_data);

    /* 接收校验函数 */
    uint8_t parse(CAN_RxHeaderTypeDef bsp_can_Rx, uint8_t *CAN_RX_data);
};

extern can_motor motor1;
extern can_motor motor2;

#endif