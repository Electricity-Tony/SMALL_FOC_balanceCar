#include "ctrlmotor.hpp"

float zero_angle = 0;
/* 以下这里是对电流直接进行控制，效果不理想 */
// /* 直立环完成 */
// // pid balance_pid(0.015, 0, -0.00015, 0, 0.3);

// /* 极性对，值太大了 */
// float kp=2;
// pid speed_pid(kp, kp/200, 0, 0.3, 0.3,1,1,1);

// pid balance_pid(0.009, 0, -0.00009, 0, 0.3);

/* 重新对速度进行控制 */
// pid balance_pid(15, 0, 0, 0, 500);
// pid balance_pid(25, 0, -0.04, 0, 500);
// pid balance_pid(25 * 0.8, 0, -0.08, 0, 500);
// float kp = 2000;
// pid speed_pid(kp, 0 / 200, 0, 300, 300);

/* 改为电调获取编码器 */
/* 这个是正解！！！！ 非常完美 */
// pid balance_pid(15, 0, 0, 0, 500);
// pid balance_pid(25, 0, -0.04, 0, 500);
pid balance_pid(15 * 0.6, 0, -0.048, 0, 500);
float kp = 0.6;
pid speed_pid(kp, kp / 200, 0, 100, 500);

pid turn_pid(-1, 0, 0, 0, 500);

float balance_Mspeed;
float speed_Mspeed;
float turn_Mspeed;

float motor1_Mspeed;
float motor2_Mspeed;

/**
 * @brief  直立环函数
 * @details
 * @param  target_angle 目标角度值
 * @retval
 */
void car_balance(float target_angle)
{
    balance_Mspeed = balance_pid.pid_run(target_angle - app_imu_data.Pitch);
}

/**
 * @brief  速度环函数
 * @details
 * @param  target_speed 目标速度值
 * @retval
 */
void car_speed(float target_speed)
{
    // motor1Encoder.get_speed();
    // motor2Encoder.get_speed();
    // float Encoder_Least;
    // static float Encoder = 0;
    // Encoder_Least = (motor1Encoder.speed - motor2Encoder.speed) - target_speed;
    // Encoder = 0.7 * Encoder + 0.3 * Encoder_Least;
    // speed_Mspeed = speed_pid.pid_run(Encoder);
    // printf("Encoder = %5.5f", Encoder);

    // motor1Encoder.get_speed();
    // motor2Encoder.get_speed();

    /* 事实证明 这里需要二阶低通滤波 */
    /* 直接电调获取速度 这个是有低通滤波的，外围再加了个低通滤波 */
    float Encoder_Least;
    static float Encoder = 0;
    Encoder_Least = (motor1.true_speed - motor2.true_speed) - target_speed;
    Encoder = 0.7 * Encoder + 0.3 * Encoder_Least;
    speed_Mspeed = speed_pid.pid_run(Encoder);
    // printf("Encoder = %5.5f", Encoder);
}

/**
 * @brief  转向环函数
 * @details
 * @param  target_turn 目标转向值
 * @retval
 */
void car_turn(float target_speed)
{
    turn_Mspeed = turn_pid.pid_run(target_speed  - ((float)app_imu_data.original.Gyro[2])/100);
}

/**
 * @brief  三环运行函数
 * @details
 * @param
 * @retval
 */
void car_3RingCtrl(float balance_value, float speed_value, float turn_value)
{

    motor1_Mspeed = balance_value + speed_value + turn_value;
    motor2_Mspeed = -(balance_value + speed_value - turn_value);

    // motor1.ctrl(currentMode, motor1_Mspeed);
    // motor2.ctrl(currentMode, motor1_Mspeed);

    motor1.ctrl(speedMode, motor1_Mspeed);
    motor2.ctrl(speedMode, motor2_Mspeed);
}