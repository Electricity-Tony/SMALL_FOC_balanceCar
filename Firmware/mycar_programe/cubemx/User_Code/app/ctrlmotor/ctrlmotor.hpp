#ifndef __CTRLMOTOR_H
#define __CTRLMOTOR_H

#include "stm32f1xx.h"
#include "main.h"

#include "txmotor/app_txmotor.hpp"
#include "mpu6500/app_imu.h"
#include "PID_cpp/bsp_pid.hpp"
#include "app_encoder/app_encoder.hpp"
#include "usart_cpp/bsp_usart.hpp"

void car_balance(float target_angle);
void car_speed(float target_speed);
void car_turn(float target_speed);

void car_3RingCtrl(float balance_value, float speed_value, float turn_value);

extern float balance_Mspeed;
extern float speed_Mspeed;
extern float turn_Mspeed;

extern float motor1_Mspeed;
extern float motor2_Mspeed;

extern pid balance_pid;
#endif