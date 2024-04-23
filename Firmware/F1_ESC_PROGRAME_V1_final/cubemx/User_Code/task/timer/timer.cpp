/**
 * @file timer.cpp
 * @brief timer
 * @author Tony_Wang
 * @version 1.0
 * @date 2023-8-28
 * @copyright
 * @par 日志:
 *   V1.0 建立定时器调用库，避免每次都要翻阅资料
 *
 */

#include "timer.hpp"

/* 添加的功能函数头文件 */
#include "foc/bsp_foc.hpp"
#include "led_cpp/bsp_led.hpp"
#include "key_cpp/bsp_key.hpp"
#include "motor/app_motor.hpp"

void timer_init(void)
{
    /* 这里放需要启动定时器中断的定时器 */
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_Base_Start_IT(&htim2);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim2)
    {
        static uint8_t Tick1 = 0;
        static uint8_t Tick2 = 0;
        if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) // 50us 20k Hz
        {
            if (Tick1 < 1) // 1: 50us
            {
                Tick1++;
            }
            else
            {
                Tick1 = 0;
                // motor_1612._current_sensor->update();
                // motor_1612.Clark_Park(motor_1612._current_sensor->phase_u.data,
                //                       motor_1612._current_sensor->phase_v.data,
                //                       motor_1612._current_sensor->phase_w.data,
                //                       motor_1612.electrical_angle);
                // motor_1612.I_q = motor_1612._Iq_LowPassFilter->run(motor_1612.I_q);
                // motor_1612.I_d = motor_1612._Id_LowPassFilter->run(motor_1612.I_d);
                motor_1612._current_sensor->adc_dma->getonce();
            }

            if (Tick2 < 19) // 19:1000us
            {
                Tick2++;
            }
            else
            {
                Tick2 = 0;
                motor_1612.run();
                // app_motor_can_Basic_postback(&motor_1612);
            }
        }
    }

    /* 依次放进入中断的定时器 */
    if (htim == (&htim1)) // 100US
    {
        /* foc run function start */
        /* 依次放该定时器下不同功能的触发间隔变量 */
        static uint16_t foc_run_flag = 0;
        /* 以上每个触发间隔变量 ++  */
        foc_run_flag += 1;
        /* 编写触发功能函数 */
        if (foc_run_flag == 10)
        {
            app_motor_can_Basic_postback(&motor_1612);

            /* 一定要添加间隔变量复位！！ */
            foc_run_flag = 0;
        }

        /* foc run function end */

        /* ***************************************************************************** */

        /* led update function start */
        /* 依次放该定时器下不同功能的触发间隔变量 */
        static uint16_t led_update_flag = 0;
        /* 以上每个触发间隔变量 ++  */
        led_update_flag += 1;
        /* 编写触发功能函数 */
        if (led_update_flag == 100)
        {
            BORAD_LED.twinkle_update();
            /* 一定要添加间隔变量复位！！ */
            led_update_flag = 0;
        }
        /* led update function end */

        /* ***************************************************************************** */

        /* key_change function start */
        /* 按键模式切换 */
        static uint16_t key_change_flag = 0;
        /* 以上每个触发间隔变量 ++  */
        key_change_flag += 1;
        /* 编写触发功能函数 */
        if (key_change_flag == 100)
        {
            // if (USER_Button.OUT_State == Button_OFF)
            // {
            //     // HAL_GPIO_WritePin(M_EN_GPIO_Port, M_EN_Pin, GPIO_PIN_RESET);
            //     motor_1612.disable();
            // }
            // else if (USER_Button.OUT_State == Button_ON)
            // {
            //     // HAL_GPIO_WritePin(M_EN_GPIO_Port, M_EN_Pin, GPIO_PIN_SET);
            //     motor_1612.enable();
            // }
            app_motor_key_canidset();
            /* 一定要添加间隔变量复位！！ */
            key_change_flag = 0;
        }
        /* key_change function end */

        /* ***************************************************************************** */
    }
}

// void task_timer_led_change(void)
// {
//     Button_State tempState = USER_Button.update();
//     if (tempState == Button_HOLD)
//     {
//         BORAD_LED.twinkle_Config(20, 10);
//     }
//     else if (tempState == Button_OFF)
//     {
//         BORAD_LED.twinkle_Config(200, 1);
//     }
//     else if (tempState == Button_ON)
//     {
//         BORAD_LED.twinkle_Config(100, 2);
//     }

//     if (USER_Button.OUT_State == Button_HOLD)
//     {
//         /* 此时表显为 长按 can_id 增加 */
//     }
// }