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
#include "led_cpp/bsp_led.hpp"
#include "key_cpp/bsp_key.hpp"
#include "PowerIO/powerio.hpp"

#include "mpu6500/app_imu.h"

#include "txmotor/app_txmotor.hpp"

void timer_init(void)
{
    /* 这里放需要启动定时器中断的定时器 */
    HAL_TIM_Base_Start_IT(&htim1);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* 依次放进入中断的定时器 */
    if (htim == (&htim1)) // 100us
    {
        /* foc run function start */
        /* 依次放该定时器下不同功能的触发间隔变量 */
        static uint16_t ium_run_flag = 0;
        /* 以上每个触发间隔变量 ++  */
        ium_run_flag += 1; 
        /* 编写触发功能函数 */
        if (ium_run_flag == 100)
        {
            app_imu_So3thread();
            /* 一定要添加间隔变量复位！！ */
            ium_run_flag = 0;
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
            //     motor1.command_SafeMode();
            //     motor2.command_SafeMode();
            // }
            // else if (USER_Button.OUT_State == Button_ON)
            // {
            //     motor1.command_RunMode();
            //     motor2.command_RunMode();
            // }
            USER_Button.update();
            /* 一定要添加间隔变量复位！！ */
            key_change_flag = 0;
        }
        /* key_change function end */

        /* ***************************************************************************** */

        /* power_check function start */
        /* 按键模式切换 */
        static uint16_t power_check_flag = 0;
        /* 以上每个触发间隔变量 ++  */
        power_check_flag += 1;
        /* 编写触发功能函数 */
        if (power_check_flag == 1000)
        {
            /* 长按关机与电池检测代码 */
            app_power_Check();
            /* 一定要添加间隔变量复位！！ */
            power_check_flag = 0;
        }
        /* power_check function end */
    }
}
