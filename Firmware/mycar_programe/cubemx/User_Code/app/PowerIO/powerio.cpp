#include "powerio.hpp"
#include "Key_cpp/bsp_key.hpp"
#include "Led_cpp/bsp_led.hpp"
#include "adc_cpp/bsp_adc.hpp"
#include "usart_cpp/bsp_usart.hpp"


float v_Battery;

/**
 * @brief  设置电源使能
 * @details
 * @param  电源状态 State
 * @retval
 */

void app_power_Ctrl(POWER_State State)
{
    HAL_GPIO_WritePin(POWER_EN_GPIO_Port, POWER_EN_Pin, GPIO_PinState(State));
}

void app_power_Check(void)
{
    Button_State tempState = USER_Button.update();
    /* 电池adc测试代码 */
    bsp_ADC2.update();   //这个更新就不放定时器了
    // bsp_adc_date_update(bsp_adc_date);
    v_Battery = (float)bsp_ADC2.data_list[0] / 4096 * 3.3 / 4.7 * 14.7;
    // printf("\r\n\r\nthe v_battery is %5.2f\r\n\r\n", v_Battery);
    if (v_Battery < 6.9f)
    {
        BORAD_LED.twinkle_Config(10, 20);
        // printf("\r\n\r\nthe v_battery is %5.2f\r\n\r\n", v_Battery);
    }
    else if (tempState == Button_HOLD)
    {
        BORAD_LED.twinkle_Config(20, 10);
    }
    else if (tempState == Button_OFF)
    {
        BORAD_LED.twinkle_Config(200, 1);
    }
    else if (tempState == Button_ON)
    {
        BORAD_LED.twinkle_Config(100, 2);
    }

    if (USER_Button.OUT_State == Button_HOLD)
    {
        app_power_Ctrl(POWER_DISENABLE);
    }
}
