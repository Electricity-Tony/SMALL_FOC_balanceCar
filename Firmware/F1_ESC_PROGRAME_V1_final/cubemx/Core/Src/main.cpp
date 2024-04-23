/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Key_cpp/bsp_key.hpp"
#include "led_cpp/bsp_led.hpp"
#include "tle5012/bsp_tle5012b.h"
#include "adc_cpp/bsp_adc.hpp"
#include "foc/bsp_foc.hpp"
#include "app_encoder/app_encoder.hpp"
#include "timer/timer.hpp"
#include "Can/bsp_can.h"

#include <math.h>

/* apps */
#include "motor/app_motor.hpp"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern uint8_t can_CAL_ZeroElectricalAngle_flag;
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_CAN_Init();

  /* USER CODE BEGIN 2 */

  BORAD_LED.ctrl(LED_OFF);
  HAL_Delay(1000);
  BORAD_LED.ctrl(LED_ON);

  // TLE5012初始化

  // /* led初始化 */
  // BORAD_LED.twinkle_Config(200, 2);

  /* 按键初始化 */
  USER_Button.clean();

  /* 电流采样 ADC_DMA 初始化 */
  HAL_Delay(1000);
  bsp_adc_dma1.init();

  /* foc 初始化 */
  motor_1612.set_voltage_limit(8, 8.4);
  motor_1612.init();
  motor_1612.set_encoder(&HallEncoder);
  motor_1612.set_PID_OUT(&pid_out_1612);
  motor_1612.set_PID_IN(&pid_in_1612);
  // /* 初始化得到原始电角度 */
  // HAL_GPIO_WritePin(M_EN_GPIO_Port, M_EN_Pin, GPIO_PIN_SET);
  // motor_1612.init_ZeroElectricalAngle(3000);
  // HAL_GPIO_WritePin(M_EN_GPIO_Port, M_EN_Pin, GPIO_PIN_RESET);
  // printf("zer_electrical_angle = %5.8f\r\n", motor_1612.zero_electrical_angle);
  /* 已经采样得到了  直接赋值的原始电角度 */
  // motor_1612.set_ZeroElectricalAngle(5.85807991f);
  motor_1612.set_ZeroElectricalAngle(2.165592f);

  /* foc 电流采样初始化 */
  motor_1612.set_current_sensor(&foc_current_sensor);
  motor_1612._current_sensor->init(0, 1, -1, 20);
  motor_1612.set_PID_IQ(&pid_IQ);
  motor_1612.set_PID_ID(&pid_ID);

  motor_1612._current_sensor->calibration(100);
  // printf("u_offset = %5f, v_offset = %5f, w_offset = %5f\r\n",
  //        motor_1612._current_sensor->phase_u.offset,
  //        motor_1612._current_sensor->phase_v.offset,
  //        motor_1612._current_sensor->phase_w.offset);

  /* 设置了一下霍尔传感器方向 使速度位置pid 为正 */
  motor_1612.encoder_dir = FORWARD;

  /* 功能运行定时器初始化 */
  timer_init();

  /* CAN 初始化 */
  bsp_can_Init();

  /* 电机校准测试 */
  // app_motor_can_CAL(&motor_1612);

  /* flash 中获取 pid 参数 */
  // app_motor_flash_pidsave(&motor_1612);
  app_motor_flash_pidget(&motor_1612);

  /* 默认电机上电 */
  motor_1612.enable();
  // motor_1612.disable();

  // app_motor_can_CAL(&motor_1612);

  /* 设置初始为 0 角度位置闭环 */
  // motor_1612.set_angle(0);
  // motor_1612.set_speed(120);
  motor_1612.set_current(0);
  // motor_1612.set_speed(60,openloop);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // motor_1612.set_speed(60);
    // motor_1612.set_speed(60,openloop);
    // motor_1612.set_current(0.4);

    /* foc 测试代码 */
    // 已放在了 timer 中断中处理

    /* 霍尔测试代码 */
    // float tle5012_angle = ReadAngle();
    // printf("angle = %5.5f\r\n", tle5012_angle);
    /* 霍尔整合为编码器测试 */
    // HallEncoder.get_date();
    // HallEncoder.get_speed();
    /* end */

    /* can 发送测试 */
    // bsp_can_Sendmessage(&hcan, CAN_ID, CAN_date);
    // app_motor_can_Basic_postback(&motor_1612);

    if (can_CAL_ZeroElectricalAngle_flag == 1)
    {
      can_CAL_ZeroElectricalAngle_flag = 0;
      app_motor_can_CAL_ZeroElectricalAngle(&motor_1612);
    }

    /* 运行杂项 */
    HAL_Delay(1);
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
