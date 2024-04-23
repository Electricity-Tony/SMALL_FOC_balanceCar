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
#include "powerio/powerio.hpp"
#include "usart_cpp/bsp_usart.hpp"
#include "mpu6500/bsp_mpu6500.h"
#include "mpu6500/app_imu.h"
#include "adc_cpp/bsp_adc.hpp"
// #include "serialstudio/bsp_serialstudio.hpp"
#include "timer/timer.hpp"

#include <math.h>

/* apps */
#include "usartcmd/app_usartcmd.hpp"
#include "txmotor/app_txmotor.hpp"
#include "ctrlmotor/ctrlmotor.hpp"
#include "app_encoder/app_encoder.hpp"
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
float Custom_Diff_temp;
Button_State state_temp;
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
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  BORAD_LED.ctrl(LED_OFF);
  HAL_Delay(1000);
  app_power_Ctrl(POWER_ENABLE); // 拉高电源使能端口
  BORAD_LED.ctrl(LED_ON);
  HAL_Delay(1000);

  USART_Debug_DMA_Init();

  /* mpu6500初始化 */
  BORAD_LED.ctrl(LED_OFF);
  HAL_Delay(100);
  if (bsp_mpu6500_Init() != HAL_ERROR)
  {
    HAL_Delay(100);
    if (app_imu_Init() == 0)
    {
      printf("imu id is error\r\n");
      BORAD_LED.ctrl(LED_ON);
      HAL_Delay(1000);
      BORAD_LED.ctrl(LED_OFF);
    }
  }
  else
  { /* 失败初始化 */
    printf("mpu6500 id is error\r\n");
    BORAD_LED.ctrl(LED_ON);
    HAL_Delay(1000);
    BORAD_LED.ctrl(LED_OFF);
  };

  // 电池 ADC 初始化
  bsp_ADC2.init();

  /* led初始化 */
  BORAD_LED.twinkle_Config(200, 2);

  /* 按键初始化 */
  USER_Button.clean();

  /* 功能运行定时器初始化 */
  timer_init();

  /* 串口调试工具检查 */
  // serial_test.check_frame_length();

  /* usartcmd 初始化 */
  command_usart1_init();

  /* can 初始化 */
  bsp_can_Init();

  /* 所有电机下电 */
  motor1.command_SafeMode();
  motor2.command_SafeMode();

  /* 直立环 pid 微分误差输入 */
  balance_pid.Custom_Diff = &Custom_Diff_temp;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* usartcmd 测试代码 */
    // command_usart1.run();

    /* foc 测试代码 */
    // 已放在了 timer 中断中处理

    /* 串口输出展示 */
    // static uint16_t print_flag = 0;
    // print_flag++;
    // if (print_flag == 1)
    // {
    //   /* serial studio 绘图 */
    //   serial_test.config_data(motor1.true_angle, 0);
    //   serial_test.config_data(motor1.true_speed, 1);
    //   serial_test.config_data(motor1.true_current, 2);
    //   printf("\r\n");
    //   serial_test.send_frame();

    //   print_flag = 0;
    // }

    /* end */

    /* MPU6500测试代码 */
    // app_imu_So3thread();
    // printf("pitch = %5.2f  ", app_imu_data.Pitch);
    // printf("roll = %5.2f  ", app_imu_data.Roll);
    // printf("yaw = %5.2f  \r\n", app_imu_data.Yaw);

    // printf("Accel x = %5d  ",app_imu_data.original.Accel[0]);
    // printf("y = %5d  ",app_imu_data.original.Accel[1]);
    // printf("z = %5d  ",app_imu_data.original.Accel[2]);
    // printf("gyro x = %5d  ",app_imu_data.original.Gyro[0]);
    // printf("y = %5d  ",app_imu_data.original.Gyro[1]);
    // printf("z = %5d  \r\n",app_imu_data.original.Gyro[2]);
    /* end */

    /* app 编码器测试代码 */
    // printf("motor1Encoder.speed = %5.2f  ", motor1Encoder.speed);
    // printf("motor2Encoder.speed = %5.2f  \r\n", motor2Encoder.speed);

    /* 按键电机上电代码 */
    static uint16_t key_enable_motor_flag = 0;
    key_enable_motor_flag++;
    if (key_enable_motor_flag == 10)
    {
      key_enable_motor_flag = 0;
      if (USER_Button.OUT_State == Button_OFF && state_temp == Button_ON)
      {
        motor1.command_SafeMode();
        motor2.command_SafeMode();
      }
      else if (USER_Button.OUT_State == Button_ON && state_temp == Button_OFF)
      {
        motor1.command_RunMode();
        motor2.command_RunMode();
      }
      state_temp = USER_Button.OUT_State;
    }

    /* 速度环 测试代码 */
    // printf("balance_Mspeed = %5.2f  ", balance_Mspeed);
    // printf("speed_Mspeed = %5.2f  \r\n", speed_Mspeed);

    /* 平衡小车控制 */
    // app_imu_So3thread();
    Custom_Diff_temp = app_imu_data.original.Gyro[1];
    car_balance(1);
    car_speed(car_target_speed);
    car_turn(car_target_ratate);

    // car_3RingCtrl(balance_Mspeed, 0, 0);
    // car_3RingCtrl(0, speed_Mspeed, 0);
    car_3RingCtrl(balance_Mspeed, speed_Mspeed, turn_Mspeed);

    /* 电源电压值串口发送 */
    if (v_Battery < 6.9)
    {
      printf("\r\n\r\nthe v_battery is %5.2f\r\n\r\n", v_Battery);
    }

    /* 运行杂项 */
    command_usart1.run();
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
