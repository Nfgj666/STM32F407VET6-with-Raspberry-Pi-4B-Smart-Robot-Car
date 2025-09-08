/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "encoder_motor.h"
#include "differential_chassis.h"
#include "motor_porting.h"
#include "line_follower.h"
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
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// PID 控制相关变量
static float kp = 0.5f;  // 比例系数
static float ki = 0.1f;  // 积分系数
static float kd = 0.01f; // 微分系数

void line_folower_pid_control()
{
    // 获取 PID 控制输出
    float output = line_folower(kp, ki, kd);  // 调用你之前的 line_folower 函数

    // 设定一个线性速度 vx，假设小车在前进（可以根据需要调整）
    float vx = 2.0f;  // 线性速度，单位：mm/s

    extern EncoderMotorObjectTypeDef motor1;
	extern EncoderMotorObjectTypeDef motor2;
	extern EncoderMotorObjectTypeDef motor3;
	extern EncoderMotorObjectTypeDef motor4;
	float motor_rps = motor1.rps;  // 电机转速，单位 rps（转每秒）

    // 使用 diff_chassis_move 调整小车的运动
    // output 为 PID 计算出的角速度，表示小车需要调整的转向
    if (output > 0) {
        // PID 输出为正，表示小车需要向右调整
        diff_chassis_move(&chassis, vx, -output);  // 调整角速度，右转
    } else if (output < 0) {
        // PID 输出为负，表示小车需要向左调整
        diff_chassis_move(&chassis, vx, -output);  // 调整角速度，左转
    } else {
        // PID 输出为0，表示小车应直行
        diff_chassis_move(&chassis, vx, 0.0f);  // 直行
    }
    printf("Linear Speed (vx): %f mm/s, Angular Speed (output): %f rad/s, Motor RPS: %f\n", vx, output, motor_rps);
}


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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  motor_init();  // 初始化电机
  extern EncoderMotorObjectTypeDef motor1;
  extern EncoderMotorObjectTypeDef motor2;
  extern EncoderMotorObjectTypeDef motor3;
  extern EncoderMotorObjectTypeDef motor4;
  extern DifferentialChassisTypeDef chassis;
  diff_chassis_object_init(&chassis);  // 初始化差速底盘控制对象
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // 1. 直线前进2秒
	  diff_chassis_move(&chassis, 200.0f, 0.0f);
	  HAL_Delay(200);

	  // 2. 停止1秒
	  diff_chassis_move(&chassis, 0.0f, 0.0f);
	  HAL_Delay(3000);

	  // 3. 原地左转2秒
	  diff_chassis_move(&chassis, 0.0f, 1.5f);
	  HAL_Delay(200);

	  // 4. 停止1秒
	  diff_chassis_move(&chassis, 0.0f, 0.0f);
	  HAL_Delay(3000);

	  // 5. 直线后退2秒
	  diff_chassis_move(&chassis, -200.0f, 0.0f);
	  HAL_Delay(200);

	  // 6. 停止1秒
	  diff_chassis_move(&chassis, 0.0f, 0.0f);
	  HAL_Delay(3000);

	  // 7. 原地右转2秒
	  diff_chassis_move(&chassis, 0.0f, -1.5f);
	  HAL_Delay(200);

	  // 8. 停止1秒
	  diff_chassis_move(&chassis, 0.0f, 0.0f);
	  HAL_Delay(3000);
	  // 调用 PID 控制函数来调整小车运动
	  //line_folower_pid_control(&chassis);

	  // 延时，防止 CPU 占用过高
	 // HAL_Delay(10);  // 适当延时，调整控制频率



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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM5_IRQn);
  /* TIM6_DAC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

#ifdef  USE_FULL_ASSERT
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
