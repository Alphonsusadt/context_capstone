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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim12;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// BTS7960 Motor Control Constants
//#define MOTOR_PWM_MAX 8399 // Maximum PWM value (ARR value)
#define MOTOR_SPEED_50 3600  // 50% duty cycle

// Motor Control Functions for BTS7960

//void motor_forward(uint16_t speed) {
//    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, speed);  // RPWM (PE5)  = speed
//    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);      // LPWM (PE6)  = 0
//    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, speed); // RPWM (PB14) = speed
//    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);     // LPWM (PB15) = 0
//    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, speed);  // RPWM (PB0)  = speed
//    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);	  // LPWM (PB1)  = 0
//    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed);  // RPWM (PA0)  = speed
//    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);	  // LPWM (PA1)  = 0
//}
//
//void motor_reverse(uint16_t speed) {
//    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);      // RPWM (PE5)  = 0
//    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, speed);  // LPWM (PE6)  = speed
//    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);     // RPWM (PB14) = 0
//    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, speed); // LPWM (PB15) = speed
//    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);      // RPWM (PB0)  = 0
//    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, speed);  // LPWM (PB1)  = speed
//    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);      // RPWM (PA0)  = 0
//    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, speed);  // LPWM (PA1)  = speed
//}
//
//void motor_stop(void) {
//    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);      // RPWM (PE5)  = 0
//    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);      // LPWM (PE6)  = 0
//    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);     // RPWM (PB14) = 0
//    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);     // LPWM (PB15) = 0
//    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);      // RPWM (PB0)  = 0
//    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);      // LPWM (PB1)  = 0
//    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);      // RPWM (PA0)  = 0
//    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);      // LPWM (PA1)  = 0
//}


void coba_motor(uint16_t speed){
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1,  speed);  // RPWM (PE5) = speed
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2,  speed); // LPWM (PE6) = 0
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, speed); // LPWM (PE6) = 0
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, speed); // LPWM (PE6) = 0
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,  speed); // LPWM (PE6) = 0
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4,  speed);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,  speed); // LPWM (PE6) = 0
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,  speed);
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
  MX_TIM9_Init();
  MX_TIM12_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  // Start PWM for BTS7960 Motor Driver
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);  // Start RPWM (PE5)
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);  // Start LPWM (PE6)
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);  // Start RPWM (PB14)
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);  // Start LPWM (PB15)
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);  // Start RPWM (PB0)
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);  // Start LPWM (PB1)
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);  // Start RPWM (PA0)
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);  // Start LPWM (PA1)

  // Initialize motor to stopped state
  //motor_stop();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // Motor Testing Loop: Forward -> Stop -> Reverse -> Stop

//	 // 1. Motor Forward (5 seconds)
//	 motor_forward(MOTOR_SPEED_50);  // 50% duty cycle
//	 HAL_Delay(5000);                // Run for 5 seconds
//
//	 // 2. Motor Stop (3 seconds)
//	 motor_stop();
//	 HAL_Delay(3000);                // Stop for 3 seconds
//
//	 // 3. Motor Reverse (5 seconds)
//	 motor_reverse(MOTOR_SPEED_50);  // 50% duty cycle
//	 HAL_Delay(5000);                // Run for 5 seconds
//
//	 // 4. Motor Stop (3 seconds)
//	 motor_stop();
//	 HAL_Delay(3000);                // Stop for 3 seconds                // Stop for 3 seconds

    // Loop repeats automatically

	coba_motor(MOTOR_SPEED_50);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4199;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4199;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 8399 ;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 0;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 4199;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
