/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <peripherals.h> // Where most peripheral functions are stored
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_SPEED 256
#define MIN_SPEED 0
#define PWM_DEADZONE 22        // Minimum PWM to overcome motor static friction
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c2_rx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */
static const float PI_OVER_180 = 3.14159265f / 180.0f;
static const float PWM_SCALE = 50.0f;

// Ultrasonic Sensors
Ultrasonic rightSensor = {
		.TriggerPort = 	GPIOA,
		.EchoPort =		GPIOA,
		.TriggerPin =	GPIO_PIN_9,
		.EchoPin =		GPIO_PIN_10,
		.timer = 		&htim2
};
Ultrasonic leftSensor = {
		.TriggerPort = 	GPIOG,
		.EchoPort =		GPIOG,
		.TriggerPin =	GPIO_PIN_14,
		.EchoPin =		GPIO_PIN_9,
		.timer = 		&htim2
};
Ultrasonic frontSensor = {
		.TriggerPort = 	GPIOC,
		.EchoPort =		GPIOC,
		.TriggerPin =	GPIO_PIN_10,
		.EchoPin =		GPIO_PIN_11,
		.timer = 		&htim2
};
// Motors
Motor FLM = { .PWM = &TIM1->CCR1, .directionPort = GPIOF, .directionPin = GPIO_PIN_13 }; // Front Left Motor
Motor FRM = { .PWM = &TIM1->CCR3, .directionPort = GPIOF, .directionPin = GPIO_PIN_14 }; // Front Right Motor — TIM1 CH3
Motor BLM = { .PWM = &TIM1->CCR2, .directionPort = GPIOF, .directionPin = GPIO_PIN_15 }; // Back Left Motor — TIM1 CH2
Motor BRM = { .PWM = &TIM8->CCR3, .directionPort = GPIOG, .directionPin = GPIO_PIN_0 }; // Back Right Motor
// IMU
IMU imu;
volatile uint8_t imu_dma_ready = 0; // Set by DMA complete callback
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */
int _write(int32_t file, uint8_t *ptr, int32_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	// Moustafa's PID Variables
	float dist_m = 0.0f; // Same distance in meters.
	float pos_error; //calculates the position error
	// TUNING GUIDE (tune in this order):
	//   kp: start here. Robot 20cm too far → should lean ~0.02 rad. kp=0.10 gives that.
	//        Too high → robot lurches toward/away from obstacle
	//        Too low  → robot barely reacts to distance changes
	//   kd: add next. Dampens approach speed. Prevents overshoot past target distance.
	//        Too high → robot feels sluggish, won't reach target
	//        Too low  → robot overshoots and bounces back and forth
	//   ki: add LAST, only if robot settles at wrong distance.
	//        Too high → slow hunting oscillation
	//        Too low  → steady-state offset from target
	float kp = 0.03f;   // gentler — 0.5m error → 0.015 rad lean
	float ki = 0.0f;     // off for now — add once position hold works
	float kd = 0.0f;     // off for now — sonar too noisy for derivative
	float target_angle = 0.0f; // Desired angle to return to stabilization.
	int test_speed;
	float target_dist;
	static float theta;
	float balance_error;
	float motor_effort;
	int final_speed;
	GPIO_PinState drive_dir;

	static uint32_t prev_time;
	float dt;
	static float filtered_gyro;
	float theta_dot;
	float pitch_accel;
	static float balance_integral;
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
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  // Starting all necessary timers.
  HAL_TIM_Base_Start(&htim2); // Ultrasonic sensor Timer
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // FLM Timer
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // FRM Timer
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // BLM TImer
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3); // BRM Timer
  HAL_Delay(1);

  IMU_Init(&imu, &hi2c2);

  // Initialize theta from accelerometer — retry up to 5 times
  theta = 0.0f;
  for (int attempt = 0; attempt < 5; attempt++) {
      Read_IMU(&imu, &hi2c2);
      if (imu.status == HAL_OK) {
          theta = atan2(imu.accel.g_z, -imu.accel.g_x);
          break;
      }
      HAL_Delay(10);
  }

  int loop_counter = 0;
  int debug_counter = 0;
  const uint32_t LOOP_PERIOD_US = 5000; // 5 ms target → 200 Hz control rate
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // ── 1. READ IMU ──
	  Read_IMU(&imu, &hi2c2);

	  // Raw gyro — zero lag
	  theta_dot = (imu.gyro.dps_y - imu.gyro.bias_y) * PI_OVER_180;
	  pitch_accel = atan2(imu.accel.g_z, -imu.accel.g_x);

	  // ── 2. COMPLEMENTARY FILTER — 70% gyro, 30% accel ──
	  // Fast enough to respond (converges in ~3 loops = 15ms)
	  // Smooth enough to kill motor vibration noise from accel
	  theta = 0.85f * (theta + (theta_dot * 0.005f)) + 0.15f * pitch_accel;

	  // ── 3. PD CONTROLLER (Ziegler-Nichols tuned) ──
	  // Ku≈38, Tu≈1.0s → Kp=0.8*Ku=30, Kd=Kp*Tu/8=3.75
	  motor_effort = (38.0f * theta)
	               + (4.0f * theta_dot);

	  // ── 4. ACTUATION — match the accel test: scale * |effort| + deadzone ──
	  final_speed = (int)(fabs(motor_effort) * PWM_SCALE) + PWM_DEADZONE;
	  if (final_speed > MAX_SPEED) final_speed = MAX_SPEED;
	  if (fabs(motor_effort) < 0.01f) final_speed = 0;

	  drive_dir = (motor_effort > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;

	  setSpeed(&FLM, final_speed, drive_dir);
	  setSpeed(&FRM, final_speed, drive_dir);
	  setSpeed(&BLM, final_speed, drive_dir);
	  setSpeed(&BRM, final_speed, drive_dir);

	  HAL_Delay(5);
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 31;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 255;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;  // 84MHz/(83+1) = 1MHz = 1μs ticks (was 167 = 2μs WRONG)
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 31;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 255;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0|Trig2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Trig1_GPIO_Port, Trig1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Trig_Front_GPIO_Port, Trig_Front_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PF13 PF14 PF15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PG0 Trig2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|Trig2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : Trig1_Pin */
  GPIO_InitStruct.Pin = Trig1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Trig1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Echo1_Pin */
  GPIO_InitStruct.Pin = Echo1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;  // was RISING only — never caught falling edge
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Echo1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Trig_Front_Pin */
  GPIO_InitStruct.Pin = Trig_Front_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Trig_Front_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Echo_Front_Pin */
  GPIO_InitStruct.Pin = Echo_Front_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Echo_Front_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Echo2_Pin */
  GPIO_InitStruct.Pin = Echo2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Echo2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

#ifdef __cplusplus
extern "C" {
#endif

// DMA complete callback — called by HAL when I2C2 DMA read finishes
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C2) {
        imu_dma_ready = 1;
    }
}

// External interrupt handler
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  // Check if the interrupt comes from an Ultrasonic Echo Pin
	// Trigger if front Sensor
	if (GPIO_Pin == frontSensor.EchoPin){
		if (HAL_GPIO_ReadPin(frontSensor.EchoPort, frontSensor.EchoPin) == GPIO_PIN_SET){
			// Record Time at Echo Start
			frontSensor.echo_start = __HAL_TIM_GET_COUNTER(frontSensor.timer);
		}
		else{
			// Record Time at Echo End
			frontSensor.echo_end = __HAL_TIM_GET_COUNTER(frontSensor.timer);
		}
	}
	// Trigger if left Sensor
	if (GPIO_Pin == leftSensor.EchoPin){
		if (HAL_GPIO_ReadPin(leftSensor.EchoPort, leftSensor.EchoPin) == GPIO_PIN_SET){
			// Record Time at Echo Start
			leftSensor.echo_start = __HAL_TIM_GET_COUNTER(leftSensor.timer);
		}
		else{
			// Record Time at Echo End
			leftSensor.echo_end = __HAL_TIM_GET_COUNTER(leftSensor.timer);
		}
	}
	// Trigger if right Sensor
	if (GPIO_Pin == rightSensor.EchoPin){
		if (HAL_GPIO_ReadPin(rightSensor.EchoPort, rightSensor.EchoPin) == GPIO_PIN_SET){
			// Record Time at Echo Start
			rightSensor.echo_start = __HAL_TIM_GET_COUNTER(rightSensor.timer);
		}
		else{
			// Record Time at Echo End
			rightSensor.echo_end = __HAL_TIM_GET_COUNTER(rightSensor.timer);
		}
	}
}

int _write(int file, uint8_t *ptr, int len)
{
  for (int i = 0; i < len; i++)
  {
      ITM_SendChar(*ptr++);
  }
  return len;
}

#ifdef __cplusplus
}
#endif
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
