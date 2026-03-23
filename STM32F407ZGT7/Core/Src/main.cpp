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
#define PWM_DEADZONE 10
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
static const float PWM_SCALE = 28.0f;

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
Motor FRM = { .PWM = &TIM1->CCR2, .directionPort = GPIOF, .directionPin = GPIO_PIN_14 }; // Front Right Motor
Motor BLM = { .PWM = &TIM1->CCR3, .directionPort = GPIOF, .directionPin = GPIO_PIN_15 }; // Back Left Motor
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
	// Control variables
	float target_angle = 0.0f;
	static float theta = 0.0f;
	float balance_error;
	float motor_effort;
	int final_speed;
	GPIO_PinState drive_dir;

	static uint32_t prev_time = 0;
	float dt;
	static float filtered_gyro = 0.0f;
	float theta_dot;
	float pitch_accel;
	static float balance_integral = 0.0f;
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
  HAL_TIM_Base_Start(&htim2); // Microsecond timer (1 MHz = 1μs ticks)
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // FLM Timer
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // FRM Timer
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // BLM Timer
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3); // BRM Timer
  HAL_Delay(1);

  // Initialize IMU — this now:
  //   1. Writes CTRL1_XL=0x70 (833Hz, ±2g) and CTRL2_G=0x70 (833Hz, ±250dps)
  //   2. Runs gyro calibration (200 samples, ~400ms, robot must be still)
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
  //                     ┌───────────────────────────────────────────────────┐
  //                     │ 500 Hz = 2000 μs per loop                        │
  //                     │ TIM2 now ticks at 1 μs (prescaler=83, 84MHz APB1)│
  //                     │ OLD BUG: prescaler was 167 → 2μs ticks           │
  //                     │   so "2000 counts" was actually 4ms = 250Hz      │
  //                     │   and dt calc was half actual → broken filter     │
  //                     └───────────────────────────────────────────────────┘
  const uint32_t LOOP_PERIOD_US = 2000; // 2 ms target → 500 Hz control rate
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  uint32_t loop_start = __HAL_TIM_GET_COUNTER(&htim2);

	  // ── Measure real dt ──
	  dt = (prev_time == 0) ? 0.002f
	             : (float)(loop_start - prev_time) * 1e-6f;
	  if (dt > 0.02f || dt < 0.0005f) dt = 0.002f; // sanity clamp for 500Hz
	  prev_time = loop_start;

	  // ── 1. READ IMU (blocking, ~0.34ms at 400kHz I2C) ──
	  Read_IMU(&imu, &hi2c2);

	  // Apply gyro bias correction (measured during Calibrate_Gyro at startup)
	  float raw_gyro_y = imu.gyro.dps_y - imu.gyro.bias_y;

	  // Low-pass filter on gyro (removes high-frequency vibration noise)
	  filtered_gyro = 0.75f * filtered_gyro + 0.25f * raw_gyro_y;
	  theta_dot = filtered_gyro * PI_OVER_180;  // convert dps → rad/s

	  // Accelerometer-derived pitch (immune to drift, noisy short-term)
	  pitch_accel = atan2(imu.accel.g_z, -imu.accel.g_x);

	  // ── 2. COMPLEMENTARY FILTER ──
	  // α = τ/(τ+dt). At 500Hz (dt=2ms), τ=0.5s → α≈0.996
	  // Heavily trusts gyro for fast response, slowly corrects drift with accel.
	  //
	  // ┌─────────────────────────────────────────────────────────┐
	  // │ OLD VALUE 0.992 was for τ≈0.25s — too much accel trust │
	  // │ Most working projects use 0.995-0.9996                  │
	  // │ Brokking YABR (the gold standard) uses ~0.9996          │
	  // └─────────────────────────────────────────────────────────┘
	  theta = 0.996f * (theta + (theta_dot * dt)) + 0.004f * pitch_accel;

	  // ── 3. BALANCE CONTROLLER (LQR-style: angle + rate + integral) ──
	  balance_error = theta - target_angle;

	  balance_integral += balance_error * dt;
	  if (balance_integral >  1.0f) balance_integral =  1.0f;
	  if (balance_integral < -1.0f) balance_integral = -1.0f;

	  // ┌─────────────────────────────────────────────────────────────┐
	  // │ GAIN NOTES after fixing IMU:                                │
	  // │                                                             │
	  // │ OLD gyro was 2x too large, so "9.0 * theta_dot" was        │
	  // │ effectively "18.0 * real_theta_dot". Now gyro is correct.   │
	  // │                                                             │
	  // │ OLD dt was 2x too small (TIM2 bug), so balance_integral     │
	  // │ accumulated at half-speed, and theta integrated at half-rate.│
	  // │                                                             │
	  // │ These gains are starting points — RETUNE on hardware:       │
	  // │   Kp (38): increase until oscillation, back off 20%         │
	  // │   Kd (9):  increase until oscillation dampens, not sluggish │
	  // │   Ki (0.3): increase only if steady-state tilt offset       │
	  // └─────────────────────────────────────────────────────────────┘
	  motor_effort = (38.0f * balance_error)
	               + (9.0f  * theta_dot)
	               + (0.3f  * balance_integral);

	  // ── 4. ACTUATION ──
	  if (fabs(motor_effort) < 0.10f) {
	      final_speed = 0;
	  } else {
	      final_speed = (int)(fabs(motor_effort) * PWM_SCALE) + PWM_DEADZONE;
	  }
	  if (final_speed > MAX_SPEED) final_speed = MAX_SPEED;

	  drive_dir = (motor_effort > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;

	  HAL_GPIO_WritePin(FLM.directionPort, FLM.directionPin, drive_dir);
	  HAL_GPIO_WritePin(FRM.directionPort, FRM.directionPin, drive_dir);
	  HAL_GPIO_WritePin(BLM.directionPort, BLM.directionPin, drive_dir);
	  HAL_GPIO_WritePin(BRM.directionPort, BRM.directionPin, drive_dir);
	  TIM1->CCR1 = final_speed;
	  TIM1->CCR2 = final_speed;
	  TIM1->CCR3 = final_speed;
	  TIM8->CCR3 = final_speed;

	  // ── Spin-wait for precise loop period ──
	  while ((__HAL_TIM_GET_COUNTER(&htim2) - loop_start) < LOOP_PERIOD_US);
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
  * @brief I2C2 Initialization Function — 400 kHz Fast Mode
  */
static void MX_I2C2_Init(void)
{
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
}

/**
  * @brief TIM1 Initialization Function — PWM for 3 motors (PE9/PE11/PE13)
  *        APB2 timer clock = 168 MHz. 168M/(31+1)/(255+1) = 20.5 kHz
  */
static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

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
  HAL_TIM_MspPostInit(&htim1);
}

/**
  * @brief TIM2 Initialization Function — Microsecond timer (loop timing + sonar)
  *        APB1 timer clock = 84 MHz.
  * ┌─────────────────────────────────────────────────────────────┐
  * │ CRITICAL FIX: Prescaler changed from 167 → 83              │
  * │                                                             │
  * │ OLD: 84MHz/(167+1) = 500kHz = 2μs per tick                 │
  * │   But ALL code assumed 1μs ticks (* 1e-6f conversions)      │
  * │   → dt was HALF actual → complementary filter broken        │
  * │   → loop ran 250Hz not 500Hz                                │
  * │   → sonar distances 2x too large                            │
  * │                                                             │
  * │ NEW: 84MHz/(83+1) = 1MHz = 1μs per tick ← CORRECT          │
  * └─────────────────────────────────────────────────────────────┘
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;  // ← WAS 167. 84MHz/(83+1) = 1MHz = 1μs ticks
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;  // 32-bit max → wraps every ~71 minutes
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
}

/**
  * @brief TIM8 Initialization Function — PWM for BRM motor (PC8)
  *        APB2 timer clock = 168 MHz. 168M/(31+1)/(255+1) = 20.5 kHz
  */
static void MX_TIM8_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 31;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 255;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();  // For encoder TIM4 on PD12/PD13

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0|Trig2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Trig1_GPIO_Port, Trig1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Trig_Front_GPIO_Port, Trig_Front_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PF13 PF14 PF15 — Motor direction pins */
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

  /*Configure GPIO pin : Trig1_Pin (PA9) */
  GPIO_InitStruct.Pin = Trig1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Trig1_GPIO_Port, &GPIO_InitStruct);

  // ┌─────────────────────────────────────────────────────────┐
  // │ FIX: Echo1 (PA10) changed from IT_RISING → IT_RISING_  │
  // │ FALLING. With RISING only, the EXTI callback could      │
  // │ never see the falling edge, so echo_end was never set   │
  // │ → sonar always returned 0.                              │
  // └─────────────────────────────────────────────────────────┘
  /*Configure GPIO pin : Echo1_Pin (PA10) */
  GPIO_InitStruct.Pin = Echo1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Echo1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Trig_Front_Pin (PC10) */
  GPIO_InitStruct.Pin = Trig_Front_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Trig_Front_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Echo_Front_Pin (PC11) */
  GPIO_InitStruct.Pin = Echo_Front_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Echo_Front_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Echo2_Pin (PG9) */
  GPIO_InitStruct.Pin = Echo2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Echo2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);  // Priority 1 (below DMA/I2C)
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  // ┌─────────────────────────────────────────────────────────┐
  // │ FIX: Enable EXTI9_5 IRQ — Echo2 is on PG9 (EXTI line 9)│
  // │ Without this, the left sensor echo interrupts were       │
  // │ silently dropped and leftSensor always returned 0.       │
  // └─────────────────────────────────────────────────────────┘
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
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
			frontSensor.echo_start = __HAL_TIM_GET_COUNTER(frontSensor.timer);
		}
		else{
			frontSensor.echo_end = __HAL_TIM_GET_COUNTER(frontSensor.timer);
		}
	}
	// Trigger if left Sensor
	if (GPIO_Pin == leftSensor.EchoPin){
		if (HAL_GPIO_ReadPin(leftSensor.EchoPort, leftSensor.EchoPin) == GPIO_PIN_SET){
			leftSensor.echo_start = __HAL_TIM_GET_COUNTER(leftSensor.timer);
		}
		else{
			leftSensor.echo_end = __HAL_TIM_GET_COUNTER(leftSensor.timer);
		}
	}
	// Trigger if right Sensor
	if (GPIO_Pin == rightSensor.EchoPin){
		if (HAL_GPIO_ReadPin(rightSensor.EchoPort, rightSensor.EchoPin) == GPIO_PIN_SET){
			rightSensor.echo_start = __HAL_TIM_GET_COUNTER(rightSensor.timer);
		}
		else{
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
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
