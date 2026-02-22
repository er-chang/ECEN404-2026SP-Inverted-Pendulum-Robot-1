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
/* USER CODE END PD */
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */
/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim8;
/* USER CODE BEGIN PV */
// Moustafa's PID Variables
float dist_m; // Same distance in meters.
float pos_error; //I DONT KNOW, PLEASE FILL IN MOUSTAFA
float kp = 0.00357f; // I ALSO DONT KNOW
float target_angle; // Desired angle to return to stabilization.
int test_speed;
float target_dist;
static float theta;
float theta_dot;
float pitch_accel;
float balance_error;
float motor_effort;
int final_speed;
GPIO_PinState drive_dir;
// Ultrasonic Sensors
Ultrasonic frontSensor = {
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
Ultrasonic rightSensor = {
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
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void MX_I2C2_Init(void);
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
MX_TIM2_Init();
MX_TIM1_Init();
MX_TIM8_Init();
MX_I2C2_Init();
/* USER CODE BEGIN 2 */
// Starting all necessary timers.
HAL_TIM_Base_Start(&htim2); // Ultrasonic sensor Timer
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // FLM Timer
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // FRM Timer
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // BLM TImer
HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3); // BRM Timer
HAL_Delay(1);
//printf("\nTest Statement\n");
IMU_Init(&imu, &hi2c2);
/* USER CODE END 2 */
/* Infinite loop */
/* USER CODE BEGIN WHILE */
// MAIN WHILE LOOP - FOR MOST CONTROL CODE
int loop_counter = 0;
    while (1)
    {
          // 1.  SENSORS
          Read_Accel(&imu, &hi2c2);
          Read_Gyro(&imu, &hi2c2);
          static float filtered_gyro = 0.0f;
          filtered_gyro = 0.8f * filtered_gyro + 0.2f * imu.gyro.dps_y;
          float theta_dot = filtered_gyro * (3.14159f / 180.0f);
          float pitch_accel = atan2(imu.accel.g_z, imu.accel.g_x);
          // Sonar: Only ping the sensor every 5th loop (50ms) so echoes don't overlap
          loop_counter++;
          if (loop_counter >= 5) {
              frontSensor.distance = getSonarDistance(&frontSensor);
              dist_m = (float)frontSensor.distance / 100.0f;
              loop_counter = 0;
          }
          // 2. FILTER & UNIT CONVERSION
          theta_dot = imu.gyro.dps_y * (3.14159f / 180.0f); // DPS to Rad/s
          pitch_accel = atan2(imu.accel.g_z, imu.accel.g_x);
          theta = 0.0f;
          theta = 0.98f * (theta + (theta_dot * 0.01f)) + 0.02f * pitch_accel;
          // 3. SAFETY KILL-SWITCH
         // if (fabs(theta) > 0.78f) {
              //TIM1->CCR1 = 0; TIM1->CCR2 = 0; TIM1->CCR3 = 0; TIM8->CCR3 = 0;
              //HAL_Delay(10);
              //continue;

          // 4. OUTER PID LOOP
          pos_error = 0.5f - dist_m;
          target_angle = -(kp * pos_error);
          // Clamp lean angle to ~15 degrees
          if (target_angle > 0.26f) target_angle = 0.26f;
          if (target_angle < -0.26f) target_angle = -0.26f;
          // 5. INNER LQR LOOP
          balance_error = theta - target_angle;
          motor_effort = (23.6952f * balance_error) + (1.9479f * theta_dot);
          // 6. ACTUATION
          final_speed = (int)(fabs(motor_effort) * 80.0f);
          // DEADBAND COMPENSATION
          if (final_speed > 0 && final_speed < 25) final_speed = 25;
          if (final_speed > MAX_SPEED) final_speed = MAX_SPEED;
          if (final_speed < MIN_SPEED) final_speed = MIN_SPEED;
          drive_dir = (motor_effort > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
          // APPLY TO MOTORS
          HAL_GPIO_WritePin(FLM.directionPort, FLM.directionPin, drive_dir);
          HAL_GPIO_WritePin(FRM.directionPort, FRM.directionPin, drive_dir);
          HAL_GPIO_WritePin(BLM.directionPort, BLM.directionPin, drive_dir);
          HAL_GPIO_WritePin(BRM.directionPort, BRM.directionPin, drive_dir);
          TIM1->CCR1 = final_speed;
          TIM1->CCR2 = final_speed;
          TIM1->CCR3 = final_speed;
          TIM8->CCR3 = final_speed;
          HAL_Delay(10);
    }
}   /* USER CODE END WHILE */
   /* USER CODE BEGIN 3 */
 /* USER CODE END 3 */
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
RCC_OscInitStruct.PLL.PLLM = 4;
RCC_OscInitStruct.PLL.PLLN = 84;
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
RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
hi2c2.Init.ClockSpeed = 100000;
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
htim1.Init.Prescaler = 16;
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
htim2.Init.Prescaler = 83;
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
TIM_MasterConfigTypeDef sMasterConfig = {0};
TIM_OC_InitTypeDef sConfigOC = {0};
TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
/* USER CODE BEGIN TIM8_Init 1 */
/* USER CODE END TIM8_Init 1 */
htim8.Instance = TIM8;
htim8.Init.Prescaler = 16;
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
/* USER CODE BEGIN TIM8_Init 2 */
/* USER CODE END TIM8_Init 2 */
HAL_TIM_MspPostInit(&htim8);
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
__HAL_RCC_GPIOC_CLK_ENABLE();
__HAL_RCC_GPIOH_CLK_ENABLE();
__HAL_RCC_GPIOF_CLK_ENABLE();
__HAL_RCC_GPIOG_CLK_ENABLE();
__HAL_RCC_GPIOE_CLK_ENABLE();
__HAL_RCC_GPIOB_CLK_ENABLE();
__HAL_RCC_GPIOA_CLK_ENABLE();
/*Configure GPIO pin Output Level */
HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);
/*Configure GPIO pin Output Level */
HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0|Trig2_Pin, GPIO_PIN_RESET);
/*Configure GPIO pin Output Level */
HAL_GPIO_WritePin(Trig1_GPIO_Port, Trig1_Pin, GPIO_PIN_RESET);
/*Configure GPIO pin Output Level */
HAL_GPIO_WritePin(Trig3_GPIO_Port, Trig3_Pin, GPIO_PIN_RESET);
/*Configure GPIO pins : PF13 PF14 */
GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
/*Configure GPIO pin : PF15 */
GPIO_InitStruct.Pin = GPIO_PIN_15;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
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
GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
GPIO_InitStruct.Pull = GPIO_NOPULL;
HAL_GPIO_Init(Echo1_GPIO_Port, &GPIO_InitStruct);
/*Configure GPIO pin : Trig3_Pin */
GPIO_InitStruct.Pin = Trig3_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(Trig3_GPIO_Port, &GPIO_InitStruct);
/*Configure GPIO pin : Echo3_Pin */
GPIO_InitStruct.Pin = Echo3_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
GPIO_InitStruct.Pull = GPIO_NOPULL;
HAL_GPIO_Init(Echo3_GPIO_Port, &GPIO_InitStruct);
/*Configure GPIO pin : Echo2_Pin */
GPIO_InitStruct.Pin = Echo2_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
GPIO_InitStruct.Pull = GPIO_NOPULL;
HAL_GPIO_Init(Echo2_GPIO_Port, &GPIO_InitStruct);
/* EXTI interrupt init*/
HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}
/* USER CODE BEGIN 4 */
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
/*printf config function*/
/* USER CODE BEGIN 4 */
#ifdef __cplusplus
extern "C" {
#endif
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
/*End printf config function*/
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


