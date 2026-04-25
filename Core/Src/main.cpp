/* USER CODE BEGIN Header */

/**

 ******************************************************************************

 * @file : main.c

 * @brief : Main program body

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

#include "peripherals.h"

#include <stdio.h>

#include "flash_storage.h"

#include <math.h>

#include "string.h"

#include "navigation.h"

/* USER CODE END Includes */



/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PTD */



/* USER CODE END PTD */



/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN PD */



#define MAX_SPEED 1024 // max PWM

#define MIN_SPEED 0 // min PWM

#define PWM_DEADZONE 68 // PWM floor to break motor static friction

#define LOG_FLAG false // toggle debug logging



#if LOG_FLAG

// data logger, view in debugger expressions

#define LOG_SIZE 5000 // 5000 samples at 500Hz = 10s

#endif



/* USER CODE END PD */



/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PM */



/* USER CODE END PM */



/* Private variables ---------------------------------------------------------*/

ADC_HandleTypeDef hadc1;

ADC_HandleTypeDef hadc3;



I2C_HandleTypeDef hi2c2;



SPI_HandleTypeDef hspi1;

DMA_HandleTypeDef hdma_spi1_tx;

DMA_HandleTypeDef hdma_spi1_rx;



TIM_HandleTypeDef htim1;

TIM_HandleTypeDef htim5;

TIM_HandleTypeDef htim14;



/* USER CODE BEGIN PV */

static const float PI_OVER_180 = 3.14159265f / 180.0f;

static const float RAD_TO_DEG = 180.0f / 3.14159265f;

static const float PWM_SCALE = 200.0f;



#if LOG_FLAG

// data logger buffers

volatile float log_theta[LOG_SIZE];

volatile float log_effort[LOG_SIZE];

volatile uint16_t log_idx = 0;

volatile uint8_t log_done = 0; // set when robot has fallen and data is ready

#endif



volatile uint16_t adc_raw[1];



Potentiometer pot = {

 .dma_buffer = (uint16_t*)adc_raw,

 .center_value = 2048,

 .raw = 0,

 .deg_per_count = 270.0f / 4095.0f,

 .current_angle = 0.0f,

 .velocity = 0.0f

};



// Ultrasonic Sensors (PCB Pin Configurations)

Ultrasonic frontSensor = {

	.TriggerPort = F_TRIG_GPIO_Port, // PC6

	.EchoPort =	F_ECHO_GPIO_Port, // PA0

	.TriggerPin = F_TRIG_Pin,

	.EchoPin =	F_ECHO_Pin,

	.timer = &htim5

};



Ultrasonic backSensor = {

	.TriggerPort = B_TRIG_GPIO_Port, // PG1

	.EchoPort =	B_ECHO_GPIO_Port, // PB1

	.TriggerPin = B_TRIG_Pin,

	.EchoPin =	B_ECHO_Pin,

	.timer = &htim5

};



Ultrasonic leftSensor = {

	.TriggerPort = L_TRIG_GPIO_Port, // PC7

	.EchoPort =	L_ECHO_GPIO_Port, // PG9

	.TriggerPin = L_TRIG_Pin,

	.EchoPin =	L_ECHO_Pin,

	.timer = &htim5

};



Ultrasonic rightSensor = {

	.TriggerPort = R_TRIG_GPIO_Port, // PC8

	.EchoPort =	R_ECHO_GPIO_Port, // PA10

	.TriggerPin = R_TRIG_Pin,

	.EchoPin =	R_ECHO_Pin,

	.timer = &htim5

};



// motors, diagonal mapping

Motor FRM = { .PWM = &TIM1->CCR1, .directionPort = GPIOF, .directionPin = GPIO_PIN_13 };

Motor FLM = { .PWM = &TIM1->CCR2, .directionPort = GPIOF, .directionPin = GPIO_PIN_14 };

Motor BRM = { .PWM = &TIM1->CCR3, .directionPort = GPIOF, .directionPin = GPIO_PIN_15 };

Motor BLM = { .PWM = &TIM1->CCR4, .directionPort = GPIOG, .directionPin = GPIO_PIN_0 };



static TelemetryPacket tx_packet;

static CommandPacket rx_packet;

static uint8_t spi_rx_raw_buffer[sizeof(TelemetryPacket)];



static volatile uint8_t spi_in_progress = 0;

volatile uint8_t system_state = 0;

volatile float theta_offset = 0.0f; // live trim from ESP32 F/B buttons



// live expression globals for debugger

volatile uint32_t g_avg_center = 0; // averaged pot center, set once after calibration

volatile uint32_t g_heartbeat = 0; // ticks so we know MCU is alive

volatile uint32_t g_pot_live = 0; // current raw pot reading



// SPI diagnostics

volatile uint32_t g_spi_tx_attempts = 0;

volatile uint32_t g_spi_rx_complete = 0;

volatile uint32_t g_spi_recoveries = 0;

volatile uint8_t g_last_cmd = 0;



volatile uint8_t current_robot_mode = 1; // 1=balance only, 2=nav with rod, 3=drive only

volatile float tune_kp = 45.0f;

volatile float tune_ki = 0.0f;

volatile float tune_kd = 0.0f;

static uint16_t prev_encoder_count = 0;

Nav my_nav;

IMU my_imu;

uint32_t sensor_timer = 0;



/* USER CODE END PV */



/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);

static void MPU_Config(void);

static void MX_GPIO_Init(void);

static void MX_DMA_Init(void);

static void MX_I2C2_Init(void);

static void MX_TIM1_Init(void);

static void MX_ADC3_Init(void);

static void MX_SPI1_Init(void);

static void MX_TIM5_Init(void);

static void MX_TIM14_Init(void);

static void MX_ADC1_Init(void);

/* USER CODE BEGIN PFP */



int _write(int32_t file, uint8_t *ptr, int32_t len); // printf to SWV console, don't delete

static void SPI1_Recover(void);



/* USER CODE END PFP */



/* Private user code ---------------------------------------------------------*/

/* USER CODE BEGIN 0 */



static void SPI1_Recover(void) {

 g_spi_recoveries++;

 HAL_SPI_DeInit(&hspi1);

 HAL_Delay(2);

 MX_SPI1_Init();

 HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

}



uint16_t Read_AS5600(I2C_HandleTypeDef* hi2c) {

 uint8_t rx_data[2];

 const uint8_t AS5600_ADDR = 0x36 << 1;

 const uint8_t RAW_ANGLE_REG = 0x0C;



 HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, AS5600_ADDR, RAW_ANGLE_REG, I2C_MEMADD_SIZE_8BIT, rx_data, 2, 5);



 if (status == HAL_OK) {

 return (uint16_t)((rx_data[0] << 8) | rx_data[1]);

 }

 return 0;

}



uint16_t Read_ADC_Channel(ADC_HandleTypeDef* hadc, uint32_t channel) {

 ADC_ChannelConfTypeDef sConfig = {0};

 sConfig.Channel = channel;

 sConfig.Rank = ADC_REGULAR_RANK_1;

 sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;



 HAL_ADC_ConfigChannel(hadc, &sConfig);



 HAL_ADC_Start(hadc);

 HAL_ADC_PollForConversion(hadc, 5);

 uint16_t value = HAL_ADC_GetValue(hadc);

 HAL_ADC_Stop(hadc);



 return value;

}



float get_median(volatile float *array, uint8_t size) {

 float temp[10]; // max buffer size



 for (uint8_t i = 0; i < size; i++) {

 temp[i] = array[i];

 }



 // insertion sort

 for (uint8_t i = 1; i < size; i++) {

 float key = temp[i];

 int8_t j = i - 1;

 while (j >= 0 && temp[j] > key) {

 temp[j + 1] = temp[j];

 j = j - 1;

 }

 temp[j + 1] = key;

 }



 return temp[size / 2];

}



/* USER CODE END 0 */



/**

 * @brief The application entry point.

 * @retval int

 */

int main(void)

{



 /* USER CODE BEGIN 1 */

	float target_angle = 0.0f;

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



 /* MPU Configuration--------------------------------------------------------*/

 MPU_Config();



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

 MX_ADC3_Init();

 MX_SPI1_Init();

 MX_TIM5_Init();

 MX_TIM14_Init();

 MX_ADC1_Init();

 /* USER CODE BEGIN 2 */

	// unbuffered stdout so printf hits SWV ITM right away

	setvbuf(stdout, NULL, _IONBF, 0);



	// start timers

 HAL_TIM_Base_Start_IT(&htim14);

 HAL_TIM_Base_Start(&htim5);

 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);



	// pot center calibration

#if true

 {

 const int N_SAMPLES = 200;

 uint32_t acc = 0;

 for (int i = 0; i < N_SAMPLES; i++) {

 HAL_ADC_Start(&hadc3);

 if (HAL_ADC_PollForConversion(&hadc3, 10) == HAL_OK) {

 uint32_t sample = HAL_ADC_GetValue(&hadc3);

 acc += sample;

 g_pot_live = sample;

 }

 g_heartbeat++;

 HAL_Delay(10);

 }

 pot.center_value = acc / N_SAMPLES;

 g_avg_center = pot.center_value;

 }

#endif



 const float POT_RAD_PER_COUNT = (270.0f * 3.14159265f / 180.0f) / 4095.0f;

 theta = 0.0f;

 const uint32_t LOOP_PERIOD_US = 1000; // 1ms loop, 1kHz



 Nav_Init(&my_nav);

 IMU_Init(&my_imu, &hi2c2); // keep robot still here



 // ESP32 telemetry

 uint32_t telemetry_counter = 0;

 uint8_t spi_fail_count = 0;

 memset(&tx_packet, 0, sizeof(tx_packet));

 memset(&rx_packet, 0, sizeof(rx_packet));



 static float pid_ITerm = 0.0f;

 static float pid_lastInput = 0.0f;



 system_state = 0; // start disarmed



 /* USER CODE END 2 */



 /* Infinite loop */

 /* USER CODE BEGIN WHILE */

	while (1)

	{

 /* USER CODE END WHILE */



 /* USER CODE BEGIN 3 */

#if true

	uint32_t loop_start = __HAL_TIM_GET_COUNTER(&htim5);



	HAL_ADC_Start(&hadc3);

	if (HAL_ADC_PollForConversion(&hadc3, 1) == HAL_OK) {

	pot.raw = (uint16_t)HAL_ADC_GetValue(&hadc3);

	g_pot_live = pot.raw;

	}

	HAL_ADC_Stop(&hadc3);



	// real dt from the hardware timer

	dt = (prev_time == 0) ? 0.002f : (float)(loop_start - prev_time) * 1e-6f;

	if (dt > 0.02f || dt < 0.0005f) dt = 0.002f;

	prev_time = loop_start;



 // odometry from the four wheel pots

	static uint16_t prev_fl = 0, prev_fr = 0, prev_bl = 0, prev_br = 0;



	uint16_t raw_fl = Read_ADC_Channel(&hadc1, ADC_CHANNEL_10);

	uint16_t raw_fr = Read_ADC_Channel(&hadc1, ADC_CHANNEL_11);

	uint16_t raw_bl = Read_ADC_Channel(&hadc1, ADC_CHANNEL_12);

	uint16_t raw_br = Read_ADC_Channel(&hadc1, ADC_CHANNEL_13);



	// handle 360 deg wrap

	int32_t d_fl = raw_fl - prev_fl; if (d_fl > 2048) d_fl -= 4096; else if (d_fl < -2048) d_fl += 4096;

	int32_t d_fr = raw_fr - prev_fr; if (d_fr > 2048) d_fr -= 4096; else if (d_fr < -2048) d_fr += 4096;

	int32_t d_bl = raw_bl - prev_bl; if (d_bl > 2048) d_bl -= 4096; else if (d_bl < -2048) d_bl += 4096;

	int32_t d_br = raw_br - prev_br; if (d_br > 2048) d_br -= 4096; else if (d_br < -2048) d_br += 4096;



	prev_fl = raw_fl; prev_fr = raw_fr; prev_bl = raw_bl; prev_br = raw_br;



	float theta_raw = (((float)pot.raw - (float)pot.center_value) * POT_RAD_PER_COUNT) + theta_offset;

	theta = theta_raw;



	// derivative on measurement, low-pass filtered

	#define THETA_DOT_ALPHA 0.15f // lower = more filtering

	{

	static float prev_theta_for_dot = 0.0f;

	static float theta_dot_filt = 0.0f;



	float theta_dot_raw = (theta - prev_theta_for_dot) / dt;

	prev_theta_for_dot = theta;



	theta_dot_filt = (THETA_DOT_ALPHA * theta_dot_raw) + ((1.0f - THETA_DOT_ALPHA) * theta_dot_filt);

	theta_dot = theta_dot_filt;

	}



	// PID control

	// 2-sample sliding window on the integral so it can't wind up

	{

	static float prev_theta_dt = 0.0f;

	float cur_theta_dt = theta * dt;

	balance_integral = cur_theta_dt + prev_theta_dt;

	prev_theta_dt = cur_theta_dt;

	}



	motor_effort = (90.0f * theta) // kp

	+ (1.5f * theta_dot) // kd

	+ (64.0f * balance_integral); // ki



	// kill motors if we fall past 25 deg

	if (fabsf(theta) > (25.0f * PI_OVER_180)) {

	system_state = 0;

	}



	// round-robin ultrasonic trigger

	sensor_timer++;



	// front + back pair

	if (sensor_timer == 1) {

	HAL_GPIO_WritePin(frontSensor.TriggerPort, frontSensor.TriggerPin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(backSensor.TriggerPort, backSensor.TriggerPin, GPIO_PIN_SET);

	} else if (sensor_timer == 2) {

	HAL_GPIO_WritePin(frontSensor.TriggerPort, frontSensor.TriggerPin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(backSensor.TriggerPort, backSensor.TriggerPin, GPIO_PIN_RESET);

	}

	// left + right pair

	else if (sensor_timer == 21) {

	HAL_GPIO_WritePin(leftSensor.TriggerPort, leftSensor.TriggerPin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(rightSensor.TriggerPort, rightSensor.TriggerPin, GPIO_PIN_SET);

	} else if (sensor_timer == 22) {

	HAL_GPIO_WritePin(leftSensor.TriggerPort, leftSensor.TriggerPin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(rightSensor.TriggerPort, rightSensor.TriggerPin, GPIO_PIN_RESET);

	}

	// restart every 40ms

	else if (sensor_timer >= 40) {

	sensor_timer = 0;

	}



	// navigation + turning

	float turn_effort = 0.0f;

	if (current_robot_mode > 0) {

	Nav_UpdateStrategy(&my_nav, frontSensor.distance, leftSensor.distance, rightSensor.distance, current_robot_mode);

	Read_Gyro(&my_imu, &hi2c2);

	float current_gyro_z = my_imu.gyro.dps_z - my_imu.gyro.bias_z;

	turn_effort = Nav_UpdateSteering(&my_nav, current_gyro_z, dt);

	}



	// mode state machine

	float final_motor_effort = 0.0f;

	float final_turn_effort = 0.0f;



	switch (current_robot_mode) {

	case 1: // balance only

	final_motor_effort = motor_effort;

	final_turn_effort = 0.0f;

	break;



	case 2: // full nav with rod

	final_motor_effort = motor_effort;

	final_turn_effort = turn_effort;

	break;



	case 3: // drive only, no rod

	final_motor_effort = my_nav.target_forward_effort;

	final_turn_effort = turn_effort;

	balance_integral = 0.0f;

	break;

	}





	// differential drive mix

	float final_l_effort = final_motor_effort - final_turn_effort;

	float final_r_effort = final_motor_effort + final_turn_effort;



	// deadzone compensation per track

	float abs_eff_l = fabsf(final_l_effort);

	float abs_eff_r = fabsf(final_r_effort);

	int left_speed = 0, right_speed = 0;



	if (abs_eff_l >= 0.003f) {

	left_speed = (int)(abs_eff_l * PWM_SCALE) + PWM_DEADZONE;

	}

	if (abs_eff_r >= 0.003f) {

	right_speed = (int)(abs_eff_r * PWM_SCALE) + PWM_DEADZONE;

	}



	if (left_speed > MAX_SPEED) left_speed = MAX_SPEED;

	if (right_speed > MAX_SPEED) right_speed = MAX_SPEED;



	// zero PWM on disarm so ESP32 sees a clean cutoff in telemetry

	if (system_state != 1) {

	left_speed = 0;

	right_speed = 0;

	}



	GPIO_PinState left_dir = (final_l_effort > 0) ? GPIO_PIN_RESET : GPIO_PIN_SET;

	GPIO_PinState right_dir = (final_r_effort > 0) ? GPIO_PIN_RESET : GPIO_PIN_SET;



	// pack telemetry

	tx_packet.pwm_fl = (float)(left_dir == GPIO_PIN_RESET ? left_speed : -left_speed);

	tx_packet.pwm_bl = tx_packet.pwm_fl;

	tx_packet.pwm_fr = (float)(right_dir == GPIO_PIN_RESET ? right_speed : -right_speed);

	tx_packet.pwm_br = tx_packet.pwm_fr;



	tx_packet.raw_pot = (float)pot.raw;



	// pot resistance from raw reading (10k pot)

	const float R_TOTAL = 10000.0f;

	tx_packet.pot_ohms = R_TOTAL * ((float)pot.raw / 4095.0f);



	// e-stop gate

	if (system_state == 1) {

	setSpeed(&FLM, left_speed, left_dir);

	setSpeed(&FRM, right_speed, right_dir);

	setSpeed(&BLM, left_speed, left_dir);

	setSpeed(&BRM, right_speed, right_dir);

	} else {

	setSpeed(&FLM, 0, GPIO_PIN_RESET);

	setSpeed(&FRM, 0, GPIO_PIN_RESET);

	setSpeed(&BLM, 0, GPIO_PIN_RESET);

	setSpeed(&BRM, 0, GPIO_PIN_RESET);

	balance_integral = 0.0f;

	}



	// ESP32 telemetry at 50Hz (every 20 control loops)

	if (++telemetry_counter >= 20) {

 telemetry_counter = 0;

 if (spi_in_progress == 0) {

 spi_fail_count = 0;

 tx_packet.pitch_raw = theta_raw * RAD_TO_DEG;

 tx_packet.pitch_filtered = theta * RAD_TO_DEG;

 tx_packet.angular_velocity = theta_dot * RAD_TO_DEG;

 tx_packet.angular_velocity = theta_dot;



 tx_packet.dist_front = frontSensor.distance;

 tx_packet.dist_back = backSensor.distance;

	tx_packet.dist_left = leftSensor.distance;

	tx_packet.dist_right = rightSensor.distance;

	tx_packet.system_state = system_state;



	uint32_t work_time_us = __HAL_TIM_GET_COUNTER(&htim5) - loop_start;

	tx_packet.cpu_load = ((float)work_time_us / 1000.0f) * 100.0f;



	memset(spi_rx_raw_buffer, 0, sizeof(spi_rx_raw_buffer));

 spi_in_progress = 1;

 HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

 for (volatile int i = 0; i < 300; i++); // ESP32 sync delay

 g_spi_tx_attempts++;



 HAL_SPI_TransmitReceive_DMA(&hspi1,

 (uint8_t *)&tx_packet,

	spi_rx_raw_buffer,

 sizeof(TelemetryPacket));

 } else {

 if (++spi_fail_count >= 5) {

 SPI1_Recover();

 spi_in_progress = 0;

 spi_fail_count = 0;

 }

 }

	}



	while ((__HAL_TIM_GET_COUNTER(&htim5) - loop_start) < LOOP_PERIOD_US);

#endif

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



 /** Configure LSE Drive Capability

 */

 HAL_PWR_EnableBkUpAccess();



 /** Configure the main internal regulator output voltage

 */

 __HAL_RCC_PWR_CLK_ENABLE();

 __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);



 /** Initializes the RCC Oscillators according to the specified parameters

 * in the RCC_OscInitTypeDef structure.

 */

 RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;

 RCC_OscInitStruct.HSEState = RCC_HSE_ON;

 RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;

 RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

 RCC_OscInitStruct.PLL.PLLM = 4;

 RCC_OscInitStruct.PLL.PLLN = 168;

 RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;

 RCC_OscInitStruct.PLL.PLLQ = 7;

 RCC_OscInitStruct.PLL.PLLR = 2;

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

 * @brief ADC1 Initialization Function

 * @param None

 * @retval None

 */

static void MX_ADC1_Init(void)

{



 /* USER CODE BEGIN ADC1_Init 0 */



 /* USER CODE END ADC1_Init 0 */



 ADC_ChannelConfTypeDef sConfig = {0};



 /* USER CODE BEGIN ADC1_Init 1 */



 /* USER CODE END ADC1_Init 1 */



 /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)

 */

 hadc1.Instance = ADC1;

 hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;

 hadc1.Init.Resolution = ADC_RESOLUTION_12B;

 hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;

 hadc1.Init.ContinuousConvMode = DISABLE;

 hadc1.Init.DiscontinuousConvMode = DISABLE;

 hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;

 hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;

 hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;

 hadc1.Init.NbrOfConversion = 1;

 hadc1.Init.DMAContinuousRequests = DISABLE;

 hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

 if (HAL_ADC_Init(&hadc1) != HAL_OK)

 {

 Error_Handler();

 }



 /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.

 */

 sConfig.Channel = ADC_CHANNEL_10;

 sConfig.Rank = ADC_REGULAR_RANK_1;

 sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

 if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)

 {

 Error_Handler();

 }

 /* USER CODE BEGIN ADC1_Init 2 */



 /* USER CODE END ADC1_Init 2 */



}



/**

 * @brief ADC3 Initialization Function

 * @param None

 * @retval None

 */

static void MX_ADC3_Init(void)

{



 /* USER CODE BEGIN ADC3_Init 0 */



 /* USER CODE END ADC3_Init 0 */



 ADC_ChannelConfTypeDef sConfig = {0};



 /* USER CODE BEGIN ADC3_Init 1 */



 /* USER CODE END ADC3_Init 1 */



 /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)

 */

 hadc3.Instance = ADC3;

 hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;

 hadc3.Init.Resolution = ADC_RESOLUTION_12B;

 hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;

 hadc3.Init.ContinuousConvMode = DISABLE;

 hadc3.Init.DiscontinuousConvMode = DISABLE;

 hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;

 hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;

 hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;

 hadc3.Init.NbrOfConversion = 1;

 hadc3.Init.DMAContinuousRequests = DISABLE;

 hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

 if (HAL_ADC_Init(&hadc3) != HAL_OK)

 {

 Error_Handler();

 }



 /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.

 */

 sConfig.Channel = ADC_CHANNEL_9;

 sConfig.Rank = ADC_REGULAR_RANK_1;

 sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

 if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)

 {

 Error_Handler();

 }

 /* USER CODE BEGIN ADC3_Init 2 */



 /* USER CODE END ADC3_Init 2 */



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

 hi2c2.Init.Timing = 0x0040154A;

 hi2c2.Init.OwnAddress1 = 0;

 hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;

 hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;

 hi2c2.Init.OwnAddress2 = 0;

 hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;

 hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;

 hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

 if (HAL_I2C_Init(&hi2c2) != HAL_OK)

 {

 Error_Handler();

 }



 /** Configure Analogue filter

 */

 if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)

 {

 Error_Handler();

 }



 /** Configure Digital filter

 */

 if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)

 {

 Error_Handler();

 }

 /* USER CODE BEGIN I2C2_Init 2 */



 /* USER CODE END I2C2_Init 2 */



}



/**

 * @brief SPI1 Initialization Function

 * @param None

 * @retval None

 */

static void MX_SPI1_Init(void)

{



 /* USER CODE BEGIN SPI1_Init 0 */



 /* USER CODE END SPI1_Init 0 */



 /* USER CODE BEGIN SPI1_Init 1 */



 /* USER CODE END SPI1_Init 1 */

 /* SPI1 parameter configuration*/

 hspi1.Instance = SPI1;

 hspi1.Init.Mode = SPI_MODE_MASTER;

 hspi1.Init.Direction = SPI_DIRECTION_2LINES;

 hspi1.Init.DataSize = SPI_DATASIZE_8BIT;

 hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;

 hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;

 hspi1.Init.NSS = SPI_NSS_SOFT;

 hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;

 hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;

 hspi1.Init.TIMode = SPI_TIMODE_DISABLE;

 hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;

 hspi1.Init.CRCPolynomial = 7;

 hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;

 hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;

 if (HAL_SPI_Init(&hspi1) != HAL_OK)

 {

 Error_Handler();

 }

 /* USER CODE BEGIN SPI1_Init 2 */



 /* USER CODE END SPI1_Init 2 */



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

 htim1.Init.Prescaler = 8;

 htim1.Init.CounterMode = TIM_COUNTERMODE_UP;

 htim1.Init.Period = 1023;

 htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

 htim1.Init.RepetitionCounter = 0;

 htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

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

 sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;

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

 if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)

 {

 Error_Handler();

 }

 sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;

 sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;

 sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;

 sBreakDeadTimeConfig.DeadTime = 0;

 sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;

 sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;

 sBreakDeadTimeConfig.BreakFilter = 0;

 sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;

 sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;

 sBreakDeadTimeConfig.Break2Filter = 0;

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

 * @brief TIM5 Initialization Function

 * @param None

 * @retval None

 */

static void MX_TIM5_Init(void)

{



 /* USER CODE BEGIN TIM5_Init 0 */



 /* USER CODE END TIM5_Init 0 */



 TIM_ClockConfigTypeDef sClockSourceConfig = {0};

 TIM_MasterConfigTypeDef sMasterConfig = {0};



 /* USER CODE BEGIN TIM5_Init 1 */



 /* USER CODE END TIM5_Init 1 */

 htim5.Instance = TIM5;

 htim5.Init.Prescaler = 83;

 htim5.Init.CounterMode = TIM_COUNTERMODE_UP;

 htim5.Init.Period = 4294967295;

 htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

 htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

 if (HAL_TIM_Base_Init(&htim5) != HAL_OK)

 {

 Error_Handler();

 }

 sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

 if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)

 {

 Error_Handler();

 }

 sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;

 sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

 if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)

 {

 Error_Handler();

 }

 /* USER CODE BEGIN TIM5_Init 2 */



 /* USER CODE END TIM5_Init 2 */



}



/**

 * @brief TIM14 Initialization Function

 * @param None

 * @retval None

 */

static void MX_TIM14_Init(void)

{



 /* USER CODE BEGIN TIM14_Init 0 */



 /* USER CODE END TIM14_Init 0 */



 /* USER CODE BEGIN TIM14_Init 1 */



 /* USER CODE END TIM14_Init 1 */

 htim14.Instance = TIM14;

 htim14.Init.Prescaler = 83;

 htim14.Init.CounterMode = TIM_COUNTERMODE_UP;

 htim14.Init.Period = 60000-1;

 htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

 htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

 if (HAL_TIM_Base_Init(&htim14) != HAL_OK)

 {

 Error_Handler();

 }

 /* USER CODE BEGIN TIM14_Init 2 */



 /* USER CODE END TIM14_Init 2 */



}



/**

 * Enable DMA controller clock

 */

static void MX_DMA_Init(void)

{



 /* DMA controller clock enable */

 __HAL_RCC_DMA2_CLK_ENABLE();



 /* DMA interrupt init */

 /* DMA2_Stream0_IRQn interrupt configuration */

 HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);

 HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

 /* DMA2_Stream3_IRQn interrupt configuration */

 HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);

 HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);



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

 __HAL_RCC_GPIOE_CLK_ENABLE();

 __HAL_RCC_GPIOC_CLK_ENABLE();

 __HAL_RCC_GPIOF_CLK_ENABLE();

 __HAL_RCC_GPIOH_CLK_ENABLE();

 __HAL_RCC_GPIOA_CLK_ENABLE();

 __HAL_RCC_GPIOB_CLK_ENABLE();

 __HAL_RCC_GPIOG_CLK_ENABLE();

 __HAL_RCC_GPIOD_CLK_ENABLE();



 /*Configure GPIO pin Output Level */

 HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);



 /*Configure GPIO pin Output Level */

 HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_SET);



 /*Configure GPIO pin Output Level */

 HAL_GPIO_WritePin(B_TRIG_GPIO_Port, B_TRIG_Pin, GPIO_PIN_RESET);



 /*Configure GPIO pin Output Level */

 HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);



 /*Configure GPIO pin Output Level */

 HAL_GPIO_WritePin(GPIOC, F_TRIG_Pin|L_TRIG_Pin|R_TRIG_Pin, GPIO_PIN_RESET);



 /*Configure GPIO pins : PE2 PE3 PE4 PE5

 PE6 PE7 PE8 PE10

 PE12 PE15 PE0 PE1 */

 GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5

 |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_10

 |GPIO_PIN_12|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1;

 GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;

 GPIO_InitStruct.Pull = GPIO_NOPULL;

 HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);



 /*Configure GPIO pins : PC13 PC4 PC5 PC9

 PC10 PC11 PC12 */

 GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_9

 |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;

 GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;

 GPIO_InitStruct.Pull = GPIO_NOPULL;

 HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);



 /*Configure GPIO pins : PF0 PF1 PF2 PF4

 PF5 PF6 PF7 PF8

 PF9 PF10 PF11 PF12 */

 GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4

 |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8

 |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;

 GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;

 GPIO_InitStruct.Pull = GPIO_NOPULL;

 HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);



 /*Configure GPIO pins : F_ECHO_Pin PA3 R_ECHO_Pin */

 GPIO_InitStruct.Pin = F_ECHO_Pin|GPIO_PIN_3|R_ECHO_Pin;

 GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;

 GPIO_InitStruct.Pull = GPIO_NOPULL;

 HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);



 /*Configure GPIO pins : PA2 PA4 PA7 PA8

 PA9 PA11 PA12 PA15 */

 GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_7|GPIO_PIN_8

 |GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;

 GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;

 GPIO_InitStruct.Pull = GPIO_NOPULL;

 HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);



 /*Configure GPIO pins : PB0 PB2 PB12 PB13

 PB14 PB15 PB4 PB6

 PB7 PB8 PB9 */

 GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_13

 |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_4|GPIO_PIN_6

 |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;

 GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;

 GPIO_InitStruct.Pull = GPIO_NOPULL;

 HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);



 /*Configure GPIO pin : B_ECHO_Pin */

 GPIO_InitStruct.Pin = B_ECHO_Pin;

 GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;

 GPIO_InitStruct.Pull = GPIO_NOPULL;

 HAL_GPIO_Init(B_ECHO_GPIO_Port, &GPIO_InitStruct);



 /*Configure GPIO pins : PF13 PF14 PF15 */

 GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;

 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

 GPIO_InitStruct.Pull = GPIO_NOPULL;

 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

 HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);



 /*Configure GPIO pin : PG0 */

 GPIO_InitStruct.Pin = GPIO_PIN_0;

 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

 GPIO_InitStruct.Pull = GPIO_NOPULL;

 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

 HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);



 /*Configure GPIO pin : B_TRIG_Pin */

 GPIO_InitStruct.Pin = B_TRIG_Pin;

 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

 GPIO_InitStruct.Pull = GPIO_NOPULL;

 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

 HAL_GPIO_Init(B_TRIG_GPIO_Port, &GPIO_InitStruct);



 /*Configure GPIO pins : PD8 PD9 PD10 PD11

 PD12 PD13 PD15 PD0

 PD1 PD2 PD3 PD4

 PD5 PD6 PD7 */

 GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11

 |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15|GPIO_PIN_0

 |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4

 |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;

 GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;

 GPIO_InitStruct.Pull = GPIO_NOPULL;

 HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);



 /*Configure GPIO pin : SPI1_CS_Pin */

 GPIO_InitStruct.Pin = SPI1_CS_Pin;

 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

 GPIO_InitStruct.Pull = GPIO_NOPULL;

 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

 HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);



 /*Configure GPIO pins : PG2 PG3 PG4 PG5

 PG6 PG7 PG8 PG10

 PG11 PG12 PG13 PG14

 PG15 */

 GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5

 |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_10

 |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14

 |GPIO_PIN_15;

 GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;

 GPIO_InitStruct.Pull = GPIO_NOPULL;

 HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);



 /*Configure GPIO pins : F_TRIG_Pin L_TRIG_Pin R_TRIG_Pin */

 GPIO_InitStruct.Pin = F_TRIG_Pin|L_TRIG_Pin|R_TRIG_Pin;

 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

 GPIO_InitStruct.Pull = GPIO_NOPULL;

 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

 HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);



 /*Configure GPIO pin : L_ECHO_Pin */

 GPIO_InitStruct.Pin = L_ECHO_Pin;

 GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;

 GPIO_InitStruct.Pull = GPIO_NOPULL;

 HAL_GPIO_Init(L_ECHO_GPIO_Port, &GPIO_InitStruct);



 /* EXTI interrupt init*/

 HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);

 HAL_NVIC_EnableIRQ(EXTI0_IRQn);



 HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);

 HAL_NVIC_EnableIRQ(EXTI1_IRQn);



 HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);

 HAL_NVIC_EnableIRQ(EXTI3_IRQn);



 HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);

 HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);



 HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);

 HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);



 /* USER CODE BEGIN MX_GPIO_Init_2 */



 /* USER CODE END MX_GPIO_Init_2 */

}



/* USER CODE BEGIN 4 */

#ifdef __cplusplus

extern "C" {

#endif



void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {

 if (hspi->Instance == SPI1) {

 HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

 spi_in_progress = 0;



 memcpy(&rx_packet, spi_rx_raw_buffer, sizeof(CommandPacket));



 if (rx_packet.command == 'A') {

 system_state = 1; // arm

 } else if (rx_packet.command == 'E') {

 system_state = 0; // e-stop

 } else if (rx_packet.command == 'M') {

 current_robot_mode = rx_packet.mode;

 } else if (rx_packet.command == 'T') {

 tune_kp = rx_packet.kp;

 tune_ki = rx_packet.ki;

 tune_kd = rx_packet.kd;

 }

 }

}



void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {

 if (hspi->Instance == SPI1) {

 // drop CS high and release the lock so main can retry

 HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

 spi_in_progress = 0;

 }

}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)

{

 // front sensor

 if (GPIO_Pin == frontSensor.EchoPin){

 if (HAL_GPIO_ReadPin(frontSensor.EchoPort, frontSensor.EchoPin) == GPIO_PIN_SET){

 frontSensor.echo_start = __HAL_TIM_GET_COUNTER(frontSensor.timer);

 } else {

 frontSensor.echo_end = __HAL_TIM_GET_COUNTER(frontSensor.timer);

 uint32_t pulse = (frontSensor.echo_end >= frontSensor.echo_start) ?

 (frontSensor.echo_end - frontSensor.echo_start) :

 ((0xFFFFFFFF - frontSensor.echo_start) + frontSensor.echo_end);



 float raw_dist = (float)pulse / 58.0f;

 if (raw_dist < 400.0f && raw_dist > 2.0f) {

 frontSensor.history[frontSensor.history_idx] = raw_dist;

 frontSensor.history_idx = (frontSensor.history_idx + 1) % MEDIAN_WINDOW;

 frontSensor.distance = get_median(frontSensor.history, MEDIAN_WINDOW);

 }

 }

 }



 // back sensor

 if (GPIO_Pin == backSensor.EchoPin){

 if (HAL_GPIO_ReadPin(backSensor.EchoPort, backSensor.EchoPin) == GPIO_PIN_SET){

 backSensor.echo_start = __HAL_TIM_GET_COUNTER(backSensor.timer);

 } else {

 backSensor.echo_end = __HAL_TIM_GET_COUNTER(backSensor.timer);

 uint32_t pulse = (backSensor.echo_end >= backSensor.echo_start) ?

 (backSensor.echo_end - backSensor.echo_start) :

 ((0xFFFFFFFF - backSensor.echo_start) + backSensor.echo_end);



 float raw_dist = (float)pulse / 58.0f;

 if (raw_dist < 400.0f && raw_dist > 2.0f) {

 backSensor.history[backSensor.history_idx] = raw_dist;

 backSensor.history_idx = (backSensor.history_idx + 1) % MEDIAN_WINDOW;

 backSensor.distance = get_median(backSensor.history, MEDIAN_WINDOW);

 }

 }

 }



 // left sensor

 if (GPIO_Pin == leftSensor.EchoPin){

 if (HAL_GPIO_ReadPin(leftSensor.EchoPort, leftSensor.EchoPin) == GPIO_PIN_SET){

 leftSensor.echo_start = __HAL_TIM_GET_COUNTER(leftSensor.timer);

 } else {

 leftSensor.echo_end = __HAL_TIM_GET_COUNTER(leftSensor.timer);

 uint32_t pulse = (leftSensor.echo_end >= leftSensor.echo_start) ?

 (leftSensor.echo_end - leftSensor.echo_start) :

 ((0xFFFFFFFF - leftSensor.echo_start) + leftSensor.echo_end);



 float raw_dist = (float)pulse / 58.0f;

 if (raw_dist < 400.0f && raw_dist > 2.0f) {

 leftSensor.history[leftSensor.history_idx] = raw_dist;

 leftSensor.history_idx = (leftSensor.history_idx + 1) % MEDIAN_WINDOW;

 leftSensor.distance = get_median(leftSensor.history, MEDIAN_WINDOW);

 }

 }

 }



 // right sensor

 if (GPIO_Pin == rightSensor.EchoPin){

 if (HAL_GPIO_ReadPin(rightSensor.EchoPort, rightSensor.EchoPin) == GPIO_PIN_SET){

 rightSensor.echo_start = __HAL_TIM_GET_COUNTER(rightSensor.timer);

 } else {

 rightSensor.echo_end = __HAL_TIM_GET_COUNTER(rightSensor.timer);

 uint32_t pulse = (rightSensor.echo_end >= rightSensor.echo_start) ?

 (rightSensor.echo_end - rightSensor.echo_start) :

 ((0xFFFFFFFF - rightSensor.echo_start) + rightSensor.echo_end);



 float raw_dist = (float)pulse / 58.0f;

 if (raw_dist < 400.0f && raw_dist > 2.0f) {

 rightSensor.history[rightSensor.history_idx] = raw_dist;

 rightSensor.history_idx = (rightSensor.history_idx + 1) % MEDIAN_WINDOW;

 rightSensor.distance = get_median(rightSensor.history, MEDIAN_WINDOW);

 }

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



 /* MPU Configuration */



void MPU_Config(void)

{

 MPU_Region_InitTypeDef MPU_InitStruct = {0};



 /* Disables the MPU */

 HAL_MPU_Disable();



 /** Initializes and configures the Region and the memory to be protected

 */

 MPU_InitStruct.Enable = MPU_REGION_ENABLE;

 MPU_InitStruct.Number = MPU_REGION_NUMBER0;

 MPU_InitStruct.BaseAddress = 0x0;

 MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;

 MPU_InitStruct.SubRegionDisable = 0x87;

 MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;

 MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;

 MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;

 MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;

 MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;

 MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;



 HAL_MPU_ConfigRegion(&MPU_InitStruct);

 /* Enables the MPU */

 HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);



}



/**

 * @brief This function is executed in case of error occurrence.

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

 * @brief Reports the name of the source file and the source line number

 * where the assert_param error has occurred.

 * @param file: pointer to the source file name

 * @param line: assert_param error line source number

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
