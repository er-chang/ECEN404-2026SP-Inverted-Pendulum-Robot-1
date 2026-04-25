#ifndef PERIPHERALS_H

#define PERIPHERALS_H



/*Includes*/

#include "main.h"

#include <stdio.h>

/*End Includes*/





/*Defines*/

#define MEDIAN_WINDOW 5



// Motor Constants

#define MOTOR_MAX_SPEED 256 // Maximum CCR value for PWM channels

#define MOTOR_MIN_SPEED 0 // Minimum CCR value for PWM channels



// IMU Constants

#define IMU_ADDR 0x6A // Memory address of the IMU

#define WHO_AM_I_REG  	0x0F  // Location of Who am I register

#define WHO_AM_I_VALUE 0x6C  // Value that should be in the who am i register

#define CTRL1_XL      	0x10  // Location of First Accelerometer control register

#define CTRL2_G			0x11  // Location of First Gyroscope control register

#define OUTX_L_A      	0x28  // Location of Accelerometer output register

#define OUTX_L_G	  	0x22  // Location of Gyroscope output register



// ESP SPI Constants

#define SPI_CS_Pin        GPIO_PIN_14

#define SPI_CS_GPIO_Port  GPIOD

#define SPI_FAIL_REINIT_COUNT   10U



/*End Defines*/





/*Prototypes*/

// --- NEW: Added AS5600 Prototype ---

uint16_t Read_AS5600(I2C_HandleTypeDef* hi2c);

/*End Prototypes*/





/*Typedefs*/



// Potentiometer

typedef struct Potentiometer {

   uint16_t* dma_buffer;    // Pointer to ADC DMA array

    uint16_t  center_value;  // ADC value when rod is vertical

    uint16_t  raw;

    float     deg_per_count; // (e.g., 270.0 / 4095.0)

    float     current_angle; // Calculated angle in degrees

    float     velocity;      // Filtered angular velocity (deg/s)

} Potentiometer;



// Ultrasonic Sensors

typedef struct Ultrasonic{

	GPIO_TypeDef* TriggerPort;

	GPIO_TypeDef* EchoPort;

	uint16_t			TriggerPin;

	uint16_t			EchoPin;

	TIM_HandleTypeDef* timer;

	volatile uint32_t   echo_start;

	volatile uint32_t	echo_end;

	volatile float 		distance;



	// array for median filter, removes outlier data

	volatile float      history[MEDIAN_WINDOW];

	volatile uint8_t    history_idx;

} Ultrasonic;



// Motors

typedef struct Motor{

	__IO uint32_t* PWM;

	GPIO_TypeDef* directionPort;

	uint16_t		directionPin;

	GPIO_PinState	direction {GPIO_PIN_SET};

} Motor;



// Accelerometer

typedef struct Accelerometer{

	volatile float 	g_x {0};

	volatile float 	g_y {0};

	volatile float 	g_z {0};

	uint8_t 		addr {OUTX_L_A};

	uint8_t 		control_addr {CTRL1_XL};

	uint8_t 		control {0x70};  // ODR=833Hz, FS=±2g (was 0x5E: wrong FS and ODR)

	uint8_t 		out[6] {0};

	float 			drift_cancel;

} Accelerometer;



// Gyroscope

typedef struct Gyroscope{

	volatile float 	dps_x {0};

	volatile float 	dps_y {0};

	volatile float 	dps_z {0};

	uint8_t 		addr {OUTX_L_G};

	uint8_t 		control_addr {CTRL2_G};

	uint8_t 		control {0x70};  // ODR=833Hz, FS=±250dps (was 0x5E: ±125dps, gyro 2x too large)

	uint8_t 		out[6];

	float 			drift_cancel;

	float           bias_z {0.0f};  // Gyro -axis bias measured at startup

} Gyroscope;



// IMU

typedef struct IMU{

	uint16_t 			addr {IMU_ADDR};

	uint16_t 			who_am_i_reg {WHO_AM_I_REG};

	uint16_t 			who_am_i_value {WHO_AM_I_VALUE};

	uint8_t				who_am_i_data[1] {0};

	uint8_t				init_buffer[1] {0};

	uint8_t				data[12];

	HAL_StatusTypeDef	status {HAL_OK};

	Accelerometer 		accel;

	Gyroscope			gyro;

} IMU;



// ESP Packets

typedef struct __attribute__((packed)) {

    float pitch_raw;

    float pitch_filtered;

    float angular_velocity;



    // 4 Ultrasonic Sensors

    float dist_front;

    float dist_back;

    float dist_left;

    float dist_right;



    float system_state;



    // 4 Motors

    float pwm_fl;

    float pwm_fr;

    float pwm_bl;

    float pwm_br;



    // Potentiometer

    float raw_pot;

    float pot_ohms;



    // CPU Metrics

    float cpu_load;

} TelemetryPacket;



typedef struct __attribute__((packed)) {

    uint8_t command;

    uint8_t mode;        // mode tracking byte

    uint8_t padding1[2]; // FPU Memory Alignment

    float kp;

    float ki;

    float kd;

    uint8_t padding2[40]; // memory padding increased from 8 to 28 to 40 bytes

} CommandPacket;



/*End Typedefs*/



/*FUNCTIONS*/



/*MICROSECOND DELAY*/

void Delay_us(uint16_t us, TIM_HandleTypeDef* tim){

	uint32_t start = __HAL_TIM_GET_COUNTER(tim);

	while ((__HAL_TIM_GET_COUNTER(tim) - start) < us);

}



/*READ POTENTIOMETER*/

void Read_Pot(Potentiometer* pot, ADC_HandleTypeDef* adc){

	HAL_ADC_Start(adc);

	HAL_ADC_PollForConversion(adc, 1);

	pot->raw = (uint16_t)HAL_ADC_GetValue(adc);

	HAL_ADC_Stop(adc);

}



/*CALIBRATE POTENTIOMETER*/

void Calibrate_Pot(Potentiometer* pot, ADC_HandleTypeDef* adc){

	uint32_t pot_sum = 0;



	for (int i = 0; i < 1000; i++) {

	  HAL_ADC_Start(adc);

	  HAL_ADC_PollForConversion(adc, 1);

	  pot_sum += HAL_ADC_GetValue(adc);

	  HAL_ADC_Stop(adc);

	  HAL_Delay(2);

	}

	pot->center_value = (uint16_t)(pot_sum / 1000);

}



/*SONAR DISTANCE*/

void getSonarDistance(Ultrasonic* sensor){

	sensor->distance = 0.0f;

	if (sensor->echo_end > sensor->echo_start) {

		uint32_t echo_width = sensor->echo_end - sensor->echo_start;

		sensor->distance = (float)echo_width * 0.01715f;

	}

	sensor->echo_start = 0;

	sensor->echo_end   = 0;

	HAL_GPIO_WritePin(sensor->TriggerPort, sensor->TriggerPin, GPIO_PIN_SET);

	Delay_us(10, sensor->timer);

	HAL_GPIO_WritePin(sensor->TriggerPort, sensor->TriggerPin, GPIO_PIN_RESET);

}



/*MOTOR SPEED and DIRECTION*/

void setSpeed(Motor* motor, uint32_t speed, GPIO_PinState direction){

	HAL_GPIO_WritePin(motor->directionPort, motor->directionPin, direction);

	*motor->PWM = speed;

	motor->direction = direction;

}



/*Accelerometer Read Function*/

void Read_Accel(IMU* imu, I2C_HandleTypeDef* i2c)

{

	volatile int16_t x;

	volatile int16_t y;

	volatile int16_t z;

   imu->status = HAL_I2C_Mem_Read(i2c, (imu->addr << 1), imu->accel.addr, 1, imu->accel.out, 6, 1000);

   x = (int16_t)((imu->accel.out[1] << 8) | imu->accel.out[0]);

   y = (int16_t)((imu->accel.out[3] << 8) | imu->accel.out[2]);

   z = (int16_t)((imu->accel.out[5] << 8) | imu->accel.out[4]);

   imu->accel.g_x = (float)x * 0.061f / 1000.0f;

   imu->accel.g_y = (float)y * 0.061f / 1000.0f;

   imu->accel.g_z = (float)(z * 0.061f / 1000.0f);

}



/*Gyroscope Read Function*/

void Read_Gyro(IMU* imu, I2C_HandleTypeDef* i2c)

{

	volatile int16_t x;

	volatile int16_t y;

	volatile int16_t z;

	imu->status = HAL_I2C_Mem_Read(i2c, (imu->addr << 1), imu->gyro.addr, 1, imu->gyro.out, 6, 1000);

   x = (int16_t)((imu->gyro.out[1] << 8) | imu->gyro.out[0]);

   y = (int16_t)((imu->gyro.out[3] << 8) | imu->gyro.out[2]);

   z = (int16_t)((imu->gyro.out[5] << 8) | imu->gyro.out[4]);

   imu->gyro.dps_x = (float)x * 8.75f / 1000.0f;

   imu->gyro.dps_y = (float)y * 8.75f / 1000.0f;

   imu->gyro.dps_z = (float)z * 8.75f / 1000.0f;

}



/*READ IMU - Reads Both Gyro and Accel Values from IMU (BLOCKING)*/

void Read_IMU(IMU* imu, I2C_HandleTypeDef* i2c) {

    imu->status = HAL_I2C_Mem_Read(i2c, (imu->addr << 1), imu->gyro.addr, 1, imu->data, 12, 10);

    imu->gyro.dps_x = (float)((int16_t)((imu->data[1] << 8) | imu->data[0])) * 0.00875f;

    imu->gyro.dps_y = (float)((int16_t)((imu->data[3] << 8) | imu->data[2])) * 0.00875f;

    imu->gyro.dps_z = (float)((int16_t)((imu->data[5] << 8) | imu->data[4])) * 0.00875f;

    imu->accel.g_x = (float)((int16_t)((imu->data[7] << 8) | imu->data[6])) * 0.000061f;

    imu->accel.g_y = (float)((int16_t)((imu->data[9] << 8) | imu->data[8])) * 0.000061f;

    imu->accel.g_z = (float)((int16_t)((imu->data[11] << 8) | imu->data[10])) * 0.000061f;

}



/*START IMU DMA - Kicks off a non-blocking 12-byte I2C DMA read into imu->data*/

HAL_StatusTypeDef Start_IMU_DMA(IMU* imu, I2C_HandleTypeDef* i2c) {

    return HAL_I2C_Mem_Read_DMA(i2c, (imu->addr << 1), imu->gyro.addr, 1, imu->data, 12);

}



/*PROCESS IMU DATA - Converts raw bytes in imu->data to float values (call after DMA completes)*/

void Process_IMU_Data(IMU* imu) {

    imu->gyro.dps_x = (float)((int16_t)((imu->data[1] << 8) | imu->data[0])) * 0.00875f;

    imu->gyro.dps_y = (float)((int16_t)((imu->data[3] << 8) | imu->data[2])) * 0.00875f;

    imu->gyro.dps_z = (float)((int16_t)((imu->data[5] << 8) | imu->data[4])) * 0.00875f;

    imu->accel.g_x = (float)((int16_t)((imu->data[7] << 8) | imu->data[6])) * 0.000061f;

    imu->accel.g_y = (float)((int16_t)((imu->data[9] << 8) | imu->data[8])) * 0.000061f;

    imu->accel.g_z = (float)((int16_t)((imu->data[11] << 8) | imu->data[10])) * 0.000061f;

}



/*GYRO CALIBRATION — call once at startup, robot must be stationary*/

void Calibrate_Gyro(IMU* imu, I2C_HandleTypeDef* i2c, uint16_t samples) {

    float sum_z = 0.0f;

    uint16_t good = 0;

    for (uint16_t i = 0; i < samples; i++) {

        Read_IMU(imu, i2c);

        if (imu->status == HAL_OK) {

            sum_z += imu->gyro.dps_z;

            good++;

        }

        HAL_Delay(2);

    }

    if (good > 0) {

        imu->gyro.bias_z = sum_z / (float)good;

    }

}



/*IMU Initialization Function*/

void IMU_Init(IMU* imu, I2C_HandleTypeDef* i2c){

	 /*-[ IMU channel check ]-*/

	  	printf("---Checking IMU Readiness---\n\n");

	  	imu->status = HAL_I2C_IsDeviceReady(i2c, (imu->addr << 1), 3, 5);

	  	switch(imu->status){

	  	case HAL_BUSY:

	  		printf("IMU is BUSY\n\n");

	  		break;

	  	case HAL_TIMEOUT:

	  		printf("IMU read TIMEDOUT\n\n");

	  		break;

	  	case HAL_ERROR:

	  		printf("IMU read had an ERROR\n\n");

	  		break;

	  	case HAL_OK:

	  		printf("IMU is OKAY\n\n");

	  		break;

	  	default:

	  		printf("SOMETHING WENT WRONG\n\n");

	  		break;

	  	};

	    printf("---IMU Readiness Checking Done---\n\n\n");

	     /*--[ Checking Done ]--*/

	    /* Who am I Test*/

	    printf("---Starting WHO AM I test---\n\n");

	    imu->status = HAL_I2C_Mem_Read(i2c, (imu->addr << 1), imu->who_am_i_reg, 1, imu->who_am_i_data, 1, 1000);

	    if (imu->who_am_i_data[0] != imu->who_am_i_value){

	    	printf("SOMETHING IS WRONG\n");

	    	printf("Who am I Value = %x \n\n", imu->who_am_i_data[0]);

	    }

	    else{

	    	printf("Who am I value is GOOD\n\n");

	    }

	    if (imu->status != HAL_OK){

	    	printf("SOMETHING IS WRONG WITH STATUS\n\n");

	    }

	    printf("---WHO AM I test is DONE---\n\n\n");

		/* End Who am I Test*/

		/* Accelerometer Initialization: ODR = 1.66 kHz, FS = +- 2g*/

	    printf("---Initializing Accelerometer---\n\n");

		HAL_I2C_Mem_Write(i2c, (imu->addr << 1), imu->accel.control_addr, 1, &(imu->accel.control), 1, 1000);

	    imu->status = HAL_I2C_Mem_Read(i2c, (imu->addr << 1), imu->accel.control_addr, 1, imu->init_buffer, 1, 1000);

	    if (imu->init_buffer[0] != imu->accel.control){

	    	printf("SOMETHING IS WRONG\n");

	    	printf("CTRL1_XL data = %x \n\n", imu->init_buffer[0]);

	    }

	    else{

	    	printf("Accelerometer init. data is GOOD\n\n");

	    }

	    if (imu->status != HAL_OK){

	    	printf("SOMETHING IS WRONG WITH STATUS\n\n");

	    }

	    printf("---Accelerometer Initialization is DONE---\n\n\n");

	    /*End Accelerometer Initialization*/

		/* Gyroscope Initialization: ODR = 1.66 kHz, FS = +- 250 dps*/

	    printf("---Initializing Gyroscope---\n\n");

		HAL_I2C_Mem_Write(i2c, (imu->addr << 1), imu->gyro.control_addr, 1, &(imu->gyro.control), 1, 1000);

	    imu->status = HAL_I2C_Mem_Read(i2c, (imu->addr << 1), imu->gyro.control_addr, 1, imu->init_buffer, 1, 1000);

	    if (imu->init_buffer[0] != imu->gyro.control){

	    	printf("SOMETHING IS WRONG\n");

	    	printf("CTRL2_G data = %x \n\n", imu->init_buffer[0]);

	    }

	    else{

	    	printf("Gyroscope init. data is GOOD\n\n");

	    }

	    if (imu->status != HAL_OK){

	    	printf("SOMETHING IS WRONG WITH STATUS\n\n");

	    }

	    printf("---Gyroscope Initialization is DONE---\n\n\n");

	    /*End Gyroscope Initialization*/

		HAL_Delay(20);

		Calibrate_Gyro(imu, i2c, 50);

}



#endif /* PERIPHERALS_H */
