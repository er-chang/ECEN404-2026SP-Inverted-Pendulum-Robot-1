#ifndef PERIPHERALS_H
#define PERIPHERALS_H

/*Includes*/
#include "main.h"
#include <stdio.h>
/*End Includes*/


/*Defines*/
#define MOTOR_MAX_SPEED 256 // Maximum CCR value for PWM channels
#define MOTOR_MIN_SPEED 0 // Minimum CCR value for PWM channels
#define IMU_ADDR 0x6A // Memory address of the IMU
// Register addresses
#define WHO_AM_I_REG  	0x0F  // Location of Who am I register
#define WHO_AM_I_VALUE 0x6C  // Value that should be in the who am i register
#define CTRL1_XL      	0x10  // Location of First Accelerometer control register
#define CTRL2_G			0x11  // Location of First Gyroscope control register
#define OUTX_L_G	  	0x22  // Location of Gyroscope output register
#define OUTX_L_A      	0x28  // Location of Accelerometer output register
/*End Defines*/


/*Prototypes*/
/*End Prototypes*/


/*Typedefs*/
// Ultrasonic Sensors
typedef struct Ultrasonic{
	GPIO_TypeDef* 		TriggerPort;
	GPIO_TypeDef* 		EchoPort;
	uint16_t			TriggerPin;
	uint16_t			EchoPin;
	TIM_HandleTypeDef* 	timer;
	volatile uint32_t   echo_start;
	volatile uint32_t	echo_end;
	float 				distance;
} Ultrasonic;

// Motors
typedef struct Motor{
	__IO uint32_t* 	PWM;
	GPIO_TypeDef* 	directionPort;
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
	// ┌─────────────────────────────────────────────────────────┐
	// │ CTRL1_XL = 0x70 → ODR=833Hz, FS=±2g, LPF2=off         │
	// │ Sensitivity: 0.061 mg/LSB (used in Process_IMU_Data)    │
	// │                                                         │
	// │ OLD VALUE 0x5E WAS WRONG:                               │
	// │   ODR=208Hz (stale reads), FS=±8g (code assumed ±2g),   │
	// │   LPF2=on (extra latency)                               │
	// └─────────────────────────────────────────────────────────┘
	uint8_t 		control {0x70};
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
	// ┌─────────────────────────────────────────────────────────┐
	// │ CTRL2_G = 0x70 → ODR=833Hz, FS=±250dps, FS_125=off     │
	// │ Sensitivity: 8.75 mdps/LSB (used in Process_IMU_Data)   │
	// │                                                         │
	// │ OLD VALUE 0x5E WAS WRONG:                               │
	// │   ODR=208Hz, FS_125=1 forced ±125dps (4.375 mdps/LSB)  │
	// │   but code used 8.75 mdps/LSB → gyro reads 2x too large│
	// └─────────────────────────────────────────────────────────┘
	uint8_t 		control {0x70};
	uint8_t 		out[6];
	float 			drift_cancel;
	float           bias_y {0.0f};  // Gyro Y-axis bias measured at startup
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
/*End Typedefs*/


/*FUNCTIONS*/
/*MICROSECOND DELAY — does NOT reset the timer so TIM2 can double as a loop clock*/
void Delay_us(uint16_t us, TIM_HandleTypeDef* tim){
	uint32_t start = __HAL_TIM_GET_COUNTER(tim);
	while ((__HAL_TIM_GET_COUNTER(tim) - start) < us);
}


/*SONAR DISTANCE*/
void getSonarDistance(Ultrasonic* sensor){
	// A. Compute distance from the previous echo captured asynchronously by the EXTI ISR
	sensor->distance = 0.0f;
	if (sensor->echo_end > sensor->echo_start) {
		uint32_t echo_width = sensor->echo_end - sensor->echo_start;
		sensor->distance = (float)echo_width * 0.01715f;
	}
	// B. Reset timestamps and fire a new trigger pulse for the next call
	sensor->echo_start = 0;
	sensor->echo_end   = 0;
	HAL_GPIO_WritePin(sensor->TriggerPort, sensor->TriggerPin, GPIO_PIN_SET);
	Delay_us(10, sensor->timer);
	HAL_GPIO_WritePin(sensor->TriggerPort, sensor->TriggerPin, GPIO_PIN_RESET);
}


/*MOTOR SPEED*/
void setSpeed(Motor* motor, uint32_t speed, GPIO_PinState direction){
	HAL_GPIO_WritePin(motor->directionPort, motor->directionPin, direction);
	*motor->PWM = speed;
	motor->direction = direction;
}


/*READ IMU - Reads Both Gyro and Accel in a single 12-byte burst (BLOCKING)
  Register map: 0x22..0x27 = gyro XYZ, 0x28..0x2D = accel XYZ (contiguous, auto-increment) */
void Read_IMU(IMU* imu, I2C_HandleTypeDef* i2c) {
    imu->status = HAL_I2C_Mem_Read(i2c, (imu->addr << 1), imu->gyro.addr, 1, imu->data, 12, 2);

    // --- Process Gyro (First 6 bytes: 0x22-0x27) ---
    // ±250 dps → 8.75 mdps/LSB = 0.00875 dps/LSB
    imu->gyro.dps_x = (float)((int16_t)((imu->data[1] << 8) | imu->data[0])) * 0.00875f;
    imu->gyro.dps_y = (float)((int16_t)((imu->data[3] << 8) | imu->data[2])) * 0.00875f;
    imu->gyro.dps_z = (float)((int16_t)((imu->data[5] << 8) | imu->data[4])) * 0.00875f;

    // --- Process Accel (Next 6 bytes: 0x28-0x2D) ---
    // ±2g → 0.061 mg/LSB = 0.000061 g/LSB
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
    // --- Gyro (First 6 bytes) — ±250 dps, 8.75 mdps/LSB ---
    imu->gyro.dps_x = (float)((int16_t)((imu->data[1] << 8) | imu->data[0])) * 0.00875f;
    imu->gyro.dps_y = (float)((int16_t)((imu->data[3] << 8) | imu->data[2])) * 0.00875f;
    imu->gyro.dps_z = (float)((int16_t)((imu->data[5] << 8) | imu->data[4])) * 0.00875f;

    // --- Accel (Next 6 bytes) — ±2g, 0.061 mg/LSB ---
    imu->accel.g_x = (float)((int16_t)((imu->data[7] << 8) | imu->data[6])) * 0.000061f;
    imu->accel.g_y = (float)((int16_t)((imu->data[9] << 8) | imu->data[8])) * 0.000061f;
    imu->accel.g_z = (float)((int16_t)((imu->data[11] << 8) | imu->data[10])) * 0.000061f;
}


/*GYRO CALIBRATION — call once at startup, robot must be stationary
  Averages N samples to find the zero-rate bias offset */
void Calibrate_Gyro(IMU* imu, I2C_HandleTypeDef* i2c, uint16_t samples) {
    float sum_y = 0.0f;
    uint16_t good = 0;
    for (uint16_t i = 0; i < samples; i++) {
        Read_IMU(imu, i2c);
        if (imu->status == HAL_OK) {
            sum_y += imu->gyro.dps_y;
            good++;
        }
        HAL_Delay(2);  // ~500 Hz sample rate during cal
    }
    if (good > 0) {
        imu->gyro.bias_y = sum_y / (float)good;
    }
    printf("Gyro cal: bias_y = %.4f dps (%d samples)\n", imu->gyro.bias_y, good);
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

		/* Accelerometer Initialization: ODR=833Hz, FS=±2g (CTRL1_XL=0x70) */
	    printf("---Initializing Accelerometer (0x%02X)---\n\n", imu->accel.control);
		HAL_I2C_Mem_Write(i2c, (imu->addr << 1), imu->accel.control_addr, 1, &(imu->accel.control), 1, 1000);
	    imu->status = HAL_I2C_Mem_Read(i2c, (imu->addr << 1), imu->accel.control_addr, 1, imu->init_buffer, 1, 1000);
	    if (imu->init_buffer[0] != imu->accel.control){
	    	printf("SOMETHING IS WRONG\n");
	    	printf("CTRL1_XL data = %x (expected %x)\n\n", imu->init_buffer[0], imu->accel.control);
	    }
	    else{
	    	printf("Accelerometer init. data is GOOD\n\n");
	    }
	    printf("---Accelerometer Initialization is DONE---\n\n\n");
	    /*End Accelerometer Initialization*/

		/* Gyroscope Initialization: ODR=833Hz, FS=±250dps (CTRL2_G=0x70) */
	    printf("---Initializing Gyroscope (0x%02X)---\n\n", imu->gyro.control);
		HAL_I2C_Mem_Write(i2c, (imu->addr << 1), imu->gyro.control_addr, 1, &(imu->gyro.control), 1, 1000);
	    imu->status = HAL_I2C_Mem_Read(i2c, (imu->addr << 1), imu->gyro.control_addr, 1, imu->init_buffer, 1, 1000);
	    if (imu->init_buffer[0] != imu->gyro.control){
	    	printf("SOMETHING IS WRONG\n");
	    	printf("CTRL2_G data = %x (expected %x)\n\n", imu->init_buffer[0], imu->gyro.control);
	    }
	    else{
	    	printf("Gyroscope init. data is GOOD\n\n");
	    }
	    printf("---Gyroscope Initialization is DONE---\n\n\n");
	    /*End Gyroscope Initialization*/

		HAL_Delay(50);  // Let ODR stabilize before calibration
		Calibrate_Gyro(imu, i2c, 200);  // ~400ms, robot must be still
}
/* End IMU INIT FUNCTION*/

#endif /* PERIPHERALS_H */
