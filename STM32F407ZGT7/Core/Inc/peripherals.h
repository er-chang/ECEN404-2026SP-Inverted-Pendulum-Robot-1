/*Private Typedefs*/
// Ultrasonic Sensors
typedef struct Ultrasonic{
	GPIO_TypeDef* 		TriggerPort;
	GPIO_TypeDef* 		EchoPort;

	uint16_t			TriggerPin;
	uint16_t			EchoPin;

	TIM_HandleTypeDef* 	timer;

	volatile float 		echo_start;
	volatile float		echo_end;
	float 				distance;
} Ultrasonic;

// Motors
typedef struct Motor{
	__IO uint32_t* PWM;
} Motor;


/*MICROSECOND DELAY*/
void Delay_us(uint16_t us, TIM_HandleTypeDef* tim){
	__HAL_TIM_SET_COUNTER(tim, 0);
	while(__HAL_TIM_GET_COUNTER(tim) < us);
}


/*SONAR DISTANCE*/
float getSonarDistance(Ultrasonic sensor){


	// A. Trigger the pulse
	HAL_GPIO_WritePin(sensor.TriggerPort, sensor.TriggerPin, GPIO_PIN_SET);
	Delay_us(10, sensor.timer); // 10us pulse
	HAL_GPIO_WritePin(sensor.TriggerPort, sensor.TriggerPin, GPIO_PIN_RESET);

	// B. Wait for measurement
	HAL_Delay(30); // Max range ~5m takes 30ms

	// C. Calculate
	if (sensor.echo_end > sensor.echo_start) {
		uint32_t echo_width = sensor.echo_end - sensor.echo_start;
		return (float)echo_width * 0.01715f;
	}
	return 0.0f; // Error / Timeout
}


/*MOTOR SPEED*/
void setSpeed(Motor motor, uint32_t speed){
	*motor.PWM = speed;
}

