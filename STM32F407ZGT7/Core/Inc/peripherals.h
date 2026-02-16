
volatile float echo_end;
volatile float echo_start;

void Delay_us(uint16_t us, TIM_HandleTypeDef* tim){
	__HAL_TIM_SET_COUNTER(tim, 0);
	while(__HAL_TIM_GET_COUNTER(tim) < us);
}


float getSonarDistance(GPIO_TypeDef* output_port, uint16_t output_pin, GPIO_TypeDef* input_port, uint16_t input_pin, TIM_HandleTypeDef* tim){
	// A. Trigger the pulse
	HAL_GPIO_WritePin(output_port, output_pin, GPIO_PIN_SET);
	Delay_us(10, tim); // 10us pulse
	HAL_GPIO_WritePin(output_port, output_pin, GPIO_PIN_RESET);

	// B. Wait for measurement
	HAL_Delay(60); // Max range ~5m takes 30ms

	// C. Calculate
	if (echo_end > echo_start) {
		uint32_t echo_width = echo_end - echo_start;
		return (float)echo_width * 0.01715f;
	}
	return 0.0f; // Error / Timeout
}

