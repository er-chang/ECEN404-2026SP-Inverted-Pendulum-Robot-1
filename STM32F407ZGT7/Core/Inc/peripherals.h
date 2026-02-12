#include <stdio.h>


void Delay_us(uint16_t us){
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	while(__HAL_TIM_GET_COUNTER(&htim2) < us);
}


void HCSR04_Trigger(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
    Delay_us(10);// ≈ 10µs on most STM32F4
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
}


float getSonarDistance(void){
	HCSR04_Trigger();

	HAL_Delay(60);

	uint32_t echo_width =  echo_end - echo_start;

	return echo_width * 0.01715;
}
