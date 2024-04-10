#include "task3.h"


void task3(I2C_HandleTypeDef *hi2c1){

	// Variables
	const uint8_t tmp_address = (0b1001000)<<1;
	const uint8_t tmp_register = 0x00;

	uint8_t buffer[1000]; // Reading 12 bits from temperature register
	buffer[0] = tmp_register; // Read from tmp register.
	int temp_in_celcius = 0;


	// Send read temperature command
	int ret = HAL_I2C_Master_Transmit(hi2c1, tmp_address, buffer, 1, HAL_MAX_DELAY);
	if (ret != HAL_OK ){
		return;
	}

	// Receive raw temperature measurement
	ret = HAL_I2C_Master_Receive(hi2c1, tmp_address, buffer, 2, HAL_MAX_DELAY);
	if (ret != HAL_OK ){
		return;
	}

	// Compute temperature in celcius.
	uint16_t temperature = (((uint16_t)buffer[0])<<4) | (((uint16_t)buffer[1])>>4); // Assume all bits are zero except the ones we occupy for tmp?
	if ((temperature & 0x1000) == 0x1000 ){
		// Temperature is negative
		// First compute the two's complement.
		uint16_t temp_twos_complement = (~temperature) + 1;
		// Use conversion and set to negative
		temp_in_celcius = -((int)temp_twos_complement*0.0625);
	} else{
		// Temperature is positive
		// Using simple conversion model to compute the temperature in celcius from raw temperature data.
		temp_in_celcius = (int)temperature*0.0625;
	}

	if (temp_in_celcius>27){
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12); // Red
	}else if(temp_in_celcius<27){
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14); // Green
	}else if (temp_in_celcius==27){
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13); // Orange
	}

	sprintf((char*) &buffer, "Temperature is %d \r\n", temp_in_celcius); // Store string in buffer
	size_t length = strlen((char*) &buffer); // Extract size of string. Need \r\n to give proper length of string.
	CDC_Transmit_FS(buffer, length); // Transmit temperature data over USB.
}
