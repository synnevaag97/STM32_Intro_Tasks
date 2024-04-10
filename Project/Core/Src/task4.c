#include "task4.h"


void task4_init(SPI_HandleTypeDef *hspi1){
	uint8_t buffer_rx[2];
	uint8_t buffer_tx[2];
	int ret = 0;

	// Enable Acc_x and Acc_y measuring and set frequency to lowest Hz.
	buffer_tx[0] = (write_bit | CTRL_REG4);
	buffer_tx[1] = 0x01 | (0x01 << 1) | (0x01 << 4);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
	ret += HAL_SPI_Transmit(hspi1, buffer_tx, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

	// Enable interrupts
	buffer_tx[0] = (write_bit | CTRL_REG3);
	buffer_tx[1] = (0x01 << 3) | (0x01 << 7);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
	ret += HAL_SPI_Transmit(hspi1, buffer_tx, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

	// Read ID and check if it is correct. Default is 0b00111111
	buffer_tx[0] = (read_bit | who_am_i_reg);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
	ret += HAL_SPI_Transmit(hspi1, buffer_tx, 1, HAL_MAX_DELAY);
	ret += HAL_SPI_Receive(hspi1, buffer_rx, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
	uint8_t id = buffer_rx[0]; // ID Should be checked.
	if (id != 0b00111111 ){
		// ID dosen't match.
		return;
	}

	if (ret != HAL_OK ){
		// Error, do something.
		return;
	}
}
void task4(SPI_HandleTypeDef *hspi1){

	int ret = 0;
	size_t length = 0;
	uint8_t buffer_rx[2];
	uint8_t buffer_tx[2];
	uint8_t buffer_str[100];

	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_0) != RESET)
	{
		// Clear flag for next interrupt.
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);

		// Get Acc_x
		buffer_tx[0] = (read_bit | REG_OUT_X_H);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
		ret += HAL_SPI_Transmit(hspi1, buffer_tx, 1, HAL_MAX_DELAY);
		ret += HAL_SPI_Receive(hspi1, buffer_rx, 1, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
		uint8_t x_val_h = (buffer_rx[0]);
		if (ret != HAL_OK ){
			// Error, do something.
			return;
		}

		buffer_tx[0] = (read_bit | REG_OUT_X_L);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
		ret += HAL_SPI_Transmit(hspi1, buffer_tx, 1, HAL_MAX_DELAY);
		ret += HAL_SPI_Receive(hspi1, buffer_rx, 1, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
		uint8_t x_val_l = (buffer_rx[0]);
		if (ret != HAL_OK ){
			// Error, do something.
			return;
		}

		// Get Acc_y
		buffer_tx[0] = (read_bit | REG_OUT_Y_H);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
		ret += HAL_SPI_Transmit(hspi1, buffer_tx, 1, HAL_MAX_DELAY);
		ret += HAL_SPI_Receive(hspi1, buffer_rx, 1, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
		uint8_t y_val_h = (buffer_rx[0]);
		if (ret != HAL_OK ){
			// Error, do something.
			return;
		}

		buffer_tx[0] = (read_bit | REG_OUT_Y_L);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
		ret += HAL_SPI_Transmit(hspi1, buffer_tx, 1, HAL_MAX_DELAY);
		ret += HAL_SPI_Receive(hspi1, buffer_rx, 1, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
		uint8_t y_val_l = (buffer_rx[0]);
		if (ret != HAL_OK ){
			// Error, do something.
			return;
		}

		// Combine H and L into one uint16_t.
		uint16_t acc_x = (x_val_h << 8) | x_val_l;
		uint16_t acc_y = (y_val_h << 8) | y_val_l;

		// Compute acceleration in m/s^2 from raw data.
		double acc_x_in_g = 0;
		if ((acc_x & 0x8000) == 0x8000 ){
			// Acceleration is negative
			// First compute the two's complement.
			uint16_t acc_x_twos_complement = (~acc_x) + 1;
			// Use conversion and set to negative
			acc_x_in_g = -((double)acc_x_twos_complement*0.00006);
		} else{
			// Acceleration is positive
			// Using simple conversion model to compute the acceleration in gravitational forces also noted m/s^2.
			acc_x_in_g = (double)acc_x*0.00006;
		}

		double acc_y_in_g = 0;
		if ((acc_y & 0x8000) == 0x8000 ){
			// Acceleration is negative
			// First compute the two's complement.
			uint16_t acc_y_twos_complement = (~acc_y) + 1;
			// Use conversion and set to negative
			acc_y_in_g = -((double)acc_y_twos_complement*0.00006);
		} else{
			// Acceleration is positive
			// Using simple conversion model to compute the acceleration in gravitational forces also noted m/s^2.
			acc_y_in_g = (double)acc_y*0.00006;
		}

		// Set Leds according to orientation.
		if ((acc_x_in_g > -0.05) && (acc_x_in_g < 0.05)){
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		} else if ( (x_val_h >> 7) == 0x01) {
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		} else if ( (x_val_h >> 7) == 0x00){
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
		}

		if ((acc_y_in_g > -0.05) && (acc_y_in_g < 0.05)){
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
		} else if ( (y_val_h>>7) == 0x01) {
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
		} else if ((y_val_h >> 7) == 0x00){
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
		}

		sprintf((char*) &buffer_str, "Data received and leds set. \r\n"); // Store string in buffer
		length = strlen((char*) &buffer_str); // Extract size of string. Need \r\n to give proper length of string.
		CDC_Transmit_FS(buffer_str, length); // Transmit temperature data over USB.
	}

	return;
}
