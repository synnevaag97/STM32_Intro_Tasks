#include "task5.h"



void task5(SPI_HandleTypeDef *hspi1){

	lis3dsh_config parameters;
	parameters.ctrl_reg4_config = CTRL_REG4_ODR0 | CTRL_REG4_Xen | CTRL_REG4_Yen;
	parameters.ctrl_reg3_config = CTRL_REG3_INT1_EN | CTRL_REG3_DR_EN;

	lis3dsh_init(hspi1, &parameters);

	int ret = 0;
	size_t length = 0;
	uint8_t buffer_rx[2];
	uint8_t buffer_str[100];

	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_0) != RESET)
	{
		// Clear flag for next interrupt.
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);

		// Get Acc_x
		ret = lis3dsh_read(LIS3DSH_OUT_X_H_ADDR, buffer_rx, 1);
		uint8_t x_val_h = (buffer_rx[0]);

		ret = lis3dsh_read(LIS3DSH_OUT_X_L_ADDR, buffer_rx, 1);
		uint8_t x_val_l = (buffer_rx[0]);
		if (ret != HAL_OK ){
			// Error, do something.
			return;
		}

		// Get Acc_y
		ret = lis3dsh_read(LIS3DSH_OUT_Y_H_ADDR, buffer_rx, 1);
		uint8_t y_val_h = (buffer_rx[0]);

		ret = lis3dsh_read(LIS3DSH_OUT_Y_L_ADDR, buffer_rx, 1);
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
}
