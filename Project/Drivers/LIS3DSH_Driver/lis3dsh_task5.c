#include "lis3dsh_task5.h"

static SPI_HandleTypeDef *handle_spi1;

int lis3dsh_write(uint8_t reg_address, uint8_t data){
	uint8_t buffer_tx[2];
	buffer_tx[0] = (WRITE_CMD | reg_address);
	buffer_tx[1] = data;


	SET_CS_PIN();
	int ret = HAL_SPI_Transmit(handle_spi1, buffer_tx, 2, HAL_MAX_DELAY);
	CLEAR_CS_PIN();

	return ret;
}
int lis3dsh_read(uint8_t reg_address, uint8_t *buffer_rx, size_t length){
	uint8_t buffer_tx[1];
	buffer_tx[0] = (READ_CMD | reg_address);

	SET_CS_PIN();
	int ret = HAL_SPI_Transmit(handle_spi1, buffer_tx, 1, HAL_MAX_DELAY);
	ret += HAL_SPI_Receive(handle_spi1, buffer_rx, length, HAL_MAX_DELAY);
	CLEAR_CS_PIN();

	return ret;
}

int lis3dsh_init(SPI_HandleTypeDef *hspi1, lis3dsh_config *parameters){
	handle_spi1 = hspi1;
	int ret = 0;

	// Set CRTL_REG4
	ret += lis3dsh_write(LIS3DSH_CTRL_REG4_ADDR, parameters->ctrl_reg4_config);

	// Set CTRL_REG3
	ret += lis3dsh_write(LIS3DSH_CTRL_REG3_ADDR, parameters->ctrl_reg3_config);

	// Read and check ID
	uint8_t buffer_rx[1];
	ret += lis3dsh_read(LIS3DSH_WHO_AM_I_ADDR,buffer_rx,1);
	uint8_t id = buffer_rx[0]; // ID Should be checked.

	if (id != 0b00111111 ){
		// ID dosen't match.
		return -1;
	}

	if (ret != HAL_OK){
		// Initialization failed.
		return -1;
	}
	return 0;
}

