#ifndef __LIS3DSH_TASK5_H
#define __LIS3DSH_TASK5_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"
#include "usbd_cdc_if.h"

/* Registers ------------------------------------------------------------------*/
#define LIS3DSH_WHO_AM_I_ADDR 0x0F
#define LIS3DSH_CTRL_REG4_ADDR 0x20
#define LIS3DSH_CTRL_REG3_ADDR 0x23
#define LIS3DSH_CTRL_REG6_ADDR 0x25
#define LIS3DSH_OUT_X_L_ADDR 0x28
#define LIS3DSH_OUT_X_H_ADDR 0x29
#define LIS3DSH_OUT_Y_L_ADDR 0x2A
#define LIS3DSH_OUT_Y_H_ADDR 0x2B


/* Configurations ------------------------------------------------------------------*/
#define CTRL_REG4_ODR0 (0x01 << 4) // Set frequency to lowest
#define CTRL_REG4_Xen 0x01 // Enable x measurements
#define CTRL_REG4_Yen (0x01 << 1) // Enable y measurements
#define CTRL_REG4_Zen (0x01 << 2) // Enable Z measurements
#define CTRL_REG4_XYZen ((0x01 << 2) | (0x01 << 1) | (0x01)) // Enable XYZ measurements

#define CTRL_REG3_INT1_EN (0x01 << 3) // Enable interrupt 1
#define CTRL_REG3_DR_EN (0x01 << 7) // Enable data ready signal, connect to INT1.

#define READ_CMD (0x01 << 7)
#define WRITE_CMD (0x00)

#define SET_CS_PIN()   (HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET)) // Setting the CS pin requires RESET the GPIO pin.
#define CLEAR_CS_PIN() (HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET))


/* Members ------------------------------------------------------------------*/

typedef struct {
	uint8_t ctrl_reg4_config; // Frequency, Acc enabling
	uint8_t ctrl_reg3_config; // Interrupts enabling
} lis3dsh_config;

/* Functions ------------------------------------------------------------------*/
int lis3dsh_init(SPI_HandleTypeDef *hspi1, lis3dsh_config *parameters);
int lis3dsh_write(uint8_t reg_address, uint8_t data);
int lis3dsh_read(uint8_t reg_address, uint8_t *buffer_rx, size_t length);

#ifdef __cplusplus
}
#endif

#endif /* __TASK2_H */
