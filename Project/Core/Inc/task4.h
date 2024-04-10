#ifndef __TASK4_H
#define __TASK4_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"
#include "usbd_cdc_if.h"

/*
 * The STM32F4 have a ST-MEMS motion sensor which is an ultra-compact low-power three-axis linear accelerometer.
 * We can communicate with it using both I2C and SPI. We will use SPI.
 * The CS pin for the MEMS is set to GPIO Output.
 * 10 MHz max clock frequency.
 * SPI1 is connected to APB2 based on Table 33, examining register RCC_APB2ENR.
 * */
#define read_bit (0x01 << 7)
#define write_bit (0x00)
#define who_am_i_reg 0x0F



#define REG_OUT_X_L 0x28
#define REG_OUT_X_H 0x29
#define REG_OUT_Y_L 0x2A
#define REG_OUT_Y_H 0x2B
#define REG_OUT_Z_L 0x2
#define REG_OUT_Z_H 0x2D
#define CTRL_REG4 0x20
#define CTRL_REG3 0x23
#define CTRL_REG6 0x25

/*
uint8_t buffer_rx[2];
uint8_t buffer_tx[2];
uint8_t buffer_str[100];
*/
void task4_init(SPI_HandleTypeDef *hspi1);
void task4(SPI_HandleTypeDef *hspi1);

#ifdef __cplusplus
}
#endif

#endif /* __TASK2_H */
