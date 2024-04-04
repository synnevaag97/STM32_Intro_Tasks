#ifndef __TASK3_H
#define __TASK3_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "usbd_cdc_if.h"

void task3(I2C_HandleTypeDef *hi2c1);

#ifdef __cplusplus
}
#endif

#endif /* __TASK2_H */
