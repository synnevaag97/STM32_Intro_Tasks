/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "task2.h"
#include "task3.h"
#include "task4.h"
#include "task5.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

osThreadId accTaskHandle;
osThreadId tmpTaskHandle;
osThreadId ledGreenTaskHandle;
osThreadId ledOrangeTaskHandle;
osThreadId ledRedTaskHandle;
osThreadId ledBlueTaskHandle;
osMessageQId myQ1Handle;
osMutexId blinking_resourceHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
void acc_fetch(void const * argument);
void tmp_fetch(void const * argument);
void ledGreen(void const * argument);
void ledOrange(void const * argument);
void ledRed(void const * argument);
void ledBlue(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of blinking_resource */
  osMutexDef(blinking_resource);
  blinking_resourceHandle = osMutexCreate(osMutex(blinking_resource));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of myQ1 */
  osMessageQDef(myQ1, 8, uint16_t);
  myQ1Handle = osMessageCreate(osMessageQ(myQ1), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of accTask */
  osThreadDef(accTask, acc_fetch, osPriorityLow, 0, 1000);
  accTaskHandle = osThreadCreate(osThread(accTask), NULL);

  /* definition and creation of tmpTask */
  osThreadDef(tmpTask, tmp_fetch, osPriorityLow, 0, 1000);
  tmpTaskHandle = osThreadCreate(osThread(tmpTask), NULL);

  /* definition and creation of ledGreenTask */
  osThreadDef(ledGreenTask, ledGreen, osPriorityNormal, 0, 128);
  ledGreenTaskHandle = osThreadCreate(osThread(ledGreenTask), NULL);

  /* definition and creation of ledOrangeTask */
  osThreadDef(ledOrangeTask, ledOrange, osPriorityNormal, 0, 128);
  ledOrangeTaskHandle = osThreadCreate(osThread(ledOrangeTask), NULL);

  /* definition and creation of ledRedTask */
  osThreadDef(ledRedTask, ledRed, osPriorityNormal, 0, 128);
  ledRedTaskHandle = osThreadCreate(osThread(ledRedTask), NULL);

  /* definition and creation of ledBlueTask */
  osThreadDef(ledBlueTask, ledBlue, osPriorityNormal, 0, 128);
  ledBlueTaskHandle = osThreadCreate(osThread(ledBlueTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void blink_led_5_time(uint16_t GPIO_PIN){
	osMutexWait(blinking_resourceHandle, HAL_MAX_DELAY);
	for (int i = 0; i < 5; i++){
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN);
		HAL_Delay(500);
	}
	osMutexRelease(blinking_resourceHandle);
}

/*
 * Uncomment callback if using NVIC for task 4.
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// GPIO_Pin = E0?
	uint8_t buffer_str[200];
	sprintf((char*) &buffer_str, "Data is availible \r\n"); // Store string in buffer
	size_t length = strlen((char*) &buffer_str); // Extract size of string. Need \r\n to give proper length of string.
	CDC_Transmit_FS(buffer_str, length);
}*/

/* USER CODE END 4 */

/* USER CODE BEGIN Header_acc_fetch */
/**
  * @brief  Function implementing the accTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_acc_fetch */
void acc_fetch(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  uint8_t buffer[100];
  for(;;)
  {
	  sprintf((char*) &buffer, "In for loop for acc \r\n");
	  size_t length = strlen((char*) &buffer);
	  CDC_Transmit_FS(buffer, length);
	  //task5(&hspi1);
    osDelay(100);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_tmp_fetch */
/**
* @brief Function implementing the tmpTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_tmp_fetch */
void tmp_fetch(void const * argument)
{
  /* USER CODE BEGIN tmp_fetch */
	/* Infinite loop */
	uint8_t buffer[100];
	for(;;)
	{
		sprintf((char*) &buffer, "In for loop for temp \r\n");
		size_t length = strlen((char*) &buffer);
		CDC_Transmit_FS(buffer, length);

		//task3(&hi2c1);
		osDelay(100);
	}
  /* USER CODE END tmp_fetch */
}

/* USER CODE BEGIN Header_ledGreen */
/**
* @brief Function implementing the ledGreenTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ledGreen */
void ledGreen(void const * argument)
{
  /* USER CODE BEGIN ledGreen */
  /* Infinite loop */
  for(;;)
  {
	blink_led_5_time(GPIO_PIN_12);
    osDelay(1);
  }
  /* USER CODE END ledGreen */
}

/* USER CODE BEGIN Header_ledOrange */
/**
* @brief Function implementing the ledOrangeTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ledOrange */
void ledOrange(void const * argument)
{
  /* USER CODE BEGIN ledOrange */
  /* Infinite loop */
  for(;;)
  {
	  blink_led_5_time(GPIO_PIN_13);
    osDelay(1);
  }
  /* USER CODE END ledOrange */
}

/* USER CODE BEGIN Header_ledRed */
/**
* @brief Function implementing the ledRedTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ledRed */
void ledRed(void const * argument)
{
  /* USER CODE BEGIN ledRed */
  /* Infinite loop */
  for(;;)
  {
	  blink_led_5_time(GPIO_PIN_14);
    osDelay(1);
  }
  /* USER CODE END ledRed */
}

/* USER CODE BEGIN Header_ledBlue */
/**
* @brief Function implementing the ledBlueTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ledBlue */
void ledBlue(void const * argument)
{
  /* USER CODE BEGIN ledBlue */
  /* Infinite loop */
  for(;;)
  {
	  blink_led_5_time(GPIO_PIN_15);
    osDelay(1);
  }
  /* USER CODE END ledBlue */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
