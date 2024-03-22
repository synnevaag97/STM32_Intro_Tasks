
#include "task2.h"

void task2(int subtask){
	 if (subtask == 0){
		 // Set the orange light high.
		 GPIOD->BSRR = (uint32_t)(1<<13); //or (uint32_t)(GPIO_PIN_13);
		 HAL_Delay(1000);
		 // Set the orange light low
		 GPIOD->BSRR = (uint32_t)((1<<13) << 16);
		 HAL_Delay(1000);
	 } else if(subtask == 1){
		 // Toggle the Green led using HAL code.
		 HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		 HAL_Delay(1000);
	 } else if(subtask == 2){
		 // Turn on Red light when button B1 is pressed.
		 uint32_t B1_mask_state = (uint32_t)(1<<0);
		 uint32_t idr_a = GPIOA->IDR;
		 if( ((idr_a & B1_mask_state) == B1_mask_state)){
			 GPIOD->BSRR = (uint32_t)(1<<14);
		 } else if ((idr_a & B1_mask_state) != B1_mask_state ){
			 GPIOD->BSRR = (uint32_t)((1<<14) << 16);
		 }
	 }
}
