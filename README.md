# STM32_Intro_Tasks
List of tasks completed on an STM32. 




| Task  | Name | Description | Completed |
| ------------- |  ------------- | ------------- | ------------- |
| 1 | Start up | Become familiar with setting up STM32CubeIDE, which is the program we use to program STM32F407G-DISC1 or STM32F411.  | ✅ |
| 2 | LED & Button | In this task, you will use the LED lights, button and the switch on the STM32F4. | ✅ |
| 3   | Temperaturesensor| In this task, you will become familiar with how to use I2C communication to retrieve data from a temperature sensor. Once you have successfully received temperature data, you will combine that knowledge with what you learned in the previous task. Finally, you will log the temperature data using USB.  | ✅ |
| 4   | Accelerometer | Use SPI communication to extract data from the accelerometer/gyroscope on the STM32F4 (on F411, we use the gyroscope instead). Once you have established communication with the accelerometer, you will use the LED lights to indicate the direction in which the circuit board tilts. This will then be handled with interrupts. You will also become familiar with another program called STM Studio, which is useful for visual plotting of variables. | ✅ |
| 5 | Accelerometer driver | Create a simple driver for the accelerometer | ✅ |  
| 6   | RTOS | Use an RTOS (Real-Time Operating System) to run temperature and accelerometer measurements in parallel. You will also use mutexes and queues to control multiple LED lights. | ✅ |
| 7 | DAC and DMA| Use the CS43L22, which is an Audio-DAC with an integrated amplifier. Generate tones with different frequencies that will eventually be combined with accelerometer data. | ✅ |
<!---               | 3B   | Oscilloskop | This task provides an introduction and basic training in the use of an oscilloscope. First, some very simple measurements generated from a signal generator will be taken. Then, I²C measurements to and from a temperature sensor will be decoded from the oscilloscope.  |  |-->

For more projects and explanations take a look at:https://legacy.cs.indiana.edu/~geobrown/book.pdf.

## Task 1
Downloaded STM32CubeIDE from https://www.st.com/en/development-tools/stm32cubeide.html. Familiarized myself with the STM32F4 user manual and datasheet found at https://www.st.com/en/microcontrollers-microprocessors/stm32f4-series/documentation.html. 
Created a new project, choose the STM32F407G-DISC1, and the reset all pins and clock configrautions before starting this new project. 

## Task 2
Enabled the three LEDs on the STM32 to be GPIO OUTPUT, and enabled Button B1 to be GPIO input. The correct pins where found in the datasheet. Then by reading reading an writing to BSRR AND IDR registers of GPIOD and GPIOA the logic for toggling LEDs and using buttons was implemented.

## Task 3
Connect TMP102 sensor through I2C. Set the correct I2C1 pins to correct modus. Under connectivity enable the I2C1. 


## Task 4
### Communicate with the accelerometer with SPI
Activate measurment in control register 4. 

### Use LED to indicate the direction of the dev kit

### Extract data only when data is availible using interrupt
When implementing interrupt for the accelerometer there is two ways to go about it. First is to simply check interrupt ourself with __HAL_GPIO_EXTI_GET_IT(GPIO_PIN_0) and clear it with __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0). Or we can activate NVIC which then controls the interrupts for us and we can simply override the callback function HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin). However, i am unsure if this callback function is for all itnerrupt and we then have to switch between cases depending of the GPIO_Pin which the interrupt originate from. 
Both these cases require us to activate interrupt in the accelerometer and set the GPIOE pin 0 to GPIO_EXTI0. This pin is the pin connected to INT1 on the accelerometer. We activate interrupt1 by setting bit ... to high in control register 3 of the accelerometer.


## Task5
Created a simple driver for the accelerometer with a init, write and read function. These functions then replaced the code previously implemented in task 4. The driver is found under Drivers folder. 


## Task 6

Threads and mutex was created with the CMSIS_V2 interface with FreeRTOS. First i have two threads running the temperature and accelerometer tasks (3 and 4/5). Then i add four new threads to blink the four leds on the board. I then implement a mutex such that only one led blinks at a time, instead of all leds blinking asynchrounsly. 

Queue was then implemented with the CMSIS_V1 interface with FreeRTOS. The queue was not implemented completely. 


## Task 7
Using a discrete generated sinusoidal signal to send digital sound to the DAC through the DMA. Sending a constant sound, and also sending a varying sound depending on the degree of acceleration on the x-axis. If acc_x is -9.81 the note is lowest, and acc_x is 9.81 the note is highest. 
When sending with DMA the signal is continously being sent. When the data is finished it repeats over and over again. To change what signal is being sent we can simply alter the data where the signal is stored in memory. 

The sinusoidal signal was generated with the simple function: 
$A sin(2 \pi f t + \phi)$

The different notes was generated by altering the frequency. Human can detect sound in the frequency span 20Hz to 20kHz. This is also the reason why you normally have a sampling rate of 41kHz. This comes from the Nyquist-Shannon sampling theorem stating that the sample rate must be at least twice the bandwith of the signal to avoid aliasing. Therefore, if we are operating on max frequency of 20kHz, the sampling rate of 41kHz is enough to represent the original signal without misleading or incorrect result. 

Reasource on notes and frequency. https://www.omnicalculator.com/other/note-frequency#the-frequency-of-musical-notes, https://celik-muhammed.medium.com/how-to-generate-440-hz-a-la-note-sin-wave-with-44-1-1e41f6ed9653. 
