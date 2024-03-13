# STM32_Intro_Tasks
List of tasks completed on an STM32. 


Task 1 - Start up

| Task  | Name | Description | Completed |
| ------------- | ------------- | ------------- |
| 1 | Start up | Become familiar with setting up STM32CubeIDE, which is the program we use to program STM32F407G-DISC1 or STM32F411.  | ✅ |
| 2 | LED  | In this task, you will use the LED lights and the switch on the STM32F4. | ✅ |
| 3   | Temperaturesensor| In this task, you will become familiar with how to use I2C communication to retrieve data from a temperature sensor. Once you have successfully received temperature data, you will combine that knowledge with what you learned in the previous task. Finally, you will log the temperature data using USB.  | ✅ |
| 3B   | Oscilloskop | This task provides an introduction and basic training in the use of an oscilloscope. First, some very simple measurements generated from a signal generator will be taken. Then, I²C measurements to and from a temperature sensor will be decoded from the oscilloscope.  | ✅ |
| 4   | Accelerometer | In this task, you will use SPI communication to extract data from the accelerometer/gyroscope on the STM32F4 (on F411, we use the gyroscope instead). Once you have established communication with the accelerometer, you will use the LED lights to indicate the direction in which the circuit board tilts. You will also become familiar with another program called STM Studio, which is useful for visual plotting of variables. |  |
| 5   | RTOS | In this task, you will use an RTOS (Real-Time Operating System) to run temperature and accelerometer measurements in parallel. You will also use mutexes and queues to control multiple LED lights. | ✅ |
| 6 | Accelerometer driver | Create a driver for the accelerometer | ✅ |  
| 7 | DAC and DMA| In this task, you will use the CS43L22, which is an Audio-DAC with an integrated amplifier. You will generate tones with different frequencies that will eventually be combined with accelerometer data. | |
