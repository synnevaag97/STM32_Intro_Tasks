################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/LIS3DSH_Driver/lis3dsh_task5.c 

OBJS += \
./Drivers/LIS3DSH_Driver/lis3dsh_task5.o 

C_DEPS += \
./Drivers/LIS3DSH_Driver/lis3dsh_task5.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/LIS3DSH_Driver/%.o Drivers/LIS3DSH_Driver/%.su Drivers/LIS3DSH_Driver/%.cyclo: ../Drivers/LIS3DSH_Driver/%.c Drivers/LIS3DSH_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/LIS3DSH_Driver/ -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-LIS3DSH_Driver

clean-Drivers-2f-LIS3DSH_Driver:
	-$(RM) ./Drivers/LIS3DSH_Driver/lis3dsh_task5.cyclo ./Drivers/LIS3DSH_Driver/lis3dsh_task5.d ./Drivers/LIS3DSH_Driver/lis3dsh_task5.o ./Drivers/LIS3DSH_Driver/lis3dsh_task5.su

.PHONY: clean-Drivers-2f-LIS3DSH_Driver

