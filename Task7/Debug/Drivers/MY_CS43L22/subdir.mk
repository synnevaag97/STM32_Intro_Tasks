################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/MY_CS43L22/MY_CS43L22.c 

OBJS += \
./Drivers/MY_CS43L22/MY_CS43L22.o 

C_DEPS += \
./Drivers/MY_CS43L22/MY_CS43L22.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/MY_CS43L22/%.o Drivers/MY_CS43L22/%.su Drivers/MY_CS43L22/%.cyclo: ../Drivers/MY_CS43L22/%.c Drivers/MY_CS43L22/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/LIS3DSH_Driver/ -I../Drivers/MY_CS43L22/ -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-MY_CS43L22

clean-Drivers-2f-MY_CS43L22:
	-$(RM) ./Drivers/MY_CS43L22/MY_CS43L22.cyclo ./Drivers/MY_CS43L22/MY_CS43L22.d ./Drivers/MY_CS43L22/MY_CS43L22.o ./Drivers/MY_CS43L22/MY_CS43L22.su

.PHONY: clean-Drivers-2f-MY_CS43L22

