################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Components/Attitude/Src/attitude.c 

OBJS += \
./Components/Attitude/Src/attitude.o 

C_DEPS += \
./Components/Attitude/Src/attitude.d 


# Each subdirectory must supply rules for building sources it contributes
Components/Attitude/Src/%.o Components/Attitude/Src/%.su Components/Attitude/Src/%.cyclo: ../Components/Attitude/Src/%.c Components/Attitude/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Components/Attitude/Inc -I../Components/Mahony/Inc -I../Components/Mahony9/Inc -I../Components/MotorPWM/Inc -I../Components/DroneProtocol/Inc -I../Components/DroneControl/Inc -I../Components/App/Inc -I"/home/lqdung/Downloads/11.drone/stm32cube/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/home/lqdung/Downloads/11.drone/stm32cube/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"/home/lqdung/Downloads/11.drone/stm32cube/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"/home/lqdung/Downloads/11.drone/stm32cube/USB_DEVICE/App" -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Components/ICM20948/Inc -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Components-2f-Attitude-2f-Src

clean-Components-2f-Attitude-2f-Src:
	-$(RM) ./Components/Attitude/Src/attitude.cyclo ./Components/Attitude/Src/attitude.d ./Components/Attitude/Src/attitude.o ./Components/Attitude/Src/attitude.su

.PHONY: clean-Components-2f-Attitude-2f-Src

