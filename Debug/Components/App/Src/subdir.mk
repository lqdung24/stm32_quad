################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Components/App/Src/app.c 

OBJS += \
./Components/App/Src/app.o 

C_DEPS += \
./Components/App/Src/app.d 


# Each subdirectory must supply rules for building sources it contributes
Components/App/Src/%.o Components/App/Src/%.su Components/App/Src/%.cyclo: ../Components/App/Src/%.c Components/App/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Components/Attitude/Inc -I../Components/Mahony/Inc -I../Components/Mahony9/Inc -I../Components/MotorPWM/Inc -I../Components/DroneProtocol/Inc -I../Components/DroneControl/Inc -I../Components/App/Inc -I"/home/lqdung/Downloads/11.drone/stm32cube/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/home/lqdung/Downloads/11.drone/stm32cube/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"/home/lqdung/Downloads/11.drone/stm32cube/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"/home/lqdung/Downloads/11.drone/stm32cube/USB_DEVICE/App" -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Components/ICM20948/Inc -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/RTOS2/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Components-2f-App-2f-Src

clean-Components-2f-App-2f-Src:
	-$(RM) ./Components/App/Src/app.cyclo ./Components/App/Src/app.d ./Components/App/Src/app.o ./Components/App/Src/app.su

.PHONY: clean-Components-2f-App-2f-Src

