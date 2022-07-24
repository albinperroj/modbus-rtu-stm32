################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ModbusRTULibrary/rtuHandler.c 

OBJS += \
./ModbusRTULibrary/rtuHandler.o 

C_DEPS += \
./ModbusRTULibrary/rtuHandler.d 


# Each subdirectory must supply rules for building sources it contributes
ModbusRTULibrary/rtuHandler.o: ../ModbusRTULibrary/rtuHandler.c ModbusRTULibrary/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/User/Desktop/Projects/ModbusMaster/ModbusRTULibrary" -I/ModbusMaster/Drivers/STM32F1xx_HAL_Driver/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"ModbusRTULibrary/rtuHandler.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

