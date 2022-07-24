################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NTC-master/ntc.c 

OBJS += \
./NTC-master/ntc.o 

C_DEPS += \
./NTC-master/ntc.d 


# Each subdirectory must supply rules for building sources it contributes
NTC-master/ntc.o: ../NTC-master/ntc.c NTC-master/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/User/Desktop/Projects/ModbusSlave/ModbusRTULibrary" -I"C:/Users/User/Desktop/Projects/ModbusSlave/NTC-master" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"NTC-master/ntc.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

