################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/MPU6050_V2.1/IIR_filter.c \
../Drivers/MPU6050_V2.1/MPU_V2.1.c 

OBJS += \
./Drivers/MPU6050_V2.1/IIR_filter.o \
./Drivers/MPU6050_V2.1/MPU_V2.1.o 

C_DEPS += \
./Drivers/MPU6050_V2.1/IIR_filter.d \
./Drivers/MPU6050_V2.1/MPU_V2.1.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/MPU6050_V2.1/%.o Drivers/MPU6050_V2.1/%.su: ../Drivers/MPU6050_V2.1/%.c Drivers/MPU6050_V2.1/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/MPU6050_V2.1 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-MPU6050_V2-2e-1

clean-Drivers-2f-MPU6050_V2-2e-1:
	-$(RM) ./Drivers/MPU6050_V2.1/IIR_filter.d ./Drivers/MPU6050_V2.1/IIR_filter.o ./Drivers/MPU6050_V2.1/IIR_filter.su ./Drivers/MPU6050_V2.1/MPU_V2.1.d ./Drivers/MPU6050_V2.1/MPU_V2.1.o ./Drivers/MPU6050_V2.1/MPU_V2.1.su

.PHONY: clean-Drivers-2f-MPU6050_V2-2e-1

