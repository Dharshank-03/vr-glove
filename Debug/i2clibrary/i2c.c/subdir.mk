################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../i2clibrary/i2c.c/i2c.c 

OBJS += \
./i2clibrary/i2c.c/i2c.o 

C_DEPS += \
./i2clibrary/i2c.c/i2c.d 


# Each subdirectory must supply rules for building sources it contributes
i2clibrary/i2c.c/%.o i2clibrary/i2c.c/%.su: ../i2clibrary/i2c.c/%.c i2clibrary/i2c.c/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I"C:/Users/Admin/OneDrive/Desktop/embedded c/instructor/host/vrgloves/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/Admin/OneDrive/Desktop/embedded c/instructor/host/vrgloves/i2clibrary/i2c.h" -I"C:/Users/Admin/OneDrive/Desktop/embedded c/instructor/host/vrgloves/i2clibrary/i2c.h" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-i2clibrary-2f-i2c-2e-c

clean-i2clibrary-2f-i2c-2e-c:
	-$(RM) ./i2clibrary/i2c.c/i2c.d ./i2clibrary/i2c.c/i2c.o ./i2clibrary/i2c.c/i2c.su

.PHONY: clean-i2clibrary-2f-i2c-2e-c

