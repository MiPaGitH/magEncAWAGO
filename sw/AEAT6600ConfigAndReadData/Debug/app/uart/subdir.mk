################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../app/uart/u.c 

OBJS += \
./app/uart/u.o 

C_DEPS += \
./app/uart/u.d 


# Each subdirectory must supply rules for building sources it contributes
app/uart/%.o app/uart/%.su app/uart/%.cyclo: ../app/uart/%.c app/uart/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../Core/Inc -I../../app/uart -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-app-2f-uart

clean-app-2f-uart:
	-$(RM) ./app/uart/u.cyclo ./app/uart/u.d ./app/uart/u.o ./app/uart/u.su

.PHONY: clean-app-2f-uart

