################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../system/src/cortexm/exception-handlers.c \
../system/src/cortexm/initialize-hardware.c \
../system/src/cortexm/reset-hardware.c 

C_DEPS += \
./system/src/cortexm/exception-handlers.d \
./system/src/cortexm/initialize-hardware.d \
./system/src/cortexm/reset-hardware.d 

OBJS += \
./system/src/cortexm/exception-handlers.o \
./system/src/cortexm/initialize-hardware.o \
./system/src/cortexm/reset-hardware.o 


# Each subdirectory must supply rules for building sources it contributes
system/src/cortexm/%.o: ../system/src/cortexm/%.c system/src/cortexm/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU Arm Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -Wall -Wextra -g -DNDEBUG -DSTM32F407xx -DUSE_HAL_DRIVER -DHSE_VALUE=8000000 -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f4-hal" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


