################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ILI9341/my_lib_ILI9341.c \
../ILI9341/my_lib_XPT2046.c \
../ILI9341/my_lib_gui.c 

OBJS += \
./ILI9341/my_lib_ILI9341.o \
./ILI9341/my_lib_XPT2046.o \
./ILI9341/my_lib_gui.o 

C_DEPS += \
./ILI9341/my_lib_ILI9341.d \
./ILI9341/my_lib_XPT2046.d \
./ILI9341/my_lib_gui.d 


# Each subdirectory must supply rules for building sources it contributes
ILI9341/%.o: ../ILI9341/%.c ILI9341/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L4R5xx -DDEBUG -c -I../Inc -I../Drivers/CMSIS/Include -I"C:/Users/Pavel/STM32CubeIDE/workspace_1.0.2/test_pripravku_L4/ILI9341" -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

