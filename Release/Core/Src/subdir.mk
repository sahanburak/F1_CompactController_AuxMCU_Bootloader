################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/aes.c \
../Core/Src/crc16.c \
../Core/Src/dma.c \
../Core/Src/flash_if.c \
../Core/Src/gpio.c \
../Core/Src/iap.c \
../Core/Src/io.c \
../Core/Src/main.c \
../Core/Src/rt_app_info.c \
../Core/Src/rt_bus_proto.c \
../Core/Src/rt_info.c \
../Core/Src/rt_init.c \
../Core/Src/sha1.c \
../Core/Src/spi.c \
../Core/Src/stm32f1xx_hal_msp.c \
../Core/Src/stm32f1xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f1xx.c \
../Core/Src/usart.c 

OBJS += \
./Core/Src/aes.o \
./Core/Src/crc16.o \
./Core/Src/dma.o \
./Core/Src/flash_if.o \
./Core/Src/gpio.o \
./Core/Src/iap.o \
./Core/Src/io.o \
./Core/Src/main.o \
./Core/Src/rt_app_info.o \
./Core/Src/rt_bus_proto.o \
./Core/Src/rt_info.o \
./Core/Src/rt_init.o \
./Core/Src/sha1.o \
./Core/Src/spi.o \
./Core/Src/stm32f1xx_hal_msp.o \
./Core/Src/stm32f1xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f1xx.o \
./Core/Src/usart.o 

C_DEPS += \
./Core/Src/aes.d \
./Core/Src/crc16.d \
./Core/Src/dma.d \
./Core/Src/flash_if.d \
./Core/Src/gpio.d \
./Core/Src/iap.d \
./Core/Src/io.d \
./Core/Src/main.d \
./Core/Src/rt_app_info.d \
./Core/Src/rt_bus_proto.d \
./Core/Src/rt_info.d \
./Core/Src/rt_init.d \
./Core/Src/sha1.d \
./Core/Src/spi.d \
./Core/Src/stm32f1xx_hal_msp.d \
./Core/Src/stm32f1xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f1xx.d \
./Core/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DBOARD_VERSION=50 -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

