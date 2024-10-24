################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/stm32f1_can.c \
../drivers/Src/stm32f1_gpio.c \
../drivers/Src/stm32f1_i2c.c \
../drivers/Src/stm32f1_rcc.c \
../drivers/Src/stm32f1_spi.c \
../drivers/Src/stm32f1_uart.c 

OBJS += \
./drivers/Src/stm32f1_can.o \
./drivers/Src/stm32f1_gpio.o \
./drivers/Src/stm32f1_i2c.o \
./drivers/Src/stm32f1_rcc.o \
./drivers/Src/stm32f1_spi.o \
./drivers/Src/stm32f1_uart.o 

C_DEPS += \
./drivers/Src/stm32f1_can.d \
./drivers/Src/stm32f1_gpio.d \
./drivers/Src/stm32f1_i2c.d \
./drivers/Src/stm32f1_rcc.d \
./drivers/Src/stm32f1_spi.d \
./drivers/Src/stm32f1_uart.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/%.o drivers/Src/%.su drivers/Src/%.cyclo: ../drivers/Src/%.c drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -c -I../Inc -I"E:/stm32/driver/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-drivers-2f-Src

clean-drivers-2f-Src:
	-$(RM) ./drivers/Src/stm32f1_can.cyclo ./drivers/Src/stm32f1_can.d ./drivers/Src/stm32f1_can.o ./drivers/Src/stm32f1_can.su ./drivers/Src/stm32f1_gpio.cyclo ./drivers/Src/stm32f1_gpio.d ./drivers/Src/stm32f1_gpio.o ./drivers/Src/stm32f1_gpio.su ./drivers/Src/stm32f1_i2c.cyclo ./drivers/Src/stm32f1_i2c.d ./drivers/Src/stm32f1_i2c.o ./drivers/Src/stm32f1_i2c.su ./drivers/Src/stm32f1_rcc.cyclo ./drivers/Src/stm32f1_rcc.d ./drivers/Src/stm32f1_rcc.o ./drivers/Src/stm32f1_rcc.su ./drivers/Src/stm32f1_spi.cyclo ./drivers/Src/stm32f1_spi.d ./drivers/Src/stm32f1_spi.o ./drivers/Src/stm32f1_spi.su ./drivers/Src/stm32f1_uart.cyclo ./drivers/Src/stm32f1_uart.d ./drivers/Src/stm32f1_uart.o ./drivers/Src/stm32f1_uart.su

.PHONY: clean-drivers-2f-Src

