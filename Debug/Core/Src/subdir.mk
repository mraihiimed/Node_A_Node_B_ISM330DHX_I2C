################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Read_Function.c \
../Core/Src/can.c \
../Core/Src/can_handler.c \
../Core/Src/dht.c \
../Core/Src/gpio.c \
../Core/Src/i2c1.c \
../Core/Src/ism330dhcx.c \
../Core/Src/main.c \
../Core/Src/peripherals.c \
../Core/Src/peripherals_init.c \
../Core/Src/stm32f1xx_hal_msp.c \
../Core/Src/stm32f1xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f1xx.c \
../Core/Src/systemclock.c \
../Core/Src/tim.c \
../Core/Src/uart.c \
../Core/Src/vehiclefulectri.c 

OBJS += \
./Core/Src/Read_Function.o \
./Core/Src/can.o \
./Core/Src/can_handler.o \
./Core/Src/dht.o \
./Core/Src/gpio.o \
./Core/Src/i2c1.o \
./Core/Src/ism330dhcx.o \
./Core/Src/main.o \
./Core/Src/peripherals.o \
./Core/Src/peripherals_init.o \
./Core/Src/stm32f1xx_hal_msp.o \
./Core/Src/stm32f1xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f1xx.o \
./Core/Src/systemclock.o \
./Core/Src/tim.o \
./Core/Src/uart.o \
./Core/Src/vehiclefulectri.o 

C_DEPS += \
./Core/Src/Read_Function.d \
./Core/Src/can.d \
./Core/Src/can_handler.d \
./Core/Src/dht.d \
./Core/Src/gpio.d \
./Core/Src/i2c1.d \
./Core/Src/ism330dhcx.d \
./Core/Src/main.d \
./Core/Src/peripherals.d \
./Core/Src/peripherals_init.d \
./Core/Src/stm32f1xx_hal_msp.d \
./Core/Src/stm32f1xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f1xx.d \
./Core/Src/systemclock.d \
./Core/Src/tim.d \
./Core/Src/uart.d \
./Core/Src/vehiclefulectri.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/Read_Function.cyclo ./Core/Src/Read_Function.d ./Core/Src/Read_Function.o ./Core/Src/Read_Function.su ./Core/Src/can.cyclo ./Core/Src/can.d ./Core/Src/can.o ./Core/Src/can.su ./Core/Src/can_handler.cyclo ./Core/Src/can_handler.d ./Core/Src/can_handler.o ./Core/Src/can_handler.su ./Core/Src/dht.cyclo ./Core/Src/dht.d ./Core/Src/dht.o ./Core/Src/dht.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/i2c1.cyclo ./Core/Src/i2c1.d ./Core/Src/i2c1.o ./Core/Src/i2c1.su ./Core/Src/ism330dhcx.cyclo ./Core/Src/ism330dhcx.d ./Core/Src/ism330dhcx.o ./Core/Src/ism330dhcx.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/peripherals.cyclo ./Core/Src/peripherals.d ./Core/Src/peripherals.o ./Core/Src/peripherals.su ./Core/Src/peripherals_init.cyclo ./Core/Src/peripherals_init.d ./Core/Src/peripherals_init.o ./Core/Src/peripherals_init.su ./Core/Src/stm32f1xx_hal_msp.cyclo ./Core/Src/stm32f1xx_hal_msp.d ./Core/Src/stm32f1xx_hal_msp.o ./Core/Src/stm32f1xx_hal_msp.su ./Core/Src/stm32f1xx_it.cyclo ./Core/Src/stm32f1xx_it.d ./Core/Src/stm32f1xx_it.o ./Core/Src/stm32f1xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f1xx.cyclo ./Core/Src/system_stm32f1xx.d ./Core/Src/system_stm32f1xx.o ./Core/Src/system_stm32f1xx.su ./Core/Src/systemclock.cyclo ./Core/Src/systemclock.d ./Core/Src/systemclock.o ./Core/Src/systemclock.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/uart.cyclo ./Core/Src/uart.d ./Core/Src/uart.o ./Core/Src/uart.su ./Core/Src/vehiclefulectri.cyclo ./Core/Src/vehiclefulectri.d ./Core/Src/vehiclefulectri.o ./Core/Src/vehiclefulectri.su

.PHONY: clean-Core-2f-Src

