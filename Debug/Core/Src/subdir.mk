################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Controlling_LED.c \
../Core/Src/GPS.c \
../Core/Src/GSM.c \
../Core/Src/Queue_GSM.c \
../Core/Src/RFID.c \
../Core/Src/RS232-UART1.c \
../Core/Src/RS232-UART2.c \
../Core/Src/RTC.c \
../Core/Src/freertos.c \
../Core/Src/main.c \
../Core/Src/spi_flash.c \
../Core/Src/stm32f3xx_hal_msp.c \
../Core/Src/stm32f3xx_hal_timebase_tim.c \
../Core/Src/stm32f3xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_management.c \
../Core/Src/system_stm32f3xx.c 

OBJS += \
./Core/Src/Controlling_LED.o \
./Core/Src/GPS.o \
./Core/Src/GSM.o \
./Core/Src/Queue_GSM.o \
./Core/Src/RFID.o \
./Core/Src/RS232-UART1.o \
./Core/Src/RS232-UART2.o \
./Core/Src/RTC.o \
./Core/Src/freertos.o \
./Core/Src/main.o \
./Core/Src/spi_flash.o \
./Core/Src/stm32f3xx_hal_msp.o \
./Core/Src/stm32f3xx_hal_timebase_tim.o \
./Core/Src/stm32f3xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_management.o \
./Core/Src/system_stm32f3xx.o 

C_DEPS += \
./Core/Src/Controlling_LED.d \
./Core/Src/GPS.d \
./Core/Src/GSM.d \
./Core/Src/Queue_GSM.d \
./Core/Src/RFID.d \
./Core/Src/RS232-UART1.d \
./Core/Src/RS232-UART2.d \
./Core/Src/RTC.d \
./Core/Src/freertos.d \
./Core/Src/main.d \
./Core/Src/spi_flash.d \
./Core/Src/stm32f3xx_hal_msp.d \
./Core/Src/stm32f3xx_hal_timebase_tim.d \
./Core/Src/stm32f3xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_management.d \
./Core/Src/system_stm32f3xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xC -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -Oz -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/Controlling_LED.cyclo ./Core/Src/Controlling_LED.d ./Core/Src/Controlling_LED.o ./Core/Src/Controlling_LED.su ./Core/Src/GPS.cyclo ./Core/Src/GPS.d ./Core/Src/GPS.o ./Core/Src/GPS.su ./Core/Src/GSM.cyclo ./Core/Src/GSM.d ./Core/Src/GSM.o ./Core/Src/GSM.su ./Core/Src/Queue_GSM.cyclo ./Core/Src/Queue_GSM.d ./Core/Src/Queue_GSM.o ./Core/Src/Queue_GSM.su ./Core/Src/RFID.cyclo ./Core/Src/RFID.d ./Core/Src/RFID.o ./Core/Src/RFID.su ./Core/Src/RS232-UART1.cyclo ./Core/Src/RS232-UART1.d ./Core/Src/RS232-UART1.o ./Core/Src/RS232-UART1.su ./Core/Src/RS232-UART2.cyclo ./Core/Src/RS232-UART2.d ./Core/Src/RS232-UART2.o ./Core/Src/RS232-UART2.su ./Core/Src/RTC.cyclo ./Core/Src/RTC.d ./Core/Src/RTC.o ./Core/Src/RTC.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/spi_flash.cyclo ./Core/Src/spi_flash.d ./Core/Src/spi_flash.o ./Core/Src/spi_flash.su ./Core/Src/stm32f3xx_hal_msp.cyclo ./Core/Src/stm32f3xx_hal_msp.d ./Core/Src/stm32f3xx_hal_msp.o ./Core/Src/stm32f3xx_hal_msp.su ./Core/Src/stm32f3xx_hal_timebase_tim.cyclo ./Core/Src/stm32f3xx_hal_timebase_tim.d ./Core/Src/stm32f3xx_hal_timebase_tim.o ./Core/Src/stm32f3xx_hal_timebase_tim.su ./Core/Src/stm32f3xx_it.cyclo ./Core/Src/stm32f3xx_it.d ./Core/Src/stm32f3xx_it.o ./Core/Src/stm32f3xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_management.cyclo ./Core/Src/system_management.d ./Core/Src/system_management.o ./Core/Src/system_management.su ./Core/Src/system_stm32f3xx.cyclo ./Core/Src/system_stm32f3xx.d ./Core/Src/system_stm32f3xx.o ./Core/Src/system_stm32f3xx.su

.PHONY: clean-Core-2f-Src

