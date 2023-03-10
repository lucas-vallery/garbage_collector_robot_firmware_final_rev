################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Capteur_TOF.c \
../Core/Src/app_freertos.c \
../Core/Src/color_sensor.c \
../Core/Src/control_loop.c \
../Core/Src/drv_timer.c \
../Core/Src/gpio.c \
../Core/Src/i2c.c \
../Core/Src/main.c \
../Core/Src/robot.c \
../Core/Src/shell.c \
../Core/Src/stm32g0xx_hal_msp.c \
../Core/Src/stm32g0xx_hal_timebase_tim.c \
../Core/Src/stm32g0xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g0xx.c \
../Core/Src/tim.c \
../Core/Src/uart_drv.c \
../Core/Src/uart_half_duplex_driver.c \
../Core/Src/usart.c \
../Core/Src/xl320_driver.c \
../Core/Src/zxbm_driver.c 

OBJS += \
./Core/Src/Capteur_TOF.o \
./Core/Src/app_freertos.o \
./Core/Src/color_sensor.o \
./Core/Src/control_loop.o \
./Core/Src/drv_timer.o \
./Core/Src/gpio.o \
./Core/Src/i2c.o \
./Core/Src/main.o \
./Core/Src/robot.o \
./Core/Src/shell.o \
./Core/Src/stm32g0xx_hal_msp.o \
./Core/Src/stm32g0xx_hal_timebase_tim.o \
./Core/Src/stm32g0xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g0xx.o \
./Core/Src/tim.o \
./Core/Src/uart_drv.o \
./Core/Src/uart_half_duplex_driver.o \
./Core/Src/usart.o \
./Core/Src/xl320_driver.o \
./Core/Src/zxbm_driver.o 

C_DEPS += \
./Core/Src/Capteur_TOF.d \
./Core/Src/app_freertos.d \
./Core/Src/color_sensor.d \
./Core/Src/control_loop.d \
./Core/Src/drv_timer.d \
./Core/Src/gpio.d \
./Core/Src/i2c.d \
./Core/Src/main.d \
./Core/Src/robot.d \
./Core/Src/shell.d \
./Core/Src/stm32g0xx_hal_msp.d \
./Core/Src/stm32g0xx_hal_timebase_tim.d \
./Core/Src/stm32g0xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g0xx.d \
./Core/Src/tim.d \
./Core/Src/uart_drv.d \
./Core/Src/uart_half_duplex_driver.d \
./Core/Src/usart.d \
./Core/Src/xl320_driver.d \
./Core/Src/zxbm_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G070xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0 -I"C:/Users/lucas/STM32CubeIDE/workspace_1.9.0/garbage_collector_robot_firmware/Drivers/VL53L0X/core/inc" -I"C:/Users/lucas/STM32CubeIDE/workspace_1.9.0/garbage_collector_robot_firmware/Drivers/VL53L0X/platform/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/Capteur_TOF.d ./Core/Src/Capteur_TOF.o ./Core/Src/Capteur_TOF.su ./Core/Src/app_freertos.d ./Core/Src/app_freertos.o ./Core/Src/app_freertos.su ./Core/Src/color_sensor.d ./Core/Src/color_sensor.o ./Core/Src/color_sensor.su ./Core/Src/control_loop.d ./Core/Src/control_loop.o ./Core/Src/control_loop.su ./Core/Src/drv_timer.d ./Core/Src/drv_timer.o ./Core/Src/drv_timer.su ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/i2c.d ./Core/Src/i2c.o ./Core/Src/i2c.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/robot.d ./Core/Src/robot.o ./Core/Src/robot.su ./Core/Src/shell.d ./Core/Src/shell.o ./Core/Src/shell.su ./Core/Src/stm32g0xx_hal_msp.d ./Core/Src/stm32g0xx_hal_msp.o ./Core/Src/stm32g0xx_hal_msp.su ./Core/Src/stm32g0xx_hal_timebase_tim.d ./Core/Src/stm32g0xx_hal_timebase_tim.o ./Core/Src/stm32g0xx_hal_timebase_tim.su ./Core/Src/stm32g0xx_it.d ./Core/Src/stm32g0xx_it.o ./Core/Src/stm32g0xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g0xx.d ./Core/Src/system_stm32g0xx.o ./Core/Src/system_stm32g0xx.su ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/uart_drv.d ./Core/Src/uart_drv.o ./Core/Src/uart_drv.su ./Core/Src/uart_half_duplex_driver.d ./Core/Src/uart_half_duplex_driver.o ./Core/Src/uart_half_duplex_driver.su ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su ./Core/Src/xl320_driver.d ./Core/Src/xl320_driver.o ./Core/Src/xl320_driver.su ./Core/Src/zxbm_driver.d ./Core/Src/zxbm_driver.o ./Core/Src/zxbm_driver.su

.PHONY: clean-Core-2f-Src

