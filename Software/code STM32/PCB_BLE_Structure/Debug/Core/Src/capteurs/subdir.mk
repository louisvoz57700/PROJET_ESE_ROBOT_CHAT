################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/capteurs/VL53L0X.c \
../Core/Src/capteurs/lidar.c \
../Core/Src/capteurs/multiplexer.c \
../Core/Src/capteurs/sensor.c 

OBJS += \
./Core/Src/capteurs/VL53L0X.o \
./Core/Src/capteurs/lidar.o \
./Core/Src/capteurs/multiplexer.o \
./Core/Src/capteurs/sensor.o 

C_DEPS += \
./Core/Src/capteurs/VL53L0X.d \
./Core/Src/capteurs/lidar.d \
./Core/Src/capteurs/multiplexer.d \
./Core/Src/capteurs/sensor.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/capteurs/%.o Core/Src/capteurs/%.su Core/Src/capteurs/%.cyclo: ../Core/Src/capteurs/%.c Core/Src/capteurs/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/RTOS2/Include -I../STM32_WPAN/App -I../Utilities/lpm/tiny_lpm -I../Middlewares/ST/STM32_WPAN -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci -I../Middlewares/ST/STM32_WPAN/utilities -I../Middlewares/ST/STM32_WPAN/ble/core -I../Middlewares/ST/STM32_WPAN/ble/core/auto -I../Middlewares/ST/STM32_WPAN/ble/core/template -I../Middlewares/ST/STM32_WPAN/ble/svc/Inc -I../Middlewares/ST/STM32_WPAN/ble/svc/Src -I../Middlewares/ST/STM32_WPAN/ble -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-capteurs

clean-Core-2f-Src-2f-capteurs:
	-$(RM) ./Core/Src/capteurs/VL53L0X.cyclo ./Core/Src/capteurs/VL53L0X.d ./Core/Src/capteurs/VL53L0X.o ./Core/Src/capteurs/VL53L0X.su ./Core/Src/capteurs/lidar.cyclo ./Core/Src/capteurs/lidar.d ./Core/Src/capteurs/lidar.o ./Core/Src/capteurs/lidar.su ./Core/Src/capteurs/multiplexer.cyclo ./Core/Src/capteurs/multiplexer.d ./Core/Src/capteurs/multiplexer.o ./Core/Src/capteurs/multiplexer.su ./Core/Src/capteurs/sensor.cyclo ./Core/Src/capteurs/sensor.d ./Core/Src/capteurs/sensor.o ./Core/Src/capteurs/sensor.su

.PHONY: clean-Core-2f-Src-2f-capteurs

