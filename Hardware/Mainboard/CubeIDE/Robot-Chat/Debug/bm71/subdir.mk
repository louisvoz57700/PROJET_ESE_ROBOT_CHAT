################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bm71/bm71_driver.c 

OBJS += \
./bm71/bm71_driver.o 

C_DEPS += \
./bm71/bm71_driver.d 


# Each subdirectory must supply rules for building sources it contributes
bm71/%.o bm71/%.su bm71/%.cyclo: ../bm71/%.c bm71/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../USB_Device/App -I../USB_Device/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I"C:/Users/knn64/git/PROJET_ESE_ROBOT_CHAT/Hardware/Mainboard/CubeIDE/Robot-Chat/adxl345" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/knn64/git/PROJET_ESE_ROBOT_CHAT/Hardware/Mainboard/CubeIDE/Robot-Chat/motors" -I"C:/Users/knn64/git/PROJET_ESE_ROBOT_CHAT/Hardware/Mainboard/CubeIDE/Robot-Chat/bm71" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-bm71

clean-bm71:
	-$(RM) ./bm71/bm71_driver.cyclo ./bm71/bm71_driver.d ./bm71/bm71_driver.o ./bm71/bm71_driver.su

.PHONY: clean-bm71

