################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../motors/moteur.c 

OBJS += \
./motors/moteur.o 

C_DEPS += \
./motors/moteur.d 


# Each subdirectory must supply rules for building sources it contributes
motors/%.o motors/%.su motors/%.cyclo: ../motors/%.c motors/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../USB_Device/App -I../USB_Device/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I"C:/Users/knn64/git/PROJET_ESE_ROBOT_CHAT/Hardware/Mainboard/CubeIDE/Robot-Chat/adxl345" -I"C:/Users/knn64/git/PROJET_ESE_ROBOT_CHAT/Hardware/Mainboard/CubeIDE/Robot-Chat/motors" -I"C:/Users/knn64/git/PROJET_ESE_ROBOT_CHAT/Hardware/Mainboard/CubeIDE/Robot-Chat/bm71" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-motors

clean-motors:
	-$(RM) ./motors/moteur.cyclo ./motors/moteur.d ./motors/moteur.o ./motors/moteur.su

.PHONY: clean-motors

