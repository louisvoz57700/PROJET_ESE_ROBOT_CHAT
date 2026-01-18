################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/tasks/task_FSM.c \
../Core/Src/tasks/task_comm.c \
../Core/Src/tasks/task_control.c \
../Core/Src/tasks/task_sensor.c 

OBJS += \
./Core/Src/tasks/task_FSM.o \
./Core/Src/tasks/task_comm.o \
./Core/Src/tasks/task_control.o \
./Core/Src/tasks/task_sensor.o 

C_DEPS += \
./Core/Src/tasks/task_FSM.d \
./Core/Src/tasks/task_comm.d \
./Core/Src/tasks/task_control.d \
./Core/Src/tasks/task_sensor.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/tasks/%.o Core/Src/tasks/%.su Core/Src/tasks/%.cyclo: ../Core/Src/tasks/%.c Core/Src/tasks/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/RTOS2/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-tasks

clean-Core-2f-Src-2f-tasks:
	-$(RM) ./Core/Src/tasks/task_FSM.cyclo ./Core/Src/tasks/task_FSM.d ./Core/Src/tasks/task_FSM.o ./Core/Src/tasks/task_FSM.su ./Core/Src/tasks/task_comm.cyclo ./Core/Src/tasks/task_comm.d ./Core/Src/tasks/task_comm.o ./Core/Src/tasks/task_comm.su ./Core/Src/tasks/task_control.cyclo ./Core/Src/tasks/task_control.d ./Core/Src/tasks/task_control.o ./Core/Src/tasks/task_control.su ./Core/Src/tasks/task_sensor.cyclo ./Core/Src/tasks/task_sensor.d ./Core/Src/tasks/task_sensor.o ./Core/Src/tasks/task_sensor.su

.PHONY: clean-Core-2f-Src-2f-tasks

