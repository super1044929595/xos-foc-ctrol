################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS/DSP/Examples/ARM/arm_variance_example/arm_variance_example_f32.c 

OBJS += \
./Drivers/CMSIS/DSP/Examples/ARM/arm_variance_example/arm_variance_example_f32.o 

C_DEPS += \
./Drivers/CMSIS/DSP/Examples/ARM/arm_variance_example/arm_variance_example_f32.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/DSP/Examples/ARM/arm_variance_example/%.o Drivers/CMSIS/DSP/Examples/ARM/arm_variance_example/%.su Drivers/CMSIS/DSP/Examples/ARM/arm_variance_example/%.cyclo: ../Drivers/CMSIS/DSP/Examples/ARM/arm_variance_example/%.c Drivers/CMSIS/DSP/Examples/ARM/arm_variance_example/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS-2f-DSP-2f-Examples-2f-ARM-2f-arm_variance_example

clean-Drivers-2f-CMSIS-2f-DSP-2f-Examples-2f-ARM-2f-arm_variance_example:
	-$(RM) ./Drivers/CMSIS/DSP/Examples/ARM/arm_variance_example/arm_variance_example_f32.cyclo ./Drivers/CMSIS/DSP/Examples/ARM/arm_variance_example/arm_variance_example_f32.d ./Drivers/CMSIS/DSP/Examples/ARM/arm_variance_example/arm_variance_example_f32.o ./Drivers/CMSIS/DSP/Examples/ARM/arm_variance_example/arm_variance_example_f32.su

.PHONY: clean-Drivers-2f-CMSIS-2f-DSP-2f-Examples-2f-ARM-2f-arm_variance_example
