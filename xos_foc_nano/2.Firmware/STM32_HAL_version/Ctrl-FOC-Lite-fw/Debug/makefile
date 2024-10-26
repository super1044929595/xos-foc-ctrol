################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include startup/subdir.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := Ctrl-FOC-Lite-fw
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Ctrl-FOC-Lite-fw.elf \

MAP_FILES += \
Ctrl-FOC-Lite-fw.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Ctrl-FOC-Lite-fw.list \

OBJCOPY_HEX += \
Ctrl-FOC-Lite-fw.hex \


# All Target
all: main-build

# Main-build Target
main-build: Ctrl-FOC-Lite-fw.elf secondary-outputs

# Tool invocations
Ctrl-FOC-Lite-fw.elf Ctrl-FOC-Lite-fw.map: $(OBJS) $(USER_OBJS) ../STM32F103CBTx_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "Ctrl-FOC-Lite-fw.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"../STM32F103CBTx_FLASH.ld" --specs=nosys.specs -Wl,-Map="Ctrl-FOC-Lite-fw.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Ctrl-FOC-Lite-fw.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Ctrl-FOC-Lite-fw.list"
	@echo 'Finished building: $@'
	@echo ' '

Ctrl-FOC-Lite-fw.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "Ctrl-FOC-Lite-fw.hex"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) Ctrl-FOC-Lite-fw.elf Ctrl-FOC-Lite-fw.hex Ctrl-FOC-Lite-fw.list Ctrl-FOC-Lite-fw.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
