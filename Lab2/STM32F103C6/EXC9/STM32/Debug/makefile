################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
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


BUILD_ARTIFACT_NAME := VXL_VDK_lab2_exc9
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
VXL_VDK_lab2_exc9.elf \

MAP_FILES += \
VXL_VDK_lab2_exc9.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
VXL_VDK_lab2_exc9.list \

OBJCOPY_HEX += \
VXL_VDK_lab2_exc9.hex \

OBJCOPY_BIN += \
VXL_VDK_lab2_exc9.bin \


# All Target
all: main-build

# Main-build Target
main-build: VXL_VDK_lab2_exc9.elf secondary-outputs

# Tool invocations
VXL_VDK_lab2_exc9.elf VXL_VDK_lab2_exc9.map: $(OBJS) $(USER_OBJS) C:\Users\taiko\HCMUT\K251\CO3009_VXL_VDK\LAB\Lab2\STM32F103C6\EXC9\STM32\STM32F103C6UX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "VXL_VDK_lab2_exc9.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\taiko\HCMUT\K251\CO3009_VXL_VDK\LAB\Lab2\STM32F103C6\EXC9\STM32\STM32F103C6UX_FLASH.ld" --specs=nosys.specs -Wl,-Map="VXL_VDK_lab2_exc9.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

VXL_VDK_lab2_exc9.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "VXL_VDK_lab2_exc9.list"
	@echo 'Finished building: $@'
	@echo ' '

VXL_VDK_lab2_exc9.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "VXL_VDK_lab2_exc9.hex"
	@echo 'Finished building: $@'
	@echo ' '

VXL_VDK_lab2_exc9.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "VXL_VDK_lab2_exc9.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) VXL_VDK_lab2_exc9.bin VXL_VDK_lab2_exc9.elf VXL_VDK_lab2_exc9.hex VXL_VDK_lab2_exc9.list VXL_VDK_lab2_exc9.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
