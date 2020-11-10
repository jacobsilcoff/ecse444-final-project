################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32L4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
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

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
ecse444-final-project.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
ecse444-final-project.list \

OBJCOPY_BIN += \
ecse444-final-project.bin \


# All Target
all: ecse444-final-project.elf secondary-outputs

# Tool invocations
ecse444-final-project.elf: $(OBJS) $(USER_OBJS) /home/jacob/school/444ecse/ecse444-final-project/STM32L475VGTX_FLASH.ld
	arm-none-eabi-gcc -o "ecse444-final-project.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/home/jacob/school/444ecse/ecse444-final-project/STM32L475VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="ecse444-final-project.map" -Wl,--gc-sections -static -L"/home/jacob/school/444ecse/ecse444-final-project/Core/Lib" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

ecse444-final-project.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "ecse444-final-project.list"
	@echo 'Finished building: $@'
	@echo ' '

ecse444-final-project.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "ecse444-final-project.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
