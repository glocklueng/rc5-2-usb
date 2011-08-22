################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: RDB1768cmsis_usb_hid_Keybd2.axf

# Tool invocations
RDB1768cmsis_usb_hid_Keybd2.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"L:\CAS\PSE\Workspaces\Mio\V6\Lib_CMSIS\Debug\Core\CM3\CoreSupport" -L"L:\CAS\PSE\Workspaces\Mio\V6\Lib_CMSIS\Debug\Core\CM3\DeviceSupport\NXP\LPC17xx" -L"L:\CAS\PSE\Workspaces\Mio\V6\Lib_CMSIS\Debug" -L"L:\CAS\PSE\Workspaces\Mio\V6\RDB1768cmsis_usbstack\Debug" -Xlinker --gc-sections -Xlinker -Map=RDB1768cmsis_usb_hid_Keybd2.map -mcpu=cortex-m3 -mthumb -T "RDB1768cmsis_usb_hid_Keybd2_Debug.ld" -o"RDB1768cmsis_usb_hid_Keybd2.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(EXECUTABLES) RDB1768cmsis_usb_hid_Keybd2.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size RDB1768cmsis_usb_hid_Keybd2.axf; arm-none-eabi-objcopy -O binary RDB1768cmsis_usb_hid_Keybd2.axf RDB1768cmsis_usb_hid_Keybd2.bin ; checksum -p LPC1769 -d RDB1768cmsis_usb_hid_Keybd2.bin;
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
