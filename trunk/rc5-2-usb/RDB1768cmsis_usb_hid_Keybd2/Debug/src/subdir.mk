################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/RC5.c \
../src/cr_startup_lpc176x.c \
../src/main_kbd.c 

OBJS += \
./src/RC5.o \
./src/cr_startup_lpc176x.o \
./src/main_kbd.o 

C_DEPS += \
./src/RC5.d \
./src/cr_startup_lpc176x.d \
./src/main_kbd.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DDEBUG -D__USE_CMSIS=CMSISv1p30_LPC17xx -D__CODE_RED -D__REDLIB__ -I"L:\CAS\PSE\Workspaces\Mio\V6\Lib_CMSIS\Core\CM3\CoreSupport" -I"L:\CAS\PSE\Workspaces\Mio\V6\Lib_CMSIS\Core\CM3\DeviceSupport\NXP\LPC17xx" -I"L:\CAS\PSE\Workspaces\Mio\V6\Lib_CMSIS\Drivers\include" -I"L:\CAS\PSE\Workspaces\Mio\V6\RDB1768cmsis_usbstack\inc" -O0 -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -mcpu=cortex-m3 -mthumb -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/cr_startup_lpc176x.o: ../src/cr_startup_lpc176x.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DDEBUG -D__USE_CMSIS=CMSISv1p30_LPC17xx -D__CODE_RED -D__REDLIB__ -I"L:\CAS\PSE\Workspaces\Mio\V6\Lib_CMSIS\Core\CM3\CoreSupport" -I"L:\CAS\PSE\Workspaces\Mio\V6\Lib_CMSIS\Core\CM3\DeviceSupport\NXP\LPC17xx" -I"L:\CAS\PSE\Workspaces\Mio\V6\Lib_CMSIS\Drivers\include" -I"L:\CAS\PSE\Workspaces\Mio\V6\RDB1768cmsis_usbstack\inc" -O0 -Os -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -mcpu=cortex-m3 -mthumb -MMD -MP -MF"$(@:%.o=%.d)" -MT"src/cr_startup_lpc176x.d" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


