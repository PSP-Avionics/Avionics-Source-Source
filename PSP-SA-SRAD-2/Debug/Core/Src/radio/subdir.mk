################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/radio/RF95.c \
../Core/Src/radio/RF95_FSK.c \
../Core/Src/radio/rfm96.c \
../Core/Src/radio/telemetry.c 

OBJS += \
./Core/Src/radio/RF95.o \
./Core/Src/radio/RF95_FSK.o \
./Core/Src/radio/rfm96.o \
./Core/Src/radio/telemetry.o 

C_DEPS += \
./Core/Src/radio/RF95.d \
./Core/Src/radio/RF95_FSK.d \
./Core/Src/radio/rfm96.d \
./Core/Src/radio/telemetry.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/radio/%.o Core/Src/radio/%.su Core/Src/radio/%.cyclo: ../Core/Src/radio/%.c Core/Src/radio/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-radio

clean-Core-2f-Src-2f-radio:
	-$(RM) ./Core/Src/radio/RF95.cyclo ./Core/Src/radio/RF95.d ./Core/Src/radio/RF95.o ./Core/Src/radio/RF95.su ./Core/Src/radio/RF95_FSK.cyclo ./Core/Src/radio/RF95_FSK.d ./Core/Src/radio/RF95_FSK.o ./Core/Src/radio/RF95_FSK.su ./Core/Src/radio/rfm96.cyclo ./Core/Src/radio/rfm96.d ./Core/Src/radio/rfm96.o ./Core/Src/radio/rfm96.su ./Core/Src/radio/telemetry.cyclo ./Core/Src/radio/telemetry.d ./Core/Src/radio/telemetry.o ./Core/Src/radio/telemetry.su

.PHONY: clean-Core-2f-Src-2f-radio

