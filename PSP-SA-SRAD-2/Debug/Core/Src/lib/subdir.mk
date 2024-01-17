################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/lib/W25Q.c \
../Core/Src/lib/bme280.c \
../Core/Src/lib/bmp280.c \
../Core/Src/lib/gps.c \
../Core/Src/lib/mpu6050.c 

OBJS += \
./Core/Src/lib/W25Q.o \
./Core/Src/lib/bme280.o \
./Core/Src/lib/bmp280.o \
./Core/Src/lib/gps.o \
./Core/Src/lib/mpu6050.o 

C_DEPS += \
./Core/Src/lib/W25Q.d \
./Core/Src/lib/bme280.d \
./Core/Src/lib/bmp280.d \
./Core/Src/lib/gps.d \
./Core/Src/lib/mpu6050.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/lib/%.o Core/Src/lib/%.su Core/Src/lib/%.cyclo: ../Core/Src/lib/%.c Core/Src/lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-lib

clean-Core-2f-Src-2f-lib:
	-$(RM) ./Core/Src/lib/W25Q.cyclo ./Core/Src/lib/W25Q.d ./Core/Src/lib/W25Q.o ./Core/Src/lib/W25Q.su ./Core/Src/lib/bme280.cyclo ./Core/Src/lib/bme280.d ./Core/Src/lib/bme280.o ./Core/Src/lib/bme280.su ./Core/Src/lib/bmp280.cyclo ./Core/Src/lib/bmp280.d ./Core/Src/lib/bmp280.o ./Core/Src/lib/bmp280.su ./Core/Src/lib/gps.cyclo ./Core/Src/lib/gps.d ./Core/Src/lib/gps.o ./Core/Src/lib/gps.su ./Core/Src/lib/mpu6050.cyclo ./Core/Src/lib/mpu6050.d ./Core/Src/lib/mpu6050.o ./Core/Src/lib/mpu6050.su

.PHONY: clean-Core-2f-Src-2f-lib

