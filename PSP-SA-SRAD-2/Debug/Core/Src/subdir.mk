################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/accelerometer.c \
../Core/Src/active_controls.c \
../Core/Src/altimeter.c \
../Core/Src/altimeter_old.c \
../Core/Src/audio.c \
../Core/Src/flight.c \
../Core/Src/fs_storage.c \
../Core/Src/main.c \
../Core/Src/prometheus.c \
../Core/Src/smintf.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/tracking.c 

OBJS += \
./Core/Src/accelerometer.o \
./Core/Src/active_controls.o \
./Core/Src/altimeter.o \
./Core/Src/altimeter_old.o \
./Core/Src/audio.o \
./Core/Src/flight.o \
./Core/Src/fs_storage.o \
./Core/Src/main.o \
./Core/Src/prometheus.o \
./Core/Src/smintf.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/tracking.o 

C_DEPS += \
./Core/Src/accelerometer.d \
./Core/Src/active_controls.d \
./Core/Src/altimeter.d \
./Core/Src/altimeter_old.d \
./Core/Src/audio.d \
./Core/Src/flight.d \
./Core/Src/fs_storage.d \
./Core/Src/main.d \
./Core/Src/prometheus.d \
./Core/Src/smintf.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/tracking.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/accelerometer.cyclo ./Core/Src/accelerometer.d ./Core/Src/accelerometer.o ./Core/Src/accelerometer.su ./Core/Src/active_controls.cyclo ./Core/Src/active_controls.d ./Core/Src/active_controls.o ./Core/Src/active_controls.su ./Core/Src/altimeter.cyclo ./Core/Src/altimeter.d ./Core/Src/altimeter.o ./Core/Src/altimeter.su ./Core/Src/altimeter_old.cyclo ./Core/Src/altimeter_old.d ./Core/Src/altimeter_old.o ./Core/Src/altimeter_old.su ./Core/Src/audio.cyclo ./Core/Src/audio.d ./Core/Src/audio.o ./Core/Src/audio.su ./Core/Src/flight.cyclo ./Core/Src/flight.d ./Core/Src/flight.o ./Core/Src/flight.su ./Core/Src/fs_storage.cyclo ./Core/Src/fs_storage.d ./Core/Src/fs_storage.o ./Core/Src/fs_storage.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/prometheus.cyclo ./Core/Src/prometheus.d ./Core/Src/prometheus.o ./Core/Src/prometheus.su ./Core/Src/smintf.cyclo ./Core/Src/smintf.d ./Core/Src/smintf.o ./Core/Src/smintf.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/tracking.cyclo ./Core/Src/tracking.d ./Core/Src/tracking.o ./Core/Src/tracking.su

.PHONY: clean-Core-2f-Src

