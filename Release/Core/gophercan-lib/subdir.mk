################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/babbo/git/gophercan-lib/GopherCAN.c \
C:/Users/babbo/git/gophercan-lib/GopherCAN_ids.c \
C:/Users/babbo/git/gophercan-lib/GopherCAN_ring_buffer.c 

OBJS += \
./Core/gophercan-lib/GopherCAN.o \
./Core/gophercan-lib/GopherCAN_ids.o \
./Core/gophercan-lib/GopherCAN_ring_buffer.o 

C_DEPS += \
./Core/gophercan-lib/GopherCAN.d \
./Core/gophercan-lib/GopherCAN_ids.d \
./Core/gophercan-lib/GopherCAN_ring_buffer.d 


# Each subdirectory must supply rules for building sources it contributes
Core/gophercan-lib/GopherCAN.o: C:/Users/babbo/git/gophercan-lib/GopherCAN.c Core/gophercan-lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F756xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/gophercan-lib/GopherCAN.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/gophercan-lib/GopherCAN_ids.o: C:/Users/babbo/git/gophercan-lib/GopherCAN_ids.c Core/gophercan-lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F756xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/gophercan-lib/GopherCAN_ids.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/gophercan-lib/GopherCAN_ring_buffer.o: C:/Users/babbo/git/gophercan-lib/GopherCAN_ring_buffer.c Core/gophercan-lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F756xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/gophercan-lib/GopherCAN_ring_buffer.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

