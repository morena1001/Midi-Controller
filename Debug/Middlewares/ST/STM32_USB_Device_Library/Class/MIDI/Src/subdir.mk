################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/ST/STM32_USB_Device_Library/Class/MIDI/Src/usbd_midi.c 

OBJS += \
./Middlewares/ST/STM32_USB_Device_Library/Class/MIDI/Src/usbd_midi.o 

C_DEPS += \
./Middlewares/ST/STM32_USB_Device_Library/Class/MIDI/Src/usbd_midi.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ST/STM32_USB_Device_Library/Class/MIDI/Src/%.o Middlewares/ST/STM32_USB_Device_Library/Class/MIDI/Src/%.su Middlewares/ST/STM32_USB_Device_Library/Class/MIDI/Src/%.cyclo: ../Middlewares/ST/STM32_USB_Device_Library/Class/MIDI/Src/%.c Middlewares/ST/STM32_USB_Device_Library/Class/MIDI/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/HID/Inc -I"C:/Users/josue/STM32CubeIDE/workspace_1.16.0/Blue Pill USB Test/Middlewares/ST/STM32_USB_Device_Library/Class/MIDI" -I"C:/Users/josue/STM32CubeIDE/workspace_1.16.0/Blue Pill USB Test/Middlewares/ST/STM32_USB_Device_Library/Class/MIDI/Src" -I"C:/Users/josue/STM32CubeIDE/workspace_1.16.0/Blue Pill USB Test/Middlewares/ST/STM32_USB_Device_Library/Class/MIDI/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Middlewares-2f-ST-2f-STM32_USB_Device_Library-2f-Class-2f-MIDI-2f-Src

clean-Middlewares-2f-ST-2f-STM32_USB_Device_Library-2f-Class-2f-MIDI-2f-Src:
	-$(RM) ./Middlewares/ST/STM32_USB_Device_Library/Class/MIDI/Src/usbd_midi.cyclo ./Middlewares/ST/STM32_USB_Device_Library/Class/MIDI/Src/usbd_midi.d ./Middlewares/ST/STM32_USB_Device_Library/Class/MIDI/Src/usbd_midi.o ./Middlewares/ST/STM32_USB_Device_Library/Class/MIDI/Src/usbd_midi.su

.PHONY: clean-Middlewares-2f-ST-2f-STM32_USB_Device_Library-2f-Class-2f-MIDI-2f-Src

