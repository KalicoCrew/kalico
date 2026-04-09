CROSS_PREFIX=arm-none-eabi-

dirs-y += src/gd32 src/generic lib/fast-hash
dirs-$(CONFIG_MACH_GD32F30X) += lib/gd32f30x
dirs-$(CONFIG_MACH_GD32E23X) += lib/gd32e23x

MCU := $(shell echo $(CONFIG_MCU))
MCU_UPPER := $(shell echo $(CONFIG_MCU) | tr a-z A-Z | tr X x)

CFLAGS-$(CONFIG_MACH_GD32F30X_HD) += -DGD32F30X_HD
CFLAGS-$(CONFIG_MACH_GD32F30X_XD) += -DGD32F30X_XD
CFLAGS-$(CONFIG_MACH_GD32F30X_CL) += -DGD32F30X_CL
CFLAGS-$(CONFIG_MACH_GD32F30X) += -mcpu=cortex-m4
CFLAGS-$(CONFIG_MACH_GD32F30X) += -Ilib/stm32f1/include -Ilib/gd32f30x
CFLAGS-$(CONFIG_MACH_GD32F30X) += -D$(MCU_UPPER) -DUSE_STDPERIPH_DRIVER
CFLAGS-$(CONFIG_MACH_GD32E23X) += -mcpu=cortex-m23 -DSTM32F051x8
CFLAGS-$(CONFIG_MACH_GD32E23X) += -Ilib/stm32f0/include -Ilib/gd32e23x
CFLAGS += $(CFLAGS-y) -mthumb -Ilib/cmsis-core -Ilib/fast-hash

CFLAGS_klipper.elf += -nostdlib -lgcc -lc_nano
CFLAGS_klipper.elf += -T $(OUT)src/generic/armcm_link.ld
$(OUT)klipper.elf: $(OUT)src/generic/armcm_link.ld

src-y += generic/armcm_boot.c generic/armcm_irq.c generic/armcm_reset.c
src-y += generic/crc16_ccitt.c
src-$(CONFIG_MACH_GD32F30X) += generic/armcm_timer.c
src-$(CONFIG_MACH_GD32E23X) += generic/timer_irq.c gd32/timer.c
src-$(CONFIG_MACH_GD32F30X) += gd32/gd32f30x.c
src-$(CONFIG_MACH_GD32E23X) += gd32/gd32e23x.c
src-y += gd32/gpio.c gd32/clockline.c gd32/watchdog.c
src-$(CONFIG_WANT_ADC) += gd32/adc.c
src-$(CONFIG_WANT_GPIO_SPI) += gd32/spi.c
src-$(CONFIG_WANT_GPIO_I2C) += gd32/i2c.c
src-$(CONFIG_SERIAL) += gd32/serial.c generic/serial_irq.c
src-$(CONFIG_USBSERIAL) += gd32/usbfs.c gd32/chipid.c generic/usb_cdc.c
src-$(CONFIG_WANT_HARD_PWM) += gd32/hard_pwm.c

target-y += $(OUT)klipper.bin

$(OUT)klipper.bin: $(OUT)klipper.elf
	@echo "  Creating bin file $@"
	$(Q)$(OBJCOPY) -O binary $< $@

serialflash: $(OUT)klipper.bin
	@echo "  Flashing $< to $(FLASH_DEVICE) via stm32flash"
	$(Q)stm32flash -w $< -v -g 0 $(FLASH_DEVICE)
