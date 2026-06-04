# Additional STM32 build rules

# Setup the toolchain
CROSS_PREFIX=arm-none-eabi-

dirs-y += src/stm32 src/generic lib/fast-hash
dirs-$(CONFIG_MACH_STM32F0) += lib/stm32f0
dirs-$(CONFIG_MACH_STM32F1) += lib/stm32f1
dirs-$(CONFIG_MACH_N32G45x) += lib/n32g45x
dirs-$(CONFIG_MACH_STM32F2) += lib/stm32f2
dirs-$(CONFIG_MACH_STM32F4) += lib/stm32f4
dirs-$(CONFIG_MACH_STM32F7) += lib/stm32f7
dirs-$(CONFIG_MACH_STM32G0) += lib/stm32g0
dirs-$(CONFIG_MACH_STM32G4) += lib/stm32g4
dirs-$(CONFIG_MACH_STM32H7) += lib/stm32h7
dirs-$(CONFIG_MACH_STM32L4) += lib/stm32l4

MCU := $(shell echo $(CONFIG_MCU))
MCU_UPPER := $(shell echo $(CONFIG_MCU) | tr a-z A-Z | tr X x)

CFLAGS-$(CONFIG_MACH_STM32F0) += -mcpu=cortex-m0 -Ilib/stm32f0/include
CFLAGS-$(CONFIG_MACH_STM32F103) += -mcpu=cortex-m3
CFLAGS-$(CONFIG_MACH_N32G45x) += -mcpu=cortex-m4 -Ilib/n32g45x/include
CFLAGS-$(CONFIG_MACH_STM32F1) += -Ilib/stm32f1/include
CFLAGS-$(CONFIG_MACH_STM32F2) += -mcpu=cortex-m3 -Ilib/stm32f2/include
CFLAGS-$(CONFIG_MACH_STM32F4) += -mcpu=cortex-m4 -Ilib/stm32f4/include
CFLAGS-$(CONFIG_MACH_STM32F7) += -mcpu=cortex-m7 -Ilib/stm32f7/include
CFLAGS-$(CONFIG_MACH_STM32G0) += -mcpu=cortex-m0plus -Ilib/stm32g0/include
CFLAGS-$(CONFIG_MACH_STM32G4) += -mcpu=cortex-m4 -Ilib/stm32g4/include
CFLAGS-$(CONFIG_MACH_STM32H7) += -mcpu=cortex-m7 -Ilib/stm32h7/include
CFLAGS-$(CONFIG_MACH_STM32L4) += -mcpu=cortex-m4 -Ilib/stm32l4/include
CFLAGS-$(CONFIG_MACH_AT32F403) = -mcpu=cortex-m4 -Ilib/stm32f1/include
CFLAGS += $(CFLAGS-y) -D$(MCU_UPPER) -mthumb -Ilib/cmsis-core -Ilib/fast-hash

CFLAGS += -Irust/kalico-c-api/include

CFLAGS_klipper.elf += -nostdlib -lgcc -lc_nano
CFLAGS_klipper.elf += -T $(OUT)src/generic/armcm_link.ld
$(OUT)klipper.elf: $(OUT)src/generic/armcm_link.ld

# Link libkalico_c_api.a. Place ahead of -lgcc/-lc_nano on the link line —
# archive extraction is demand-driven and order-sensitive (spec §2.4 link-line
# pitfalls). Empirically, prepending to CFLAGS_klipper.elf works because
# Klipper's link rule is `$(CC) $(OBJS) $(CFLAGS_klipper.elf) -o $@` and
# the existing -lgcc tail stays at the end.
#
# --start-group / --end-group: a live Rust function pulled from the archive may
# reference C symbols (e.g. `timer_read_time`, `stats_send_time`) that GNU ld
# has already passed in its single forward sweep. The grouped pair forces ld
# to iterate archives until cross-references settle. Matches src/linux/Makefile.
# KALICO_LIB is set by src/Makefile based on the active MCU (target-h7 vs
# target-f4 cargo target dir); see comments there.
CFLAGS_klipper.elf := -Wl,--start-group $(KALICO_LIB) -Wl,--end-group $(CFLAGS_klipper.elf)
$(OUT)klipper.elf: $(KALICO_LIB)

# Add source files
src-y += stm32/watchdog.c stm32/clockline.c stm32/dfu_reboot.c
src-y += generic/crc16_ccitt.c
src-y += generic/armcm_boot.c generic/armcm_irq.c generic/armcm_reset.c
src-y += generic/fault_handler.c
src-y += generic/mpu_protect.c

# Kalico runtime glue. Per-family motion-timer backend selected by
# CONFIG_MACH_STM32*. H7/F4 use TIM5; G0B1 has no TIM5 and uses a configurable
# basic timer (default TIM7) plus a software CYCCNT (no DWT on Cortex-M0+).
src-$(CONFIG_MACH_STM32H7) += stm32/runtime_tick_h7.c
src-$(CONFIG_MACH_STM32F4) += stm32/runtime_tick_f4.c
src-$(CONFIG_MACH_STM32G0) += stm32/runtime_tick_g0.c
src-y += runtime_tick.c
src-y += runtime_commands.c
# Phase-stepping XDIRECT SPI helper (Step 7-D / phase-stepping sim slice).
# Called from the Rust modulator via FFI; see src/stm32/phase_stepping_spi.h.
src-y += stm32/phase_stepping_spi.c
# Sim-only: software CYCCNT under CONFIG_KALICO_SIM. Per Step-6 spec §3.1.
src-$(CONFIG_KALICO_SIM) += stm32/kalico_sim_clock.c

src-$(CONFIG_MACH_STM32F0) += stm32/stm32f0.c ../lib/stm32f0/system_stm32f0xx.c
src-$(CONFIG_MACH_STM32F1) += stm32/stm32f1.c ../lib/stm32f1/system_stm32f1xx.c
src-$(CONFIG_MACH_STM32F2) += stm32/stm32f4.c ../lib/stm32f2/system_stm32f2xx.c
src-$(CONFIG_MACH_STM32F4) += stm32/stm32f4.c ../lib/stm32f4/system_stm32f4xx.c
src-$(CONFIG_MACH_STM32F7) += stm32/stm32f7.c ../lib/stm32f7/system_stm32f7xx.c
src-$(CONFIG_MACH_STM32G0) += stm32/stm32g0.c
src-$(CONFIG_MACH_STM32G4) += stm32/stm32g4.c ../lib/stm32g4/system_stm32g4xx.c
src-$(CONFIG_MACH_STM32H7) += stm32/stm32h7.c ../lib/stm32h7/system_stm32h7xx.c
src-$(CONFIG_MACH_STM32L4) += stm32/stm32l4.c ../lib/stm32l4/system_stm32l4xx.c

timer-src-y := generic/armcm_timer.c
timer-src-$(CONFIG_MACH_STM32F0) := generic/timer_irq.c stm32/stm32f0_timer.c
timer-src-$(CONFIG_MACH_STM32G0) := generic/timer_irq.c stm32/stm32f0_timer.c
gpio-src-y := stm32/gpio.c stm32/gpioperiph.c
gpio-src-$(CONFIG_MACH_STM32F1) := stm32/gpio.c
gpio-src-$(CONFIG_MACH_STM32H7) := stm32/stm32h7_gpio.c stm32/gpioperiph.c
src-y += $(timer-src-y) $(gpio-src-y)

adc-src-y := stm32/adc.c
adc-src-$(CONFIG_MACH_STM32F0) := stm32/stm32f0_adc.c
adc-src-$(CONFIG_MACH_N32G45x) := ../lib/n32g45x/n32g45x_adc.c stm32/n32g45x_adc.c
adc-src-$(CONFIG_MACH_STM32G0) := stm32/stm32f0_adc.c
adc-src-$(CONFIG_MACH_STM32G4) := stm32/stm32h7_adc.c
adc-src-$(CONFIG_MACH_STM32H7) := stm32/stm32h7_adc.c
adc-src-$(CONFIG_MACH_STM32L4) := stm32/stm32h7_adc.c
src-$(CONFIG_WANT_ADC) += $(adc-src-y)

spi-src-y := stm32/spi.c
spi-src-$(CONFIG_MACH_STM32H7) := stm32/stm32h7_spi.c
src-$(CONFIG_WANT_GPIO_SPI) += $(spi-src-y)

i2c-src-y := stm32/stm32f0_i2c.c
i2c-src-$(CONFIG_MACH_STM32F1) := stm32/i2c.c
i2c-src-$(CONFIG_MACH_STM32F2) := stm32/i2c.c
i2c-src-$(CONFIG_MACH_STM32F4) := stm32/i2c.c
src-$(CONFIG_WANT_GPIO_I2C) += $(i2c-src-y)

serial-src-y := stm32/serial.c
serial-src-$(CONFIG_MACH_STM32F0) := stm32/stm32f0_serial.c
serial-src-$(CONFIG_MACH_STM32G0) := stm32/stm32f0_serial.c
serial-src-$(CONFIG_MACH_STM32G4) := stm32/stm32f0_serial.c
serial-src-$(CONFIG_MACH_STM32H7) := stm32/stm32f0_serial.c
src-$(CONFIG_SERIAL) += $(serial-src-y) generic/serial_irq.c

usb-src-$(CONFIG_HAVE_STM32_USBFS) := stm32/usbfs.c
usb-src-$(CONFIG_HAVE_STM32_USBOTG) := stm32/usbotg.c
src-$(CONFIG_USBSERIAL) += $(usb-src-y) stm32/chipid.c generic/usb_cdc.c

canbus-src-y := generic/canserial.c ../lib/fast-hash/fasthash.c
canbus-src-$(CONFIG_HAVE_STM32_CANBUS) += stm32/can.c
canbus-src-$(CONFIG_HAVE_STM32_FDCANBUS) += stm32/fdcan.c
src-$(CONFIG_CANSERIAL) += $(canbus-src-y) generic/canbus.c stm32/chipid.c
src-$(CONFIG_USBCANBUS) += $(usb-src-y) $(canbus-src-y)
src-$(CONFIG_USBCANBUS) += stm32/chipid.c generic/usb_canbus.c
src-$(CONFIG_WANT_HARD_PWM) += stm32/hard_pwm.c

sdio-src-y := stm32/sdio.c
src-$(CONFIG_HAVE_GPIO_SDIO) += $(sdio-src-y)

# Binary output file rules
target-y += $(OUT)klipper.bin

$(OUT)klipper.bin: $(OUT)klipper.elf
	@echo "  Creating hex file $@"
	$(Q)$(OBJCOPY) -O binary $< $@

# Flash rules
lib/hidflash/hid-flash:
	@echo "  Building hid-flash"
	$(Q)make -C lib/hidflash

flash: $(OUT)klipper.bin lib/hidflash/hid-flash
	@echo "  Flashing $< to $(FLASH_DEVICE)"
	$(Q)$(PYTHON) ./scripts/flash_usb.py -t $(CONFIG_MCU) -d "$(FLASH_DEVICE)" -s "$(CONFIG_FLASH_APPLICATION_ADDRESS)" $(if $(NOSUDO),--no-sudo) $(OUT)klipper.bin

serialflash: $(OUT)klipper.bin
	@echo "  Flashing $< to $(FLASH_DEVICE) via stm32flash"
	$(Q)stm32flash -w $< -v -g 0 $(FLASH_DEVICE)
