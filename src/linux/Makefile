# Additional linux build rules

dirs-y += src/linux src/generic

src-y += linux/main.c linux/timer.c linux/console.c linux/watchdog.c
src-y += linux/pca9685.c linux/spidev.c linux/analog.c linux/hard_pwm.c
src-y += linux/i2c.c linux/gpio.c generic/crc16_ccitt.c generic/alloc.c
src-y += linux/sensor_ds18b20.c

# Kalico host-tick driver — pthread-based modulation tick replacing TIM5
# on the host build. Provides the kalico_h7_* symbol surface that the
# Rust runtime's producer-protocol calls across the FFI boundary.
src-$(CONFIG_KALICO_RUNTIME) += linux/kalico_host_tick.c
src-$(CONFIG_KALICO_RUNTIME) += runtime_tick.c

# Link libkalico_c_api.a (host-target staticlib).
ifeq ($(CONFIG_KALICO_RUNTIME),y)
KALICO_LIB := rust/target/release/libkalico_c_api.a
CFLAGS_klipper.elf := $(KALICO_LIB) $(CFLAGS_klipper.elf)
$(OUT)klipper.elf: $(KALICO_LIB)
endif

CFLAGS_klipper.elf += -lutil -lrt -lpthread

flash: $(OUT)klipper.elf
	@echo "  Flashing"
	$(Q)sudo ./scripts/flash-linux.sh $(OUT)
