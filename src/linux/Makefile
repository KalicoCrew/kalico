# Additional linux build rules

dirs-y += src/linux src/generic

src-y += linux/main.c linux/timer.c linux/console.c linux/watchdog.c
src-y += linux/pca9685.c linux/spidev.c linux/analog.c linux/hard_pwm.c
src-$(CONFIG_KALICO_SIM_TMCUART_BYPASS) += linux/sim_chip_socket.c
src-y += linux/i2c.c linux/gpio.c generic/crc16_ccitt.c generic/alloc.c
src-y += linux/sensor_ds18b20.c

# Kalico host-tick driver — pthread-based modulation tick replacing TIM5
# on the host build. Provides the kalico_h7_* symbol surface that the
# Rust runtime's producer-protocol calls across the FFI boundary.
src-$(CONFIG_KALICO_RUNTIME) += linux/runtime_tick_host.c
src-$(CONFIG_KALICO_RUNTIME) += runtime_tick.c
src-$(CONFIG_KALICO_RUNTIME) += runtime_commands.c

# Generated FFI header (cbindgen) shared with the H7 build.
ifeq ($(CONFIG_KALICO_RUNTIME),y)
CFLAGS += -Irust/kalico-c-api/include
endif

# Link libkalico_c_api.a. The Linux build has a mutual-callback edge:
# host_tick.o calls into libkalico (kalico_runtime_tick) and libkalico
# calls back out (runtime_tick_enable / read_cyccnt). Wrap with
# --start-group/--end-group so ld iterates until both sides resolve.
ifeq ($(CONFIG_KALICO_RUNTIME),y)
KALICO_LIB := rust/target/release/libkalico_c_api.a
CFLAGS_klipper.elf := -Wl,--start-group $(KALICO_LIB) -Wl,--end-group $(CFLAGS_klipper.elf)
$(OUT)klipper.elf: $(KALICO_LIB)
endif

CFLAGS_klipper.elf += -lutil -lrt -lpthread

flash: $(OUT)klipper.elf
	@echo "  Flashing"
	$(Q)sudo ./scripts/flash-linux.sh $(OUT)
