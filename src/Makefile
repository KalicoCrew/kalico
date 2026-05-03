# Main code build rules

src-y += sched.c command.c basecmd.c debugcmds.c
src-$(CONFIG_HAVE_GPIO) += initial_pins.c gpiocmds.c stepper.c endstop.c \
    trsync.c
src-$(CONFIG_WANT_ADC) += adccmds.c
src-$(CONFIG_WANT_SPI) += spicmds.c
src-$(CONFIG_WANT_I2C) += i2ccmds.c
src-$(CONFIG_WANT_HARD_PWM) += pwmcmds.c
src-$(CONFIG_HAVE_GPIO_SDIO) += sdiocmds.c

src-$(CONFIG_WANT_BUTTONS) += buttons.c
src-$(CONFIG_WANT_TMCUART) += tmcuart.c
src-$(CONFIG_WANT_NEOPIXEL) += neopixel.c
src-$(CONFIG_WANT_PULSE_COUNTER) += pulse_counter.c
src-$(CONFIG_WANT_ST7920) += lcd_st7920.c
src-$(CONFIG_WANT_HD44780) += lcd_hd44780.c
src-$(CONFIG_WANT_SOFTWARE_SPI) += spi_software.c
src-$(CONFIG_WANT_SOFTWARE_I2C) += i2c_software.c
src-$(CONFIG_WANT_THERMOCOUPLE) += thermocouple.c
src-$(CONFIG_WANT_ADXL345) += sensor_adxl345.c
src-$(CONFIG_WANT_LIS2DW) += sensor_lis2dw.c
src-$(CONFIG_WANT_LDC1612) += sensor_ldc1612.c
src-$(CONFIG_WANT_MPU9250) += sensor_mpu9250.c
src-$(CONFIG_WANT_ICM20948) += sensor_icm20948.c
src-$(CONFIG_WANT_HX71X) += sensor_hx71x.c
src-$(CONFIG_WANT_ADS1220) += sensor_ads1220.c
src-$(CONFIG_WANT_ADS131M0X) += sensor_ads131m0x.c
src-$(CONFIG_WANT_SENSOR_ANGLE) += sensor_angle.c
src-$(CONFIG_NEED_SENSOR_BULK) += sensor_bulk.c
src-$(CONFIG_NEED_SOS_FILTER) += sos_filter.c
src-$(CONFIG_NEED_LOAD_CELL_PROBE) += load_cell_probe.c

# Kalico Rust staticlib build hook. The C-side dependency on the staticlib
# is declared in src/stm32/Makefile (klipper.elf: $(KALICO_LIB)); this rule
# tells Make how to build the staticlib in the first place. FORCE ensures
# cargo is invoked every build — cargo itself handles up-to-date checks
# (incremental compilation), so this is cheap on no-op builds.
ifeq ($(CONFIG_KALICO_RUNTIME),y)
ifeq ($(CONFIG_MACH_STM32H7),y)
# Production firmware: thumbv7em + f32 mcu-h7 features. Sim build
# (CONFIG_KALICO_SIM=y) appends `kalico-sim` to compile in the sim-only
# fixture path (sim_fixtures.rs + kalico_runtime_load_fixture FFI), per
# Step-6 plan Phase 0 Task 0.2. Never include in production firmware —
# the fixture path skips curve validation; it's safe only under sim.
KALICO_RUST_FEATURES := mcu-h7,header-nurbs,header-runtime
ifeq ($(CONFIG_KALICO_SIM),y)
KALICO_RUST_FEATURES := $(KALICO_RUST_FEATURES),kalico-sim
endif

rust/target/thumbv7em-none-eabi/release/libkalico_c_api.a: FORCE
	cd rust && PATH="$(HOME)/.cargo/bin:$$PATH" cargo build -p kalico-c-api \
		--no-default-features \
		--features $(KALICO_RUST_FEATURES) \
		--target thumbv7em-none-eabi --release
endif

ifeq ($(CONFIG_MACH_LINUX),y)
# Host build: f64 host features for klippy-in-loop integration testing.
# Always includes `kalico-sim` (the Linux build is itself a sim variant).
KALICO_RUST_FEATURES := host,header-nurbs,header-runtime,kalico-sim

rust/target/release/libkalico_c_api.a: FORCE
	cd rust && PATH="$(HOME)/.cargo/bin:$$PATH" cargo build -p kalico-c-api \
		--no-default-features \
		--features $(KALICO_RUST_FEATURES) \
		--release
endif

.PHONY: FORCE
endif
