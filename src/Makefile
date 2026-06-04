# Main code build rules

src-y += sched.c command.c basecmd.c debugcmds.c
# Kalico-native transport — stream-level demuxer + Layer 4 dispatcher.
# Spec: docs/superpowers/specs/2026-05-04-kalico-native-transport-design.md.
src-y += kalico_demux.c kalico_dispatch.c \
    runtime_storage.c runtime_panic.c step_queue.c \
    spi_queue.c kalico_log.c
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
# Production firmware: thumbv7em + per-MCU-family Rust features. Sim build
# (CONFIG_KALICO_SIM=y) appends `kalico-sim` to keep MCU-style tick paths
# active under the sim clock. Never in production firmware.
ifeq ($(CONFIG_MACH_STM32H7),y)
KALICO_RUST_FEATURES := mcu-h7,header-nurbs,header-runtime
# Per-MCU cargo target dir. H7 and F4 both use the thumbv7em-none-eabi
# rustc triple but emit different `#[link_section]` attributes (`.axi_bss`
# vs `.bss` for RT_CELL); a shared incremental cache forces a full rebuild
# on every MCU switch (~minutes on the Pi). Separate dirs keep both caches
# warm so each MCU switch is incremental, not from-scratch.
KALICO_CARGO_TARGET_DIR := target-h7
KALICO_RUST_TARGET := thumbv7em-none-eabi
else ifeq ($(CONFIG_MACH_STM32F4),y)
KALICO_RUST_FEATURES := mcu-f4,header-nurbs,header-runtime
KALICO_CARGO_TARGET_DIR := target-f4
KALICO_RUST_TARGET := thumbv7em-none-eabi
else ifeq ($(CONFIG_MACH_STM32G0),y)
# STM32G0B1 (Cortex-M0+) = ARMv6-M = thumbv6m-none-eabi, a DIFFERENT rustc
# triple than H7/F4. The thumbv6m target build sets
# `--cfg portable_atomic_unsafe_assume_single_core` in rust/.cargo/config.toml
# because ARMv6-M has no LDREX/STREX (see the design record). Own target dir
# keeps the thumbv6m cache from colliding with the thumbv7em caches.
KALICO_RUST_FEATURES := mcu-g0,header-nurbs,header-runtime
KALICO_CARGO_TARGET_DIR := target-g0
KALICO_RUST_TARGET := thumbv6m-none-eabi
endif
ifeq ($(CONFIG_KALICO_SIM),y)
KALICO_RUST_FEATURES := $(KALICO_RUST_FEATURES),kalico-sim
endif
ifeq ($(CONFIG_MOTION_MODULE_STEPPER),y)
KALICO_RUST_FEATURES := $(KALICO_RUST_FEATURES),motion-module-stepper
endif

ifneq (,$(filter y,$(CONFIG_MACH_STM32H7) $(CONFIG_MACH_STM32F4) $(CONFIG_MACH_STM32G0)))
# Exported so src/stm32/Makefile depends on (and links against) the right
# archive without re-deriving the path. KALICO_RUST_TARGET is the rustc
# triple for the active MCU (thumbv7em for H7/F4, thumbv6m for G0).
KALICO_LIB := rust/$(KALICO_CARGO_TARGET_DIR)/$(KALICO_RUST_TARGET)/release/libkalico_c_api.a

# Per-MCU target dir gives H7 and F4 their own caches (see above). Within
# a single MCU's dir, the kalico-sim feature toggle is the only remaining
# feature-set change cargo sees; the stamp guard catches the rare case
# where cargo's incremental cache misses a feature flip and forces a
# targeted clean of just the affected crates.
$(KALICO_LIB): FORCE
	cd rust && PATH="$(HOME)/.cargo/bin:$$PATH" && \
		export CARGO_TARGET_DIR="$(KALICO_CARGO_TARGET_DIR)" && \
		mkdir -p "$$CARGO_TARGET_DIR" && \
		PREV=$$(cat "$$CARGO_TARGET_DIR/.kalico-features-stamp" 2>/dev/null || echo ""); \
		if [ "$$PREV" != "$(KALICO_RUST_FEATURES)" ]; then \
			echo "kalico-c-api features changed ($$PREV -> $(KALICO_RUST_FEATURES)); cargo clean"; \
			cargo clean -p kalico-c-api 2>/dev/null || true; \
			cargo clean -p kalico-host-rt 2>/dev/null || true; \
			cargo clean -p runtime 2>/dev/null || true; \
			cargo clean -p nurbs 2>/dev/null || true; \
		fi && \
		KALICO_RUNTIME_STORAGE_SIZE=$(if $(filter y,$(CONFIG_RUNTIME_TARGET_LARGE)),$(CONFIG_RUNTIME_STORAGE_SIZE_LARGE),$(CONFIG_RUNTIME_STORAGE_SIZE_SMALL)) \
		KALICO_RUNTIME_PIECE_RING_SIZE=$(CONFIG_RUNTIME_PIECE_RING_SIZE) \
		KALICO_RUNTIME_SAMPLE_RATE_HZ=$(CONFIG_KALICO_MOTION_SAMPLE_RATE_HZ) \
		cargo build -p kalico-c-api \
		--no-default-features \
		--features $(KALICO_RUST_FEATURES) \
		--target $(KALICO_RUST_TARGET) --release && \
		echo "$(KALICO_RUST_FEATURES)" > "$$CARGO_TARGET_DIR/.kalico-features-stamp"
endif

ifeq ($(CONFIG_MACH_LINUX),y)
# First-class Linux MCU build. The `mcu-linux` feature is the f64 host numeric
# profile (the Pi has an FPU) PLUS the "real MCU firmware" marker that links
# the C step/SPI FFI (linux/phase_stepping_spi.c) — distinct from a bare `host`
# build (the motion-bridge planner cdylib / unit tests), which must NOT
# reference that symbol. kalico-sim is OPT-IN here exactly like the STM32
# families (CONFIG_KALICO_SIM), not hardwired: a Linux MCU drives real
# /dev/gpiochip / /dev/spidev by default.
KALICO_RUST_FEATURES := mcu-linux,header-nurbs,header-runtime
ifeq ($(CONFIG_KALICO_SIM),y)
KALICO_RUST_FEATURES := $(KALICO_RUST_FEATURES),kalico-sim
endif
ifeq ($(CONFIG_MOTION_MODULE_STEPPER),y)
KALICO_RUST_FEATURES := $(KALICO_RUST_FEATURES),motion-module-stepper
endif
# Own target dir: the motion-bridge cdylib builds the shared crates (runtime,
# nurbs, ...) with different host features into rust/target/, and a kalico-sim
# flip changes this firmware's feature set — keeping them separate avoids
# mutual incremental-cache thrash (slow on the Pi). Mirrors the per-arch STM32
# dirs. Exported for src/linux/Makefile's link rule.
KALICO_CARGO_TARGET_DIR := target-linux
KALICO_LIB := rust/$(KALICO_CARGO_TARGET_DIR)/release/libkalico_c_api.a

$(KALICO_LIB): FORCE
	cd rust && PATH="$(HOME)/.cargo/bin:$$PATH" && \
		export CARGO_TARGET_DIR="$(KALICO_CARGO_TARGET_DIR)" && \
		mkdir -p "$$CARGO_TARGET_DIR" && \
		PREV=$$(cat "$$CARGO_TARGET_DIR/.kalico-features-stamp" 2>/dev/null || echo ""); \
		if [ "$$PREV" != "$(KALICO_RUST_FEATURES)" ]; then \
			echo "kalico-c-api features changed ($$PREV -> $(KALICO_RUST_FEATURES)); cargo clean"; \
			cargo clean -p kalico-c-api 2>/dev/null || true; \
			cargo clean -p kalico-host-rt 2>/dev/null || true; \
			cargo clean -p runtime 2>/dev/null || true; \
			cargo clean -p nurbs 2>/dev/null || true; \
		fi && \
		KALICO_RUNTIME_PIECE_RING_SIZE=$(CONFIG_RUNTIME_PIECE_RING_SIZE) \
		KALICO_RUNTIME_SAMPLE_RATE_HZ=$(CONFIG_KALICO_MOTION_SAMPLE_RATE_HZ) \
		cargo build -p kalico-c-api \
		--no-default-features \
		--features $(KALICO_RUST_FEATURES) \
		--release && \
		echo "$(KALICO_RUST_FEATURES)" > "$$CARGO_TARGET_DIR/.kalico-features-stamp"
endif

.PHONY: FORCE
