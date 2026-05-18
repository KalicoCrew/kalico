"""Unit tests for MCU_endstop._resolve_bridge_gpio_index.

The resolver converts a pin string into the numeric GPIO index the
bridge firmware's `gpio_in_setup()` expects. Two namespaces:

  * Linux MCU (sim) — "gpiochipN/gpioM" → port * MAX_GPIO_LINES + num
    (MAX_GPIO_LINES=288, mirrors src/linux/internal.h::GPIO).
  * STM32 (real hardware) — "P[A-I][0-15]" → (port-'A') * 16 + num
    (mirrors src/stm32/internal.h::GPIO).

The STM32 branch is what makes sensorless homing on H7/F4 hardware work:
the TMC DIAG endstop pin name (e.g. "PG9" for stepper_x1 on the BTT
Octopus Pro) must round-trip through the host into the same numeric
index the firmware-side pin table uses, otherwise `runtime_commands.c`'s
`endstop_pin_table_populate` opens the wrong physical pin.
"""

from klippy.mcu import MCU_endstop


def test_linux_gpiochip_zero():
    assert MCU_endstop._resolve_bridge_gpio_index("gpiochip0/gpio200") == 200


def test_linux_gpiochip_nonzero_port():
    # MAX_GPIO_LINES=288 in src/linux/internal.h::GPIO.
    assert MCU_endstop._resolve_bridge_gpio_index("gpiochip2/gpio5") == 2 * 288 + 5


def test_stm32_pa0_is_zero():
    assert MCU_endstop._resolve_bridge_gpio_index("PA0") == 0


def test_stm32_pa15():
    assert MCU_endstop._resolve_bridge_gpio_index("PA15") == 15


def test_stm32_pb0():
    assert MCU_endstop._resolve_bridge_gpio_index("PB0") == 16


def test_stm32_pg9_for_stepper_x1_diag():
    # PG9 is the DIAG_0 pin on the BTT Octopus Pro H723 and the wired
    # endstop for stepper_x1 in the user's printer.cfg. GPIO('G', 9) =
    # (6) * 16 + 9 = 105. This is the regression case for sensorless
    # homing on real hardware — prior to the fix the resolver returned
    # 0 and the firmware sampled the wrong pin.
    assert MCU_endstop._resolve_bridge_gpio_index("PG9") == 105


def test_stm32_pg6_for_stepper_y1_diag():
    # PG6 is the DIAG_1 pin on the BTT Octopus Pro H723. (6)*16 + 6 = 102.
    assert MCU_endstop._resolve_bridge_gpio_index("PG6") == 102


def test_stm32_pi15_highest():
    # GPIO('I', 15) = 8 * 16 + 15 = 143 — the highest STM32 pin index
    # the firmware enumerates (src/stm32/gpio.c). Must still fit u8.
    idx = MCU_endstop._resolve_bridge_gpio_index("PI15")
    assert idx == 143
    assert idx < 256  # fits in u8 for gpio_in_setup cast


def test_unknown_string_returns_zero():
    # Default fallthrough for anything the resolver doesn't recognize.
    assert MCU_endstop._resolve_bridge_gpio_index("") == 0
    assert MCU_endstop._resolve_bridge_gpio_index("virtual_endstop") == 0


def test_stm32_out_of_range_pin_rejected():
    # STM32 pins above 15 are invalid (each port has 16 pins).
    # Don't accidentally return a confusing value — fall through to 0.
    assert MCU_endstop._resolve_bridge_gpio_index("PA16") == 0
    assert MCU_endstop._resolve_bridge_gpio_index("PA99") == 0


def test_stm32_invalid_port_rejected():
    # Only ports A-I are enumerated in src/stm32/gpio.c.
    assert MCU_endstop._resolve_bridge_gpio_index("PZ0") == 0
