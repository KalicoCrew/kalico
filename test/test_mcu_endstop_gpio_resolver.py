from klippy.mcu import MCU_endstop


def test_linux_gpiochip_zero():
    assert MCU_endstop._resolve_bridge_gpio_index("gpiochip0/gpio200") == 200


def test_linux_gpiochip_nonzero_port():
    assert (
        MCU_endstop._resolve_bridge_gpio_index("gpiochip2/gpio5") == 2 * 288 + 5
    )


def test_stm32_pa0_is_zero():
    assert MCU_endstop._resolve_bridge_gpio_index("PA0") == 0


def test_stm32_pa15():
    assert MCU_endstop._resolve_bridge_gpio_index("PA15") == 15


def test_stm32_pb0():
    assert MCU_endstop._resolve_bridge_gpio_index("PB0") == 16


def test_stm32_pg9_for_stepper_x1_diag():
    assert MCU_endstop._resolve_bridge_gpio_index("PG9") == 105


def test_stm32_pg6_for_stepper_y1_diag():
    assert MCU_endstop._resolve_bridge_gpio_index("PG6") == 102


def test_stm32_pi15_highest():
    idx = MCU_endstop._resolve_bridge_gpio_index("PI15")
    assert idx == 143
    assert idx < 256  # fits in u8 for gpio_in_setup cast


def test_unknown_string_returns_zero():
    assert MCU_endstop._resolve_bridge_gpio_index("") == 0
    assert MCU_endstop._resolve_bridge_gpio_index("virtual_endstop") == 0


def test_stm32_out_of_range_pin_rejected():
    assert MCU_endstop._resolve_bridge_gpio_index("PA16") == 0
    assert MCU_endstop._resolve_bridge_gpio_index("PA99") == 0


def test_stm32_invalid_port_rejected():
    assert MCU_endstop._resolve_bridge_gpio_index("PZ0") == 0
