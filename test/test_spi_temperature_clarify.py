from klippy.extras import spi_temperature


def _make_sensor():
    sensor = spi_temperature.MAX31855.__new__(spi_temperature.MAX31855)
    sensor.name = "extruder"
    sensor._last_value = sensor._last_fault = None
    sensor.min_temp, sensor.max_temp = 0.0, 300.0
    sensor.setup_minmax(0.0, 300.0)
    return sensor


def test_no_reading_yet():
    assert _make_sensor()._clarify_fault("MCU shutdown", {}) is None


def test_reports_fault_bits():
    sensor = _make_sensor()
    sensor._last_value, sensor._last_fault = 0, 0x1
    msg = sensor._clarify_fault("MCU shutdown", {})
    assert msg == "Sensor 'extruder' reported fault 0x1"


def test_reports_out_of_range_temperature():
    sensor = _make_sensor()
    sensor._last_value = sensor.calc_adc(350.0)
    sensor._last_fault = 0
    msg = sensor._clarify_fault("MCU shutdown", {})
    assert msg.startswith("Sensor 'extruder' temperature 350.")
    assert msg.endswith("not in range 0.000:300.000")


def test_in_range_reading_is_not_reported():
    sensor = _make_sensor()
    sensor._last_value = sensor.calc_adc(200.0)
    sensor._last_fault = 0
    assert sensor._clarify_fault("MCU shutdown", {}) is None
