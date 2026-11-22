from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from klippy.extras.load_cell.load_cell import LoadCell
from klippy.extras.load_cell.load_cell_probe import (
    LoadCellPrimitives,
    LoadCellProbeConfigHelper,
    LoadCellTareParams,
)
from klippy.gcode import GCodeCommand


@pytest.mark.parametrize("offset", [-400.0, -0.125, 0.0, 0.125, 400.0])
@pytest.mark.parametrize("invert", [-1.0, 1.0])
@pytest.mark.parametrize("channels", [1, 3])
def test_tare_reading_includes_offset(offset, invert, channels):
    samples = tuple(10000 + channel for channel in range(channels))
    cell = LoadCell.__new__(LoadCell)
    cell.printer = Mock()
    cell.invert = invert
    cell.channel_count = channels
    cell.counts_per_gram = 10.0
    cell.reference_tare_counts = sum(samples)
    cell.reference_tare_counts_per_channel = samples
    cell.avg_counts = Mock(return_value=samples)
    primitives = LoadCellPrimitives.__new__(LoadCellPrimitives)
    primitives._load_cell = cell
    primitives._printer = Mock()
    primitives._config_helper = Mock(
        spec=LoadCellProbeConfigHelper, assert_force_safety_limit=Mock()
    )
    primitives._continuous_tare_filter_helper = Mock()
    primitives._mcu_load_cell_probe = Mock()
    params = LoadCellTareParams(5, offset, 75, 2000, 1000, None)

    primitives.tare(params)

    assert cell.counts_to_grams(sum(samples)) == pytest.approx(
        offset, abs=0.5 / cell.counts_per_gram
    )
    assert cell.reference_tare_counts_per_channel == samples
    assert all(isinstance(value, int) for value in cell.tare_counts_per_channel)
    cell.avg_counts.assert_called_once_with(5)
    primitives._mcu_load_cell_probe.set_endstop_range.assert_called_once_with(
        params
    )


@pytest.mark.parametrize("configured", [None, -400.0, 400.0])
@pytest.mark.parametrize("override", [None, -200.5, 0.0, 200.5])
def test_tare_offset_config_and_command(configured, override):
    config = Mock()
    config.getfloat.side_effect = lambda name, default, *args: (
        configured
        if name == "tare_offset" and configured is not None
        else default
    )
    config.getint.side_effect = lambda name, default, *args: default
    config.getboolean.side_effect = lambda name, default: default
    cell = SimpleNamespace(
        get_sensor=lambda: SimpleNamespace(get_samples_per_second=lambda: 100)
    )
    helper = LoadCellProbeConfigHelper(config, cell)
    parameters = {} if override is None else {"TARE_OFFSET": str(override)}
    gcmd = GCodeCommand(Mock(), "PROBE", "PROBE", parameters, False, None)
    expected = override if override is not None else (configured or 0.0)
    assert helper.get_tare_offset_grams(gcmd) == expected

    primitives = LoadCellPrimitives.__new__(LoadCellPrimitives)
    primitives._config_helper = helper
    primitives._continuous_tare_filter_helper = Mock()
    assert primitives.build_tare_params(gcmd).tare_offset == expected


@pytest.mark.parametrize("invert", [-1.0, 1.0])
@pytest.mark.parametrize(
    "force,offset,allowed",
    [
        (2300, 400, False),
        (-2300, -400, False),
        (0, 3000, True),
        (0, -3000, True),
        (1900, 0, True),
    ],
)
def test_force_safety_limit_ignores_tare_offset(invert, force, offset, allowed):
    cell = LoadCell.__new__(LoadCell)
    cell.printer = Mock()
    cell.invert = invert
    cell.channel_count = 1
    cell.counts_per_gram = 10.0
    cell.reference_tare_counts = 100000
    cell.reference_tare_counts_per_channel = (100000,)
    cell.tare_counts = 100000
    cell.tare_counts_per_channel = (100000,)
    samples = (round(100000 + force * invert * cell.counts_per_gram),)
    cell.avg_counts = Mock(return_value=samples)
    cell.get_sensor = lambda: SimpleNamespace(
        get_range=lambda: (-1000000, 1000000)
    )
    helper = LoadCellProbeConfigHelper.__new__(LoadCellProbeConfigHelper)
    helper._load_cell = cell
    helper._printer = SimpleNamespace(command_error=RuntimeError)
    primitives = LoadCellPrimitives.__new__(LoadCellPrimitives)
    primitives._load_cell = cell
    primitives._printer = Mock()
    primitives._config_helper = helper
    primitives._continuous_tare_filter_helper = Mock()
    primitives._mcu_load_cell_probe = Mock()
    params = LoadCellTareParams(5, offset, 75, 2000, 1000, None)

    if allowed:
        primitives.tare(params)
        assert cell.counts_to_grams(sum(samples)) == offset
    else:
        with pytest.raises(RuntimeError, match="exceeds force_safety_limit"):
            primitives.tare(params)
        assert cell.tare_counts_per_channel == (100000,)
        primitives._mcu_load_cell_probe.set_endstop_range.assert_not_called()
