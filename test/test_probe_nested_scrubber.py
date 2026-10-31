from __future__ import annotations

from typing import Optional

import pytest

from klippy.extras import probe


class _GCodeError(Exception):
    pass


class _FakeGCmd:
    error = _GCodeError

    def __init__(self, **params):
        self._params = params

    def get(self, name, default=None, **_):
        return self._params.get(name, default)

    def get_int(self, name, default=None, **_):
        return int(self._params.get(name, default))

    def get_float(self, name, default=None, **_):
        return float(self._params.get(name, default))

    def respond_info(self, msg, log=True):
        pass


class _FakeToolhead:
    def get_position(self):
        return [10.0, 10.0, 5.0, 0.0]


class _FakeTemplate:
    def __init__(self):
        self.on_run = lambda: None

    def create_template_context(self):
        return {}

    def run_gcode_from_command(self, context):
        self.on_run()


class _FakePrinter:
    command_error = _GCodeError

    def __init__(self, template):
        self._objects = {
            "toolhead": _FakeToolhead(),
            "gcode_macro": self,
        }
        self._template = template

    def get_printer(self):
        return self

    def load_object(self, config, name):
        return self._objects[name]

    def load_template(self, config, option, default):
        return self._template

    def lookup_object(self, name, default=None):
        return self._objects.get(name, default)


class _FakeConfig:
    def __init__(self, printer, **values):
        self._printer = printer
        self._values = values

    def get_printer(self):
        return self._printer

    def getfloat(self, name, default=None, **_):
        return float(self._values.get(name, default))

    def getint(self, name, default=None, **_):
        return int(self._values.get(name, default))

    def getchoice(self, name, choices, default=None):
        return choices[self._values.get(name, default)]


class _FakeProbe(probe.PrinterProbe):
    """PrinterProbe with the hardware-facing methods replaced"""

    def __init__(self, printer, retry_policy_factory, results):
        self.printer = printer
        self.retry_policy_factory = retry_policy_factory
        self.results = results
        self.speed = 5.0
        self.retry_speed = 5.0
        self.sample_count = 1
        self.samples_tolerance = 0.1
        self.samples_retries = 0
        self.samples_result = "average"
        self.multi_probe_pending = False

    def multi_probe_begin(self, always_restore_toolhead=False):
        self.multi_probe_pending = True

    def multi_probe_end(self):
        self.multi_probe_pending = False

    def _move(self, coord, speed):
        pass

    def _retract(self, gcmd):
        pass

    def _probe(self, speed, gcmd):
        return [10.0, 10.0, 1.0], self.results.pop(0)


def _build_probe(results):
    probe.GcodeNozzleScrubber.is_scrubbing = False
    template = _FakeTemplate()
    printer = _FakePrinter(template)
    config = _FakeConfig(printer)
    factory = probe.RetryPolicyFactory(config)
    return _FakeProbe(printer, factory, results), template


def _outer_gcmd():
    return _FakeGCmd(
        BAD_PROBE_STRATEGY=probe.RetryStrategy.RETRY.name,
        BAD_PROBE_RETRIES=2,
        SCRUBBING_FREQUENCY=1,
    )


@pytest.mark.parametrize(
    "nested_strategy, raises_exception",
    [
        (probe.RetryStrategy.IGNORE, False),
        (probe.RetryStrategy.FAIL, False),
        (probe.RetryStrategy.RETRY, True),
        (probe.RetryStrategy.CIRCLE, True),
    ],
)
def test_probe_from_nozzle_scrubber_gcode(nested_strategy, raises_exception):
    # The outer probe is bad, so it retries and runs the scrubber gcode.
    # The nested probe inside the scrubber gcode is good.
    prober, template = _build_probe([False, True, True])
    nested_results = []
    nested_gcmd = _FakeGCmd(BAD_PROBE_STRATEGY=nested_strategy.name)
    template.on_run = lambda: nested_results.append(
        prober.run_probe(nested_gcmd)
    )
    error: Optional[_GCodeError] = None
    try:
        prober.run_probe(_outer_gcmd())
    except _GCodeError as e:
        error = e
    assert (error is not None) is raises_exception
    assert ("cannot use retries" in str(error)) is raises_exception
    assert len(nested_results) == (0 if raises_exception else 1)
    assert probe.GcodeNozzleScrubber.is_scrubbing is False


def test_scrubber_gcode_calling_itself_raises():
    prober, template = _build_probe([False, False, True])
    outer_gcmd = _outer_gcmd()
    scrubber = prober.retry_policy_factory.customize(outer_gcmd).scrubber
    template.on_run = lambda: scrubber.clean_nozzle(outer_gcmd, 1, 2)
    with pytest.raises(_GCodeError, match="called itself"):
        prober.run_probe(outer_gcmd)
    assert probe.GcodeNozzleScrubber.is_scrubbing is False
