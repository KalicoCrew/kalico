"""Probe contexts pair nested sessions and preserve backend errors."""

import pytest

from klippy.extras import probe


def test_external_probe_without_context_keeps_working():
    calls = []
    context = probe.ProbingContext(gcmd=object())

    class ExternalProbe:
        def multi_probe_begin(self):
            calls.append("begin")

        def multi_probe_end(self):
            calls.append("end")

    wrapper = probe.PrinterProbe.__new__(probe.PrinterProbe)
    external = ExternalProbe()
    with probe.multi_probe_session(external, context):
        assert calls == ["begin"]
    assert calls == ["begin", "end"]
    calls.clear()
    wrapper.mcu_probe = external
    wrapper.multi_probe_count = 0
    with probe.multi_probe_session(wrapper, context):
        for _ in range(2):
            with probe.multi_probe_session(wrapper, context):
                assert wrapper.multi_probe_count == 2
                assert calls == ["begin"]
            assert wrapper.multi_probe_count == 1
            assert calls == ["begin"]
    assert calls == ["begin", "end"]
    assert wrapper.multi_probe_count == 0


def test_probe_session_preserves_internal_type_errors():
    context = probe.ProbingContext(gcmd=object(), always_restore_toolhead=True)
    calls = []

    class Probe:
        def multi_probe_begin(self, ctx=None):
            calls.append(("begin", ctx))
            if failure == "begin" and ctx is not None:
                raise TypeError("sensor initialization failed")

        def multi_probe_end(self, ctx=None):
            calls.append(("end", ctx))
            if failure == "end" and ctx is not None:
                raise TypeError("sensor cleanup failed")

    wrapper = probe.PrinterProbe.__new__(probe.PrinterProbe)
    wrapper.mcu_probe = Probe()
    wrapper.multi_probe_count = 0
    for failure, message in [
        ("begin", "sensor initialization failed"),
        ("end", "sensor cleanup failed"),
    ]:
        calls.clear()
        with pytest.raises(TypeError, match=message):
            with probe.multi_probe_session(wrapper, context):
                assert failure == "end"
        assert calls == [("begin", context), ("end", context)]
        assert all(ctx is context for _, ctx in calls)
        assert wrapper.multi_probe_count == 0
