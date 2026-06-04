"""Contract tests for the SpiRouter chip dispatcher."""

import pytest

from tools.sim_klippy.orchestrator.spi_router import SpiRouter

pytestmark = pytest.mark.sim_unit


def test_attach_dispatches_by_cs():
    seen_a, seen_b = [], []

    def chip_a(payload: bytes) -> bytes:
        seen_a.append(payload)
        return b"A" + payload[1:]

    def chip_b(payload: bytes) -> bytes:
        seen_b.append(payload)
        return b"B" + payload[1:]

    router = SpiRouter()
    router.attach(5, chip_a)
    router.attach(40, chip_b)

    assert router(5, b"\x00\x11\x22") == b"A\x11\x22"
    assert router(40, b"\xff\x33") == b"B\x33"
    assert seen_a == [b"\x00\x11\x22"]
    assert seen_b == [b"\xff\x33"]


def test_unknown_cs_raises():
    router = SpiRouter()
    router.attach(5, lambda p: p)
    with pytest.raises(KeyError) as exc:
        router(99, b"\x00")
    assert "no chip on CS 99" in str(exc.value)


def test_double_attach_raises():
    router = SpiRouter()
    router.attach(5, lambda p: p)
    with pytest.raises(ValueError):
        router.attach(5, lambda p: p)
