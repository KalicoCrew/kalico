import json
import zlib

from klippy import msgproto


def _identify_payload(**overrides):
    data = {
        "enumerations": {},
        "commands": {},
        "responses": {},
        "output": {},
        "app": "Kalico",
        "version": "v0.13.0-241-g5fdf0dd3e",
    }
    data.update(overrides)
    return zlib.compress(json.dumps(data).encode())


def test_get_kconfig_before_identify_returns_none():
    mp = msgproto.MessageParser()
    assert mp.get_kconfig() is None


def test_process_identify_without_kconfig_key_stays_none():
    # Firmware predating this feature has no "kconfig" key at all.
    mp = msgproto.MessageParser()
    mp.process_identify(_identify_payload())
    assert mp.get_kconfig() is None


def test_process_identify_with_kconfig_key():
    payload = _identify_payload(kconfig="CONFIG_MACH_STM32=y\n")
    mp = msgproto.MessageParser()
    mp.process_identify(payload)
    assert mp.get_kconfig() == "CONFIG_MACH_STM32=y\n"
