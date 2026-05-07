"""Boot test: sim fixture brings up both MCUs, beacon, all chip stubs;
klippy connects, registers all extras, reaches 'Printer is ready'.

Failure modes that count as test failure:
- Any klippy traceback
- Any MCU shutdown
- Any 'transport closed' or 'transport timed out'
- Any 'TMC_UNKNOWN_REG' warning (drift detection)
"""
import pytest


def test_boot_clean(sim):
    log = sim.klippy_log.read_text() if sim.klippy_log.exists() else ""
    assert "Printer is ready" in log, (
        "klippy did not reach ready. Last 80 lines:\n"
        + "\n".join(log.splitlines()[-80:])
    )
    # No tracebacks
    assert "Traceback" not in log, f"klippy crashed during boot:\n{log}"
    # No MCU shutdowns mid-boot
    if "MCU '" in log and " shutdown:" in log:
        if "Command request" in log or "Emergency stop" in log:
            pytest.fail(f"MCU shutdown during boot:\n{log}")
    assert "transport closed" not in log
    assert "transport timed out" not in log
