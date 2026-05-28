"""Pytest for sim_control_client.py against a running shim."""

import os
import subprocess
import time

import pytest

from tools.sim_klippy.orchestrator.sim_control_client import (
    SimControlClient,
    SimControlError,
)

pytestmark = pytest.mark.sim_unit

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))


@pytest.fixture
def shim_under_sleep(tmp_path):
    """Spawn /bin/sleep with the shim loaded; yield the control-socket path."""
    sock_dir = tmp_path / "sim"
    sock_dir.mkdir()
    shim = os.path.join(
        REPO_ROOT, "tools/sim_klippy/preload/libsim_intercept.so"
    )
    assert os.path.exists(shim), (
        "build shim first: make -C tools/sim_klippy/preload"
    )
    env = os.environ.copy()
    env["LD_PRELOAD"] = shim
    env["KALICO_SIM_SOCK_DIR"] = str(sock_dir)
    p = subprocess.Popen(["/bin/sleep", "10"], env=env)
    deadline = time.time() + 3.0
    sock_path = sock_dir / "sim_control"
    while time.time() < deadline:
        if sock_path.exists():
            break
        time.sleep(0.05)
    else:
        p.terminate()
        pytest.fail("control socket never appeared")
    yield str(sock_path)
    p.terminate()
    p.wait()


def test_ping(shim_under_sleep):
    with SimControlClient(shim_under_sleep) as c:
        c.ping()


def test_set_and_get_gpio(shim_under_sleep):
    with SimControlClient(shim_under_sleep) as c:
        c.set_gpio_input(chip=0, line=20, value=1)
        # Note: set_gpio_input updates the shared GPIO table; reading
        # back via get_gpio_output reads the same table.
        assert c.get_gpio_output(chip=0, line=20) == 1


def test_set_adc(shim_under_sleep):
    with SimControlClient(shim_under_sleep) as c:
        c.set_adc(channel=3, value=2048)
        # No direct getter for ADC — verify by absence of error.


def test_unknown_verb(shim_under_sleep):
    with SimControlClient(shim_under_sleep) as c:
        with pytest.raises(SimControlError):
            c._send_recv("bogus_verb")
