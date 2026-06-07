import os

import pytest

from tools.sim_klippy.orchestrator.launcher import McuHandle, spawn_mcus

pytestmark = pytest.mark.needs_elf


def test_spawn_brings_up_both_mcus(tmp_path):
    h7_socket = str(tmp_path / "klipper_sim_h7")
    f4_socket = str(tmp_path / "klipper_sim_f4")
    handles = spawn_mcus(
        h7_elf="out/klipper-h7-sim.elf",
        f4_elf="out/klipper-f4-sim.elf",
        h7_socket=h7_socket,
        f4_socket=f4_socket,
        log_dir=str(tmp_path),
    )
    try:
        assert isinstance(handles.h7, McuHandle)
        assert isinstance(handles.f4, McuHandle)
        assert os.path.exists(handles.h7.socket_path)
        assert os.path.exists(handles.f4.socket_path)
        assert handles.h7.process.poll() is None
        assert handles.f4.process.poll() is None
    finally:
        handles.shutdown()
        assert not os.path.exists(h7_socket)
        assert not os.path.exists(f4_socket)
