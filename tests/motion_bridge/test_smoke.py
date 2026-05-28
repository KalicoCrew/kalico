"""Smoke tests for the motion_bridge PyO3 extension module."""

import pytest


def test_module_imports():
    import motion_bridge

    assert hasattr(motion_bridge, "MotionBridge")


def test_bridge_instantiates():
    import motion_bridge

    bridge = motion_bridge.MotionBridge()
    assert bridge.version() != ""


# ── Task 32–33: claim / release MCU ─────────────────────────────────────


def test_claim_mcu_returns_int():
    import motion_bridge

    bridge = motion_bridge.MotionBridge()
    handle = bridge.claim_mcu("mcu", "/dev/ttyACM0", 250000)
    assert isinstance(handle, int)


def test_claim_two_mcus_returns_distinct_handles():
    import motion_bridge

    bridge = motion_bridge.MotionBridge()
    h1 = bridge.claim_mcu("mcu", "/dev/ttyACM0", 250000)
    h2 = bridge.claim_mcu("mcu2", "/dev/ttyACM1", 250000)
    assert h1 != h2


def test_release_mcu_then_alloc_fails():
    import motion_bridge

    bridge = motion_bridge.MotionBridge()
    h = bridge.claim_mcu("mcu", "/dev/ttyACM0", 250000)
    bridge.release_mcu(h)
    with pytest.raises(RuntimeError):
        bridge.alloc_command_queue(h)


# ── Task 34: alloc_command_queue ─────────────────────────────────────────


def test_alloc_command_queue():
    import motion_bridge

    bridge = motion_bridge.MotionBridge()
    h = bridge.claim_mcu("mcu", "/dev/ttyACM0", 250000)
    q = bridge.alloc_command_queue(h)
    assert isinstance(q, int)


def test_alloc_two_queues_distinct():
    import motion_bridge

    bridge = motion_bridge.MotionBridge()
    h = bridge.claim_mcu("mcu", "/dev/ttyACM0", 250000)
    q1 = bridge.alloc_command_queue(h)
    q2 = bridge.alloc_command_queue(h)
    assert q1 != q2


# ── Task 35: passthrough_send ────────────────────────────────────────────


def test_passthrough_send_does_not_crash():
    import motion_bridge

    bridge = motion_bridge.MotionBridge()
    h = bridge.claim_mcu("mcu", "/dev/ttyACM0", 250000)
    q = bridge.alloc_command_queue(h)
    # fire-and-forget, should not raise
    bridge.passthrough_send(h, q, b"\x01\x02\x03")


def test_passthrough_send_with_clocks():
    import motion_bridge

    bridge = motion_bridge.MotionBridge()
    h = bridge.claim_mcu("mcu", "/dev/ttyACM0", 250000)
    q = bridge.alloc_command_queue(h)
    bridge.passthrough_send(h, q, b"\xaa", min_clock=100, req_clock=200)


# ── Task 36: passthrough_query ───────────────────────────────────────────


def test_passthrough_query_returns_notify_id():
    import motion_bridge

    bridge = motion_bridge.MotionBridge()
    h = bridge.claim_mcu("mcu", "/dev/ttyACM0", 250000)
    q = bridge.alloc_command_queue(h)
    nid = bridge.passthrough_query(h, q, b"\x01")
    assert isinstance(nid, int)
    assert nid > 0  # notify ids start at 1


# ── Task 37: passthrough_send_wait_ack ───────────────────────────────────


def test_send_wait_ack_raises_not_implemented():
    import motion_bridge

    bridge = motion_bridge.MotionBridge()
    h = bridge.claim_mcu("mcu", "/dev/ttyACM0", 250000)
    q = bridge.alloc_command_queue(h)
    with pytest.raises(NotImplementedError, match="Phase 2"):
        bridge.passthrough_send_wait_ack(h, q, b"\x01", 1.0)


# ── Task 38: passthrough_register_handler ────────────────────────────────


def test_register_handler_does_not_crash():
    import motion_bridge

    bridge = motion_bridge.MotionBridge()
    h = bridge.claim_mcu("mcu", "/dev/ttyACM0", 250000)
    bridge.passthrough_register_handler(h, "get_status", 0, lambda params: None)


# ── Task 39: passthrough_register_flush_callback ─────────────────────────


def test_register_flush_callback_does_not_crash():
    import motion_bridge

    bridge = motion_bridge.MotionBridge()
    h = bridge.claim_mcu("mcu", "/dev/ttyACM0", 250000)
    bridge.passthrough_register_flush_callback(h, lambda: None)


# ── Task 40: poll_event ──────────────────────────────────────────────────


def test_poll_event_returns_none_when_empty():
    import motion_bridge

    bridge = motion_bridge.MotionBridge()
    assert bridge.poll_event() is None


# ── Additional API: config / stats / clock ───────────────────────────────


def test_add_config_cmd_and_begin_config_phase():
    import motion_bridge

    bridge = motion_bridge.MotionBridge()
    h = bridge.claim_mcu("mcu", "/dev/ttyACM0", 250000)
    # add config command — returns True during Collecting phase
    added = bridge.add_config_cmd(h, b"\x10\x20")
    assert added is True
    # begin config phase
    bridge.begin_config_phase(h)
    # adding after begin_config_phase returns False
    added_after = bridge.add_config_cmd(h, b"\x30\x40")
    assert added_after is False


def test_add_init_cmd():
    import motion_bridge

    bridge = motion_bridge.MotionBridge()
    h = bridge.claim_mcu("mcu", "/dev/ttyACM0", 250000)
    added = bridge.add_init_cmd(h, b"\xaa")
    assert added is True


def test_add_restart_cmd():
    import motion_bridge

    bridge = motion_bridge.MotionBridge()
    h = bridge.claim_mcu("mcu", "/dev/ttyACM0", 250000)
    added = bridge.add_restart_cmd(h, b"\xbb")
    assert added is True


def test_get_stats_returns_dict():
    import motion_bridge

    bridge = motion_bridge.MotionBridge()
    h = bridge.claim_mcu("mcu", "/dev/ttyACM0", 250000)
    stats = bridge.get_stats(h)
    assert isinstance(stats, dict)
    assert stats["bytes_write"] == 0
    assert stats["send_seq"] == 0
    assert "ready_bytes" in stats


def test_set_clock_est():
    import motion_bridge

    bridge = motion_bridge.MotionBridge()
    h = bridge.claim_mcu("mcu", "/dev/ttyACM0", 250000)
    # Should not raise
    bridge.set_clock_est(h, 48_000_000.0, 0.0, 1000)


def test_next_config_entry_after_config_phase():
    import motion_bridge

    bridge = motion_bridge.MotionBridge()
    h = bridge.claim_mcu("mcu", "/dev/ttyACM0", 250000)
    bridge.add_config_cmd(h, b"\x01")
    bridge.add_config_cmd(h, b"\x02")
    bridge.begin_config_phase(h)
    # Should drain the two config entries
    e1 = bridge.next_config_entry(h)
    assert e1 is not None
    e2 = bridge.next_config_entry(h)
    assert e2 is not None
    e3 = bridge.next_config_entry(h)
    # After config entries, init entries follow (none added here)
    # Eventually returns None
    # (Exact behavior depends on ConfigStage internals)


def test_extract_old_returns_dict():
    import motion_bridge

    bridge = motion_bridge.MotionBridge()
    h = bridge.claim_mcu("mcu", "/dev/ttyACM0", 250000)
    result = bridge.extract_old(h)
    assert isinstance(result, dict)
    assert "sent" in result
    assert "received" in result
    assert isinstance(result["sent"], list)
    assert isinstance(result["received"], list)


# ── Error handling ───────────────────────────────────────────────────────


def test_unknown_mcu_raises_runtime_error():
    import motion_bridge

    bridge = motion_bridge.MotionBridge()
    with pytest.raises(RuntimeError, match="unknown MCU"):
        bridge.alloc_command_queue(999)


def test_unknown_mcu_get_stats_raises():
    import motion_bridge

    bridge = motion_bridge.MotionBridge()
    with pytest.raises(RuntimeError, match="unknown MCU"):
        bridge.get_stats(999)
