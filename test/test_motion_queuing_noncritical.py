from klippy import clocksync
from klippy.extras import motion_queuing

MAIN_FREQ = 64000000.0
SECONDARY_FREQ = 48000000.0


class _FakeReactor:
    NOW = 0.0
    NEVER = 9999999999999999.0

    def register_timer(self, callback, waketime=NEVER):
        return None

    def update_timer(self, timer, waketime):
        pass

    def monotonic(self):
        return 0.0


class _FakeFFILib:
    def __init__(self):
        self.set_time_calls = []

    def steppersync_set_time(self, ss, offset, freq):
        self.set_time_calls.append((ss, offset, freq))


class _FakeMCU:
    def __init__(self, name, clocksync_obj, disconnected=False):
        self.name = name
        self.non_critical_disconnected = disconnected
        self._clocksync = clocksync_obj
        self.calibrate_calls = 0

    def calibrate_clock(self, print_time, eventtime):
        self.calibrate_calls += 1
        return self._clocksync.calibrate_clock(print_time, eventtime)

    def estimated_print_time(self, eventtime):
        return self._clocksync.estimated_print_time(eventtime)


def _main_sync(boot_time=3600.0):
    sync = clocksync.ClockSync(_FakeReactor())
    sync.mcu_freq = MAIN_FREQ
    # Main mcu booted 'boot_time' seconds before host time 0
    sync.clock_est = (0.0, boot_time * MAIN_FREQ, MAIN_FREQ)
    return sync


def _connected_secondary(main_sync):
    # State as left behind by a successful SecondarySync.connect()
    sync = clocksync.SecondarySync(_FakeReactor(), main_sync)
    sync.mcu_freq = SECONDARY_FREQ
    sync.clock_est = (0.0, 100.0 * SECONDARY_FREQ, SECONDARY_FREQ)
    sync.clock_adj = (3600.0 - 100.0, SECONDARY_FREQ)
    return sync


def _make_motion_queuing(monkeypatch, steppersyncs, primary):
    ffi_lib = _FakeFFILib()
    monkeypatch.setattr(
        motion_queuing.chelper, "get_ffi", lambda: (None, ffi_lib)
    )
    mq = motion_queuing.PrinterMotionQueuing.__new__(
        motion_queuing.PrinterMotionQueuing
    )
    mq.steppersyncs = steppersyncs
    mq.last_step_gen_time = 0.0
    mq.clear_history_time = 0.0
    mq.mcu = primary
    return mq, ffi_lib


def test_never_connected_secondary_sync_does_not_divide_by_zero():
    # Without a clock estimate the old calculation raised
    # ZeroDivisionError after a few calls (1-27 depending on uptime)
    sync = clocksync.SecondarySync(_FakeReactor(), _main_sync())
    for i in range(120):
        eventtime = 1.0 + i
        assert sync.calibrate_clock(0.0, eventtime) == (0.0, 1.0)


def test_connected_secondary_sync_still_calibrates():
    sync = _connected_secondary(_main_sync())
    offset, freq = sync.calibrate_clock(3601.0, 1.0)
    assert abs(freq / SECONDARY_FREQ - 1.0) < 1e-6
    assert (offset, freq) == sync.clock_adj


def test_stats_skips_disconnected_non_critical_mcu(monkeypatch):
    main_sync = _main_sync()
    primary = _FakeMCU("mcu", main_sync)
    connected = _FakeMCU("toolboard", _connected_secondary(main_sync))
    disconnected = _FakeMCU(
        "mmu",
        clocksync.SecondarySync(_FakeReactor(), main_sync),
        disconnected=True,
    )
    mq, ffi_lib = _make_motion_queuing(
        monkeypatch,
        [(primary, "ss_main"), (connected, "ss_tb"), (disconnected, "ss_mmu")],
        primary,
    )
    for i in range(60):
        assert mq.stats(1.0 + i) == (False, "")
    assert disconnected.calibrate_calls == 0
    assert primary.calibrate_calls == 60
    assert connected.calibrate_calls == 60
    synced = {ss for ss, _, _ in ffi_lib.set_time_calls}
    assert synced == {"ss_main", "ss_tb"}
    # The primary mcu is never adjusted, the secondary one is
    assert ("ss_main", 0.0, MAIN_FREQ) in ffi_lib.set_time_calls
    _, tb_offset, tb_freq = ffi_lib.set_time_calls[-1]
    assert abs(tb_freq / SECONDARY_FREQ - 1.0) < 1e-6


def test_stats_calibrates_non_critical_mcu_again_after_reconnect(
    monkeypatch,
):
    main_sync = _main_sync()
    primary = _FakeMCU("mcu", main_sync)
    mmu = _FakeMCU("mmu", _connected_secondary(main_sync), disconnected=True)
    mq, ffi_lib = _make_motion_queuing(
        monkeypatch, [(primary, "ss_main"), (mmu, "ss_mmu")], primary
    )
    mq.stats(1.0)
    assert mmu.calibrate_calls == 0
    # recon_mcu() clears non_critical_disconnected after a reconnect
    mmu.non_critical_disconnected = False
    mq.stats(2.0)
    assert mmu.calibrate_calls == 1
    ss, offset, freq = ffi_lib.set_time_calls[-1]
    assert ss == "ss_mmu"
    assert abs(freq / SECONDARY_FREQ - 1.0) < 1e-6


def test_stats_during_reconnect_attempt_of_never_connected_mcu(monkeypatch):
    # recon_mcu() clears non_critical_disconnected before the clock is
    # synchronized; stats() may run in that window (reactor pauses while
    # connecting) and must not crash
    main_sync = _main_sync()
    primary = _FakeMCU("mcu", main_sync)
    mmu = _FakeMCU("mmu", clocksync.SecondarySync(_FakeReactor(), main_sync))
    mq, ffi_lib = _make_motion_queuing(
        monkeypatch, [(primary, "ss_main"), (mmu, "ss_mmu")], primary
    )
    for i in range(60):
        mq.stats(1.0 + i)
    assert mmu.calibrate_calls == 60
