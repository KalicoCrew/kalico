import os

from klippy import chelper

MCU_FREQ = 16000000.0
MAX_ERROR = int(0.000025 * MCU_FREQ)
STEP_DIST = 0.0125
ACCEL, CRUISE_V = 3000.0, 150.0
ACCEL_T = CRUISE_V / ACCEL
CRUISE_T = 0.2


def _expected_position(print_time):
    if print_time <= ACCEL_T:
        return 0.5 * ACCEL * print_time**2
    return 0.5 * ACCEL * ACCEL_T**2 + CRUISE_V * (print_time - ACCEL_T)


def test_high_precision_steps_on_syncemitter_stepcompress():
    ffi_main, ffi_lib = chelper.get_ffi()
    read_fd, write_fd = os.pipe()
    serialqueue = ffi_main.gc(
        ffi_lib.serialqueue_alloc(write_fd, b"u", 0, b"hp_test"),
        ffi_lib.serialqueue_free,
    )
    ssm = ffi_main.gc(
        ffi_lib.steppersyncmgr_alloc(), ffi_lib.steppersyncmgr_free
    )
    ss = ffi_lib.steppersyncmgr_alloc_steppersync(ssm)
    se = ffi_lib.steppersync_alloc_syncemitter(ss, b"hp_test", 1)
    sc = ffi_lib.syncemitter_get_stepcompress(se)
    ffi_lib.stepcompress_hp_enable(sc)
    ffi_lib.stepcompress_fill(sc, 1, MAX_ERROR, 1, 2)
    ffi_lib.steppersync_setup_movequeue(ss, serialqueue, 512)
    ffi_lib.steppersync_set_time(ss, 0.0, MCU_FREQ)

    trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
    ffi_lib.trapq_append(
        trapq,
        0.0,
        ACCEL_T,
        CRUISE_T,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        CRUISE_V,
        ACCEL,
    )
    sk = ffi_main.gc(ffi_lib.cartesian_stepper_alloc(b"x"), ffi_lib.free)
    ffi_lib.itersolve_set_trapq(sk, trapq, STEP_DIST)
    ffi_lib.syncemitter_set_stepper_kinematics(se, sk)

    end_time = ACCEL_T + CRUISE_T
    assert ffi_lib.steppersyncmgr_gen_steps(ssm, end_time, end_time, 0.0) == 0

    history = ffi_main.new("struct pull_history_steps[]", 8192)
    records = ffi_lib.stepcompress_extract_old(sc, history, 8192, 0, 2**63 - 1)
    moves = [history[i] for i in range(records)]
    assert moves
    # Only the high precision protocol emits these parameters
    assert any(m.add2 or m.shift for m in moves)
    expected_steps = _expected_position(end_time) / STEP_DIST
    assert abs(sum(m.step_count for m in moves) - expected_steps) <= 2

    for index in range(1, 40):
        print_time = end_time * index / 40.0
        clock = int(print_time * MCU_FREQ)
        position = ffi_lib.stepcompress_find_past_position(sc, clock)
        assert abs(position * STEP_DIST - _expected_position(print_time)) < 0.05

    ffi_lib.serialqueue_exit(serialqueue)
    os.close(read_fd)
