#!/usr/bin/env python3
"""Energization-by-motion regression tests.

Pin the architectural invariant: stepper enable callbacks fire iff the
corresponding *motor-slot* has non-zero delta under the same kinematic
transform the runtime uses for stepping (rust/runtime/src/kinematics.rs).

Specifically guards against the CoreXY pure-X regression where motor B
(stepper_y) stayed de-energized because the enable filter was keyed on
cartesian-axis-letter membership instead of post-transform motor motion.
"""
from __future__ import annotations

from klippy import motion_kinematics
from klippy.motion_toolhead import (
    MotionToolhead,
    _MOTOR_SLOT_PREFIXES,
    _name_motor_slot,
    _stepper_motor_slot,
)


# --- motor_deltas: parity with rust/runtime/src/kinematics.rs ---------------


def test_motor_deltas_corexy_pure_x_moves_both_belts():
    # CoreXY belt geometry: A = X + Y, B = X - Y. Pure-X command must
    # produce non-zero deltas on both A and B — that's the whole reason
    # both motors have to be energized on a pure-X cartesian move.
    a, b, z, e = motion_kinematics.motor_deltas("corexy", 10.0, 0.0, 0.0, 0.0)
    assert a == 10.0 and b == 10.0
    assert z == 0.0 and e == 0.0


def test_motor_deltas_corexy_pure_y_moves_both_belts_opposite():
    a, b, z, e = motion_kinematics.motor_deltas("corexy", 0.0, 7.5, 0.0, 0.0)
    assert a == 7.5 and b == -7.5
    assert z == 0.0 and e == 0.0


def test_motor_deltas_corexy_z_and_e_passthrough():
    a, b, z, e = motion_kinematics.motor_deltas("corexy", 0.0, 0.0, 3.0, 2.0)
    assert a == 0.0 and b == 0.0
    assert z == 3.0 and e == 2.0


def test_motor_deltas_corexy_mirrors_rust_transform_cases():
    # Bit-for-bit match against the test vectors in
    # rust/runtime/src/kinematics.rs::corexy_with_e_round_trip. If this
    # diverges, the host enable decision can disagree with the runtime
    # stepping decision — which is exactly the class of bug this fix
    # exists to make impossible.
    cases = [
        ((0.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0)),
        ((1.0, 0.0, 0.0, 0.0), (1.0, 1.0, 0.0, 0.0)),
        ((0.0, 1.0, 0.0, 0.0), (1.0, -1.0, 0.0, 0.0)),
        ((1.5, 2.5, 3.0, 7.0), (4.0, -1.0, 3.0, 7.0)),
        ((-3.0, 4.0, 1.0, -2.0), (1.0, -7.0, 1.0, -2.0)),
    ]
    for (dx, dy, dz, de), expected in cases:
        assert motion_kinematics.motor_deltas("corexy", dx, dy, dz, de) == expected


def test_motor_deltas_cartesian_is_identity():
    assert motion_kinematics.motor_deltas(
        "cartesian", 1.0, 2.0, 3.0, 4.0
    ) == (1.0, 2.0, 3.0, 4.0)


def test_motor_deltas_unknown_kinematic_falls_back_to_cartesian():
    # hybrid_corexy is accepted by BridgeKinematics but
    # _configure_axes_per_mcu treats it as not-yet-driven; until the
    # runtime supports it, motor_deltas falls through to the cartesian
    # identity (which matches the documented bridge behavior).
    assert motion_kinematics.motor_deltas(
        "hybrid_corexy", 1.0, 2.0, 3.0, 4.0
    ) == (1.0, 2.0, 3.0, 4.0)


# --- Slot mapping: name → motor slot ----------------------------------------


def test_motor_slot_prefix_order_is_canonical():
    # Slot order is load-bearing: motion_kinematics.motor_deltas, the
    # runtime config blob, and the enable filter all index this same
    # tuple. Pin it explicitly.
    assert _MOTOR_SLOT_PREFIXES == (
        (0, "stepper_x"),
        (1, "stepper_y"),
        (2, "stepper_z"),
        (3, "extruder"),
    )


def test_name_motor_slot_primary_steppers():
    assert _name_motor_slot("stepper_x") == (0, True)
    assert _name_motor_slot("stepper_y") == (1, True)
    assert _name_motor_slot("stepper_z") == (2, True)
    assert _name_motor_slot("extruder") == (3, True)


def test_name_motor_slot_awd_partners_share_primary_slot():
    assert _name_motor_slot("stepper_x1") == (0, False)
    assert _name_motor_slot("stepper_y1") == (1, False)
    assert _name_motor_slot("stepper_z1") == (2, False)
    assert _name_motor_slot("stepper_z2") == (2, False)
    assert _name_motor_slot("stepper_z3") == (2, False)


def test_name_motor_slot_rejects_unrelated_names():
    # Random "stepper_*" sections we don't own (e.g. mock test fixtures)
    # must not silently land in a motor slot.
    assert _name_motor_slot("stepper_a") is None
    assert _name_motor_slot("stepper_xa") is None
    assert _name_motor_slot("manual_stepper") is None
    assert _name_motor_slot("") is None


class _FakeStepper:
    def __init__(self, name):
        self._name = name

    def get_name(self):
        return self._name


def test_stepper_motor_slot_extracts_slot_from_stepper_object():
    assert _stepper_motor_slot(_FakeStepper("stepper_x")) == 0
    assert _stepper_motor_slot(_FakeStepper("stepper_x1")) == 0
    assert _stepper_motor_slot(_FakeStepper("stepper_y")) == 1
    assert _stepper_motor_slot(_FakeStepper("stepper_z2")) == 2
    assert _stepper_motor_slot(_FakeStepper("extruder")) == 3
    assert _stepper_motor_slot(_FakeStepper("stepper_a")) is None


# --- _fire_active_callbacks: integration via stub MotionToolhead ------------


class _FakeStepperWithCallback(_FakeStepper):
    def __init__(self, name):
        super().__init__(name)
        self._active_callbacks = []
        self.fired = []  # captures print_time of every callback fire

    def add_active_callback(self, cb):
        self._active_callbacks.append(cb)


class _FakeKin:
    def __init__(self, steppers):
        self._steppers = steppers

    def get_steppers(self):
        return self._steppers


class _StubToolhead:
    """Minimal shape of MotionToolhead that _fire_active_callbacks needs.

    We bind the real `_fire_active_callbacks` to this stub so the test
    exercises the actual code path without booting klippy.
    """

    def __init__(self, kinematics_name, steppers):
        self.kinematics_name = kinematics_name
        self.kin = _FakeKin(steppers)

    def get_last_move_time(self):
        return 1.0


def _register_enable_callbacks(steppers):
    """Wire a capturing callback onto every stepper, mimicking
    stepper_enable.StepperEnable.motor_enable registration."""
    for s in steppers:
        s.add_active_callback(lambda t, s=s: s.fired.append(t))


def _make_corexy_steppers():
    return [
        _FakeStepperWithCallback("stepper_x"),
        _FakeStepperWithCallback("stepper_x1"),  # AWD partner on motor A
        _FakeStepperWithCallback("stepper_y"),
        _FakeStepperWithCallback("stepper_y1"),  # AWD partner on motor B
        _FakeStepperWithCallback("stepper_z"),
    ]


def test_corexy_pure_x_move_energizes_both_belts_and_all_awd_partners():
    """The bug this fix exists to prevent. Pure X command on CoreXY:
    motor_deltas = (dx, dx, 0, 0) → both A-slot and B-slot steppers
    (including AWD partners) energize; Z stays de-energized.
    """
    steppers = _make_corexy_steppers()
    _register_enable_callbacks(steppers)
    stub = _StubToolhead("corexy", steppers)
    MotionToolhead._fire_active_callbacks(stub, 10.0, 0.0, 0.0, 0.0)
    fired = {s._name: bool(s.fired) for s in steppers}
    assert fired == {
        "stepper_x": True,
        "stepper_x1": True,
        "stepper_y": True,
        "stepper_y1": True,
        "stepper_z": False,
    }


def test_corexy_pure_y_move_also_energizes_both_belts():
    steppers = _make_corexy_steppers()
    _register_enable_callbacks(steppers)
    stub = _StubToolhead("corexy", steppers)
    MotionToolhead._fire_active_callbacks(stub, 0.0, 5.0, 0.0, 0.0)
    fired = {s._name: bool(s.fired) for s in steppers}
    assert fired == {
        "stepper_x": True,
        "stepper_x1": True,
        "stepper_y": True,
        "stepper_y1": True,
        "stepper_z": False,
    }


def test_corexy_pure_z_move_only_energizes_z():
    steppers = _make_corexy_steppers()
    _register_enable_callbacks(steppers)
    stub = _StubToolhead("corexy", steppers)
    MotionToolhead._fire_active_callbacks(stub, 0.0, 0.0, 3.0, 0.0)
    fired = {s._name: bool(s.fired) for s in steppers}
    assert fired == {
        "stepper_x": False,
        "stepper_x1": False,
        "stepper_y": False,
        "stepper_y1": False,
        "stepper_z": True,
    }


def test_cartesian_pure_x_move_energizes_only_x():
    steppers = [
        _FakeStepperWithCallback("stepper_x"),
        _FakeStepperWithCallback("stepper_y"),
        _FakeStepperWithCallback("stepper_z"),
    ]
    _register_enable_callbacks(steppers)
    stub = _StubToolhead("cartesian", steppers)
    MotionToolhead._fire_active_callbacks(stub, 10.0, 0.0, 0.0, 0.0)
    assert bool(steppers[0].fired) is True
    assert bool(steppers[1].fired) is False
    assert bool(steppers[2].fired) is False


def test_zero_motion_is_a_noop():
    steppers = _make_corexy_steppers()
    _register_enable_callbacks(steppers)
    stub = _StubToolhead("corexy", steppers)
    MotionToolhead._fire_active_callbacks(stub, 0.0, 0.0, 0.0, 0.0)
    for s in steppers:
        assert s.fired == []
        # Critically, the callback must NOT be drained on a no-op call —
        # otherwise the next real move would have nothing to fire.
        assert len(s._active_callbacks) == 1


def test_callbacks_are_drained_once_per_fire():
    # Matches the "fire-then-clear" pattern in
    # stepper.generate_steps / stepper_enable: enable is one-shot per
    # arm; the next register-then-fire cycle has to re-add the callback.
    steppers = _make_corexy_steppers()
    _register_enable_callbacks(steppers)
    stub = _StubToolhead("corexy", steppers)
    MotionToolhead._fire_active_callbacks(stub, 10.0, 0.0, 0.0, 0.0)
    for s in steppers[:4]:  # X/Y belts + AWD partners
        assert s._active_callbacks == []
    # Z was never fired, so its callback should still be queued.
    assert len(steppers[4]._active_callbacks) == 1
