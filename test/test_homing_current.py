from klippy.extras.homing import Homing


class _FakeToolhead:
    def __init__(self):
        self.dwells = []

    def get_last_move_time(self):
        return 7.5

    def dwell(self, delay):
        self.dwells.append(delay)


class _FakeCurrentHelper:
    def __init__(self, dwell_time):
        self._dwell_time = dwell_time
        self.calls = []

    def set_current_for_homing(self, print_time, pre_homing):
        self.calls.append((print_time, pre_homing))
        return self._dwell_time


class _FakeRail:
    def __init__(self, helpers):
        self._helpers = helpers

    def get_tmc_current_helpers(self):
        return self._helpers


def _homing():
    return Homing.__new__(Homing)


def test_applies_to_every_helper_and_dwells_for_the_slowest():
    fast = _FakeCurrentHelper(0.5)
    slow = _FakeCurrentHelper(1.0)
    toolhead = _FakeToolhead()

    _homing()._set_homing_current(
        toolhead, _FakeRail([fast, slow]), pre_homing=True
    )

    assert fast.calls == [(7.5, True)]
    assert slow.calls == [(7.5, True)]
    assert toolhead.dwells == [1.0]


def test_skips_steppers_without_tmc_drivers():
    helper = _FakeCurrentHelper(0.5)
    toolhead = _FakeToolhead()

    _homing()._set_homing_current(
        toolhead, _FakeRail([None, helper]), pre_homing=False
    )

    assert helper.calls == [(7.5, False)]
    assert toolhead.dwells == [0.5]


def test_no_dwell_when_no_current_change_needed():
    toolhead = _FakeToolhead()

    _homing()._set_homing_current(
        toolhead, _FakeRail([_FakeCurrentHelper(0.0)]), pre_homing=True
    )

    assert toolhead.dwells == []
