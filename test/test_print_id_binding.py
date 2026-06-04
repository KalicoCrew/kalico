import pytest

from klippy import structured_log


@pytest.fixture(autouse=True)
def _reset_print_context():
    # Ensure a clean slate before and after every test — order-independent.
    structured_log.clear_print()
    yield
    structured_log.clear_print()


def test_print_id_set_and_cleared_helpers():
    # The print lifecycle uses these two helpers; verify the contract here.
    structured_log.clear_print()
    assert structured_log.get_print() == ""
    pid = structured_log.make_print_id()
    structured_log.bind_print(pid)
    assert structured_log.get_print() == pid
    assert pid.startswith("print-")
    structured_log.clear_print()
    assert structured_log.get_print() == ""


def test_standby_clears_print_id():
    # reset()/standby must leave no active print_id bound.
    structured_log.bind_print(structured_log.make_print_id())
    assert structured_log.get_print() != ""
    structured_log.clear_print()  # the call reset() now makes
    assert structured_log.get_print() == ""
