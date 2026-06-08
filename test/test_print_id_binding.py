import pytest

from klippy import structured_log


@pytest.fixture(autouse=True)
def _reset_print_context():
    structured_log.clear_print()
    yield
    structured_log.clear_print()


def test_print_id_set_and_cleared_helpers():
    structured_log.clear_print()
    assert structured_log.get_print() == ""
    pid = structured_log.make_print_id()
    structured_log.bind_print(pid)
    assert structured_log.get_print() == pid
    assert pid.startswith("print-")
    structured_log.clear_print()
    assert structured_log.get_print() == ""


def test_standby_clears_print_id():
    structured_log.bind_print(structured_log.make_print_id())
    assert structured_log.get_print() != ""
    structured_log.clear_print()
    assert structured_log.get_print() == ""
