import pytest


def test_import_extra():
    from klippy import import_test

    with pytest.raises(SystemExit):
        import_test()
