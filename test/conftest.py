from __future__ import annotations

import pytest
import klippy.chelper
import pathlib
import typing
import shutil
import os

# Ensure chelper is built
klippy.chelper.get_ffi()


def pytest_addoption(parser):
    parser.addoption(
        "--dictdir",
        action="store",
        default=os.environ.get("DICTDIR", "dict"),
        help="Klipper build dictionary path",
    )


# Use PYTHONPATH to enable the testing plugin
TESTING_PLUGIN = pathlib.Path(__file__).parent / "kalico_testing_plugin"


def pytest_sessionstart(session):
    old_python_path = os.environ.get("PYTHONPATH", "")
    os.environ["PYTHONPATH"] = f"{TESTING_PLUGIN}:{old_python_path}"

    @session.config.add_cleanup
    def clean_symlink():
        os.environ["PYTHONPATH"] = old_python_path


@pytest.fixture
def config_root(request, tmp_path):
    """
    This abuses type hinting to allow fixture usages to specify the source directory

    def test_foo(config_root: Annotated[pathlib.Path, "relative/path/to/my_config"]):
    """

    test_hints = typing.get_type_hints(request.function, include_extras=True)
    my_hint = test_hints[request.fixturename]
    _, src = typing.get_args(my_hint)
    src = pathlib.Path(request.node.fspath).parent / pathlib.Path(src)

    tmp_config_root = tmp_path / "printer"
    shutil.copytree(src, tmp_config_root)
    yield tmp_config_root
