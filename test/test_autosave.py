import pathlib
import typing

import pytest
from klippy_testing import PrinterShim, Restart


def test_autosave_includes(
    config_root: typing.Annotated[pathlib.Path, "test_configs/autosave"],
):
    start_args = {"config_file": str(config_root / "printer.cfg")}
    with PrinterShim(start_args) as printer:
        pconfig = printer.lookup_object("configfile")
        config = printer.load_config()
        assert (
            config.getsection("danger_options").getboolean("temp_ignore_limits")
            is True
        )

        pconfig.set("danger_options", "temp_ignore_limits", "False")
        assert pconfig.status_save_pending == {
            "danger_options": {"temp_ignore_limits": "False"}
        }
        with pytest.raises(Restart):
            printer.call("SAVE_CONFIG")

    with PrinterShim(start_args) as printer:
        config = printer.load_config()

        # Test that the changed config value is properly saved
        assert (
            config.getsection("danger_options").getboolean("temp_ignore_limits")
            is False
        )

        # Test that the autosave line is now in printer.cfg
        assert (
            "#*# temp_ignore_limits = False"
            in (config_root / "printer.cfg").read_text()
        )
        # Test that the source line in danger_options.cfg is commented out
        assert (
            "#temp_ignore_limits: True"
            in (config_root / "danger_options.cfg").read_text()
        )


HEADER = """#*# <---------------------- SAVE_CONFIG ---------------------->
#*# DO NOT EDIT THIS BLOCK OR BELOW. The contents are auto-generated.
#*#
"""
REGULAR = """[danger_options]
temp_ignore_limits: True

"""


def _autosave_values(printer):
    pconfig = printer.lookup_object("configfile")
    printer.load_config()
    fileconfig = pconfig.autosave.fileconfig
    return {s: dict(fileconfig.items(s)) for s in fileconfig.sections()}


def _save(printer, section, option, value):
    pconfig = printer.lookup_object("configfile")
    pconfig.set(section, option, value)
    with pytest.raises(Restart):
        printer.call("SAVE_CONFIG")


def test_autosave_multiple_clean_blocks(tmp_path: pathlib.Path):
    cfg = tmp_path / "printer.cfg"
    cfg.write_text(
        REGULAR
        + HEADER
        + "#*# [foo]\n#*# a = 1\n#*#\n#*# [bar]\n#*# b = 2\n#*#\n"
        + HEADER
        + "#*# [bar]\n#*# b = 3\n"
    )
    start_args = {"config_file": str(cfg)}
    with PrinterShim(start_args) as printer:
        # Blocks are merged, later values take precedence
        assert _autosave_values(printer) == {
            "foo": {"a": "1"},
            "bar": {"b": "3"},
        }
        assert not printer.lookup_object("configfile").runtime_warnings
        _save(printer, "foo", "a", "4")

    # Both blocks are replaced by a single one
    data = cfg.read_text()
    assert data.count("SAVE_CONFIG ---") == 1
    assert data.startswith(REGULAR.rstrip() + "\n")
    with PrinterShim(start_args) as printer:
        assert _autosave_values(printer) == {
            "foo": {"a": "4"},
            "bar": {"b": "3"},
        }


@pytest.mark.parametrize(
    "block",
    [
        pytest.param(
            HEADER + "#*# [foo]\n\n#*# a = 1\n", id="blank-line-in-block"
        ),
        pytest.param(
            HEADER.replace("--->\n", "---> \n") + "#*# [foo]\n#*# a = 1\n",
            id="trailing-space-on-header",
        ),
        pytest.param(
            HEADER.replace("#*#\n", "#*#\t\n") + "#*# [foo]\n#*# a = 1\n",
            id="trailing-tab-on-header",
        ),
        pytest.param(
            HEADER + "#*# [foo]\n#*# a = 1  \n", id="nbsp-after-prefix"
        ),
        pytest.param(
            HEADER + "#*# [foo]\n#*# a = 1\n\n\n  \n", id="trailing-blanks"
        ),
    ],
)
def test_autosave_tolerates_whitespace(tmp_path: pathlib.Path, block):
    cfg = tmp_path / "printer.cfg"
    cfg.write_text(REGULAR + block)
    with PrinterShim({"config_file": str(cfg)}) as printer:
        assert _autosave_values(printer) == {"foo": {"a": "1"}}
        assert not printer.lookup_object("configfile").runtime_warnings


def test_autosave_header_at_start_of_file(tmp_path: pathlib.Path):
    cfg = tmp_path / "printer.cfg"
    cfg.write_text(HEADER + "#*# [foo]\n#*# a = 1\n")
    with PrinterShim({"config_file": str(cfg)}) as printer:
        assert _autosave_values(printer) == {"foo": {"a": "1"}}


@pytest.mark.parametrize(
    "data",
    [
        pytest.param(
            REGULAR + HEADER + "#*# [foo]\n#*# a = 1\n\n[baz]\nc: 3\n",
            id="config-after-block",
        ),
        pytest.param(
            REGULAR
            + HEADER
            + "#*# [foo]\n#*# a = 1\n\n[baz]\nc: 3\n\n"
            + HEADER
            + "#*# [foo]\n#*# a = 2\n",
            id="config-between-blocks",
        ),
        pytest.param(
            REGULAR + "#*# [foo]\n#*# a = 1\n", id="block-without-header"
        ),
        pytest.param(
            REGULAR + "  " + HEADER.replace("\n", "\n  ") + "#*# [foo]\n",
            id="indented-block",
        ),
    ],
)
def test_autosave_corrupted_refuses_save(tmp_path: pathlib.Path, data):
    cfg = tmp_path / "printer.cfg"
    cfg.write_text(data)
    with PrinterShim({"config_file": str(cfg)}) as printer:
        pconfig = printer.lookup_object("configfile")
        # Corrupted autosave data is ignored, with a visible warning
        assert _autosave_values(printer) == {}
        assert len(pconfig.runtime_warnings) == 1
        assert (
            "Ignoring SAVE_CONFIG data"
            in (pconfig.runtime_warnings[0]["message"])
        )
        # SAVE_CONFIG must not append another block
        pconfig.set("foo", "a", "5")
        with pytest.raises(Exception, match="Unable to SAVE_CONFIG"):
            printer.call("SAVE_CONFIG")
    assert cfg.read_text() == data
