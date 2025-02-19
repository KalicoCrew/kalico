import typing, pathlib
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
