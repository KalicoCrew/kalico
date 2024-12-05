import shutil
import pathlib
import subprocess


class KalicoMigration:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.enabled = config.getboolean("enabled", "True")
        if not self.enabled:
            return

        self.printer.register_event_handler("klippy:ready", self.handle_ready)

        self.root_path = pathlib.Path(__file__).parent.parent.parent
        self.rename_script = self.root_path / "scripts" / "kalico-rename.sh"

        self.gcode = self.printer.lookup_object("gcode")
        self.gcode.register_command(
            "MIGRATE_TO_KALICO", self.cmd_MIGRATE_TO_KALICO, desc=""
        )

    def handle_ready(self):
        self.gcode.respond_info(
            "Danger-Klipper is now Kalico!\n"
            "\n"
            "As part of this exciting new name, we now have a new github repository and website!\n"
            "\n"
            "`MIGRATE_TO_KALICO` will automatically update your installation"
        )

    def cmd_MIGRATE_TO_KALICO(self, gcmd):
        self.gcode.respond_info("Starting the migration!")

        with subprocess.Popen(
            (
                shutil.which("bash"),
                str(self.rename_script),
                str(self.root_path),
            ),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        ) as proc:
            for line in iter(proc.stdout.readline, ""):
                self.gcode.respond_info(line)

        if proc.returncode:
            self.gcode.respond_raw("!! Error during migration")

        else:
            gcmd.respond_info("Migration complete!\n" "Restarting to Kalico")
            self.printer.request_exit("error_exit")


def load_config(config):
    return KalicoMigration(config)
