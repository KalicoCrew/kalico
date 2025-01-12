"""
Kalico Telemetry

Submit anonymized information about your printer to the Kalico project
Configuration options
"""

import json
import logging
import pathlib
import platform
import sys
import shlex
import threading
import urllib.request
import uuid


class KalicoTelementry:
    TELEMETRY_ENDPOINT = "https://telemetry.kalico.gg/collect"

    def __init__(self, config):
        self.printer = config.get_printer()
        self.enabled = config.getboolean("enabled", None)

        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "TELEMETRY_EXAMPLE", self.cmd_TELEMETRY_EXAMPLE, True
        )

        if self.enabled is not True:
            gcode.register_command(
                "ENABLE_TELEMETRY", self.cmd_ENABLE_TELEMETRY
            )

        if self.enabled is not False:
            gcode.register_command(
                "DISABLE_TELEMETRY", self.cmd_DISABLE_TELEMETRY
            )

        if self.enabled is None:
            self.printer.register_event_handler(
                "klippy:ready", self._telemetry_prompt
            )

        if self.enabled:
            self.printer.register_event_handler(
                "klippy:ready", self._handle_ready
            )

    def _telemetry_prompt(self):
        if self.printer.get_start_args().get("debuginput"):
            return

        gcode = self.printer.lookup_object("gcode")

        gcode.respond_info(
            "Welcome to Kalico\n"
            "\n"
            "Kalico is starting an opt-in analytics campaign, collecting anonymous data about our users\n"
            "Submitted data will contain an anonymized copy of your current configuration, enabled"
            " Kalico modules and plugins, and some basic information about the host machine.\n"
            "\n"
            "`ENABLE_TELEMETRY` will opt-in to the campaign\n"
            "`DISABLE_TELEMETRY` will permanently opt-out\n"
            "\n"
            "`TELEMETRY_EXAMPLE` will save a full example of what would be"
            " submitted as 'telemetry.json' in your configuration folder\n"
            "\n"
            "For more information, visit https://kalico.gg/telemetry"
        )

    def cmd_ENABLE_TELEMETRY(self, gcmd):
        "Enable Kalico telemetry reporting"

        configfile = self.printer.lookup_object("configfile")
        configfile.set("telemetry", "enabled", True)

        gcmd.respond_info(
            "Kalico telemetry enabled\n"
            "\n"
            "`SAVE_CONFIG` will update your printer config\n"
            "file with this setting and restart the printer."
        )

    def cmd_DISABLE_TELEMETRY(self, gcmd):
        "Disable Kalico telemetry reporting"

        configfile = self.printer.lookup_object("configfile")
        configfile.set("telemetry", "enabled", False)

        gcmd.respond_info(
            "Kalico telemetry disabled\n"
            "\n"
            "`SAVE_CONFIG` will update your printer config\n"
            "file with this setting and restart the printer."
        )

    def get_status(self, eventtime=None):
        return {"enabled": self.enabled}

    def _handle_ready(self):
        try:
            data = self._collect_telemetry()

        except:
            logging.exception("exception collecting telemetry")

        thread = threading.Thread(target=self._send, args=(data,))
        thread.start()

    def cmd_TELEMETRY_EXAMPLE(self, gcmd):
        "Save an example of the current machine telemetry to 'telemetry.json'"

        filename = (
            pathlib.Path(self.printer.get_start_args()["config_file"]).parent
            / "telemetry.json"
        )
        data = self._collect_telemetry()

        with filename.open("w", encoding="utf-8") as fp:
            json.dump(data, fp, indent=2)

        gcmd.respond_info(f"Example telemetry saved to {filename}")

    def _get_machine_id(self):
        """
        Get the freedesktop machine-id if available, otherwise generate and save our own

        If enabled, /etc/machine-id is automatically generated on first boot
        If not available, a unique ID will be generated and saved to `.machine-id`
         next to your printer.cfg
        """

        MACHINE_ID = pathlib.Path("/etc/machine-id")
        KALICO_MACHINE_ID = (
            pathlib.Path(self.printer.get_start_args()["config_file"]).parent
            / ".machine-id"
        )

        if MACHINE_ID.exists():
            return MACHINE_ID.read_text().strip()
        elif KALICO_MACHINE_ID.exists():
            return KALICO_MACHINE_ID.read_text().strip()
        else:
            generated_id = f"kalico-{uuid.uuid4()}"
            KALICO_MACHINE_ID.write_text(generated_id)
            return generated_id

    def _collect_anonymized_config(self):
        """
        Generate an anonymized version of the current printer configuration.

        Only section and option names are submitted

        A truncated example:

        {
            "danger_options": [
                "autosave_includes",
                "allow_plugin_override",
                "log_bed_mesh_at_startup"
            ],
            "printer": [
                "kinematics",
                "invert_kinematics",
                "max_velocity",
                "max_accel",
                "max_z_velocity",
                "max_z_accel",
                "minimum_cruise_ratio",
                "square_corner_velocity"
            ],
            ...
        }
        """
        configfile = self.printer.lookup_object("configfile")
        raw_config = configfile.status_raw_config

        return {
            section: [option for option in options.keys()]
            for section, options in raw_config.items()
        }

    def _collect_platform(self):
        """
        Collect basic platform information

        {
            "machine": "x86_64",
            "os_release": { ... },
            "version": "#1 SMP PREEMPT_DYNAMIC Debian 6.12~rc6-1~exp1 (2024-11-10)",
            "python": "3.12.7 (main, Nov  8 2024, 17:55:36) [GCC 14.2.0]"
        }
        """
        return {
            "machine": platform.machine(),
            "os_release": self._collect_os_release(),
            "version": platform.version(),
            "python": sys.version,
        }

    def _collect_os_release(self):
        """
        Collect the freedesktop OS-RELEASE information.
        See also `platform.freedesktop_os_release()` (available in Python 3.10+)

        {
            "NAME": "Debian GNU/Linux",
            "ID": "debian",
            "PRETTY_NAME": "Debian GNU/Linux trixie/sid",
            "VERSION_CODENAME": "trixie",
            "HOME_URL": "https://www.debian.org/",
            "SUPPORT_URL": "https://www.debian.org/support",
            "BUG_REPORT_URL": "https://bugs.debian.org/"
        }
        """
        paths = [
            pathlib.Path("/etc/os-release"),
            pathlib.Path("/usr/lib/os-release"),
        ]
        path = next(filter(pathlib.Path.exists, paths), None)
        if not path:
            return

        result = {}
        with path.open("r") as fp:
            for line in fp:
                if "=" not in line:
                    continue
                key, value = line.split("=", maxsplit=1)
                result[key] = shlex.split(value)[0]

        return result

    def _collect_printer_objects(self):
        """
        Collect a list of all enabled objects in the current Kalico runtime

        This may include extra modules not visible in the config, every module loaded
        may request additional modules that may not need configuration.
        """

        return list(self.printer.objects.keys())

    def _collect_telemetry(self):
        """
        Collect the full telemetry for submission

        As well as the above, the current Kalico software version
        """
        machine_id = self._get_machine_id()
        start_args = self.printer.get_start_args()

        return {
            "id": machine_id,
            # the Kalico git version e.g. "v0.12.0-527-g79013da3-dirty"
            "version": start_args.get("software_version", "unknown"),
            # name of the git branch, e.g. "master" or "bleeding-edge-v2"
            "branch": start_args.get("git_branch", "unknown"),
            # e.g. "4 core Intel(R) N100",
            "cpu_info": start_args.get("cpu_info", "unknown"),
            # git_remote, e.g. 'https://github.com/danger-klippers/dangerklipper
            "platform": self._collect_platform(),
            "config": self._collect_anonymized_config(),
            "objects": self._collect_printer_objects(),
        }

    def _send(self, collected_data: dict):
        data = json.dumps(collected_data).encode()
        headers = {
            "Content-Type": "application/json",
            "User-Agent": "Kalico {}".format(collected_data["version"]),
        }

        req = urllib.request.Request(
            self.TELEMETRY_ENDPOINT,
            headers=headers,
            data=data,
        )

        urllib.request.urlopen(req)
        logging.info(f"Telemetry sent to {self.TELEMETRY_ENDPOINT}")


def load_config(config):
    return KalicoTelementry(config)
