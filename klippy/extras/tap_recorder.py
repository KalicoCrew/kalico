# Load Cell Probe
#
# Copyright (C) 2025  Gareth Farrington <gareth@waves.ky>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import json
import logging
from pathlib import Path


class TapRecorder:
    def __init__(self, config):
        self._is_recording = False
        self._tap_file_name = None
        self._file_handle = None
        self._printer = config.get_printer()
        self._probe_name = config.get(
            "load_cell_probe_name", default="load_cell_probe"
        )
        self._register_commands()
        self._printer.register_event_handler("klippy:connect", self._on_connect)
        self._printer.register_event_handler(
            "klippy:disconnect", self._on_disconnect
        )

    def _on_connect(self):
        load_cell_probe = self._printer.lookup_object(self._probe_name)
        load_cell_probe.add_client(self._on_tap)

    def _on_disconnect(self):
        self._close_file()

    def _on_tap(self, tap_event: dict):
        if self._is_recording:
            self._save_tap(tap_event["tap"])
        return True

    def _save_tap(self, tap_data):
        try:
            self._file_handle.write(json.dumps(tap_data) + "\n")
            self._file_handle.flush()
        except Exception as e:
            logging.error("Failed to write tap data: %s", e)
            raise e

    def _close_file(self):
        if self._file_handle is not None:
            try:
                self._file_handle.close()
            except Exception as e:
                logging.error("Failed to close file: %s", e)
            finally:
                self._file_handle = None

    cmd_start_recording_help = "Start recording tap data"

    def cmd_start_recording(self, gcmd):
        full_path = "_"
        try:
            self._tap_file_name = gcmd.get("FILE", default=None)
            file_path = Path(self._tap_file_name).resolve()
            file_path.parent.mkdir(parents=True, exist_ok=True)
            full_path = str(file_path)
            gcmd.respond_info("Saving tap data to %s" % (full_path,))
            self._file_handle = file_path.open(
                "a", encoding="utf-8", buffering=1
            )
            self._is_recording = True
        except Exception as e:
            error_msg = "Failed to open file %s: %s" % (full_path, e)
            raise gcmd.error(error_msg)

    cmd_stop_recording_help = "Stop recording tap data"

    def cmd_stop_recording(self, gcmd):
        self._is_recording = False
        self._close_file()

    def _register_commands(self):
        # Register commands
        gcode = self._printer.lookup_object("gcode")
        gcode.register_command(
            "TAP_RECORDER_START",
            self.cmd_start_recording,
            desc=self.cmd_start_recording_help,
        )
        gcode.register_command(
            "TAP_RECORDER_STOP",
            self.cmd_stop_recording,
            desc=self.cmd_stop_recording_help,
        )


def load_config(config):
    return TapRecorder(config)
