import logging
import os

from . import servo_axis

# Default endpoint binary: ethercat_node.py lives at
# <repo>/klippy/extras/, so three os.path.dirname hops reach <repo>.
_REPO_ROOT = os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
)
_DEFAULT_ENDPOINT = os.path.join(
    _REPO_ROOT, "rust", "target", "release", "kalico-ethercat-rt"
)


class EtherCatNode:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        socket_path = config.get("socket").strip()
        if not socket_path:
            raise config.error(
                "ethercat_node %s: 'socket' must be a non-empty path"
                % (self.name,)
            )
        self.socket_path = socket_path
        interface = config.get("interface").strip()
        if not interface:
            raise config.error(
                "ethercat_node %s: 'interface' must be a non-empty "
                "NIC name (e.g. eth0)" % (self.name,)
            )
        self.interface = interface
        self.endpoint = os.path.abspath(
            config.get("endpoint", _DEFAULT_ENDPOINT)
        )
        self.bridge_handle = None
        # Derived at claim time, not __init__: the [servo_*] sections are parsed
        # by the toolhead AFTER [ethercat_node] sections (printer._read_config
        # loads prefix sections before motion_toolhead), so the matching
        # ServoRail does not exist yet here.
        self._counts_per_mm = None
        # Claim during mcu-identify. printer._connect sends
        # "klippy:mcu_identify" before invoking the "klippy:connect"
        # handlers (klippy/printer.py), and motion_toolhead._init_planner
        # runs on "klippy:connect" — so the handle is populated before the
        # planner is built. This mirrors MCU._mcu_identify's claim_mcu call.
        self.printer.register_event_handler("klippy:mcu_identify", self._claim)
        self.printer.load_object(config, "servo_param")

    def _find_rail(self):
        # ServoRails are not printer objects (the toolhead builds them directly
        # into kin.rails), so iterate the toolhead's rails rather than
        # printer.lookup_objects.
        toolhead = self.printer.lookup_object("toolhead")
        for rail in getattr(toolhead.get_kinematics(), "rails", ()):
            if (
                isinstance(rail, servo_axis.ServoRail)
                and rail.get_node_name() == self.name
            ):
                return rail
        raise self.printer.config_error(
            "ethercat_node %s: no [servo_*] section with node=%s — "
            "cannot derive counts_per_mm" % (self.name, self.name)
        )

    def _claim(self):
        if self.bridge_handle is not None:
            return
        rail = self._find_rail()
        self._counts_per_mm = rail.get_counts_per_mm()
        bridge = self.printer.lookup_object("motion_bridge")
        try:
            self.bridge_handle = bridge.claim_ethercat_node(
                self.name,
                self.socket_path,
                self.interface,
                self.endpoint,
                self._counts_per_mm,
            )
        except RuntimeError as e:
            raise self.printer.config_error(str(e))
        logging.info(
            "ethercat_node %s: claimed handle=%s socket=%s interface=%s "
            "endpoint=%s counts_per_mm=%s",
            self.name,
            self.bridge_handle,
            self.socket_path,
            self.interface,
            self.endpoint,
            self._counts_per_mm,
        )
        self._push_drive_params(rail)

    def _push_drive_params(self, rail):
        params = rail.get_sdo_params()
        if not params:
            return
        bridge = self.printer.lookup_object("motion_bridge")
        for index, subindex, size, value in params:
            try:
                bridge.sdo_write(
                    self.bridge_handle, index, subindex, size, value
                )
            except RuntimeError as e:
                raise self.printer.config_error(
                    "ethercat_node %s: claim-time drive param "
                    "0x%04x.%d = %d failed: %s"
                    % (self.name, index, subindex, value, e)
                )
            logging.info(
                "ethercat_node %s: drive param 0x%04x.%d = %d pushed",
                self.name,
                index,
                subindex,
                value,
            )

    def get_bridge_handle(self):
        return self.bridge_handle

    def get_counts_per_mm(self):
        return self._counts_per_mm


def load_config_prefix(config):
    return EtherCatNode(config)
