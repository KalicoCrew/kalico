# EtherCAT node — a kalico-native motion endpoint reached over a Unix socket.
#
# An EtherCAT node is NOT a Klipper MCU: it has no GPIO pins and is not a
# command-queue MCU. It is the connection a [servo_<axis>] device (Part A,
# Task 8) binds to. The node is claimed with the Rust motion_bridge during
# the mcu-identify phase, mirroring how serial MCUs claim themselves in
# MCU._mcu_identify; the claim returns a handle the motion toolhead reads
# later (Task 9) when it builds the planner.
#
# Part A scope is motion only. DI / temperature / status are future
# capabilities and are intentionally not implemented here.
import logging


class EtherCatNode:
    def __init__(self, config):
        self.printer = config.get_printer()
        # [ethercat_node node_x] -> node_x
        self.name = config.get_name().split()[-1]
        socket_path = config.get("socket").strip()
        if not socket_path:
            raise config.error(
                "ethercat_node %s: 'socket' must be a non-empty path"
                % (self.name,)
            )
        self.socket_path = socket_path
        self.bridge_handle = None
        # Claim during mcu-identify. printer._connect sends
        # "klippy:mcu_identify" before invoking the "klippy:connect"
        # handlers (klippy/printer.py), and motion_toolhead._init_planner
        # runs on "klippy:connect" — so the handle is populated before the
        # planner is built. This mirrors MCU._mcu_identify's claim_mcu call.
        self.printer.register_event_handler("klippy:mcu_identify", self._claim)

    def _claim(self):
        if self.bridge_handle is not None:
            return
        bridge = self.printer.lookup_object("motion_bridge")
        self.bridge_handle = bridge.claim_ethercat_node(
            self.name, self.socket_path
        )
        logging.info(
            "ethercat_node %s: claimed handle=%s socket=%s",
            self.name,
            self.bridge_handle,
            self.socket_path,
        )

    def get_bridge_handle(self):
        return self.bridge_handle


def load_config_prefix(config):
    return EtherCatNode(config)
