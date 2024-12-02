#!/usr/bin/env python2
# Tool to query CAN bus uuids
#
# Copyright (C) 2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import optparse
import sys
import time
import can

CANBUS_ID_ADMIN = 0x3F0
CMD_QUERY_UNASSIGNED = 0x00
CMD_QUERY_UNASSIGNED_EXTENDED = 0x01
RESP_NEED_NODEID = 0x20
RESP_HAVE_NODEID = 0x21
CMD_SET_KLIPPER_NODEID = 0x01
CMD_SET_CANBOOT_NODEID = 0x11

RESP_DANGER_NODEID = 0x07

AppNames = {
    CMD_SET_KLIPPER_NODEID: "Klipper",
    RESP_DANGER_NODEID: "Danger-Klipper",
    CMD_SET_CANBOOT_NODEID: "CanBoot",
}


def query_unassigned(canbus_iface):
    # Open CAN socket
    filters = [
        {"can_id": CANBUS_ID_ADMIN + 1, "can_mask": 0x7FF, "extended": False}
    ]
    bus = can.interface.Bus(
        channel=canbus_iface, can_filters=filters, bustype="socketcan"
    )
    # Send query
    msg = can.Message(
        arbitration_id=CANBUS_ID_ADMIN,
        data=[CMD_QUERY_UNASSIGNED, CMD_QUERY_UNASSIGNED_EXTENDED],
        is_extended_id=False,
    )
    bus.send(msg)
    # Read responses
    found_ids = {}
    start_time = curtime = time.time()
    while 1:
        tdiff = start_time + 2.0 - curtime
        if tdiff <= 0.0:
            break
        msg = bus.recv(tdiff)
        curtime = time.time()
        if (
            msg is None
            or msg.arbitration_id != CANBUS_ID_ADMIN + 1
            or msg.dlc < 7
            or msg.data[0] not in (RESP_NEED_NODEID, RESP_HAVE_NODEID)
        ):
            continue
        uuid = sum([v << ((5 - i) * 8) for i, v in enumerate(msg.data[1:7])])
        if uuid in found_ids:
            continue
        found_ids[uuid] = 1
        app_id = CMD_SET_KLIPPER_NODEID
        node_id = None
        if msg.dlc > 7:
            app_id = msg.data[7]
        if msg.data[0] == RESP_HAVE_NODEID:
            node_id = app_id
            app_id = RESP_DANGER_NODEID
        app_name = AppNames.get(app_id, "Unknown")
        if node_id:
            sys.stdout.write(
                "Found canbus_uuid=%012x, Application: %s, Assigned: %02x\n"
                % (uuid, app_name, node_id)
            )
        else:
            sys.stdout.write(
                "Found canbus_uuid=%012x, Application: %s, Unassigned\n"
                % (uuid, app_name)
            )
    sys.stdout.write(
        "Total %d uuids found\n"
        % (
            len(
                found_ids,
            )
        )
    )


def main():
    usage = "%prog [options] <can interface>"
    opts = optparse.OptionParser(usage)
    options, args = opts.parse_args()
    if len(args) != 1:
        opts.error("Incorrect number of arguments")
    canbus_iface = args[0]
    query_unassigned(canbus_iface)


if __name__ == "__main__":
    main()
