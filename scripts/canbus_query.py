#!/usr/bin/env python3
# Tool to query CAN bus uuids
#
# Copyright (C) 2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os
import optparse
import sys
import time
import errno
import can

CANBUS_ID_ADMIN = 0x3F0
CMD_QUERY_UNASSIGNED = 0x00
CMD_QUERY_UNASSIGNED_EXTENDED = 0x01
RESP_NEED_NODEID = 0x20
RESP_HAVE_NODEID = 0x21
CMD_SET_KLIPPER_NODEID = 0x01
CMD_SET_CANBOOT_NODEID = 0x11

RESP_KALICO_NODEID = 0x07

AppNames = {
    CMD_SET_KLIPPER_NODEID: "Klipper",
    RESP_KALICO_NODEID: "Kalico",
    CMD_SET_CANBOOT_NODEID: "Katapult",
}


def get_can_interfaces():
    try:
        return [
            iface
            for iface in os.listdir("/sys/class/net")
            if iface.startswith("can")
        ]
    except Exception as e:
        print(f"Error listing /sys/class/net: {e}, assuming no interfaces.")
        return []


def query_unassigned(canbus_iface):
    # Open CAN socket
    filters = [
        {"can_id": CANBUS_ID_ADMIN + 1, "can_mask": 0x7FF, "extended": False}
    ]
    try:
        bus = can.interface.Bus(
            channel=canbus_iface, can_filters=filters, bustype="socketcan"
        )
    except OSError as e:
        if e.errno == errno.ENODEV:
            sys.stderr.write(f"Interface {canbus_iface} not found!\n")
            sys.exit(1)
        raise

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
            app_id = RESP_KALICO_NODEID
        app_name = AppNames.get(app_id, "Unknown")
        if node_id:
            sys.stdout.write(
                "[%s] Found canbus_uuid=%012x, Application: %s, Assigned: %02x\n"
                % (canbus_iface, uuid, app_name, node_id)
            )
        else:
            sys.stdout.write(
                "[%s] Found canbus_uuid=%012x, Application: %s, Unassigned\n"
                % (canbus_iface, uuid, app_name)
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
    usage = "%prog [options] [<can interface> <can interface> ...]"
    opts = optparse.OptionParser(usage)
    options, args = opts.parse_args()
    query_ifaces = []
    if len(args) == 0:
        query_ifaces = get_can_interfaces()
        if not query_ifaces:
            sys.stderr.write("No can interfaces found!\n")
            sys.exit(1)
    else:
        query_ifaces = args

    for iface in query_ifaces:
        query_unassigned(iface)


if __name__ == "__main__":
    main()
