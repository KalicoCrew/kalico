#!/usr/bin/env python

import argparse
import os
import sys
import termios
import time
import subprocess
import struct

SELF_DIR = os.path.dirname(__file__)


def serial_sys_devpath(device_path):
    device_name = os.path.basename(os.path.realpath(device_path))
    try:
        sys_path = os.path.realpath(
            os.path.join("/sys/class/tty/", device_name, "device", "..")
        )
    except OSError as e:
        return None
    return sys_path


def read_dev_file(devpath, file):
    target = os.path.join(devpath, file)
    if not os.path.exists(target):
        return None
    with open(target) as f:
        return f.read().strip()


def check_device_is_beacon(devpath):
    manufacturer = read_dev_file(devpath, "manufacturer")
    vendor = read_dev_file(devpath, "idVendor")
    if manufacturer != "Beacon" and vendor != "04d8":
        return False
    product = read_dev_file(devpath, "product")
    if product is None or not product.startswith("Beacon "):
        return False
    rev = product[7:].lower()
    if not rev.startswith("rev"):
        return False
    return rev[3:].split(" ")[0]


def get_device_fw_version(devpath):
    raw = read_dev_file(devpath, "bcdDevice")
    return (int(raw[0]), int(raw[1] + raw[2]), int(raw[3]))


def fw_path(rev):
    return os.path.join(SELF_DIR, "firmware", "rev" + rev + ".dfu")


def get_fw_file_version(fwpath):
    file_size = os.stat(fwpath).st_size
    with open(fwpath, "rb") as f:
        f.seek(file_size - 16)
        file_version = bytes(f.read(2))
    (lo, hi) = struct.unpack("BB", file_version)
    return (((hi >> 4) & 0xF), (hi & 0xF) * 10 + ((lo >> 4) & 0xF), (lo & 0xF))


def format_fw_version(version):
    return "%d.%d.%d" % version


def enter_bootloader(device_path):
    with open(device_path, "rb") as f:
        fd = f.fileno()
        t = termios.tcgetattr(fd)
        t[4] = t[5] = termios.B1200
        termios.tcsetattr(fd, termios.TCSANOW, t)


def wait_bootloader(sys_path):
    timeout = time.time() + 5
    while 1:
        time.sleep(0.1)
        if not os.path.exists(sys_path):
            continue
        product = read_dev_file(sys_path, "product")
        if "Bootloader" in product:
            return
        if time.time() >= timeout:
            return


def flash(fw_path, sudo):
    args = ["dfu-util", "-e", "-D", fw_path]
    if sudo:
        args.insert(0, "sudo")
    print("Running dfu-util: %s" % (" ".join(args),))
    p = subprocess.Popen(args, stderr=subprocess.PIPE)
    out, err = p.communicate()
    if p.returncode != 0:
        err = err.decode()
        if (
            not "Error sending completion packet" in err
            and not "unable to read DFU status after completion" in err
        ):
            print(err)
            raise Exception("Flashing failed, dfu-util returned %d" % (p.returncode,))
        else:
            print("\nDownload done.")


def do_update(device_path, sys_path, rev, no_sudo, force):
    if device_path is not None:
        actual_fw_version = format_fw_version(get_device_fw_version(sys_path))
        desired_fw_version = format_fw_version(get_fw_file_version(fw_path(rev)))
        if not force and actual_fw_version == desired_fw_version:
            serial = read_dev_file(sys_path, "serial")
            print(
                "Beacon '%s' is already flashed with the current firmware version '%s'. Skipping.\n"
                "To force update, re-run `update_firmware.py` with the `--force` flag set."
                % (serial, actual_fw_version)
            )
            return

        enter_bootloader(device_path)
        wait_bootloader(sys_path)
    flash(os.path.join(SELF_DIR, "firmware", "rev" + rev + ".dfu"), not no_sudo)


def find_beacons():
    devices = []
    prefix = "/sys/bus/usb/devices"
    for dev in os.listdir(prefix):
        try:
            sys_path = os.path.realpath(os.path.join(prefix, dev))
        except OSError as e:
            continue
        rev = check_device_is_beacon(sys_path)
        if rev == False:
            continue
        device_path = None
        for sub in os.listdir(sys_path):
            tty_path = os.path.realpath(os.path.join(sys_path, sub, "tty"))
            if not os.path.exists(tty_path):
                continue
            try:
                device_name = os.listdir(tty_path).pop()
            except IndexError:
                continue
            device_path = "/dev/" + device_name
            break

        devices.append((device_path, sys_path, rev))
    return devices


def task_check(device_path):
    sys_devpath = serial_sys_devpath(device_path)
    if sys_devpath is None:
        print("Could not look up syspath for device")
        sys.exit(255)

    rev = check_device_is_beacon(sys_devpath)
    if rev == False:
        print("Device does not appear to be a Beacon")
        sys.exit(255)

    actual_fw_version = format_fw_version(get_device_fw_version(sys_devpath))
    desired_fw_version = format_fw_version(
        get_fw_file_version(os.path.join(SELF_DIR, "firmware", "rev" + rev + ".dfu"))
    )

    if actual_fw_version != desired_fw_version:
        print(
            "Outdated Beacon firmware version %s, current version is %s.\n"
            "Please run `install.sh` or `update_firmware.py update all` to update to the latest version.\n"
            "Using an outdated firmware version can result in instability or failures."
            % (actual_fw_version, desired_fw_version)
        )


def task_update(device_path, no_sudo, force):
    if device_path == "all":
        for device_path, sys_path, rev in find_beacons():
            do_update(device_path, sys_path, rev, no_sudo, force)
    else:
        sys_devpath = serial_sys_devpath(device_path)
        if sys_devpath is None:
            print("Could not find sys entry for given device")
            sys.exit(255)

        rev = check_device_is_beacon(sys_devpath)
        if rev == False:
            print("Given device does not appear to be a Beacon")
            sys.exit(255)
        do_update(device_path, sys_devpath, rev, no_sudo, force)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Beacon firmware updater")
    subparsers = parser.add_subparsers(dest="command")
    subparsers.required = True

    parser_check = subparsers.add_parser("check")
    parser_check.add_argument("device_path")

    parser_update = subparsers.add_parser("update")
    parser_update.add_argument("device_path")
    parser_update.add_argument("--no-sudo", default=False, action="store_true")
    parser_update.add_argument("--force", default=False, action="store_true")

    kwargs = vars(parser.parse_args())
    sub = kwargs.pop("command")
    globals()["task_" + sub](**kwargs)
