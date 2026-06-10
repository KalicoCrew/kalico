---
name: neptune-bench
description: Use when you need the Neptune 3 Pro test bench's address (ssh/host URL) or its flash script — "what's the Neptune bench URL?", "ssh into the Neptune", "flash the Neptune". Gives the host + plug and points at the ready-made flash script next to this skill.
---

# Neptune 3 Pro test bench

Secondary bring-up bench — and the **EtherCAT servo bench**: the X axis is an
A6-EC servo drive (`[ethercat_node node_x]` over `eth0`, `[servo_x]` with
131072 counts/rev), not a stepper. Y/Z/E remain steppers on the stock board.
Elegoo Neptune 3 Pro bedslinger, ZNP Robin Nano DW v2.2, STM32F401RCT6.
ST-Link/SWD flash with NRST disconnected.

| | |
|---|---|
| **SSH host** | `dderg@ethercatpi5.local` |
| **Repo on Pi** | `~/kalico` |
| **Sudo password** | `password` |
| **Smart plug** | macOS Shortcuts `Plug 2 ON` / `Plug 2 OFF` (run from the Mac) |
| **MCU serial** | `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` @ 500000 |
| **EtherCAT servo** | X axis, A6-EC drive, `node_x` on `eth0`, socket `/tmp/kalico-ethercat.sock` |

## Flashing — `scripts/flash-neptune.sh`

Pull, build, and flash the F401 end to end, from the Mac.

```sh
.claude/skills/neptune-bench/scripts/flash-neptune.sh <branch>
```

`<branch>` is **required** — be explicit about what gets flashed. The Pi pulls
`origin/<branch>`, so the script first prints a reminder of how many commits are
unpushed and how many files are uncommitted on your local worktree (it does **not**
push — push yourself if you want local work on the board).

What it does: pull + checkout `<branch>` on
the Pi → verify `.config` is still an F401 build → `make` → stop klippy/moonraker and
suppress auto-restart (systemd `Restart=no` drop-in + disable the CH340 udev rule) so
PA13 stays SWDIO → power-cycle via `Plug 2 OFF`/`ON` → wait for the CH340 tty →
openocd ST-Link flash + verify at `0x8008000` (software reset, `reset halt`) → restore
auto-restart, start services, poll `printer/info` until `ready`.

Idempotent: always pulls and flashes; the restart-suppression and udev-disable steps
tolerate already-applied state; an `EXIT` trap restores the bench (re-enable udev rule,
remove drop-in, start klippy) even if the pull/build/flash fails.

It power-cycles the board and reflashes — run it only when the user has asked to flash
the Neptune.
