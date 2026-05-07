# printer_real — vendored printer config + plugin snapshot

One-time snapshot of the Trident printer's config tree and the third-party
plugin repos it depends on, used by the faithful klippy-in-loop simulator
(see `docs/superpowers/specs/2026-05-08-faithful-klippy-sim-design.md`).

## Provenance

Captured 2026-05-07T22:05Z from `dderg@trident.local`.

Kalico fork rev: `a3fc191d8` (sota-motion).

| Source | Upstream | Rev |
|---|---|---|
| `config/` (excluding broken symlink targets) | `~/printer_data/config/` on the printer | n/a — user-maintained |
| `third_party_repos/beacon_klipper/` | https://github.com/beacon3d/beacon_klipper.git | `ef987001b85e9cf18cb4029d89d8d1d97dec6cc9` |
| `third_party_repos/motors-sync/` | https://github.com/MRX8024/motors-sync | `4372a220f45454f256974780f7b840bf407ceb44` |
| `third_party_repos/Klipper-Adaptive-Meshing-Purging/` | https://github.com/kyleisah/Klipper-Adaptive-Meshing-Purging.git | `b0dad8ec9ee31cb644b94e39d4b8a8fb9d6c9ba0` |
| `third_party_repos/mainsail-config/` | https://github.com/mainsail-crew/mainsail-config.git | `ff3869a621db17ce3ef660adbbd3fa321995ac42` |
| `third_party_repos/moonraker-timelapse/` | https://github.com/mainsail-crew/moonraker-timelapse.git | `c7fff11e542b95e0e15b8bb1443cea8159ac0274` |
| `third_party_repos/chopper-resonance-tuner/` | https://github.com/MRX8024/chopper-resonance-tuner | `1f98212ca9dbfdf15d516115dd4c26e97b914a8d` |

## Layout

```
printer_real/
├── README.md                      # this file
├── config/                        # printer.cfg + .cfg includes; broken symlinks
│   │                              # rewired to point into third_party_repos/
│   ├── printer.cfg
│   ├── KAMP -> ../third_party_repos/Klipper-Adaptive-Meshing-Purging/Configuration
│   ├── chopper_tune.cfg -> ../third_party_repos/chopper-resonance-tuner/chopper_tune.cfg
│   ├── mainsail.cfg -> ../third_party_repos/mainsail-config/mainsail.cfg
│   ├── timelapse.cfg -> ../third_party_repos/moonraker-timelapse/klipper_macro/timelapse.cfg
│   └── ...                        # other .cfg files (verbatim)
└── third_party_repos/             # full source trees of git_repo install_managers
    ├── beacon_klipper/
    │   └── beacon.py              # vendored beacon plugin
    ├── motors-sync/
    │   └── motors_sync.py         # vendored motors_sync plugin
    ├── Klipper-Adaptive-Meshing-Purging/
    ├── mainsail-config/
    ├── moonraker-timelapse/
    └── chopper-resonance-tuner/
```

## Refreshing

There is no automated refresh script. If the printer's config drifts and
the sim needs to track it, redo the rsync pulls manually. The pulls used
were:

```bash
rsync -a --exclude='*.zip' --exclude='*_results' --exclude='input_shaper' \
    --exclude='scripts' --exclude='KAMP' --exclude='*.bak' \
    --exclude='*.bak.*' --exclude='*.stable-backup' \
    dderg@trident.local:~/printer_data/config/ \
    tools/sim_klippy/printer_real/config/

# For each git_repo plugin (beacon, motors-sync, KAMP, etc.):
rsync -a --exclude='__pycache__' --exclude='.git' --exclude='*.pyc' \
    dderg@trident.local:/home/dderg/<repo>/ \
    tools/sim_klippy/printer_real/third_party_repos/<repo>/
```

After re-pulling, broken symlinks under `config/` need their targets
rewritten to point into `../third_party_repos/<repo>/...`.

## Why this lives in-tree

The faithful sim's whole point is to run the same code paths the printer
runs. That means same printer.cfg, same plugins, same versions. Vendoring
makes sim runs reproducible and offline-capable; without it the sim would
have to ssh the printer on every test.

This is a snapshot, not a fork. We don't modify these files (except the
broken-symlink retargeting above, which is a path-translation, not a
content change). Upstream changes to any plugin won't reach us without a
manual re-pull.

## License notes

All vendored repos are open-source (MIT / GPLv3 / similar). Each carries
its original LICENSE file. We don't redistribute beyond this repo, and
the repo is private to the user's own work.
