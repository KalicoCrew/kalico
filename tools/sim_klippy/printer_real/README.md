# printer_real — vendored printer config + fetched plugin sources

One-time snapshot of the Trident printer's config tree and a pin file
for the third-party plugin repos it depends on. Used by the faithful
klippy-in-loop simulator (see
`docs/superpowers/specs/2026-05-08-faithful-klippy-sim-design.md`).

The config tree (printer.cfg + .cfg includes) is checked in.
The plugin sources are **not** checked in — they're fetched at pinned
revs by `tools/sim_klippy/fetch_plugins.sh` into `third_party_repos/`,
which is gitignored.

## Provenance

Captured 2026-05-07T22:05Z from `dderg@trident.local`.

Kalico fork rev: `a3fc191d8` (sota-motion).

| Source | Upstream | Pinned rev |
|---|---|---|
| `config/` (excluding broken symlink targets) | `~/printer_data/config/` on the printer | n/a — user-maintained |
| `third_party_repos/beacon_klipper/` | https://github.com/beacon3d/beacon_klipper.git | `ef987001b85e9cf18cb4029d89d8d1d97dec6cc9` |
| `third_party_repos/motors-sync/` | https://github.com/MRX8024/motors-sync | `4372a220f45454f256974780f7b840bf407ceb44` |
| `third_party_repos/Klipper-Adaptive-Meshing-Purging/` | https://github.com/kyleisah/Klipper-Adaptive-Meshing-Purging.git | `b0dad8ec9ee31cb644b94e39d4b8a8fb9d6c9ba0` |
| `third_party_repos/mainsail-config/` | https://github.com/mainsail-crew/mainsail-config.git | `ff3869a621db17ce3ef660adbbd3fa321995ac42` |
| `third_party_repos/moonraker-timelapse/` | https://github.com/mainsail-crew/moonraker-timelapse.git | `c7fff11e542b95e0e15b8bb1443cea8159ac0274` |
| `third_party_repos/chopper-resonance-tuner/` | https://github.com/MRX8024/chopper-resonance-tuner | `1f98212ca9dbfdf15d516115dd4c26e97b914a8d` |
| `third_party_repos/klipper_tmc_autotune/` | https://github.com/andrewmcgr/klipper_tmc_autotune | `f366d75fa44d177aa6fb002cdff50195e6952772` |

The pins live in `tools/sim_klippy/fetch_plugins.sh` — that file is the
source of truth; this table mirrors it.

## Layout

```
printer_real/
├── README.md                      # this file
├── config/                        # printer.cfg + .cfg includes (tracked).
│   │                              # Broken symlinks rewired to point into
│   │                              # ../third_party_repos/.
│   ├── printer.cfg
│   ├── KAMP -> ../third_party_repos/Klipper-Adaptive-Meshing-Purging/Configuration
│   ├── chopper_tune.cfg -> ../third_party_repos/chopper-resonance-tuner/chopper_tune.cfg
│   ├── mainsail.cfg -> ../third_party_repos/mainsail-config/mainsail.cfg
│   ├── timelapse.cfg -> ../third_party_repos/moonraker-timelapse/klipper_macro/timelapse.cfg
│   └── ...                        # other .cfg files (verbatim)
└── third_party_repos/             # gitignored. Populated by fetch_plugins.sh.
    ├── beacon_klipper/
    ├── motors-sync/
    ├── Klipper-Adaptive-Meshing-Purging/
    ├── mainsail-config/
    ├── moonraker-timelapse/
    ├── chopper-resonance-tuner/
    └── klipper_tmc_autotune/
```

## Fetching the plugin sources

```bash
tools/sim_klippy/fetch_plugins.sh
```

Idempotent: re-running does nothing if the checked-out revs already match
the pins. `tools/sim_klippy/conftest.py` runs this automatically at sim
fixture entry if any plugin source is missing, so the typical workflow
("clone repo, run sim tests") needs no manual step.

## Bumping a pin

Edit the rev in `tools/sim_klippy/fetch_plugins.sh`, then re-run the
script. It will fetch and check out the new rev, replacing the old one in
place. Update the table above to match.

## If a plugin needs local modifications

Don't edit files under `third_party_repos/` — they get clobbered on every
re-run. Fork the upstream repo on github, point the script's URL at the
fork, and commit changes there.

## Refreshing the printer config snapshot

There is no automated refresh script. If the printer's config drifts and
the sim needs to track it, redo the rsync pull manually:

```bash
rsync -a --exclude='*.zip' --exclude='*_results' --exclude='input_shaper' \
    --exclude='scripts' --exclude='KAMP' --exclude='*.bak' \
    --exclude='*.bak.*' --exclude='*.stable-backup' \
    dderg@trident.local:~/printer_data/config/ \
    tools/sim_klippy/printer_real/config/
```

After re-pulling, broken symlinks under `config/` need their targets
rewritten to point into `../third_party_repos/<repo>/...`.

## Why the config lives in-tree, but plugins don't

The faithful sim's whole point is to run the same code paths the printer
runs. That means same printer.cfg, same plugins, same versions.

The config tree is small (~3500 lines across the .cfg files) and is
user-authored — there's no upstream to pin against, so it's checked in.

The plugins are third-party code with their own upstream history. We
don't modify them; pinning revs and fetching on demand keeps the repo
small (~2.3 MB saved) and makes it obvious that any change to a plugin
must happen via fork, not local edit.

## License notes

All third-party repos are open-source (MIT / GPLv3 / similar). Each
carries its original LICENSE file in its clone. We don't redistribute.
