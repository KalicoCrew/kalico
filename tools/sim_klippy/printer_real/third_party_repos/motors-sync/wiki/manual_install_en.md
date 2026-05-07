1. Download a program - `git clone https://github.com/MRX8024/motors-sync`
2. Create a link to the program - `ln -sf ~/motors-sync/motors_sync.py ~/klipper/klippy/extras/motors_sync.py`
3. Exclude program from klipper git tracking - `echo "klippy/extras/motors_sync.py" >> "~/klipper/.git/info/exclude"`
4. Install packages -

    ```sudo apt-get install libatlas-base-dev libopenblas-dev```

    ```~/klippy-env/bin/pip install -r ~/motors-sync/wiki/requirements.txt```

You can also optionally add an update section to moonraker for subsequent updates via Fluidd / Mainsail update managers.
```
[update_manager motors-sync]
type: git_repo
path: ~/motors-sync/
origin: https://github.com/MRX8024/motors-sync.git
primary_branch: main
managed_services: klipper
virtualenv: ~/klippy-env/
requirements: wiki/requirements.txt
system_dependencies: wiki/packages.json
```
