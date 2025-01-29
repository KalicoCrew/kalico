# Migrating from Klipper

> [!NOTE]
> Any add-on modules you are using will need to be reinstalled after switching to Kalico. This includes things like Beacon support, led-effect, etc.
>
> Any data in ~/printer_data such as printer configs and macros will be unaffected, THOUGH do back up just in case.

### Option 1. Manually clone the repository

If desired, make a backup copy of your existing Klipper installation by running:

```bash
mv ~/klipper ~/klipper_old
```

Then clone the Kalico repo and restart the klipper service:

```bash
git clone https://github.com/KalicoCrew/kalico.git ~/klipper
sudo systemctl restart klipper
```

### Option 2. Using KIAUH

For users that are not comfortable using Git directly, [KIAUH v6](https://github.com/dw-0/kiauh) is able to use custom repositories.

To do this, add the Kalico repo to KIAUH's custom repository settings with the following steps:

From the KIAUH menu select:

- [S] Settings
- 1\) Set custom Klipper repository
- Use `https://github.com/KalicoCrew/kalico` as the new repository URL
- Use `main` or `bleeding-edge-v2` as the new branch name
- Select 'Y' to apply the changes
- Enter 'B' for back twice
- 'Q' to quit

### Option 3. Adding a git-remote to the existing installation
Can switch back to mainline klipper at any time via a `git checkout upstream_main`

```bash
cd ~/klipper
git remote add kalico https://github.com/KalicoCrew/kalico.git
git checkout -b upstream-main origin/master
git branch -D master
git fetch kalico main
git checkout -b main kalico/main
sudo systemctl restart klipper
sudo systemctl restart moonraker
```
