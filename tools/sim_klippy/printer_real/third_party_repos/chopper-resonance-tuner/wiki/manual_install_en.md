1. Create a folder for the output files, by default - `mkdir ~/printer_data/config/adxl_results/chopper_magnitude`
2. Download a program - `git clone https://github.com/MRX8024/chopper-resonance-tuner`
3. Create a link to the program - `ln -sf ~/chopper-resonance-tuner/chopper_tune.cfg ~/printer_data/config/`
4. Install via kiauh, or move the gcode_shell_command.py module from repo to the klipper - `cp -i ~/chopper-resonance-tuner/gcode_shell_command.py ~/klipper/klippy/extras/`
5. Install packages -

    ``` sudo apt-get install libatlas-base-dev libopenblas-dev ```

    ``` sudo apt-get install python3-tqdm python3-plotly python3-numpy python3-matplotlib ```

7. Add lines to the configuration -
```
[respond]
[include chopper_tune.cfg]
```
If you used NOT standard paths, be sure to edit them in `chopper_plot.py`, `[gcode_shell_commandhop_tune] in Chopper_tune.cfg`

You can also optionally add an update section to moonraker for subsequent updates via Fluidd / Mainsail update managers.
```
[update_manager chopper-resonance-tuner]
type: git_repo
path: ~/chopper-resonance-tuner/
origin: https://github.com/MRX8024/chopper-resonance-tuner.git
primary_branch: main
managed_services: klipper
```
