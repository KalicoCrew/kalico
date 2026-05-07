1. Создайте папку для выходных файлов, по умолчанию - `mkdir ~/printer_data/config/adxl_results/chopper_magnitude`
2. Скачайте репозиторий - `git clone https://github.com/MRX8024/chopper-resonance-tuner`
3. Создайте ссылку к программе - `ln -sf ~/chopper-resonance-tuner/chopper_tune.cfg ~/printer_data/config/`
4. Установите через kiauh, либо переместите модуль gcode_shell_command.py из репозитория в клиппер - `cp -i ~/chopper-resonance-tuner/gcode_shell_command.py ~/klipper/klippy/extras/`
5. Установите пакеты -

    ``` sudo apt-get install libatlas-base-dev libopenblas-dev ```

    ``` sudo apt-get install python3-tqdm python3-plotly python3-numpy python3-matplotlib ```

7. Добавьте в конфигурацию принтера строки -
```
[respond]
[include chopper_tune.cfg]
```
Если вы использовали НЕ стандартные пути, не забудьте поправить их в `chopper_plot.py`, `[gcode_shell_command chop_tune] в chopper_tune.cfg`

Также, по желанию, можно добавить раздел обновления в moonraker для последующих обновлений через ведморду в разделе обновлений.
```
[update_manager chopper-resonance-tuner]
type: git_repo
path: ~/chopper-resonance-tuner/
origin: https://github.com/MRX8024/chopper-resonance-tuner.git
primary_branch: main
managed_services: klipper
```
