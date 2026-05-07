### The method of semi-automatic calibration of driver parameters is based on Trinamic’s [manual](https://www.analog.com/en/app-notes/AN-001.html) for “behavioral” motor tuning.


### 1. Install the calibration script on the printer host. (the klipper will reboot!)
```
   cd ~
   git clone https://github.com/MRX8024/chopper-resonance-tuner
   bash ~/chopper-resonance-tuner/install.sh
```
If everything went well, you will see folder - `adxl_results` in your printer home directory (~/printer_data/config), into which the calibration results will be placed, as well as an already available macro from the macro panel on the main page of the Fluidd / Mainsail.
And if for some reason not, then install [manually](/wiki/manual_install_en.md).

2. Сonnect the accelerometer to the motor by screwing it in, this guarantees accurate vibration measurement.
   However, it is possible to connect, as for example when measuring resonances, for input_shaper - to the print head / bed, depending on the type of printer, selected measuring axis, to collect vibrations.
   This method may give incorrect data if the mechanics are crooked, but on properly assembled printers, it is not inferior to the first.

3. Calibration: (Further commands in this article will be interpreted with the minimum required parameters, all supported are listed at the bottom of the manual).

   1. We determine the resonant speeds by entering the command `CHOPPER_TUNE FIND_VIBRATIONS=1` into the web terminal.
   2. After the macro is completed, the algorithm will automatically generate a table of data and graphics, place them in the `.../adxl_results/chopper_magnitude/` directory, download and open `interactive_plot_*.html`, and see the following picture -
   ![](/wiki/pictures/img_1.png)
   The graph will usually show 2 peaks, at a speed of about 50mm/s and 100mm/s - these are resonant speeds, we need the lowest of these speeds, for example, 55mm/s.
   3. Run the macro to iterate through all the chopper options at the previously selected speed, the command will look like this
   -`CHOPPER_TUNE MIN_SPEED=55 MAX_SPEED=55`. **Check the availability of free space on the host**, possible /tmp folder limit on hosts with 1GB of RAM, about ~700mb is required for data.
   The data collection time will take approximately two hours (depending on kinematics), after completion we open the graph in the same way as the previous time, we get a graph of the form -
   ![](/wiki/pictures/img_2.png)
   In this example, the minimum vibrations are at TBL=0 and TOFF=8. Let's enlarge this area.
   ![](/wiki/pictures/img_3.png)
   4. Select the chopper option with the minimum magnitude value - these are the required parameters. It is also necessary to take into account that with large values of `TBL` and `TOFF` the motor frequency decreases, which leads to the appearance of nasty high frequency noise. 
   If the vibration decreases with the occurrence of this phenomenon, move to a pleasant range of work between vibrations and noise, by using the program functionality (entering the registers ranges you need into the macro parameters), if this bothers you. If not, then it would be preferable to leave the high-frequency squeak.
   We enter them into the drivers section in printer.cfg, example -
   ```
   [tmc**** stepper_*]
   cs_pin: PC4
   ...
   driver_TBL: 0
   driver_TOFF: 8
   driver_HSTRT: 5
   driver_HEND: 5
   ```

   5. You can repeat the procedure with smaller variations of the chopper, for example, only `TBL=0` and `TOFF=8` and iterate over the full ranges of `HSTRT` and `HEND`, but with more repetitions of `ITERATIONS`. In this case, the graph will be based on average results to reduce the influence of mechanics on the readings.
   6. If you are the lucky owner of a TMC2240 or TMC5160, then after setting all of the above registers, you have the opportunity to configure another parameter called `TPFD`.
   It is responsible for damping the average resonances of the motor, and has a value range of `0-15`. Set its parameter value to `driver_TPFD: 0`, or calibrate it.
   The command with the data registers found above, two `ITERATIONS` - for greater accuracy, and resonant speed looks like this - `CHOPPER_TUNE TBL_MIN=0 TBL_MAX=0 TOFF_MIN=8 TOFF_MAX=8 HSTRT_MIN=5 HSTRT_MAX=5 HEND_MIN=5 HEND_MAX=5 TPFD_MIN=0 TPFD_MAX=15 MIN_SPEED=55 MAX_SPEED=55 ITERATIONS=2`


Description of the program functionality -

The values `'default'` in parameters mean that if there is no argument, this variable will assign the default parameters from printer.cfg, or calculate the minimum required ones.

1. `AXIS` - direction `X/Y/Z` in which the measurement will be run.
2. `CURRENT_MIN_MA` and `CURRENT_MAX_MA` - are responsible for changes in the supplied current (mA) to stepper motors in 25mA steps. For example, if you have enough torque that the stepper motors produce, you can reduce their current to make the system quieter and reduce motor heating. This function partly allows you to analyze is it worth it, or just choose the current you need in measure.
3. `TBL_MIN-0` and `TBL_MAX-3`, `TOFF_MIN-1` and `TOFF_MAX-8`, `HSTRT_MIN-0` and `HSTRT_MAX-7`, `HEND_MIN-0` and `HEND_MAX-15`, `TPFD_MIN-0` and `TPFD_MAX-15` are actually also responsible for enumerating parameters, in this case, registers of driver/s. Their range of work and search is indicated.
4. `HSTRT_HEND_MAX-16` - limit on the sum of `HSTRT and HEND`, change is undesirable. ([more](https://www.analog.com/media/en/technical-documentation/data-sheets/TMC5160A_datasheet_rev1.17.pdf))
5. `MIN_SPEED` and `MAX_SPEED` - enumerate the speed range, with a step of `SPEED_CHANGE_STEP`. By default, it is calculated based on the required rpm by gear ratios.
6. `ITERATIONS` - the number of repetitions of measurements, for more accurate data.
7. `TRAVEL_DISTANCE` - distance `(mm)` of the print head movement during which vibrations are read. By default, is calculated based on the printer's capabilities and measurement time.
8. `ACCELEROMETER` - an accelerometer that will be used to measure vibrations, auto will be detected if one is specified in the `resonance_tester` configuration, otherwise, without specifying will be applied `adxl345`.
9. `FIND_VIBRATIONS` - mode for measuring vibrations from speed, useful in order to remove resonant speeds from everyday printing, as for step 3.1 of this article, applies registers from printer configuration. Values - `(True / False), (1 / 0)`
10. `RUN_PLOTTER` - run the graph generation script. Values - `(True / False), (1 / 0)`