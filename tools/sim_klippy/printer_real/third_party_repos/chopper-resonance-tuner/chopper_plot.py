#!/usr/bin/env python3
# TMC drivers registers calibration tool (plotter)
#
# Copyright (C) 2024  Alexander Fedorov <altzbox@gmail.com>
# Copyright (C) 2024  Maksim Bolgov <maksim8024@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

#################################################################################################################
RESULTS_FOLDER = '~/printer_data/config/adxl_results/chopper_magnitude'
DATA_FOLDER = '/tmp/'
#################################################################################################################

import os, sys, csv
import numpy as np
from tqdm import tqdm
import plotly.graph_objects as go
import plotly.io as pio
from datetime import datetime

RESULTS_FOLDER = os.path.expanduser(RESULTS_FOLDER)
FCLK = 12 # MHz
CUTOFF_RANGE = 5

def cleaner():
    os.system('rm -f /tmp/*.csv')
    sys.exit(0)

def check_export_path(path):
    if not os.path.exists(path):
        try:
            os.makedirs(path)
        except OSError as e:
            print(f'Error generate path {path}: {e}')

def parse_arguments():
    args = sys.argv[1:]
    parsed_args = {}
    for arg in args:
        name, value = arg.split('=')
        parsed_args[name] = int(value) if value.isdigit() else value
    return parsed_args

def calc_static_magnitude(file):
    data = np.array([
        [float(row["accel_x"]),
         float(row["accel_y"]),
         float(row["accel_z"])] for row in csv.DictReader(file)])
    return np.mean(data, axis=0)

def calc_magnitude(file, static_data):
    data = np.array([
        [float(row["accel_x"]),
         float(row["accel_y"]),
         float(row["accel_z"])] for row in csv.DictReader(file)]) - static_data
    trim_size = len(data) // CUTOFF_RANGE
    data = data[trim_size:-trim_size]
    md_magnitude = np.median(np.linalg.norm(data, axis=1))
    return md_magnitude

def main():
    print('Magnitude graphs generation...')
    args = parse_arguments()
    driver = args.get('driver')
    iterations = args.get('iterations')
    sense_resistor = round(float(args.get('sense_resistor')), 3)
    now = datetime.now().strftime('%Y%m%d_%H%M%S')
    # Calc static magnitude
    static_name = next((name for name in os.listdir(DATA_FOLDER) if name.endswith('stand_still.csv')), None)
    with open(f'{DATA_FOLDER}{static_name}', 'r') as file:
        static_data = calc_static_magnitude(file)
        accel_chip = static_name.split('-')[0]
    # Calc magnitudes on registers
    samples = {}
    datapoint = []
    empty_error = 0
    data_files = sorted(os.listdir(DATA_FOLDER), key=lambda x: os.
                        path.getmtime(os.path.join(DATA_FOLDER, x)), reverse=True)
    for name in data_files:
        if name.endswith('__.csv'):
            with open(f'{DATA_FOLDER}{name}', 'r') as file:
                curr, tbl, toff, hstrt, hend, tpfd, speed, freq, iter = name.split('__')[1].split('_')
                out_name = (f'current={curr}_tbl={tbl}_toff={toff}_hstrt={hstrt}_hend={hend}'
                            f'_tpfd={tpfd}_speed={float(speed)/100:.2f}_freq={float(freq)/1000:.2f}kHz')
                try:
                    md_magnitude = calc_magnitude(file, static_data)
                    datapoint.append(md_magnitude)
                    if int(iter) == iterations:
                        samples[out_name] = np.mean(datapoint, axis=0)
                        datapoint.clear()
                except:
                    datapoint.clear()
                    empty_error += 1
                    samples[out_name] = 0

    # Graphs generation
    colors = ['', '#2F4F4F', '#12B57F', '#9DB512', '#DF8816', '#1297B5', '#5912B5', '#B51284', '#127D0C']
    params = [reversed(list(samples.items())), sorted(samples.items(), key=lambda x: x[1])]
    names = ['', 'sorted_']
    for param, name in zip(params, names):
        fig = go.Figure()
        for entry in param:
            toff = int(entry[0].split('_')[2].split('=')[1])
            color = colors[toff if toff <= 8 else toff - 8]
            fig.add_trace(go.Bar(x=[entry[1]], y=[entry[0]], marker_color=color, orientation='h', showlegend=False))
        fig.update_layout(title='Median Magnitude vs Parameters', xaxis_title='Median Magnitude',
                          yaxis_title='Parameters', coloraxis_showscale=True)
        plot_html_path = os.path.join(RESULTS_FOLDER, f'{name}interactive_plot_{accel_chip}_tmc{driver}_{sense_resistor}_{now}.html')
        pio.write_html(fig, plot_html_path, auto_open=False)
        speed1 = params[1][0][0].split('_')[6].split('=')[1]
        speed2 = params[1][1][0].split('_')[6].split('=')[1]
        if speed1 != speed2:
            break

    # Export Info
    try:
        print(f'Access to interactive plot at: {"/".join(plot_html_path.split("/")[:-1] + [plot_html_path.split(names[1])[1]])}')
    except IndexError:
        print(f'Access to interactive plot at: {plot_html_path}')
    if empty_error:
        print(f'Warning!!! Empty data cells detected ({empty_error}), make sure you dont run out of memory')

if __name__ == '__main__':
    if sys.argv[1] == 'cleaner':
        cleaner()
    check_export_path(RESULTS_FOLDER)
    main()
