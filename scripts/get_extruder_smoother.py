#!/usr/bin/env python
# Script to plot extruder smoothers
#
# Copyright (C) 2024  Dmitry Butyugin <dmbutyugin@google.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import optparse, importlib, math, os, sys
import numpy as np, matplotlib
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)),
                             '..', 'klippy'))
shaper_defs = importlib.import_module('.shaper_defs', 'extras')
shaper_calibrate = importlib.import_module('.shaper_calibrate', 'extras')
extruder_smoother = importlib.import_module('.extruder_smoother', 'extras')

def plot_shaper(shaper_name, damping_ratio):
    (C_e, t_sm), (t, velocities) = extruder_smoother.get_extruder_smoother(
            shaper_name.lower(), 1.0, damping_ratio, return_velocities=True)
    tau = np.linspace(-.5 * t_sm, .5 * t_sm, t.shape[0])
    w_e = np.zeros(shape=tau.shape)
    for c in C_e[::-1]:
        w_e = (w_e * tau + c)

    normalized_velocities = velocities / (
            velocities.sum(axis=-1)[:,np.newaxis] * (t[1] - t[0]))
    mean_velocity = normalized_velocities.mean(axis=0)

    fig, ax = matplotlib.pyplot.subplots(figsize=(10,9))
    ax.set_title("Unit step input")
    styles = ['dotted', 'dashed', 'dashdot']
    dr_i = 0
    for i in range(normalized_velocities.shape[0]):
        ax.plot(t, normalized_velocities[i,:], linestyle=styles[dr_i])
    ax.plot(t, mean_velocity, 'b', linewidth=2.0)
    ax.plot(t, w_e / (t[-1] - t[0]), 'r', linewidth=2.0)
    ax.set_xlabel('Time, sec')
    ax.set_ylabel('Amplitude')
    ax.grid()
    return fig

def setup_matplotlib(output_to_file):
    global matplotlib
    if output_to_file:
        matplotlib.use('Agg')
    import matplotlib.pyplot, matplotlib.dates, matplotlib.font_manager
    import matplotlib.ticker

def main():
    # Parse command-line arguments
    usage = "%prog [options]"
    opts = optparse.OptionParser(usage)
    opts.add_option("-o", "--output", type="string", dest="output",
                    default=None, help="filename of output graph")
    opts.add_option("-s", "--shaper", type="string", dest="shaper",
                    default="ei", help="name of the shaper to plot")
    opts.add_option("--damping_ratio", type="float", dest="damping_ratio",
                    default=shaper_defs.DEFAULT_DAMPING_RATIO,
                    help="the damping ratio of the system to adapt smoother to")
    options, args = opts.parse_args()
    if len(args) != 0:
        opts.error("Incorrect number of arguments")
    if (options.shaper.lower() not in
            [s.name for s in shaper_defs.INPUT_SHAPERS] and
        options.shaper.lower() not in
            [s.name for s in shaper_defs.INPUT_SMOOTHERS]):
        opts.error("Invalid shaper name '%s'" % (opts.shaper,))

    # Draw graph
    setup_matplotlib(options.output is not None)
    fig = plot_shaper(options.shaper, options.damping_ratio)

    # Show graph
    if options.output is None:
        matplotlib.pyplot.show()
    else:
        fig.set_size_inches(8, 6)
        fig.savefig(options.output)

if __name__ == '__main__':
    main()
