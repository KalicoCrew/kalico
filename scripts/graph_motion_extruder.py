#!/usr/bin/env python
# Script to graph motion results with extruder flow
#
# Copyright (C) 2019-2024  Kevin O'Connor <kevin@koconnor.net>
# Copyright (C) 2020-2024  Dmitry Butyugin <dmbutyugin@google.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import optparse, datetime, importlib, math, os, sys
import numpy as np
import matplotlib
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)),
                             '..', 'klippy'))
shaper_defs = importlib.import_module('.shaper_defs', 'extras')
shaper_calibrate = importlib.import_module('.shaper_calibrate', 'extras')
extruder_smoother = importlib.import_module('.extruder_smoother', 'extras')

SEG_TIME = .00002
INV_SEG_TIME = 1. / SEG_TIME

SPRING_FREQ=43.0
DAMPING_RATIO=0.095

SHAPER_FREQ=45.0
SHAPER_DAMPING_RATIO=0.10

######################################################################
# Basic trapezoid motion
######################################################################

# List of moves: [(start_v, end_v, move_t), ...]
Moves = [
    (0., 0., .100),
    (6.869, 90., None), (90., 90., .120), (90., 5., None),
    (5., 5., 0.002),
    (5., 90., None), (90., 90., .130), (90., 50., None),
    (50., 50., .100), (50., .5, None),
    (0., 0., .200)
]
ACCEL = 3000.
LAYER_H = 0.3
EXTRUDE_R = (.4 * LAYER_H * 1.0) / (math.pi * (1.75 / 2.)**2)

def get_acc(start_v, end_v):
    return ACCEL

# Standard constant acceleration generator
def get_acc_pos(rel_t, start_v, accel, move_t):
    return (start_v + 0.5 * accel * rel_t) * rel_t

# Calculate positions based on 'Moves' list
def gen_positions():
    out = []
    start_d = start_t = t = 0.
    for start_v, end_v, move_t in Moves:
        if move_t is None:
            move_t = abs(end_v - start_v) / get_acc(start_v, end_v)
        accel = (end_v - start_v) / move_t
        end_t = start_t + move_t
        while t <= end_t:
            rel_t = t - start_t
            out.append(start_d + get_acc_pos(rel_t, start_v, accel, move_t))
            t += SEG_TIME
        start_d += get_acc_pos(move_t, start_v, accel, move_t)
        start_t = end_t
    return out


######################################################################
# Estimated motion with belt as spring
######################################################################

def estimate_spring(positions, freq, damping_ratio):
    ang_freq2 = (freq * 2. * math.pi)**2
    damping_factor = 4. * math.pi * damping_ratio * freq
    head_pos = head_v = 0.
    out = []
    for stepper_pos in positions:
        head_pos += head_v * SEG_TIME
        head_a = (stepper_pos - head_pos) * ang_freq2
        head_v += head_a * SEG_TIME
        head_v -= head_v * damping_factor * SEG_TIME
        out.append(head_pos)
    return out


######################################################################
# Pressure advance
######################################################################

SMOOTH_TIME = .02
SYSTEM_PRESSURE_ADVANCE = .045
LINEAR_ADVANCE = .045
LINEAR_VELOCITY = 0.5
LINEAR_OFFSET = 0. #0.16

# Calculate raw pressure advance positions
def calc_pa_raw(positions):
    pa = LINEAR_ADVANCE * INV_SEG_TIME
    recipr_lin_vel = INV_SEG_TIME / LINEAR_VELOCITY
    out = [0.] * len(positions)
    for i in indexes(positions):
        de = positions[i+1] - positions[i]
        out[i] = positions[i] + pa * de
        out[i] += LINEAR_OFFSET * math.tanh(de * recipr_lin_vel)
        #out[i] += LINEAR_OFFSET * (1. - 1. / (1. + de * recipr_lin_vel))
    return out

# Pressure advance after smoothing
def calc_pa(positions, smooth_time):
    return calc_weighted(calc_pa_raw(positions), smooth_time)
def calc_pa2(positions, smooth_time):
    return calc_weighted2(calc_pa_raw(positions), smooth_time)

# Nozzle flow assuming nozzle follows simple pressure advance model
def calc_nozzle_flow(positions, pressure_advance):
    pa = pressure_advance * INV_SEG_TIME
    last = 0.
    out = [0.] * len(positions)
    for i in range(1, len(positions)):
        out[i] = last = (positions[i] - last)/pa + last
    return out

######################################################################
# Toolhead positions and extrusion widths
######################################################################

TH_POSITION_BUCKET = 0.200
TH_OFFSET = 10

# Map toolhead locations
def calc_toolhead_positions(positions):
    th_max = round(max(positions) / TH_POSITION_BUCKET)
    th_index = [i * TH_POSITION_BUCKET
                for i in range(-TH_OFFSET, th_max + TH_OFFSET)]
    return th_index

# Determing extrusion amount at each toolhead position
def calc_extruded(th_index, th_positions, e_velocities):
    e_adjust = SEG_TIME / TH_POSITION_BUCKET * (
            math.pi * (1.75 / 2.)**2) / LAYER_H
    out = [0.] * len(th_index)
    for th_pos, e in zip(th_positions, e_velocities):
        bucket = TH_OFFSET + round(th_pos / TH_POSITION_BUCKET)
        if e > 0.:
            out[bucket] += e * e_adjust
    return out

######################################################################
# List helper functions
######################################################################

MARGIN_TIME = 0.050

def time_to_index(t):
    return int(t * INV_SEG_TIME + .5)

def indexes(positions):
    drop = time_to_index(MARGIN_TIME)
    return range(drop, len(positions)-drop)

def trim_lists(*lists):
    keep = len(lists[0]) - time_to_index(2. * MARGIN_TIME)
    for l in lists:
        del l[keep:]


######################################################################
# Common data filters
######################################################################

# Generate estimated first order derivative
def gen_deriv(data):
    return [0.] + [(data[i+1] - data[i]) * INV_SEG_TIME
                   for i in range(len(data)-1)]

# Simple average between two points smooth_time away
def calc_average(positions, smooth_time):
    offset = time_to_index(smooth_time * .5)
    out = [0.] * len(positions)
    for i in indexes(positions):
        out[i] = .5 * (positions[i-offset] + positions[i+offset])
    return out

# Average (via integration) of smooth_time range
def calc_smooth(positions, smooth_time):
    offset = time_to_index(smooth_time * .5)
    weight = 1. / (2*offset - 1)
    out = [0.] * len(positions)
    for i in indexes(positions):
        out[i] = sum(positions[i-offset+1:i+offset]) * weight
    return out

# Time weighted average (via integration) of smooth_time range
def calc_weighted(positions, smooth_time):
    offset = time_to_index(smooth_time * .5)
    weight = 1. / offset**2
    out = [0.] * len(positions)
    for i in indexes(positions):
        weighted_data = [positions[j] * (offset - abs(j-i))
                         for j in range(i-offset, i+offset)]
        out[i] = sum(weighted_data) * weight
    return out

# Weighted average (`h**2 - (t-T)**2`) of smooth_time range
def calc_weighted2(positions, smooth_time):
    offset = time_to_index(smooth_time * .5)
    weight = .75 / offset**3
    out = [0.] * len(positions)
    for i in indexes(positions):
        weighted_data = [positions[j] * (offset**2 - (j-i)**2)
                         for j in range(i-offset, i+offset)]
        out[i] = sum(weighted_data) * weight
    return out

# Weighted average (`(h**2 - (t-T)**2)**2`) of smooth_time range
def calc_weighted4(positions, smooth_time):
    offset = time_to_index(smooth_time * .5)
    weight = 15 / (16. * offset**5)
    out = [0.] * len(positions)
    for i in indexes(positions):
        weighted_data = [positions[j] * ((offset**2 - (j-i)**2))**2
                         for j in range(i-offset, i+offset)]
        out[i] = sum(weighted_data) * weight
    return out

# Weighted average (`(h - abs(t-T))**2 * (2 * abs(t-T) + h)`) of range
def calc_weighted3(positions, smooth_time):
    offset = time_to_index(smooth_time * .5)
    weight = 1. / offset**4
    out = [0.] * len(positions)
    for i in indexes(positions):
        weighted_data = [positions[j] * (offset - abs(j-i))**2
                         * (2. * abs(j-i) + offset)
                         for j in range(i-offset, i+offset)]
        out[i] = sum(weighted_data) * weight
    return out


######################################################################
# Spring motion estimation
######################################################################

def calc_spring_raw(positions, freq, damping_ratio):
    sa = (INV_SEG_TIME / (freq * 2. * math.pi))**2
    ra = 2. * damping_ratio * math.sqrt(sa)
    out = [0.] * len(positions)
    for i in indexes(positions):
        out[i] = (positions[i]
                  + sa * (positions[i-1] - 2.*positions[i] + positions[i+1])
                  + ra * (positions[i+1] - positions[i]))
    return out

def calc_spring_double_weighted(positions, freq, smooth_time):
    offset = time_to_index(smooth_time * .25)
    sa = (INV_SEG_TIME / (offset * freq * 2. * math.pi))**2
    ra = 2. * damping_ratio * math.sqrt(sa)
    out = [0.] * len(positions)
    for i in indexes(positions):
        out[i] = (positions[i]
                  + sa * (positions[i-offset] - 2.*positions[i]
                          + positions[i+offset])
                  + ra * (positions[i+1] - positions[i]))
    return calc_weighted(out, smooth_time=.5 * smooth_time)

######################################################################
# Input shapers
######################################################################

def calc_shaper(shaper, positions):
    t_offs = shaper_defs.get_shaper_offset(shaper[0], shaper[1])
    A = shaper[0]
    inv_D = 1. / sum(A)
    n = len(A)
    T = [time_to_index(-shaper[1][j]) for j in range(n)]
    t_offs_ind = time_to_index(t_offs)
    out = [0.] * len(positions)
    for i in indexes(positions):
        out[i] = sum([positions[i + T[j] + t_offs_ind] * A[j]
                      for j in range(n)]) * inv_D
    return out

def calc_smoother(smoother, positions):
    C, t_sm, t_offs, _ = smoother
    hst = 0.5 * t_sm

    tau = np.arange(-hst, hst, SEG_TIME)
    w = np.zeros(shape=tau.shape)
    for c in C[::-1]:
        w = w * tau + c

    w_dt = w * SEG_TIME
    pos = np.asarray(positions)
    t_offs_ind = time_to_index(t_offs)
    if t_offs_ind < 0:
        smoothed = np.convolve(pos[:t_offs_ind], w_dt, mode='same')
    else:
        smoothed = np.convolve(pos[t_offs_ind:], w_dt, mode='same')
    return np.concatenate((np.zeros(shape=(abs(t_offs_ind),)),
                           smoothed)).tolist()

# Ideal values

def gen_updated_position(positions, shaper_name,
                         shaper_freq, shaper_damping_ratio):
    for s in shaper_defs.INPUT_SHAPERS:
        if s.name == shaper_name.lower():
            A, T = s.init_func(shaper_freq, shaper_damping_ratio)
            ts = shaper_defs.get_shaper_offset(A, T)
            T = [t - ts for t in T]
            shaper = A, T, s.name.upper()
            return calc_shaper(shaper, positions)
    for s in shaper_defs.INPUT_SMOOTHERS:
        if s.name == shaper_name.lower():
            C, t_sm = s.init_func(shaper_freq)
            t_offs = shaper_defs.get_smoother_offset(C, t_sm)
            smoother = C, t_sm, t_offs, s.name.upper()
            return calc_smoother(smoother, positions)

def gen_extr_is_positions(positions, shaper_name,
                          shaper_freq, shaper_damping_ratio):
    for s in shaper_defs.INPUT_SHAPERS:
        if s.name == shaper_name.lower():
            A, T = s.init_func(shaper_freq, shaper_damping_ratio)
            t_sm = T[-1] - T[0]
            t_offs = shaper_defs.get_shaper_offset(A, T) - .5 * t_sm
    for s in shaper_defs.INPUT_SMOOTHERS:
        if s.name == shaper_name.lower():
            C, t_sm = s.init_func(shaper_freq)
            t_offs = shaper_defs.get_smoother_offset(C, t_sm)
    C_e, _ = extruder_smoother.get_extruder_smoother(shaper_name, t_sm,
                                                     shaper_damping_ratio)
    smoother = C_e, t_sm, t_offs, shaper_name
    return calc_smoother(smoother, positions)

######################################################################
# Plotting and startup
######################################################################

def plot_motion(shaper_name, shaper_freq, shaper_damping_ratio,
                freq, damping_ratio, smooth_time):
    # Nominal motion
    positions = gen_positions()
    velocities = gen_deriv(positions)
    accels = gen_deriv(velocities)
    # Updated motion
    upd_positions = gen_updated_position(positions, shaper_name,
                                         shaper_freq, shaper_damping_ratio)
    upd_velocities = gen_deriv(upd_positions)
    upd_accels = gen_deriv(upd_velocities)
    # Estimated position with model of belt as spring
    spring_orig = estimate_spring(positions, freq, damping_ratio)
    spring_upd = estimate_spring(upd_positions, freq, damping_ratio)
    head_velocities = gen_deriv(spring_orig)
    head_accels = gen_deriv(head_velocities)
    head_upd_velocities = gen_deriv(spring_upd)
    head_upd_accels = gen_deriv(head_upd_velocities)
    # Motion with pressure advance
    extruder_positions = [p * EXTRUDE_R for p in positions]
    extruder_velocities = [v * EXTRUDE_R for v in velocities]
    nom_flow = gen_deriv(calc_nozzle_flow(extruder_positions,
                                          SYSTEM_PRESSURE_ADVANCE))
    pa_positions = calc_pa_raw(extruder_positions)
    pa_velocities = gen_deriv(pa_positions)
    pa_accels = gen_deriv(pa_velocities)
    pa_flow = gen_deriv(calc_nozzle_flow(pa_positions,
                                         SYSTEM_PRESSURE_ADVANCE))
    # Smoothed PA motion
    sm_pa_positions = calc_pa(extruder_positions, smooth_time)
    sm_pa_velocities = gen_deriv(sm_pa_positions)
    sm_pa_accels = gen_deriv(sm_pa_velocities)
    sm_pa_flow = gen_deriv(calc_nozzle_flow(sm_pa_positions,
                                            SYSTEM_PRESSURE_ADVANCE))
    # Extruder synchronization with IS
    extr_is_pos = gen_extr_is_positions(extruder_positions, shaper_name,
                                        shaper_freq, shaper_damping_ratio)
    extr_is_vel = gen_deriv(extr_is_pos)
    extr_is_pa_pos = calc_pa_raw(extr_is_pos)
    extr_is_pa_vel = gen_deriv(extr_is_pa_pos)
    extr_is_pa_accel = gen_deriv(extr_is_pa_vel)
    extr_is_pa_flow = gen_deriv(calc_nozzle_flow(extr_is_pa_pos,
                                                 SYSTEM_PRESSURE_ADVANCE))

    # Build plot
    times = [SEG_TIME * i for i in range(len(positions))]
    trim_lists(times, velocities, accels,
               positions, spring_upd,
               upd_velocities, upd_velocities, upd_accels,
               head_velocities, head_upd_velocities,
               head_accels, head_upd_accels,
               extruder_positions, extruder_velocities,
               pa_positions, pa_velocities, pa_flow,
               sm_pa_velocities, sm_pa_flow,
               extr_is_vel, extr_is_pa_vel, extr_is_pa_flow)
    # Extrusion width
    th_index = calc_toolhead_positions(positions)
    ext_nom = calc_extruded(th_index, positions, extruder_velocities)
    ext_smooth_pa_ideal = calc_extruded(th_index, positions, sm_pa_flow)
    ext_smooth_pa_is = calc_extruded(th_index, spring_upd, sm_pa_flow)
    ext_is_pa = calc_extruded(th_index, spring_upd, extr_is_pa_flow)

    fig, axs = matplotlib.pyplot.subplot_mosaic([
        ['Extruder position', 'Extrusion width'],
        ['Extruder velocity', 'Extrusion width'],
        ['Extruder acceleration', 'Extrusion width']])
    ((_, ax1), (_, ax4), (_, ax2), (_, ax3)) = axs.items()
    ax1.set_title("Simulation: resonance freq=%.1f Hz, damping_ratio=%.3f,\n"
                  "shaper = %s, configured freq=%.1f Hz"
                  % (freq, damping_ratio, shaper_name, shaper_freq))
    ax1.set_ylabel('Velocity (mm/s)')
    ax1.plot(times, velocities, 'g', label='Nominal Velocity', alpha=0.8)
    ax1.plot(times, upd_velocities, 'r', label='New Velocity', alpha=0.8)
    ax1.plot(times, head_velocities, 'orange', label='Head Velocity', alpha=0.4)
    ax1.plot(times, head_upd_velocities, 'blue', label='New Head Velocity',
             alpha=0.4)
    fontP = matplotlib.font_manager.FontProperties()
    fontP.set_size('x-small')
    ax1.legend(loc='best', prop=fontP)
    ax1.grid(True)
    ax2.set_ylabel('Acceleration (mm/s^2)')
    #ax2.plot(times, accels, 'g', label='Nominal Accel', alpha=0.8)
    ax2.plot(times, upd_accels, 'r', label='New Accel', alpha=0.8)
    ax2.plot(times, head_accels, 'orange', label='Head Accel', alpha=0.4)
    ax2.plot(times, head_upd_accels, 'blue', label='New Head Accel', alpha=0.4)
    ax2.set_ylim([-5. * ACCEL, 5. * ACCEL])
    ax2.legend(loc='best', prop=fontP)
    ax2.grid(True)
    ax3.set_ylabel('Extruder velocity (mm/s)')
    ax3.plot(times, extruder_velocities, 'black', label='Nominal')
    ax3.plot(times, sm_pa_velocities, 'g', label='Smooth PA', alpha=0.9)
    ax3.plot(times, extr_is_vel, 'orange', label='Extr IS', alpha=0.5)
    ax3.plot(times, extr_is_pa_vel, 'blue', label='Extr IS PA', alpha=0.5)
    ax3.plot(times, pa_flow, label='PA flow')
    ax3.plot(times, sm_pa_flow, label='Smooth PA flow')
    ax3.grid(True)
    ax3.legend(loc='best', prop=fontP)
    ax3.set_xlabel('Time (s)')

    ax4.set_title('Extruion width')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Filament extruded (mm)')
    ax4.plot(th_index, ext_nom, 'black', label='Nominal')
    ax4.plot(th_index, ext_smooth_pa_ideal,
             'r', label='Smooth PA with ideal TH motion', alpha=0.5)
    ax4.plot(th_index, ext_smooth_pa_is,
            'g', label='Smooth PA with IS', alpha=0.7)
    ax4.plot(th_index, ext_is_pa,
             'blue', label='Extr IS PA', alpha=0.5)
    ax4.legend(loc='best', prop=fontP)
    ax4.set_xlabel('Toolhead Position (mm)')
    ax4.grid(True)
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
    opts.add_option("-f", "--freq", type="float", dest="freq",
                    default=SPRING_FREQ,
                    help="the frequency of the system")
    opts.add_option("--damping_ratio", type="float", dest="damping_ratio",
                    default=DAMPING_RATIO,
                    help="the damping ratio of the system")
    opts.add_option("--shaper_freq", type="float", dest="shaper_freq",
                    default=SHAPER_FREQ,
                    help="the frequency of the shaper")
    opts.add_option("--shaper_damping_ratio", type="float",
                    dest="shaper_damping_ratio", default=SHAPER_DAMPING_RATIO,
                    help="the damping ratio of the shaper")
    opts.add_option("--smooth_time", type="float", dest="smooth_time",
                    default=SMOOTH_TIME, help="extruder smooth time")
    options, args = opts.parse_args()
    if len(args) != 0:
        opts.error("Incorrect number of arguments")

    # Draw graph
    setup_matplotlib(options.output is not None)
    fig = plot_motion(options.shaper,
                      options.shaper_freq, options.shaper_damping_ratio,
                      options.freq, options.damping_ratio, options.smooth_time)

    # Show graph
    if options.output is None:
        matplotlib.pyplot.show()
    else:
        fig.set_size_inches(8, 6)
        fig.savefig(options.output)

if __name__ == '__main__':
    main()
