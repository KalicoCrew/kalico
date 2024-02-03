# Extruder smoothers to synchronize extruder pressure advance with input shaping
#
# Copyright (C) 2023-2024  Dmitry Butyugin <dmbutyugin@google.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import collections, importlib, math
shaper_defs = importlib.import_module('.shaper_defs', 'extras')
shaper_calibrate = importlib.import_module('.shaper_calibrate', 'extras')

ExtruderSmootherCfg = collections.namedtuple(
        'ExtruderSmootherCfg', ('order', 'freq_opt_range'))

EXTURDER_SMOOTHERS = {
        'default' : ExtruderSmootherCfg(-1, (1.0, 1.0, 1)),
        'zv' : ExtruderSmootherCfg(5, (0.98, 1.02, 5)),
        'mzv' : ExtruderSmootherCfg(7, (0.95, 1.05, 11)),
        'zvd' : ExtruderSmootherCfg(7, (0.93, 1.06, 14)),
        'ei' : ExtruderSmootherCfg(7, (0.83, 0.89, 7)),
        '2hump_ei' : ExtruderSmootherCfg(9, (0.65, 0.75, 11)),
        '3hump_ei' : ExtruderSmootherCfg(10, (0.54, 0.66, 13)),
        'smooth_zv' : ExtruderSmootherCfg(7, (0.98, 1.0, 3)),
        'smooth_mzv' : ExtruderSmootherCfg(9, (0.95, 1.07, 20)),
        'smooth_ei' : ExtruderSmootherCfg(9, (0.97, 1.07, 15)),
        'smooth_zvd_ei' : ExtruderSmootherCfg(11, (0.90, 1.10, 30)),
        'smooth_2hump_ei' : ExtruderSmootherCfg(11, (0.95, 1.07, 20)),
        'smooth_si' : ExtruderSmootherCfg(11, (0.95, 1.07, 20)),
}

def _estimate_shaper(np, shaper, test_damping_ratio, test_freqs):
    A, T = np.asarray(shaper[0]), np.asarray(shaper[1])
    inv_D = 1. / A.sum()
    n = len(T)
    t_s = T[-1] - T[0]
    hst = t_s * 0.5

    test_freqs = np.asarray(test_freqs)
    t_start, t_end = -hst, hst
    n_t = 1000
    unity_range = np.linspace(0., 1., n_t)
    time = (t_end - t_start) * unity_range + t_start
    dt = (time[-1] - time[0]) / n_t

    omega = 2. * math.pi * test_freqs[test_freqs > 0.]

    response = np.zeros(shape=(omega.shape[0], time.shape[-1]))
    for i in range(n):
        s_r = shaper_calibrate.step_response(np, time - T[i] + hst,
                                             omega, test_damping_ratio)
        response += A[i] * s_r
    response *= inv_D
    velocity = (response[:,1:] - response[:,:-1]) / (omega * dt)[:, np.newaxis]
    return time[:-1], velocity

def _estimate_smoother(np, smoother, test_damping_ratio, test_freqs):
    C, t_sm = smoother[0], smoother[1]
    hst = t_sm * 0.5

    test_freqs = np.asarray(test_freqs)
    t_start, t_end = -t_sm, t_sm
    n_t = 1000
    unity_range = np.linspace(0., 1., n_t)
    time = (t_end - t_start) * unity_range + t_start
    dt = (time[-1] - time[0]) / n_t
    tau = np.copy(time)
    tau[time < -hst] = 0.
    tau[time > hst] = 0.

    w = np.zeros(shape=tau.shape)
    for c in C[::-1]:
        w = w * tau + c
    w[time < -hst] = 0.
    w[time > hst] = 0.
    norms = (w * dt).sum(axis=-1)

    omega = 2. * math.pi * test_freqs[test_freqs > 0.]

    w_ind = (time >= -hst) & (time < hst)
    wl = np.count_nonzero(w_ind)

    def get_windows(m, wl):
        nrows = m.shape[-1] - wl + 1
        n = m.strides[-1]
        return np.lib.stride_tricks.as_strided(m, shape=(m.shape[0], nrows, wl),
                                               strides=(m.strides[0], n, n))

    s_r = shaper_calibrate.step_response(np, time, omega, test_damping_ratio)
    w_dt = w[w_ind] * ((1. / norms) * dt)
    response = np.einsum("ijk,k->ij", get_windows(s_r, wl), w_dt[::-1])
    velocity = (response[:,1:] - response[:,:-1]) / (omega * dt)[:, np.newaxis]
    return time[w_ind], velocity

def _calc_extruder_smoother(np, shaper_name, t, velocities, n, t_sm):
    zero_derivatives = shaper_name.startswith('smooth_')
    if n <= 3:
        return [1.5, 0, -6.]
    if n <= 5 and zero_derivatives:
        return [15. / 8., 0., -15., 0., 30.]
    m = velocities.shape[0]

    t_i = np.zeros(shape=(t.shape[0], n))
    inv_t_sm = 1. / t_sm
    t_i[:, 0] = inv_t_sm
    for i in range(1, n):
        t_i[:, i] = t_i[:, i-1] * t * inv_t_sm
    w = 1. - np.abs(2. * t / t_sm)**2

    normalized_velocities = velocities / (
            velocities.sum(axis=-1)[:,np.newaxis] * (t[1] - t[0]))
    rhs = normalized_velocities.flat * np.tile(w, m)
    A = np.tile(t_i, (m, 1))
    Aw = np.tile(t_i * w[:,np.newaxis], (m, 1))

    B = np.matmul(A.T, Aw)
    f = np.matmul(A.T, rhs)

    B[0, :] = 0.
    for i in range(0, n, 2):
        B[0, i] = 1. / ((i+1) * 2**i)
    f[0] = 1.
    B[1, :] = np.power(-0.5, np.arange(n))
    B[2, :] = np.power(0.5, np.arange(n))
    f[1] = f[2] = 0.
    if zero_derivatives:
        B[3, :] = np.power(-0.5, np.arange(n)) * np.arange(n)
        B[4, :] = np.power(0.5, np.arange(n)) * np.arange(n)
        f[3] = f[4] = 0.
    if shaper_name == '3hump_ei':
        B[3, :] = np.power(0.5, np.arange(n)) * np.arange(n)
        f[3] = 0.
    C = np.linalg.solve(B, f)
    return C

def get_extruder_smoother(shaper_name, smooth_time, damping_ratio,
                          normalize_coeffs=True, return_velocities=False):
    try:
        np = importlib.import_module('numpy')
    except ImportError:
        raise self.error(
                "Failed to import `numpy` module, make sure it was "
                "installed via `~/klippy-env/bin/pip install` (refer to "
                "docs/Measuring_Resonances.md for more details).")
    shaper_name = shaper_name.lower()
    smoother_cfg = EXTURDER_SMOOTHERS.get(
            shaper_name, EXTURDER_SMOOTHERS['default'])
    test_freqs = np.linspace(*smoother_cfg.freq_opt_range)
    n = smoother_cfg.order
    for s in shaper_defs.INPUT_SHAPERS:
        if s.name == shaper_name:
            A, T = s.init_func(1.0, damping_ratio)
            if n < 0:
                n = 2 * len(A) + 1
            t_sm = T[-1] - T[0]
            shaper = A, T
            t, velocities = _estimate_shaper(np, shaper,
                                             damping_ratio, test_freqs)
            break
    for s in shaper_defs.INPUT_SMOOTHERS:
        if s.name == shaper_name:
            C, t_sm = s.init_func(1.0)
            if n < 0:
                n = len(C)
            smoother = C, t_sm
            t, velocities = _estimate_smoother(np, smoother,
                                               damping_ratio, test_freqs)
            break
    C_e = _calc_extruder_smoother(np, shaper_name, t, velocities, n, t_sm)
    smoother = shaper_defs.init_smoother(C_e[::-1], smooth_time,
                                         normalize_coeffs)
    if not return_velocities:
        return smoother
    return smoother, (t, velocities)
