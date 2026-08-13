import math
import multiprocessing
import traceback

from klippy import queuelogger

# Thermal-model calibration (INDX_CALIBRATE) defaults / limits.
MODEL_CAL_MIN_TEMP_DEFAULT = 60.0  # start heatup from below this (deg C)
MODEL_CAL_MIN_TEMP_CEILING = 100.0  # MIN_TEMP must stay below this
MODEL_CAL_MAX_TEMP_DEFAULT = 200.0  # heat up to this (deg C)
MODEL_CAL_HOLD_TIME_DEFAULT = 10.0  # dwell at MAX_TEMP (s); 0 disables
MODEL_CAL_COOLDOWN_TIME_DEFAULT = 15.0  # measure cooldown for this long (s)
MODEL_CAL_COOLDOWN_TIME_MAX = 20.0  # cap on the cooldown window (s)
MODEL_CAL_BASELINE_TIME = 4.0  # heater-off baseline power window (s)
MODEL_CAL_STEPPER_DWELL = 0.1  # dwell around stepper enable toggles (s)
MODEL_CAL_POLL_INTERVAL = 0.2  # reactor pause granularity in waits (s)
MODEL_CAL_FIRST_READ_TIMEOUT = 5.0  # wait for the first temperature report
MODEL_CAL_PRECOOL_TIMEOUT = 600.0  # wait-to-drop-below-MIN_TEMP cap (s)
MODEL_CAL_HEATUP_TIMEOUT = 600.0  # heatup-to-MAX_TEMP cap (s)
MODEL_CAL_MIN_SAMPLES = 20  # refuse to fit with fewer samples
# Reference temperature at which the stored max_power is defined; the fitted
# temperature coefficient corrects max_power away from this point:
#   max_power(T) = max_power + max_power_temp_coeff * (T - ref)
MODEL_MAX_POWER_REF_TEMP = 200.0

FAN_CAL_BREAKS_DEFAULT = 6
FAN_CAL_HOLD_TIME_DEFAULT = 5.0
FAN_CAL_MIN_SPEED_DEFAULT = 0.20
FAN_CAL_TEMP_DEFAULT = 200.0
FAN_CAL_SETTLE_TIME_DEFAULT = 10.0
FAN_CAL_HEATUP_TIMEOUT = 600.0
FAN_CAL_MIN_SAMPLES = 2

# Calibration samples are plain tuples so the fit can run in a forked
# subprocess: (temp, power, pwm, dt, ambient, phase).
CAL_TEMP, CAL_POWER, CAL_PWM, CAL_DT, CAL_AMBIENT, CAL_PHASE = range(6)


def fit_max_power(samples, baseline_power):
    # Bilinear least squares over the heat/hold phases:
    #   power - baseline = P0*pwm + b*pwm*(T - Tref)
    # P0 is max_power at the reference temperature; b (W/K) captures how the
    # delivered power changes as the nozzle heats. Fitted through the origin
    # (P=0 at pwm=0 post-baseline); the pwm factor weights the high-duty
    # heatup, and the wide temperature span there resolves the slope. No
    # external/voltage reference is used.
    ref = MODEL_MAX_POWER_REF_TEMP
    spp = spq = sqq = spy = sqy = 0.0
    n = 0
    t_min = t_max = None
    rows = []
    for s in samples:
        if s[CAL_PHASE] not in ("heat", "hold"):
            continue
        pwm = s[CAL_PWM]
        temp = s[CAL_TEMP]
        p1 = pwm
        p2 = pwm * (temp - ref)
        y = s[CAL_POWER] - baseline_power
        spp += p1 * p1
        spq += p1 * p2
        sqq += p2 * p2
        spy += p1 * y
        sqy += p2 * y
        rows.append((p1, p2, y))
        n += 1
        if t_min is None or temp < t_min:
            t_min = temp
        if t_max is None or temp > t_max:
            t_max = temp
    if n < MODEL_CAL_MIN_SAMPLES:
        return {"error": "not enough heat/hold samples for max_power (%d)" % n}
    det = spp * sqq - spq * spq
    if det == 0.0:
        return {
            "error": "degenerate max_power fit (too little temperature "
            "range at full drive)"
        }
    p0 = (spy * sqq - sqy * spq) / det
    b = (spp * sqy - spq * spy) / det
    if not (p0 > 0.0) or not math.isfinite(p0) or not math.isfinite(b):
        return {
            "error": "implausible max_power fit (P0=%.4g, coeff=%.4g)" % (p0, b)
        }

    p0_se = b_se = None
    try:
        ssr = sum((y - (p0 * p1 + b * p2)) ** 2 for (p1, p2, y) in rows)
        if n > 2:
            sigma2 = ssr / (n - 2)
            var_p0 = sigma2 * sqq / det
            var_b = sigma2 * spp / det
            if var_p0 >= 0.0:
                p0_se = math.sqrt(var_p0)
            if var_b >= 0.0:
                b_se = math.sqrt(var_b)
    except (ValueError, ZeroDivisionError):
        pass

    return {
        "max_power": p0,
        "max_power_se": p0_se,
        "max_power_ref_temp": ref,
        "max_power_temp_coeff": b,
        "max_power_temp_coeff_se": b_se,
        "max_power_n": n,
        "fit_t_min": t_min,
        "fit_t_max": t_max,
    }


def fit_thermal_rc(samples, baseline_power):
    # Integral-form linear least squares. Integrating C*dT/dt = P - (T-Ta)/R
    # gives  T_i - T_0 = (1/C)*E_i - (1/(RC))*G_i,  with cumulative trapezoids
    #   E_i = integral of (P - baseline), G_i = integral of (T - ambient).
    # Fit y = a*E + d*G through the origin (a=1/C, d=-1/(RC)); never
    # differentiates the noisy temperature signal.
    fitset = [
        s for s in samples if s[CAL_PHASE] in ("heat", "hold", "cooldown")
    ]
    if len(fitset) < MODEL_CAL_MIN_SAMPLES:
        return {
            "error": "not enough samples for thermal fit (%d)" % len(fitset)
        }
    rows = []
    e_cum = g_cum = 0.0
    prev_p = prev_e = None
    t0 = fitset[0][CAL_TEMP]
    for idx, s in enumerate(fitset):
        p = s[CAL_POWER] - baseline_power
        e = s[CAL_TEMP] - s[CAL_AMBIENT]
        if idx > 0:
            dt = s[CAL_DT]
            e_cum += 0.5 * (p + prev_p) * dt
            g_cum += 0.5 * (e + prev_e) * dt
        rows.append((e_cum, g_cum, s[CAL_TEMP] - t0))
        prev_p, prev_e = p, e

    saa = sum(E * E for (E, G, y) in rows)
    sag = sum(E * G for (E, G, y) in rows)
    sgg = sum(G * G for (E, G, y) in rows)
    say = sum(E * y for (E, G, y) in rows)
    sgy = sum(G * y for (E, G, y) in rows)
    det = saa * sgg - sag * sag
    if det == 0.0:
        return {"error": "degenerate thermal fit (singular normal equations)"}
    a = (say * sgg - sgy * sag) / det
    d = (saa * sgy - sag * say) / det
    if a <= 0.0 or d >= 0.0:
        return {
            "error": "unphysical thermal fit (C=%.4g, slope d=%.4g)" % (a, d)
        }
    capacity = 1.0 / a
    to_ambient_r = -a / d

    # Confidence intervals via covariance sigma^2 (X^T X)^-1 + delta method.
    capacity_ci = to_ambient_r_ci = None
    try:
        n = len(rows)
        ssr = sum((y - (a * E + d * G)) ** 2 for (E, G, y) in rows)
        if n > 2:
            sigma2 = ssr / (n - 2)
            var_a = sigma2 * sgg / det
            var_d = sigma2 * saa / det
            cov_ad = -sigma2 * sag / det
            var_c = var_a / a**4
            # R = -a/d : dR/da=-1/d, dR/dd=a/d^2
            var_r = (
                var_a / d**2 + a * a * var_d / d**4 - 2.0 * a * cov_ad / d**3
            )
            if var_c >= 0.0:
                capacity_ci = 1.96 * math.sqrt(var_c)
            if var_r >= 0.0:
                to_ambient_r_ci = 1.96 * math.sqrt(var_r)
    except (ValueError, ZeroDivisionError):
        pass

    # Goodness of fit: forward-simulate (same Euler step as the model) and
    # report the RMS temperature error in deg C.
    sq = 0.0
    t_sim = t0
    for idx, s in enumerate(fitset):
        if idx > 0:
            p = s[CAL_POWER] - baseline_power
            loss = (t_sim - s[CAL_AMBIENT]) / to_ambient_r
            t_sim += (p - loss) / capacity * s[CAL_DT]
        sq += (t_sim - s[CAL_TEMP]) ** 2
    rms = math.sqrt(sq / len(fitset))

    return {
        "thermal_capacity": capacity,
        "thermal_capacity_ci": capacity_ci,
        "to_ambient_r": to_ambient_r,
        "to_ambient_r_ci": to_ambient_r_ci,
        "rms": rms,
        "n_fit": len(fitset),
    }


def thermal_model_fit(samples, baseline_power):
    # Pure function (plain data in/out) so it can run in a forked subprocess.
    result = fit_thermal_rc(samples, baseline_power)
    if result.get("error"):
        return result
    max_power = fit_max_power(samples, baseline_power)
    if max_power.get("error"):
        return max_power
    result.update(max_power)
    return result


def mean_phase_power(samples, phase_name):
    vals = [s[CAL_POWER] for s in samples if s[CAL_PHASE] == phase_name]
    if not vals:
        return None
    return sum(vals) / len(vals)


def fan_calibration_breakpoints(count, min_speed):
    if count < 2 or not 0.0 < min_speed < 1.0:
        raise ValueError("invalid fan calibration breakpoints")
    step = (1.0 - min_speed) / (count - 1)
    return [min_speed + step * i for i in range(count)]


def fit_fan_model(samples, thermal_capacity, to_ambient_r):
    points = []
    for speed, phase_samples in samples:
        if len(phase_samples) < FAN_CAL_MIN_SAMPLES:
            continue
        duration = input_energy = ambient_energy = 0.0
        first_temp = phase_samples[0][0]
        last_temp = first_temp
        last_ambient = phase_samples[0][1]
        for temp, ambient, dt, pwm_energy in phase_samples[1:]:
            if dt <= 0.0:
                continue
            duration += dt
            input_energy += pwm_energy
            ambient_energy += (
                0.5
                * ((last_temp - last_ambient) + (temp - ambient))
                / to_ambient_r
                * dt
            )
            last_temp = temp
            last_ambient = ambient
        if duration <= 0.0:
            continue
        fan_power = (
            input_energy
            - ambient_energy
            - thermal_capacity * (last_temp - first_temp)
        ) / duration
        mean_delta = sum(t - a for t, a, _, _ in phase_samples) / len(
            phase_samples
        )
        conductance = fan_power / mean_delta if mean_delta > 0.0 else 0.0
        if fan_power > 0.0 and conductance > 0.0:
            points.append((speed, fan_power, mean_delta, conductance, duration))

    if len(points) < 3:
        return {"error": "not enough usable fan breakpoints (%d)" % len(points)}

    rows = [(math.log(p[0]), math.log(p[3])) for p in points]
    n = len(rows)
    mean_x = sum(x for x, _ in rows) / n
    mean_y = sum(y for _, y in rows) / n
    sxx = sum((x - mean_x) ** 2 for x, _ in rows)
    if sxx == 0.0:
        return {"error": "degenerate fan fit"}
    k = sum((x - mean_x) * (y - mean_y) for x, y in rows) / sxx
    intercept = mean_y - k * mean_x
    a = math.exp(-intercept)
    if not (a > 0.0 and k > 0.0 and math.isfinite(a) and math.isfinite(k)):
        return {"error": "unphysical fan fit (a=%.4g, k=%.4g)" % (a, k)}

    residuals = []
    actual = []
    for speed, power, delta, _, _ in points:
        predicted = delta / a * speed**k
        residuals.append(power - predicted)
        actual.append(power)
    rms = math.sqrt(sum(v * v for v in residuals) / n)
    mean_power = sum(actual) / n
    total_sq = sum((v - mean_power) ** 2 for v in actual)
    residual_sq = sum(v * v for v in residuals)
    r_squared = 1.0 - residual_sq / total_sq if total_sq > 0.0 else None

    a_ci = k_ci = None
    if n > 2:
        log_ssr = sum((y - (intercept + k * x)) ** 2 for x, y in rows)
        sigma2 = log_ssr / (n - 2)
        k_se = math.sqrt(sigma2 / sxx)
        intercept_se = math.sqrt(sigma2 * (1.0 / n + mean_x * mean_x / sxx))
        k_ci = 1.96 * k_se
        a_ci = 1.96 * a * intercept_se

    return {
        "part_cooling_fan_a": a,
        "part_cooling_fan_a_ci": a_ci,
        "part_cooling_fan_k": k,
        "part_cooling_fan_k_ci": k_ci,
        "rms_power": rms,
        "r_squared": r_squared,
        "points": points,
    }


def run_fit_in_background(printer, func, *fit_args):
    # Run a CPU-bound fit in a separate process so the reactor (MCU comms,
    # heater safety timers) is never stalled. Mirrors
    # mathutil.background_coordinate_descent.
    parent_conn, child_conn = multiprocessing.Pipe()

    def wrapper():
        queuelogger.clear_bg_logging()
        try:
            res = func(*fit_args)
        except Exception:
            child_conn.send((True, traceback.format_exc()))
            child_conn.close()
            return
        child_conn.send((False, res))
        child_conn.close()

    proc = multiprocessing.Process(target=wrapper)
    proc.daemon = True
    proc.start()
    reactor = printer.get_reactor()
    gcode = printer.lookup_object("gcode")
    eventtime = last_report = reactor.monotonic()
    while proc.is_alive():
        if eventtime > last_report + 5.0:
            last_report = eventtime
            gcode.respond_info("INDX: fitting thermal model...", log=False)
        eventtime = reactor.pause(eventtime + 0.1)
    try:
        is_err, res = parent_conn.recv()
    except EOFError:
        # Child died without sending (e.g. killed/segfault): surface it as a
        # normal fit error rather than an opaque EOFError.
        is_err, res = True, "fit process died unexpectedly"
    proc.join()
    parent_conn.close()
    if is_err:
        raise Exception("Error in thermal model fit: %s" % (res,))
    return res
