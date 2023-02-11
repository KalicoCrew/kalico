// Extruder stepper pulse time generation
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // tanh
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "integrate.h" // struct smoother
#include "kin_shaper.h" // struct shaper_pulses
#include "pyhelper.h" // errorf
#include "trapq.h" // move_get_distance

// Without pressure advance, the extruder stepper position is:
//     extruder_position(t) = nominal_position(t)
// When pressure advance is enabled, additional filament is pushed
// into the extruder during acceleration (and retracted during
// deceleration). The formula is:
//     pa_position(t) = (nominal_position(t)
//                       + pressure_advance * nominal_velocity(t))
// The nominal position and velocity are then smoothed using a weighted average:
//     smooth_position(t) = (
//         definitive_integral(nominal_position(x+t_offs) * smoother(t-x) * dx,
//                             from=t-smooth_time/2, to=t+smooth_time/2)
//     smooth_velocity(t) = (
//         definitive_integral(nominal_velocity(x+t_offs) * smoother(t-x) * dx,
//                             from=t-smooth_time/2, to=t+smooth_time/2)
// and the final pressure advance value calculated as
//     smooth_pa_position(t) = smooth_position(t) + pa_func(smooth_velocity(t))
// where pa_func(v) = pressure_advance * v for linear velocity model or a more
// complicated function for non-linear pressure advance models.

// Calculate the definitive integral of extruder for a given move
static double
pa_move_integrate(struct move *m, int axis, double pressure_advance
                  , double base, double start, double end, double time_offset)
{
    // Calculate base position and velocity with pressure advance
    int can_pressure_advance = m->axes_r.x > 0. || m->axes_r.y > 0.;
    if (!can_pressure_advance)
        pressure_advance = 0.;
    double axis_r = m->axes_r.axis[axis - 'x'];
    double start_v = m->start_v * axis_r;
    double ha = m->half_accel * axis_r;
    base += pressure_advance * start_v;
    start_v += pressure_advance * 2. * ha;
    // Calculate definitive integral
    double iext = extruder_integrate(base, start_v, ha, start, end);
    double wgt_ext = extruder_integrate_time(base, start_v, ha, start, end);
    return wgt_ext - time_offset * iext;
}

// Calculate the definitive integral of the extruder over a range of moves
static double
pa_range_integrate(struct move *m, int axis, double move_time
                   , double pressure_advance, double hst)
{
    move_time += sm->t_offs;
    while (unlikely(move_time < 0.)) {
        m = list_prev_entry(m, node);
        move_time += m->move_t;
    }
    while (unlikely(move_time > m->move_t)) {
        move_time -= m->move_t;
        m = list_next_entry(m, node);
    }
    // Calculate integral for the current move
    double res = 0., start = move_time - hst, end = move_time + hst;
    double start_base = m->start_pos.axis[axis - 'x'];
    res += pa_move_integrate(m, axis, pressure_advance, 0.
                             , start, move_time, start);
    res -= pa_move_integrate(m, axis, pressure_advance, 0.
                             , move_time, end, end);
    // Integrate over previous moves
    const struct move *prev = m;
    while (likely(start < 0.)) {
        prev = list_prev_entry(prev, node);
        start += prev->move_t;
        double base = prev->start_pos.axis[axis - 'x'] - start_base;
        res += pa_move_integrate(prev, axis, pressure_advance, base, start
                                 , prev->move_t, start);
    }
    // Integrate over future moves
    t0 = move_time;
    while (likely(end > m->move_t)) {
        end -= m->move_t;
        t0 -= m->move_t;
        m = list_next_entry(m, node);
        double base = m->start_pos.axis[axis - 'x'] - start_base;
        res -= pa_move_integrate(m, axis, pressure_advance, base, 0., end, end);
    }
    *pos_integral += start_base;
}

static void
shaper_pa_range_integrate(const struct move *m, int axis, double move_time
                          , const struct shaper_pulses *sp
                          , const struct smoother *sm
                          , double *pos_integral, double *pa_velocity_integral)
{
    *pos_integral = *pa_velocity_integral = 0.;
    int num_pulses = sp->num_pulses, i;
    for (i = 0; i < num_pulses; ++i) {
        double t = sp->pulses[i].t, a = sp->pulses[i].a;
        double p_pos_int, p_pa_vel_int;
        pa_range_integrate(m, axis, move_time + t, sm,
                           &p_pos_int, &p_pa_vel_int);
        *pos_integral += a * p_pos_int;
        *pa_velocity_integral += a * p_pa_vel_int;
    }
}

struct pressure_advance_params {
    union {
        struct {
            double pressure_advance;
        };
        struct {
            double linear_advance, linear_offset, linearization_velocity;
        };
        double params[3];
    };
};

typedef double (*pressure_advance_func)(
        double, double, struct pressure_advance_params *pa_params);

struct extruder_stepper {
    struct stepper_kinematics sk;
    double pressure_advance, time_offset;
    double half_smooth_time, inv_half_smooth_time2;
};

double __visible
pressure_advance_linear_model_func(double position, double pa_velocity
                                   , struct pressure_advance_params *pa_params)
{
    return position + pa_velocity * pa_params->pressure_advance;
}

double __visible
pressure_advance_tanh_model_func(double position, double pa_velocity
                                 , struct pressure_advance_params *pa_params)
{
    position += pa_params->linear_advance * pa_velocity;
    if (pa_params->linear_offset) {
        double rel_velocity = pa_velocity / pa_params->linearization_velocity;
        position += pa_params->linear_offset * tanh(rel_velocity);
    }
    return position;
}

double __visible
pressure_advance_recipr_model_func(double position, double pa_velocity
                                   , struct pressure_advance_params *pa_params)
{
    position += pa_params->linear_advance * pa_velocity;
    if (pa_params->linear_offset) {
        double rel_velocity = pa_velocity / pa_params->linearization_velocity;
        position += pa_params->linear_offset * (1. - 1. / (1. + rel_velocity));
    }
    return position;
}

static double
extruder_calc_position(struct stepper_kinematics *sk, struct move *m
                       , double move_time)
{
    struct extruder_stepper *es = container_of(sk, struct extruder_stepper, sk);
    move_time += es->time_offset;
    while (unlikely(move_time < 0.)) {
        m = list_prev_entry(m, node);
        move_time += m->move_t;
    }
    while (unlikely(move_time >= m->move_t)) {
        move_time -= m->move_t;
        m = list_next_entry(m, node);
    }
    double hst = es->half_smooth_time;
    int i;
    struct coord e_pos;
    double move_dist = move_get_distance(m, move_time);
    for (i = 0; i < 3; ++i) {
        if (!hst) {
            e_pos.axis[i] = m->axes_r.axis[i] * move_dist;
        } else {
            double area = pa_range_integrate(m, 'x' + i, move_time,
                                             es->pressure_advance, hst);
            e_pos.axis[i] = area * es->inv_half_smooth_time2;
        }
        e_pos.axis[i] += m->start_pos.axis[i];
    }
    return e_pos.x + e_pos.y + e_pos.z;
}

static void
extruder_note_generation_time(struct extruder_stepper *es)
{
    double pre_active = 0., post_active = 0.;
    pre_active += es->half_smooth_time + es->time_offset;
    if (pre_active < 0.) pre_active = 0.;
    post_active += es->half_smooth_time - es->time_offset;
    if (post_active < 0.) post_active = 0.;
    es->sk.gen_steps_pre_active = pre_active;
    es->sk.gen_steps_post_active = post_active;
}

void __visible
extruder_set_pressure_advance(struct stepper_kinematics *sk
                              , double pressure_advance, double smooth_time
                              , double time_offset)
{
    struct extruder_stepper *es = container_of(sk, struct extruder_stepper, sk);
    double hst = smooth_time * .5;
    es->half_smooth_time = hst;
    es->time_offset = time_offset;
    extruder_note_generation_time(es);
    if (! hst)
        return;
    memcpy(&es->pa_params, params, n_params * sizeof(params[0]));
}

void __visible
extruder_set_pressure_advance_model_func(struct stepper_kinematics *sk
                                         , pressure_advance_func func)
{
    struct extruder_stepper *es = container_of(sk, struct extruder_stepper, sk);
    memset(&es->pa_params, 0, sizeof(es->pa_params));
    es->pa_func = func;
}

int __visible
extruder_set_shaper_params(struct stepper_kinematics *sk, char axis
                           , int n, double a[], double t[])
{
    if (axis != 'x' && axis != 'y')
        return -1;
    struct extruder_stepper *es = container_of(sk, struct extruder_stepper, sk);
    struct shaper_pulses *sp = &es->sp[axis-'x'];
    int status = init_shaper(n, a, t, sp);
    extruder_note_generation_time(es);
    return status;
}

int __visible
extruder_set_smoothing_params(struct stepper_kinematics *sk, char axis
                              , int n, double a[], double t_sm, double t_offs)
{
    if (axis != 'x' && axis != 'y' && axis != 'z')
        return -1;
    struct extruder_stepper *es = container_of(sk, struct extruder_stepper, sk);
    struct smoother *sm = &es->sm[axis-'x'];
    int status = init_smoother(n, a, t_sm, sm);
    sm->t_offs = t_offs;
    extruder_note_generation_time(es);
    return status;
}

double __visible
extruder_get_step_gen_window(struct stepper_kinematics *sk)
{
    struct extruder_stepper *es = container_of(sk, struct extruder_stepper, sk);
    return es->sk.gen_steps_pre_active > es->sk.gen_steps_post_active
         ? es->sk.gen_steps_pre_active : es->sk.gen_steps_post_active;
}

struct stepper_kinematics * __visible
extruder_stepper_alloc(void)
{
    struct extruder_stepper *es = malloc(sizeof(*es));
    memset(es, 0, sizeof(*es));
    es->sk.calc_position_cb = extruder_calc_position;
    es->sk.active_flags = AF_X | AF_Y | AF_Z;
    return &es->sk;
}
