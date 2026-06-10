#ifndef LIBECRT_H
#define LIBECRT_H
#include <stdint.h>

/* go_realtime + ec_init + CSP/DC config + map + SAFE-OP + DC align + OP,
 * then parks at CiA402 Ready-to-Switch-On (no torque). 0 on success;
 * -1 ec_init, -2 no slaves, -3 SAFE-OP, -4 OP, -5 park timeout,
 * -6 PDO remap, -7 FF routing. */
int  ec_rt_bringup(const char *ifname, int64_t cycle_ns, int rt_cpu, int rt_prio);

/* CiA402 ladder to Operation Enabled. 0 on success, -5 on timeout/fault. */
int  ec_rt_enable(void);

/* One steady-state DC cycle: sleep to next deadline, send+recv process data,
 * run the DC PI jitter correction, keep controlword=0x000F. Writes the PI
 * offset to *toff_ns. Returns the working counter (3 == healthy). */
int  ec_rt_cycle(int64_t *toff_ns);

/* Stage the CSP target for the next cycle's send. */
void ec_rt_set_target_position(int32_t counts);

int32_t  ec_rt_get_position_actual(void);
uint16_t ec_rt_get_statusword(void);
uint16_t ec_rt_get_error_code(void);
int32_t  ec_rt_get_following_error(void);

/* Stage CiA402 offsets for the next cycle's send (zeroed at bring-up and
 * on disable). Velocity in encoder counts/s, torque in 0.1% of rated. */
void ec_rt_set_velocity_offset(int32_t counts_per_s);
void ec_rt_set_torque_offset(int16_t tenths_pct);
int32_t ec_rt_get_velocity_actual(void);
int16_t ec_rt_get_torque_actual(void);

/* controlword = 0x0006 (disable voltage path), held for a few cycles. */
void ec_rt_disable(void);

void ec_rt_shutdown(void);

#endif
