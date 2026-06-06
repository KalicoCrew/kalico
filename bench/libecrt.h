#ifndef LIBECRT_H
#define LIBECRT_H
#include <stdint.h>

/* All functions operate on EtherCAT slave 1 (single-drive bring-up). */

/* go_realtime + ec_init + CSP/DC config + map + SAFE-OP + DC align + OP,
 * then parks at CiA402 Ready-to-Switch-On (no torque). 0 on success;
 * -1 ec_init, -2 no slaves, -3 SAFE-OP, -4 OP, -5 park timeout. */
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

/* controlword = 0x0006 (disable voltage path), held for a few cycles. */
void ec_rt_disable(void);

/* dcsync0 off, back to INIT, close NIC. */
void ec_rt_shutdown(void);

#endif
