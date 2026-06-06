#ifndef LIBECRT_H
#define LIBECRT_H
#include <stdint.h>

/* All functions operate on EtherCAT slave 1 (single-drive bring-up). */

/* go_realtime + ec_init + CSP/DC config + map + SAFE-OP + DC align + OP,
 * then PARK at CiA402 Ready-to-Switch-On (controlword 0x0006, no torque),
 * running an internal cyclic+DC loop with target=actual the whole time.
 * Returns 0 once parked. The drive is NOT energized; call ec_rt_enable().
 * Failure codes: -1 ec_init, -2 no slaves, -3 SAFE-OP not reached,
 *                -4 OP not reached, -5 CiA402 park timeout.
 * The caller MUST check the return is 0 before calling any other function;
 * on failure the bus is closed and the g_out/g_in pointers are not valid. */
int  ec_rt_bringup(const char *ifname, int64_t cycle_ns, int rt_cpu, int rt_prio);

/* CiA402 enable ladder to Operation Enabled (torque on), target tracking
 * actual throughout so torque engages at the current shaft position.
 * Returns 0 on success, -5 on ladder timeout/persistent fault. */
int  ec_rt_enable(void);

/* One steady-state DC cycle: sleep to next deadline, send+recv process data,
 * run the DC PI jitter correction. Enabled: controlword=0x000F. Parked:
 * controlword=0x0006 with target tracking actual. Writes the PI offset to
 * *toff_ns. Returns the working counter (3 == healthy). */
int  ec_rt_cycle(int64_t *toff_ns);

/* Stage the CSP target for the next cycle's send. */
void ec_rt_set_target_position(int32_t counts);

int32_t  ec_rt_get_position_actual(void);
uint16_t ec_rt_get_statusword(void);
uint16_t ec_rt_get_error_code(void);
int32_t  ec_rt_get_following_error(void);

/* controlword = 0x0006 (disable voltage path), held for a few cycles; leaves the cycle parked. */
void ec_rt_disable(void);

/* dcsync0 off, back to INIT, close NIC. */
void ec_rt_shutdown(void);

#endif
