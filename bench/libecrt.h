#ifndef LIBECRT_H
#define LIBECRT_H
#include <stdint.h>

/* go_realtime + ec_init + TxPDO remap (variable 1A00h) + CSP/DC config + map
 * + SAFE-OP + DC align + OP, then parks at CiA402 Ready-to-Switch-On (no
 * torque). 0 on success; -1 ec_init, -2 no slaves, -3 SAFE-OP, -4 OP,
 * -5 park timeout, -6 TxPDO remap SDO write failed, -7 mapped PDO sizes
 * disagree with out_t/in_t. */
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

/* One-shot snapshot of servo-loop fields only; touch-probe and I/O state
 * stay on the per-field accessors. Includes the staged commanded target
 * (out_t), so a 1 kHz capture costs one FFI hop per cycle. */
typedef struct {
    uint16_t error_code;
    uint16_t statusword;
    int32_t  position_actual;
    int16_t  torque_actual;
    int32_t  following_error;
    int32_t  position_demand;
    int32_t  target_position;
} ec_telemetry_t;

void ec_rt_get_telemetry(ec_telemetry_t *out);

/* controlword = 0x0006 (disable voltage path), held for a few cycles. */
void ec_rt_disable(void);

void ec_rt_shutdown(void);

#endif
