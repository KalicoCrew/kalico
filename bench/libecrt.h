#ifndef LIBECRT_H
#define LIBECRT_H
#include <stdint.h>

#define EC_RT_ERR_EC_INIT         (-1)
#define EC_RT_ERR_NO_SLAVES       (-2)
#define EC_RT_ERR_SAFE_OP_TIMEOUT (-3)
#define EC_RT_ERR_OP_TIMEOUT      (-4)
#define EC_RT_ERR_CIA402_TIMEOUT  (-5)
#define EC_RT_ERR_PDO_REMAP       (-6)
#define EC_RT_ERR_PDO_SIZE        (-7)
#define EC_RT_ERR_PREOP_TIMEOUT   (-8)
#define EC_RT_ERR_INIT_TIMEOUT    (-9)
#define EC_RT_ERR_RT_MLOCK        (-10)
#define EC_RT_ERR_RT_AFFINITY     (-11)
#define EC_RT_ERR_RT_SCHED        (-12)
#define EC_RT_ERR_RXPDO_ASSIGN    (-13)

/* Brings slave 1 to OPERATIONAL and parks it at CiA402 Ready-to-Switch-On
 * (no torque); ec_rt_enable() applies torque. 0 or an EC_RT_ERR_* above. */
int  ec_rt_bringup(const char *ifname, int64_t cycle_ns, int rt_cpu, int rt_prio);

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

/* SDO-read 6065h/6066h/6072h. 0 on success; -1/-2/-3 per failing object. */
int ec_rt_read_limits(uint32_t *ferr_counts, uint16_t *ferr_timeout_ms,
                      uint16_t *torque_tenth_pct);

/* SDO-write 6065h and 6072h. 0 on success; -1/-2 per failing object. */
int ec_rt_write_limits(uint32_t ferr_counts, uint16_t torque_tenth_pct);

/* controlword = 0x0006 (disable voltage path), held for a few cycles. */
void ec_rt_disable(void);

void ec_rt_dump_al_state(void);

void ec_rt_shutdown(void);

/* SDO upload from slave 1. On entry *size is the buffer capacity; on success
 * it holds the object's byte count. Returns 0 on success, -1 on failure with
 * *abort_code holding the CoE abort code (0 = transport-level failure). */
int ec_rt_sdo_read(uint16_t index, uint8_t sub, uint8_t *buf, int *size,
                   uint32_t *abort_code);

/* SDO download to slave 1. Same return/abort_code convention. */
int ec_rt_sdo_write(uint16_t index, uint8_t sub, const uint8_t *buf, int size,
                    uint32_t *abort_code);

#endif
