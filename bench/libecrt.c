#define _GNU_SOURCE
#include "libecrt.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>
#include "ethercat.h"

/*
 * PDO layout must match the drive's active mapping exactly.
 *
 * RxPDO 0x1701 (12 bytes):
 *   controlword      6040  uint16
 *   target_position  607A  int32
 *   touch_probe_fn   60B8  uint16
 *   phys_outputs     60FE:01 uint32
 *
 * TxPDO 0x1B01 (28 bytes):
 *   error_code       603F  uint16
 *   statusword       6041  uint16
 *   position_actual  6064  int32
 *   torque_actual    6077  int16
 *   following_error  60F4  int32
 *   tp_status        60B9  uint16
 *   tp1_pos          60BA  int32
 *   tp2_pos          60BC  int32
 *   digital_inputs   60FD  uint32
 */
#pragma pack(push, 1)
typedef struct {
    uint16_t controlword;
    int32_t  target_position;
    uint16_t touch_probe_fn;
    uint32_t phys_outputs;
} out_t;
typedef struct {
    uint16_t error_code;
    uint16_t statusword;
    int32_t  position_actual;
    int16_t  torque_actual;
    int32_t  following_error;
    uint16_t tp_status;
    int32_t  tp1_pos;
    int32_t  tp2_pos;
    uint32_t digital_inputs;
} in_t;
#pragma pack(pop)

static char    IOmap[4096];
static out_t  *g_out;
static in_t   *g_in;
static int64_t g_cycle_ns;
static struct timespec g_ts;
static int64_t g_integral;
static int g_enabled;

static void add_ts(struct timespec *ts, int64_t add) {
    int64_t ns  = add % 1000000000LL;
    int64_t sec = (add - ns) / 1000000000LL;
    ts->tv_sec  += sec;
    ts->tv_nsec += ns;
    if (ts->tv_nsec >= 1000000000LL) { ts->tv_nsec -= 1000000000LL; ts->tv_sec++; }
}

/*
 * DC PI jitter correction — identical algorithm to ec_spin.c's dc_sync().
 * Uses g_integral instead of a function-local static so the integrator state
 * persists correctly across the bringup loop and the steady-state cycle calls.
 */
static void dc_sync(int64_t reftime, int64_t cycletime, int64_t *offset) {
    int64_t delta = reftime % cycletime;
    if (delta > cycletime / 2) delta -= cycletime;
    if (delta > 0) g_integral++;
    if (delta < 0) g_integral--;
    *offset = -(delta / 100) - (g_integral / 20);
}

/*
 * Best-effort RT hardening. Failures are non-fatal (the caller may lack
 * CAP_IPC_LOCK / CAP_SYS_NICE) but they ARE reported on stderr: without RT
 * scheduling the DC loop jitters and the drive throws Er74.1 / misses SYNC0,
 * which looks like a drive bug rather than a missing-capability problem.
 */
static void go_realtime(int cpu, int prio) {
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) perror("ec_rt: mlockall (continuing)");
    cpu_set_t set; CPU_ZERO(&set); CPU_SET(cpu, &set);
    if (sched_setaffinity(0, sizeof(set), &set) != 0) perror("ec_rt: setaffinity (continuing)");
    struct sched_param sp; sp.sched_priority = prio;
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) perror("ec_rt: SCHED_FIFO (continuing)");
}

static int rt_exchange(int64_t *toff) {
    int64_t off = 0;
    add_ts(&g_ts, g_cycle_ns + (toff ? *toff : 0));
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &g_ts, NULL);
    ec_send_processdata();
    int wkc = ec_receive_processdata(EC_TIMEOUTRET);
    dc_sync(ec_DCtime, g_cycle_ns, &off);
    if (toff) *toff = off;
    return wkc;
}

int ec_rt_bringup(const char *ifname, int64_t cycle_ns, int rt_cpu, int rt_prio) {
    g_cycle_ns = cycle_ns < 250000 ? 250000 : cycle_ns;
    g_integral = 0;
    g_enabled  = 0;
    go_realtime(rt_cpu, rt_prio);

    if (!ec_init(ifname)) return -1;
    if (ec_config_init(FALSE) <= 0) { ec_close(); return -2; }

    /*
     * SDO bring-up order is identical to ec_spin.c and must be preserved
     * exactly: mode first, then both sync-type subindices, then cycle-time
     * subindices.  The drive requires SYNC0 active before SAFE-OP
     * (else AL 0x0030 / Er74.1).
     */
    int8_t  opmode  = 8;                       /* CSP */
    uint16_t sync_dc = 2;                      /* DC SYNC0 */
    uint32_t cyc    = (uint32_t)g_cycle_ns;

    ec_SDOwrite(1, 0x6060, 0x00, FALSE, sizeof(opmode),  &opmode,  EC_TIMEOUTRXM);
    ec_SDOwrite(1, 0x1C32, 0x01, FALSE, sizeof(sync_dc), &sync_dc, EC_TIMEOUTRXM);
    ec_SDOwrite(1, 0x1C33, 0x01, FALSE, sizeof(sync_dc), &sync_dc, EC_TIMEOUTRXM);
    ec_SDOwrite(1, 0x1C32, 0x02, FALSE, sizeof(cyc),     &cyc,     EC_TIMEOUTRXM);
    ec_SDOwrite(1, 0x1C33, 0x02, FALSE, sizeof(cyc),     &cyc,     EC_TIMEOUTRXM);

    ec_configdc();
    ec_dcsync0(1, TRUE, (uint32_t)g_cycle_ns, (int32_t)(g_cycle_ns / 2));
    ec_config_map(&IOmap);
    /* If SAFE-OP is not reached, PDOs are not mapped and ec_slave[1].outputs may
     * be NULL/stale — bail before dereferencing it in the stabilize loop. */
    if (ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4) != EC_STATE_SAFE_OP) {
        ec_close();
        return -3;
    }

    g_out = (out_t *) ec_slave[1].outputs;
    g_in  = (in_t  *) ec_slave[1].inputs;
    g_out->controlword    = 0;
    g_out->target_position = 0;
    g_out->touch_probe_fn  = 0;
    g_out->phys_outputs    = 0;

    clock_gettime(CLOCK_MONOTONIC, &g_ts);
    int64_t toff = 0;

    /* STABILIZE: align DC for 1.5 s with target tracking actual. Matches the
     * proven ec_spin.c STABILIZE_SEC; the Pi 3B's USB-attached NIC needs the
     * longer window for the DC PI loop to settle before OP, else Er74.1 /
     * AL 0x0030 at the SAFE-OP->OP transition. */
    for (int64_t i = 0; i < (int64_t)(1.5e9 / g_cycle_ns); i++) {
        g_out->controlword     = 0;
        g_out->target_position = g_in->position_actual;
        rt_exchange(&toff);
    }

    ec_slave[0].state = EC_STATE_OPERATIONAL;
    ec_writestate(0);
    for (int64_t i = 0; i < (int64_t)(2.0e9 / g_cycle_ns); i++) {
        g_out->target_position = g_in->position_actual;
        rt_exchange(&toff);
        if (i % 20 == 0) ec_readstate();
        if (ec_slave[0].state == EC_STATE_OPERATIONAL) break;
    }
    if (ec_slave[0].state != EC_STATE_OPERATIONAL) return -4;

    for (int64_t pc = 0; pc < 3000; pc++) {
        uint16_t sw = g_in->statusword;
        g_out->target_position = g_in->position_actual;
        if (sw & 0x0008) {
            g_out->controlword = ((pc / 10) % 2) ? 0x0080 : 0x0000; /* pulse fault reset */
        } else if ((sw & 0x006F) == 0x0021) {
            g_out->controlword = 0x0006;
            rt_exchange(&toff);
            g_enabled = 0;
            return 0;
        } else {
            g_out->controlword = 0x0006;
        }
        rt_exchange(&toff);
    }
    return -5;
}

int ec_rt_enable(void) {
    /*
     * CiA402 enable state machine — identical to ec_spin.c's ALIGN phase.
     * Masks and values match the CiA402 state-machine table exactly:
     *   sw & 0x004F == 0x0040  => Switch-On Disabled: issue 0x0006
     *   sw & 0x006F == 0x0021  => Ready-to-Switch-On: issue 0x0007
     *   sw & 0x006F == 0x0023  => Switched-On: issue 0x000F
     *   sw & 0x006F == 0x0027  => Operation Enabled: return 0
     *   sw & 0x0008            => Fault: pulse fault-reset on bit 7
     */
    int64_t toff = 0;
    for (int64_t pc = 0; pc < 3000; pc++) {
        uint16_t sw = g_in->statusword;
        g_out->target_position = g_in->position_actual;
        if (sw & 0x0008) {
            g_out->controlword = ((pc / 10) % 2) ? 0x0080 : 0x0000; /* pulse fault reset */
        } else if ((sw & 0x004F) == 0x0040) {
            g_out->controlword = 0x0006;
        } else if ((sw & 0x006F) == 0x0021) {
            g_out->controlword = 0x0007;
        } else if ((sw & 0x006F) == 0x0023) {
            g_out->controlword = 0x000F;
        } else if ((sw & 0x006F) == 0x0027) {
            g_out->controlword = 0x000F;
            rt_exchange(&toff);
            g_enabled = 1;
            return 0;
        } else {
            g_out->controlword = 0x0000;
        }
        rt_exchange(&toff);
    }
    return -5;
}

int ec_rt_cycle(int64_t *toff_ns) {
    if (g_enabled) {
        g_out->controlword = 0x000F;
    } else {
        g_out->controlword = 0x0006;
        g_out->target_position = g_in->position_actual;
    }
    return rt_exchange(toff_ns);
}

void ec_rt_set_target_position(int32_t counts) { g_out->target_position = counts; }
int32_t  ec_rt_get_position_actual(void)        { return g_in->position_actual; }
uint16_t ec_rt_get_statusword(void)             { return g_in->statusword; }
uint16_t ec_rt_get_error_code(void)             { return g_in->error_code; }
int32_t  ec_rt_get_following_error(void)        { return g_in->following_error; }

void ec_rt_disable(void) {
    g_enabled = 0;
    for (int i = 0; i < 100; i++) {
        g_out->controlword = 0x0006;
        g_out->target_position = g_in->position_actual;
        int64_t t = 0;
        rt_exchange(&t);
    }
}

void ec_rt_shutdown(void) {
    ec_dcsync0(1, FALSE, 0, 0);
    ec_slave[0].state = EC_STATE_INIT;
    ec_writestate(0);
    ec_close();
}
