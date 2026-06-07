// LD_PRELOAD shim that replaces wall-clock time with a shared-memory
// virtual clock for faster-than-real-time Klipper simulation.
//
// Intercepts: clock_gettime, clock_nanosleep, nanosleep, ppoll, poll,
//             select, timer_create, timer_settime.
//
// Virtual clock lives in /dev/shm/kalico_vtime as an atomic uint64
// (nanoseconds). Time advances when processes "sleep" — the clock is
// bumped by the requested amount and the call returns immediately.
// I/O waits (poll/ppoll/select) do NOT advance virtual time; they
// busy-poll with brief real sleeps until data arrives or the virtual
// deadline (set by other processes' clock advances) is reached.

#define _GNU_SOURCE
#include "vtime.h"

#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

static int vtime_debug = 0;
static double vtime_speed = 100.0;
static uint64_t vtime_real_t0_ns = 0;
#define VLOG(fmt, ...) do { \
    if (vtime_debug) fprintf(stderr, "[vtime:%d] " fmt "\n", \
                             (int)getpid(), ##__VA_ARGS__); \
} while (0)

static struct vtime_shm *vshm = NULL;

static int (*real_clock_gettime)(clockid_t, struct timespec *) = NULL;
static int (*real_clock_nanosleep)(clockid_t, int, const struct timespec *,
                                   struct timespec *) = NULL;
static int (*real_nanosleep)(const struct timespec *, struct timespec *) = NULL;
static int (*real_poll)(struct pollfd *, nfds_t, int) = NULL;
static int (*real_ppoll)(struct pollfd *, nfds_t, const struct timespec *,
                          const sigset_t *) = NULL;
static int (*real_select)(int, fd_set *, fd_set *, fd_set *,
                           struct timeval *) = NULL;
static int (*real_timer_create)(clockid_t, struct sigevent *,
                                 timer_t *) = NULL;
static int (*real_timer_settime)(timer_t, int, const struct itimerspec *,
                                  struct itimerspec *) = NULL;
static int (*real_usleep)(useconds_t) = NULL;

static uint64_t
real_monotonic_ns(void)
{
    struct timespec ts;
    if (real_clock_gettime)
        real_clock_gettime(CLOCK_MONOTONIC, &ts);
    else
        clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

static uint64_t
vtime_speed_cap(void)
{
    if (vtime_speed <= 0)
        return UINT64_MAX;
    uint64_t real_elapsed = real_monotonic_ns() - vtime_real_t0_ns;
    uint64_t vtime_start = 1000000000ULL;
    return vtime_start + (uint64_t)(real_elapsed * vtime_speed);
}

static struct {
    timer_t real_timer;
    int armed;
    uint64_t target_ns;
    pthread_mutex_t lock;
} vtimer = { .lock = PTHREAD_MUTEX_INITIALIZER };

static uint64_t
ts_to_ns(const struct timespec *ts)
{
    return (uint64_t)ts->tv_sec * 1000000000ULL + (uint64_t)ts->tv_nsec;
}

static void
ns_to_ts(uint64_t ns, struct timespec *ts)
{
    ts->tv_sec = ns / 1000000000ULL;
    ts->tv_nsec = ns % 1000000000ULL;
}

static uint64_t
vtime_now(void)
{
    return atomic_load_explicit(&vshm->nanos, memory_order_acquire);
}

static uint64_t
vtime_advance_by(uint64_t delta_ns)
{
    uint64_t cap = vtime_speed_cap();
    uint64_t cur = vtime_now();
    uint64_t target = cur + delta_ns;
    if (target > cap) {
        while (vtime_speed_cap() < target) {
            struct timespec w = { 0, 100000 };
            real_nanosleep(&w, NULL);
        }
    }
    return atomic_fetch_add_explicit(&vshm->nanos, delta_ns,
                                     memory_order_acq_rel) + delta_ns;
}

static void
vtime_advance_to(uint64_t target_ns)
{
    uint64_t cap = vtime_speed_cap();
    if (target_ns > cap) {
        while (vtime_speed_cap() < target_ns) {
            struct timespec w = { 0, 100000 };
            real_nanosleep(&w, NULL);
        }
    }
    uint64_t cur;
    do {
        cur = atomic_load_explicit(&vshm->nanos, memory_order_acquire);
        if (target_ns <= cur)
            return;
    } while (!atomic_compare_exchange_weak_explicit(
        &vshm->nanos, &cur, target_ns,
        memory_order_acq_rel, memory_order_acquire));
}

static void
vtimer_check_and_fire(void)
{
    pthread_mutex_lock(&vtimer.lock);
    if (vtimer.armed) {
        uint64_t now = vtime_now();
        if (now >= vtimer.target_ns) {
            vtimer.armed = 0;
            pthread_mutex_unlock(&vtimer.lock);
            raise(SIGALRM);
            return;
        }
    }
    pthread_mutex_unlock(&vtimer.lock);
}

__attribute__((constructor(102)))
static void
vtime_init(void)
{
    const char *v = getenv("KALICO_VTIME_DEBUG");
    vtime_debug = (v && v[0] == '1');

    const char *speed_env = getenv("KALICO_VTIME_SPEED");
    if (speed_env)
        vtime_speed = atof(speed_env);
    if (vtime_speed < 1.0)
        vtime_speed = 100.0;

    real_clock_gettime = dlsym(RTLD_NEXT, "clock_gettime");
    real_clock_nanosleep = dlsym(RTLD_NEXT, "clock_nanosleep");
    real_nanosleep = dlsym(RTLD_NEXT, "nanosleep");
    real_poll = dlsym(RTLD_NEXT, "poll");
    real_ppoll = dlsym(RTLD_NEXT, "ppoll");
    real_select = dlsym(RTLD_NEXT, "select");
    real_timer_create = dlsym(RTLD_NEXT, "timer_create");
    real_timer_settime = dlsym(RTLD_NEXT, "timer_settime");
    real_usleep = dlsym(RTLD_NEXT, "usleep");

    int fd = shm_open(VTIME_SHM_NAME, O_RDWR, 0);
    if (fd < 0) {
        fd = shm_open(VTIME_SHM_NAME, O_CREAT | O_RDWR, 0666);
        if (fd < 0) {
            fprintf(stderr, "[vtime] shm_open failed: %s\n", strerror(errno));
            return;
        }
        if (ftruncate(fd, sizeof(struct vtime_shm)) < 0) {
            fprintf(stderr, "[vtime] ftruncate failed: %s\n", strerror(errno));
            close(fd);
            return;
        }
    }

    vshm = mmap(NULL, sizeof(struct vtime_shm), PROT_READ | PROT_WRITE,
                MAP_SHARED, fd, 0);
    close(fd);
    if (vshm == MAP_FAILED) {
        fprintf(stderr, "[vtime] mmap failed: %s\n", strerror(errno));
        vshm = NULL;
        return;
    }

    vtime_real_t0_ns = real_monotonic_ns();
    atomic_fetch_add(&vshm->num_participants, 1);
    VLOG("init, participants=%u, vtime=%lu ns, speed=%.0fx",
         atomic_load(&vshm->num_participants),
         (unsigned long)vtime_now(), vtime_speed);
}

int
clock_gettime(clockid_t clk_id, struct timespec *tp)
{
    if (!vshm || !tp)
        return real_clock_gettime(clk_id, tp);

    if (clk_id == CLOCK_MONOTONIC || clk_id == CLOCK_MONOTONIC_RAW
        || clk_id == CLOCK_MONOTONIC_COARSE) {
        ns_to_ts(vtime_now(), tp);
        return 0;
    }
    return real_clock_gettime(clk_id, tp);
}

int
clock_nanosleep(clockid_t clk_id, int flags, const struct timespec *req,
                struct timespec *rem)
{
    if (!vshm)
        return real_clock_nanosleep(clk_id, flags, req, rem);

    if (clk_id == CLOCK_MONOTONIC || clk_id == CLOCK_MONOTONIC_RAW) {
        if (flags & TIMER_ABSTIME) {
            uint64_t target = ts_to_ns(req);
            vtime_advance_to(target);
        } else {
            uint64_t delta = ts_to_ns(req);
            if (delta > 0)
                vtime_advance_by(delta);
        }
        vtimer_check_and_fire();
        sched_yield();
        if (rem) {
            rem->tv_sec = 0;
            rem->tv_nsec = 0;
        }
        return 0;
    }
    return real_clock_nanosleep(clk_id, flags, req, rem);
}

int
nanosleep(const struct timespec *req, struct timespec *rem)
{
    if (!vshm)
        return real_nanosleep(req, rem);

    uint64_t delta = ts_to_ns(req);
    if (delta > 0)
        vtime_advance_by(delta);
    vtimer_check_and_fire();
    sched_yield();
    if (rem) {
        rem->tv_sec = 0;
        rem->tv_nsec = 0;
    }
    return 0;
}

int
usleep(useconds_t usec)
{
    if (!vshm)
        return real_usleep(usec);

    uint64_t delta = (uint64_t)usec * 1000ULL;
    if (delta > 0)
        vtime_advance_by(delta);
    vtimer_check_and_fire();
    sched_yield();
    return 0;
}

int
poll(struct pollfd *fds, nfds_t nfds, int timeout_ms)
{
    if (!vshm)
        return real_poll(fds, nfds, timeout_ms);

    int ret = real_poll(fds, nfds, 0);
    if (ret != 0 || timeout_ms == 0)
        return ret;

    uint64_t deadline_ns;
    if (timeout_ms < 0) {
        deadline_ns = UINT64_MAX;
    } else {
        deadline_ns = vtime_now() + (uint64_t)timeout_ms * 1000000ULL;
    }

    struct timespec spin = { .tv_sec = 0, .tv_nsec = 100000 };
    for (int i = 0; i < 10000; i++) {
        ret = real_poll(fds, nfds, 0);
        if (ret != 0)
            return ret;

        if (vtime_now() >= deadline_ns)
            return 0;

        pthread_mutex_lock(&vtimer.lock);
        if (vtimer.armed && vtimer.target_ns <= deadline_ns) {
            uint64_t target = vtimer.target_ns;
            vtimer.armed = 0;
            pthread_mutex_unlock(&vtimer.lock);
            vtime_advance_to(target);
            raise(SIGALRM);
            errno = EINTR;
            return -1;
        }
        pthread_mutex_unlock(&vtimer.lock);

        real_nanosleep(&spin, NULL);
    }
    vtime_advance_to(deadline_ns);
    return 0;
}

int
ppoll(struct pollfd *fds, nfds_t nfds, const struct timespec *timeout,
      const sigset_t *sigmask)
{
    if (!vshm)
        return real_ppoll(fds, nfds, timeout, sigmask);

    struct timespec zero = { 0, 0 };
    int ret = real_ppoll(fds, nfds, &zero, sigmask);
    if (ret != 0)
        return ret;

    // Klipper's console_sleep calls ppoll(fds, n, NULL, sigmask) to
    // wait for serial data OR SIGALRM. In virtual time, we advance
    // the clock to the next virtual timer target and deliver SIGALRM.
    // This is equivalent to the kernel sleeping until the timer fires.

    uint64_t deadline_ns;
    if (!timeout) {
        deadline_ns = UINT64_MAX;
    } else {
        deadline_ns = vtime_now() + ts_to_ns(timeout);
    }

    for (int i = 0; i < 100000; i++) {
        if (vtime_now() >= deadline_ns)
            return 0;

        pthread_mutex_lock(&vtimer.lock);
        if (vtimer.armed) {
            uint64_t target = vtimer.target_ns;
            if (target <= deadline_ns) {
                vtimer.armed = 0;
                pthread_mutex_unlock(&vtimer.lock);
                vtime_advance_to(target);
                raise(SIGALRM);
                errno = EINTR;
                return -1;
            }
        }
        pthread_mutex_unlock(&vtimer.lock);

        struct timespec one_ms = { .tv_sec = 0, .tv_nsec = 1000000 };
        ret = real_ppoll(fds, nfds, &one_ms, sigmask);
        if (ret != 0)
            return ret;
    }
    if (deadline_ns < UINT64_MAX)
        vtime_advance_to(deadline_ns);
    return 0;
}

int
select(int nfds, fd_set *readfds, fd_set *writefds, fd_set *exceptfds,
       struct timeval *timeout)
{
    if (!vshm)
        return real_select(nfds, readfds, writefds, exceptfds, timeout);

    // select modifies fd_sets, so we need copies for retry
    fd_set r_copy, w_copy, e_copy;
    if (readfds) r_copy = *readfds;
    if (writefds) w_copy = *writefds;
    if (exceptfds) e_copy = *exceptfds;

    struct timeval zero = { 0, 0 };
    int ret = real_select(nfds, readfds, writefds, exceptfds, &zero);
    if (ret != 0)
        return ret;

    if (timeout && timeout->tv_sec == 0 && timeout->tv_usec == 0)
        return 0;

    uint64_t deadline_ns;
    if (!timeout) {
        deadline_ns = UINT64_MAX;
    } else {
        deadline_ns = vtime_now()
            + (uint64_t)timeout->tv_sec * 1000000000ULL
            + (uint64_t)timeout->tv_usec * 1000ULL;
    }

    for (int i = 0; i < 30000; i++) {
        if (readfds) *readfds = r_copy;
        if (writefds) *writefds = w_copy;
        if (exceptfds) *exceptfds = e_copy;

        struct timeval one_ms = { .tv_sec = 0, .tv_usec = 1000 };
        ret = real_select(nfds, readfds, writefds, exceptfds, &one_ms);
        if (ret != 0)
            return ret;

        if (vtime_now() >= deadline_ns) {
            if (timeout) {
                timeout->tv_sec = 0;
                timeout->tv_usec = 0;
            }
            return 0;
        }

        vtimer_check_and_fire();
    }
}

int
timer_create(clockid_t clk_id, struct sigevent *sevp, timer_t *timerid)
{
    if (!vshm)
        return real_timer_create(clk_id, sevp, timerid);

    // Klipper creates a CLOCK_MONOTONIC timer that fires SIGALRM.
    // We intercept this and use virtual time instead.
    if (clk_id == CLOCK_MONOTONIC) {
        pthread_mutex_lock(&vtimer.lock);
        vtimer.armed = 0;
        vtimer.target_ns = 0;
        pthread_mutex_unlock(&vtimer.lock);
        *timerid = (timer_t)0xCAFE;
        VLOG("timer_create: virtual timer created");
        return 0;
    }
    return real_timer_create(clk_id, sevp, timerid);
}

int
timer_settime(timer_t timerid, int flags, const struct itimerspec *new_value,
              struct itimerspec *old_value)
{
    if (!vshm || timerid != (timer_t)0xCAFE)
        return real_timer_settime(timerid, flags, new_value, old_value);

    if (old_value) {
        memset(old_value, 0, sizeof(*old_value));
    }

    pthread_mutex_lock(&vtimer.lock);
    if (new_value->it_value.tv_sec == 0 && new_value->it_value.tv_nsec == 0) {
        vtimer.armed = 0;
    } else if (flags & TIMER_ABSTIME) {
        vtimer.target_ns = ts_to_ns(&new_value->it_value);
        vtimer.armed = 1;
    } else {
        vtimer.target_ns = vtime_now() + ts_to_ns(&new_value->it_value);
        vtimer.armed = 1;
    }
    pthread_mutex_unlock(&vtimer.lock);

    vtimer_check_and_fire();

    return 0;
}
