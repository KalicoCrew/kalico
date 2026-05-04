// TTY based IO
//
// Copyright (C) 2017-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#define _GNU_SOURCE
#include <errno.h> // errno
#include <fcntl.h> // fcntl
#include <poll.h> // ppoll
#include <pty.h> // openpty
#include <stdio.h> // fprintf
#include <string.h> // memmove
#include <sys/stat.h> // chmod
#include <sys/types.h> // mode_t gid_t
#include <time.h> // struct timespec
#include <unistd.h> // ttyname
#include "board/irq.h" // irq_wait
#include "board/misc.h" // console_sendf
#include "command.h" // command_find_block
#include "internal.h" // console_setup
#include "kalico_demux.h" // kalico_demux_*
#include "kalico_dispatch.h" // kalico_dispatch_frame
#include "sched.h" // sched_wake_task

static struct pollfd main_pfd[1];
#define MP_TTY_IDX   0

// Report 'errno' in a message written to stderr
void
report_errno(char *where, int rc)
{
    int e = errno;
    fprintf(stderr, "Got error %d in %s: (%d)%s\n", rc, where, e, strerror(e));
}


/****************************************************************
 * Setup
 ****************************************************************/

int
set_non_blocking(int fd)
{
    int flags = fcntl(fd, F_GETFL);
    if (flags < 0) {
        report_errno("fcntl getfl", flags);
        return -1;
    }
    int ret = fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    if (ret < 0) {
        report_errno("fcntl setfl", flags);
        return -1;
    }
    return 0;
}

int
set_close_on_exec(int fd)
{
    int ret = fcntl(fd, F_SETFD, FD_CLOEXEC);
    if (ret < 0) {
        report_errno("fcntl set cloexec", ret);
        return -1;
    }
    return 0;
}

int
console_setup(char *name, mode_t mode, gid_t group)
{
    // Open pseudo-tty
    struct termios ti;
    memset(&ti, 0, sizeof(ti));
    int mfd, sfd, ret = openpty(&mfd, &sfd, NULL, &ti, NULL);
    if (ret) {
        report_errno("openpty", ret);
        return -1;
    }
    ret = set_non_blocking(mfd);
    if (ret)
        return -1;
    ret = set_close_on_exec(mfd);
    if (ret)
        return -1;
    ret = set_close_on_exec(sfd);
    if (ret)
        return -1;
    main_pfd[MP_TTY_IDX].fd = mfd;
    main_pfd[MP_TTY_IDX].events = POLLIN;

    // Create symlink to tty
    unlink(name);
    char *tname = ttyname(sfd);
    if (!tname) {
        report_errno("ttyname", 0);
        return -1;
    }
    ret = symlink(tname, name);
    if (ret) {
        report_errno("symlink", ret);
        return -1;
    }
    ret = chmod(tname, mode);
    if (ret) {
        report_errno("chmod", ret);
        return -1;
    }

    if (group != (gid_t) -1) {
        ret = chown(tname, (uid_t) -1 , group);
        if (ret) {
            report_errno("chgrp", ret);
            return -1;
        }
    }

    // Make sure stderr is non-blocking
    ret = set_non_blocking(STDERR_FILENO);
    if (ret)
        return -1;

    return 0;
}


/****************************************************************
 * Console handling
 ****************************************************************/

static struct task_wake console_wake;
static uint8_t receive_buf[4096];
static int receive_pos;

// Klipper-only byte accumulator. The kalico demuxer extracts bytes belonging
// to legacy Klipper frames out of the raw read buffer; we re-assemble those
// here so command_find_and_dispatch sees a contiguous Klipper-only stream
// (it expects a buffer it can pop_count out of).
static uint8_t klipper_only_buf[4096];
static int klipper_only_pos;

void *
console_receive_buffer(void)
{
    return klipper_only_buf;
}

// Raw byte writer used by the kalico transport TX path. Bypasses Klipper's
// msgproto framing — the bytes are already a complete kalico frame.
int
kalico_console_write_raw(const uint8_t *buf, uint16_t len)
{
    int ret = write(main_pfd[MP_TTY_IDX].fd, buf, len);
    if (ret < 0) {
        report_errno("write", ret);
        return -1;
    }
    return ret;
}

// Process any incoming commands
void
console_task(void)
{
    if (!sched_check_wake(&console_wake))
        return;

    // Read data
    int ret = read(main_pfd[MP_TTY_IDX].fd, &receive_buf[receive_pos]
                   , sizeof(receive_buf) - receive_pos);
    if (ret < 0) {
        if (errno == EWOULDBLOCK) {
            ret = 0;
        } else {
            report_errno("read", ret);
            return;
        }
    }
    if (ret == 15 && receive_buf[receive_pos+14] == '\n'
        && memcmp(&receive_buf[receive_pos], "FORCE_SHUTDOWN\n", 15) == 0)
        shutdown("Force shutdown command");

    // Drive the kalico-native demuxer over the freshly-read bytes, routing
    // Klipper bytes into klipper_only_buf and dispatching kalico frames
    // inline. See spec §6 (stream-level demux).
    int new_bytes = ret;
    if (new_bytes > 0) {
        fprintf(stderr, "[mcu] read %d bytes (first 0x%02x)\n",
                new_bytes, receive_buf[receive_pos]);
        fflush(stderr);
    }
    for (int i = 0; i < new_bytes; i++) {
        uint8_t b = receive_buf[receive_pos + i];
        kalico_demux_output_t out = kalico_demux_feed_byte(b);
        switch (out) {
        case KALICO_DEMUX_OUT_NONE:
            break;
        case KALICO_DEMUX_OUT_KLIPPER: {
            const uint8_t *kbuf = kalico_demux_klipper_buf();
            uint8_t klen = kalico_demux_klipper_len();
            if (klipper_only_pos + klen
                <= (int)sizeof(klipper_only_buf)) {
                memcpy(&klipper_only_buf[klipper_only_pos], kbuf, klen);
                klipper_only_pos += klen;
            }
            kalico_demux_consume();
            break;
        }
        case KALICO_DEMUX_OUT_KALICO: {
            uint8_t channel = kalico_demux_kalico_channel();
            const uint8_t *payload = kalico_demux_kalico_payload();
            uint16_t payload_len = kalico_demux_kalico_payload_len();
            kalico_dispatch_frame(channel, payload, payload_len);
            kalico_demux_consume();
            break;
        }
        case KALICO_DEMUX_OUT_ERROR:
            fprintf(stderr, "[mcu] demux ERROR after %d bytes consumed\n", i);
            fflush(stderr);
            kalico_demux_consume();
            break;
        }
    }
    // The raw read buffer is fully consumed by the demuxer per byte.
    receive_pos = 0;

    // Drain Klipper frames from klipper_only_buf via the existing parser.
    int len = klipper_only_pos;
    while (len > 0) {
        uint_fast8_t pop_count;
        uint_fast8_t msglen = len > MESSAGE_MAX ? MESSAGE_MAX : len;
        ret = command_find_and_dispatch(klipper_only_buf, msglen, &pop_count);
        if (!ret)
            break;
        len -= pop_count;
        if (len)
            memmove(klipper_only_buf, &klipper_only_buf[pop_count], len);
    }
    klipper_only_pos = len;
    if (klipper_only_pos > 0)
        sched_wake_task(&console_wake);
}
DECL_TASK(console_task);

// Encode and transmit a "response" message
void
console_sendf(const struct command_encoder *ce, va_list args)
{
    // Generate message
    uint8_t buf[MESSAGE_MAX];
    uint_fast8_t msglen = command_encode_and_frame(buf, ce, args);

    // Transmit message
    int ret = write(main_pfd[MP_TTY_IDX].fd, buf, msglen);
    if (ret < 0)
        report_errno("write", ret);
}

// Sleep until a signal received (waking early for console input if needed)
void
console_sleep(sigset_t *sigset)
{
    int ret = ppoll(main_pfd, ARRAY_SIZE(main_pfd), NULL, sigset);
    if (ret <= 0) {
        if (errno != EINTR)
            report_errno("ppoll main_pfd", ret);
        return;
    }
    if (main_pfd[MP_TTY_IDX].revents)
        sched_wake_task(&console_wake);
}
