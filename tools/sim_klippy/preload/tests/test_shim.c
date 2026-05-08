// Direct test harness for libsim_intercept.so. Loads the shim via
// LD_PRELOAD (not dlopen — we rely on the constructor running in the
// process before main() runs), then exercises each handler.
//
// Run via: LD_PRELOAD=../libsim_intercept.so ./test_shim
// _GNU_SOURCE is defined via -D_GNU_SOURCE in CFLAGS.
#include <assert.h>
#include <fcntl.h>
#include <linux/gpio.h>
#include <linux/spi/spidev.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#define ASSERT_EQ(a, b) do { \
    long _a = (long)(a), _b = (long)(b); \
    if (_a != _b) { \
        fprintf(stderr, "FAIL %s:%d: %s = %ld != %ld\n", __FILE__, __LINE__, #a, _a, _b); \
        return 1; \
    } \
} while (0)

static int test_gpio_open_returns_fake_fd(void) {
    int fd = open("/dev/gpiochip0", O_RDWR);
    ASSERT_EQ(fd >= 0x10000000, 1);
    close(fd);
    return 0;
}

static int test_gpio_line_handle_roundtrip(void) {
    int chip = open("/dev/gpiochip0", O_RDWR);
    ASSERT_EQ(chip >= 0, 1);
    struct gpiohandle_request req = {0};
    req.lines = 1;
    req.flags = GPIOHANDLE_REQUEST_OUTPUT;
    req.lineoffsets[0] = 5;
    req.default_values[0] = 1;
    snprintf(req.consumer_label, sizeof(req.consumer_label), "test");
    ASSERT_EQ(ioctl(chip, GPIO_GET_LINEHANDLE_IOCTL, &req), 0);
    ASSERT_EQ(req.fd >= 0x10000000, 1);
    struct gpiohandle_data data = {0};
    ASSERT_EQ(ioctl(req.fd, GPIOHANDLE_GET_LINE_VALUES_IOCTL, &data), 0);
    ASSERT_EQ(data.values[0], 1);
    data.values[0] = 0;
    ASSERT_EQ(ioctl(req.fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data), 0);
    data.values[0] = 99;
    ASSERT_EQ(ioctl(req.fd, GPIOHANDLE_GET_LINE_VALUES_IOCTL, &data), 0);
    ASSERT_EQ(data.values[0], 0);
    close(req.fd);
    close(chip);
    return 0;
}

static int test_iio_default_value(void) {
    int fd = open("/sys/bus/iio/devices/iio:device0/in_voltage3_raw", O_RDONLY);
    ASSERT_EQ(fd >= 0x10000000, 1);
    char buf[16] = {0};
    ssize_t n = pread(fd, buf, sizeof(buf) - 1, 0);
    ASSERT_EQ(n > 0, 1);
    ASSERT_EQ(atoi(buf), 3900);
    close(fd);
    return 0;
}

static int test_pwm_write_absorbed(void) {
    int fd = open("/sys/class/pwm/pwmchip0/pwm0/period", O_WRONLY);
    ASSERT_EQ(fd >= 0x10000000, 1);
    const char *s = "1000000";
    ASSERT_EQ(write(fd, s, 7), 7);
    close(fd);
    return 0;
}

static int test_control_socket_ping(void) {
    const char *dir = getenv("KALICO_SIM_SOCK_DIR");
    if (!dir) { fprintf(stderr, "SKIP: KALICO_SIM_SOCK_DIR not set\n"); return 0; }
    char path[256];
    snprintf(path, sizeof(path), "%s/sim_control", dir);
    int sock = socket(AF_UNIX, SOCK_STREAM, 0);
    ASSERT_EQ(sock >= 0, 1);
    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    size_t plen = strlen(path);
    if (plen >= sizeof(addr.sun_path)) {
        fprintf(stderr, "socket path too long\n");
        return 1;
    }
    memcpy(addr.sun_path, path, plen + 1);
    if (connect(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("connect");
        return 1;
    }
    const char *req = "ping\n";
    ASSERT_EQ(write(sock, req, 5), 5);
    char reply[8] = {0};
    ssize_t n = read(sock, reply, sizeof(reply) - 1);
    ASSERT_EQ(n > 0, 1);
    ASSERT_EQ(strncmp(reply, "ok", 2), 0);
    close(sock);
    return 0;
}

#define RUN(t) do { \
    fprintf(stderr, "RUN %s\n", #t); \
    if (t() != 0) { fails++; fprintf(stderr, "FAIL %s\n", #t); } \
    else { fprintf(stderr, "PASS %s\n", #t); } \
} while (0)

int main(void) {
    int fails = 0;
    RUN(test_gpio_open_returns_fake_fd);
    RUN(test_gpio_line_handle_roundtrip);
    RUN(test_iio_default_value);
    RUN(test_pwm_write_absorbed);
    RUN(test_control_socket_ping);
    fprintf(stderr, "DONE fails=%d\n", fails);
    return fails ? 1 : 0;
}
