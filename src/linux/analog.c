// Read analog values from Linux IIO device
//
// Copyright (C) 2017  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <fcntl.h> // open
#include <stdio.h> // snprintf
#include <stdlib.h> // atoi
#include <unistd.h> // read
#include "command.h" // shutdown
#include "gpio.h" // gpio_adc_setup
#include "internal.h" // report_errno
#include "sched.h" // sched_shutdown

DECL_CONSTANT("ADC_MAX", 4095); // Assume 12bit adc

#define ANALOG_START (1<<12)

DECL_ENUMERATION_RANGE("pin", "analog0", ANALOG_START, 8);

#define IIO_PATH "/sys/bus/iio/devices/iio:device0/in_voltage%d_raw"

struct gpio_adc
gpio_adc_setup(uint32_t pin)
{
    char fname[256];
    uint8_t idx = (uint8_t)(pin - ANALOG_START);
    snprintf(fname, sizeof(fname), IIO_PATH, (int)idx);

    int fd = open(fname, O_RDONLY|O_CLOEXEC);
    if (fd < 0) {
#if CONFIG_KALICO_SIM
        // Sim builds: the IIO sysfs path doesn't exist on a host running
        // the test sim binary. Fall back to a synthetic gpio_adc whose
        // reads are served by analog_get_simulated_value. Channels that
        // haven't been seeded with runtime_sim_adc_set still return 0
        // (ADC_MIN), which klippy will treat as out-of-range — the test
        // fixture is responsible for seeding sane ADC values before
        // klippy attaches.
        return (struct gpio_adc){ .fd = -1, .adc_pin = idx };
#else
        report_errno("analog open", fd);
        goto fail;
#endif
    }
    int ret = set_non_blocking(fd);
    if (ret < 0)
        goto fail;
    return (struct gpio_adc){ .fd = fd, .adc_pin = idx };
fail:
    if (fd >= 0)
        close(fd);
    shutdown("Unable to open adc device");
}

// Sim-only: orchestrator-driven simulated ADC values for thermistors
// and heater feedback. Indexed by adc_pin (= pin - ANALOG_START, a
// small 0-based channel index). Set by command_runtime_sim_adc_set via
// this strong definition, which overrides the weak stub in
// src/runtime_sim_commands.c at link time.
//
// Default seed: ADC value matching ~25 °C on a typical NTC thermistor
// with 4.7 kΩ pull-up. For Generic 3950 (R0=100 kΩ @ 25 °C) the
// divider ratio R/(R+R_pull) ≈ 100/(100+4.7) ≈ 0.955 → ADC ≈ 3910 of
// 4095. Picking 3900 keeps every reasonable [min_temp, max_temp] band
// (e.g. min_temp=0, max_temp=120 for a heated bed) inside-bounds at
// boot. Tests that need a specific temperature seed via
// runtime_sim_adc_set; the seeded value overrides this default.
#define MAX_SIM_ADC 32
#define SIM_ADC_DEFAULT 3900
static uint16_t sim_adc_values[MAX_SIM_ADC] = {
    [0 ... MAX_SIM_ADC - 1] = SIM_ADC_DEFAULT,
};
static uint8_t  sim_adc_set[MAX_SIM_ADC] = {
    [0 ... MAX_SIM_ADC - 1] = 1,
};

void
analog_set_simulated_value(uint8_t adc_pin, uint16_t value)
{
    if (adc_pin < MAX_SIM_ADC) {
        sim_adc_values[adc_pin] = value;
        sim_adc_set[adc_pin]    = 1;
    }
}

static uint16_t
analog_get_simulated_value(uint8_t adc_pin, uint8_t *is_set_out)
{
    if (adc_pin < MAX_SIM_ADC && sim_adc_set[adc_pin]) {
        *is_set_out = 1;
        return sim_adc_values[adc_pin];
    }
    *is_set_out = 0;
    return 0;
}

uint32_t
gpio_adc_sample(struct gpio_adc g)
{
    return 0;
}

uint16_t
gpio_adc_read(struct gpio_adc g)
{
    // Sim shim: if a simulated value has been set for this channel,
    // return it instead of reading the (potentially unconfigured) sysfs ADC.
    uint8_t is_set = 0;
    uint16_t sv = analog_get_simulated_value(g.adc_pin, &is_set);
    if (is_set)
        return sv;

#if CONFIG_KALICO_SIM
    // Sim build with no seeded value: return 0. Tests should seed an
    // appropriate value via runtime_sim_adc_set; if they don't, klippy
    // will see ADC=0 and report out-of-range, which is a clearer
    // diagnostic than a sysfs read failure.
    if (g.fd < 0)
        return 0;
#endif

    char buf[64];
    int ret = pread(g.fd, buf, sizeof(buf)-1, 0);
    if (ret <= 0) {
        report_errno("analog read", ret);
        try_shutdown("Error on analog read");
        return 0;
    }
    buf[ret] = '\0';
    return atoi(buf);
}

void
gpio_adc_cancel_sample(struct gpio_adc g)
{
}
