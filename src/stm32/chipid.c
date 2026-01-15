// Support for extracting the hardware chip id on stm32
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "generic/canserial.h" // canserial_set_uuid
#include "generic/usb_cdc.h"   // usb_fill_serial
#include "generic/usbstd.h"    // usb_string_descriptor
#include "internal.h"          // UID_BASE
#include "sched.h"             // DECL_INIT

#define CHIP_UID_LEN 12

static struct {
    struct usb_string_descriptor desc;
    uint16_t data[CHIP_UID_LEN * 2];
} cdc_chipid;

struct usb_string_descriptor *
usbserial_get_serialid(void)
{
    return &cdc_chipid.desc;
}

void
chipid_init(void)
{
    void *uid = (void *)UID_BASE;

// stm32h5 requires 32 bit reads
// see: https://community.st.com/t5/stm32-mcus/how-to-avoid-a-hardfault-when-icache-is-enabled-on-the-stm32h5/ta-p/630085
#if CONFIG_MACH_STM32H5
    uint32_t buf[CHIP_UID_LEN / sizeof(uint32_t)];
    for (uint8_t i = 0; i < CHIP_UID_LEN / sizeof(uint32_t); i++)
        buf[i] = ((uint32_t *)UID_BASE)[i];
    uid = buf;
#endif

    if (CONFIG_USB_SERIAL_NUMBER_CHIPID)
        usb_fill_serial(&cdc_chipid.desc, ARRAY_SIZE(cdc_chipid.data), uid);

    if (CONFIG_CANBUS) {
        if (CONFIG_CAN_UUID_USE_CHIPID) {
            canserial_set_uuid(uid, CHIP_UID_LEN);
        } else {
            canserial_set_uuid((uint8_t *)CONFIG_CAN_UUID_CUSTOM, CHIP_UID_LEN);
        }
    }
}

DECL_INIT(chipid_init);
