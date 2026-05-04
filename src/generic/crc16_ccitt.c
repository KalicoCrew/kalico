// Code for crc16_ccitt
//
// Copyright (C) 2016  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "misc.h" // crc16_ccitt

// Implement the standard crc "ccitt" algorithm on the given buffer
//
// Phase C kalico-native transport: `len` widened from `uint_fast8_t` (8-bit
// on glibc x86_64) to a true 32-bit type so kalico frames up to 64 KB can
// CRC correctly. Klipper-protocol callers pass tiny lengths (≤ MESSAGE_MAX)
// so they are unaffected.
uint16_t
crc16_ccitt(uint8_t *buf, uint32_t len)
{
    uint16_t crc = 0xffff;
    while (len--) {
        uint8_t data = *buf++;
        data ^= crc & 0xff;
        data ^= data << 4;
        crc = ((((uint16_t)data << 8) | (crc >> 8)) ^ (uint8_t)(data >> 4)
               ^ ((uint16_t)data << 3));
    }
    return crc;
}
