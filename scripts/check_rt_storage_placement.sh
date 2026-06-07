#!/usr/bin/env bash
# Usage: ./scripts/check_rt_storage_placement.sh out/klipper.elf
#
# On H7 builds: rt_storage must land in `.axi_bss` at an address in
# [0x24000000, 0x24050000).
# On F4 builds: rt_storage must land in `.bss` (or `.bss.*`) at an address
# in [0x20000000, 0x20020000) and must be inside the [_bss_start,_bss_end]
# span (verified via boot-zeroing coverage cross-check).

set -euo pipefail

ELF="${1:-out/klipper.elf}"
if [ ! -f "$ELF" ]; then
    echo "ERROR: ELF not found: $ELF" >&2
    exit 1
fi

SYM_LINE="$("${OBJDUMP:-arm-none-eabi-objdump}" -t "$ELF" | awk '/ rt_storage$/{print}')"
if [ -z "$SYM_LINE" ]; then
    echo "ERROR: rt_storage symbol not found in $ELF" >&2
    echo "  (expected to be a C-declared uint8_t array in src/runtime_storage.c)" >&2
    exit 2
fi

# "${OBJDUMP:-arm-none-eabi-objdump}" -t format: <addr> <flags> <section> <size> <name>
ADDR_HEX="$(echo "$SYM_LINE" | awk '{print $1}')"
SECTION="$(echo "$SYM_LINE" | awk '{print $(NF-2)}')"
ADDR=$((16#${ADDR_HEX#0x}))

# Use awk instead of grep -q here: with `set -o pipefail`, grep -q exits
# early when it matches, SIGPIPE-killing the upstream objdump, and the
# pipe returns non-zero. awk reads stdin to EOF so the upstream finishes
# normally regardless of match position.
if "${OBJDUMP:-arm-none-eabi-objdump}" -t "$ELF" \
   | awk '/_axi_bss_start/{found=1} END{exit !found}'; then
    MCU=H7
    EXPECTED_SECTION=".axi_bss"
    MIN_ADDR=$((0x24000000))
    MAX_ADDR=$((0x24050000))
else
    MCU=F4
    EXPECTED_SECTION=".bss"
    MIN_ADDR=$((0x20000000))
    MAX_ADDR=$((0x20020000))
fi

echo "rt_storage: section=$SECTION addr=$ADDR_HEX MCU=$MCU"

if [ "$SECTION" != "$EXPECTED_SECTION" ]; then
    # Allow .bss subsections on F4 (.bss.runtime_storage, etc.)
    if [ "$MCU" = "F4" ] && [[ "$SECTION" == .bss* ]]; then
        :
    else
        echo "ERROR: rt_storage is in section '$SECTION', expected '$EXPECTED_SECTION' on $MCU" >&2
        exit 3
    fi
fi

if [ "$ADDR" -lt "$MIN_ADDR" ] || [ "$ADDR" -ge "$MAX_ADDR" ]; then
    printf "ERROR: rt_storage address 0x%x outside expected range [0x%x, 0x%x) on %s\n" \
        "$ADDR" "$MIN_ADDR" "$MAX_ADDR" "$MCU" >&2
    exit 4
fi

# (awk without early `exit` to avoid the pipefail + SIGPIPE issue that bit
# the MCU-detection heuristic above. The last match wins; there's only
# one _bss_start / _bss_end symbol so it doesn't matter.)
if [ "$MCU" = "F4" ]; then
    BSS_START_HEX="$("${OBJDUMP:-arm-none-eabi-objdump}" -t "$ELF" | awk '/_bss_start$/{addr=$1} END{print addr}')"
    BSS_END_HEX="$("${OBJDUMP:-arm-none-eabi-objdump}" -t "$ELF" | awk '/_bss_end$/{addr=$1} END{print addr}')"
    if [ -z "$BSS_START_HEX" ] || [ -z "$BSS_END_HEX" ]; then
        echo "WARNING: could not locate _bss_start/_bss_end symbols; skipping boot-zero coverage check" >&2
    else
        BSS_START=$((16#${BSS_START_HEX#0x}))
        BSS_END=$((16#${BSS_END_HEX#0x}))
        if [ "$ADDR" -lt "$BSS_START" ] || [ "$ADDR" -ge "$BSS_END" ]; then
            printf "ERROR: rt_storage 0x%x outside boot-zeroed [_bss_start=0x%x, _bss_end=0x%x) — orphan section escaped zeroing\n" \
                "$ADDR" "$BSS_START" "$BSS_END" >&2
            exit 5
        fi
    fi
fi

echo "OK: rt_storage placed correctly on $MCU"
