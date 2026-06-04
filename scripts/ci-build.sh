#!/bin/bash
# Test script for continuous integration: klippy host software only.
#
# Firmware is NOT built here. The only supported firmware is the kalico
# motion-runtime firmware (test/configs/kalico-*.config: H723, F446, G0B1
# boards + the Linux MCU), and building it requires the Rust toolchain,
# which this image deliberately does not carry. The single firmware gate
# is .github/workflows/ci-mcu-firmware.yaml (full C + Rust staticlib link
# per arch via scripts/ci-build-mcu-kalico.sh). There is no other firmware
# flavor — no legacy motion planner, no step compression — so there is
# nothing else to compile.
#
# The klippy integration (.test) cases that boot a printer skip honestly
# when the native motion bridge / firmware dicts are absent (see
# test/klippy/conftest.py); they light up in environments that build the
# real engine.

# Stop script early on any error; check variables
set -eu

######################################################################
# Section grouping output message helpers
######################################################################

start_test()
{
    echo "::group::=============== $1 $2"
    set -x
}

finish_test()
{
    set +x
    echo "=============== Finished $2"
    echo "::endgroup::"
}

######################################################################
# Verify klippy host software
######################################################################

start_test klippy "py.test suite"
py.test
finish_test klippy "py.test suite"
