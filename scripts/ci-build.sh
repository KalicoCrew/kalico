#!/bin/bash
# Test script for continuous integration.

# Stop script early on any error; check variables
set -eu

# Paths to tools installed by ci-install.sh
MAIN_DIR=${PWD}
BUILD_DIR=/ci_build
export PATH=${BUILD_DIR}/or1k-linux-musl-cross/bin:${PATH}
PYTHON=${BUILD_DIR}/python-env/bin/python


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
# Run compile tests for several different MCU types
######################################################################

compile()
{
    for TARGET in test/configs/*.config ; do
        start_test mcu_compile "$TARGET"
        make clean
        make distclean
        unset CC
        cp ${TARGET} .config
        make olddefconfig
        make V=1 -j2
        size out/*.elf
        ./scripts/check-software-div.sh .config out/*.elf
        finish_test mcu_compile "$TARGET"
        cp out/klipper.dict ${1}/$(basename ${TARGET} .config).dict
    done
    make clean
    make distclean
}

export DICTDIR=${DICTDIR:-${BUILD_DIR}/dict}

if [ ! -d "${DICTDIR}" ]; then
    mkdir -p ${DICTDIR}
    compile ${DICTDIR}
elif [ ! -z "${1-}" ] && [ $1 == "compile" ]; then
    compile ${DICTDIR}
fi

######################################################################
# Verify klippy host software
######################################################################

start_test klippy "py.test suite"
py.test
finish_test klippy "py.test suite"
