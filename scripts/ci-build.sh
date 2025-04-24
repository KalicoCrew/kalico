#!/bin/bash
# Test script for continuous integration.

# Stop script early on any error; check variables
set -eu

# Paths to tools installed by ci-install.sh
MAIN_DIR=${PWD}
BUILD_DIR=/ci_build
export PATH=${BUILD_DIR}/pru-gcc/bin:${PATH}
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
# List available test configs in JSON format
######################################################################

list_configs()
{
    local configs=()
    for TARGET in test/configs/*.config; do
        configs+=("\"$(basename ${TARGET} .config)\"")
    done
    
    # Join array elements with commas
    local IFS=","
    # Output compact single-line JSON to avoid escaping issues
    echo "{\"configs\":[${configs[*]}]}"
}

######################################################################
# Run compile tests for several different MCU types
######################################################################

compile()
{
    local dict_dir=$1
    local specific_config=$2
    
    if [ -n "$specific_config" ]; then
        # Run only the specified config
        if [ ! -f "test/configs/${specific_config}.config" ]; then
            echo "Error: Config file test/configs/${specific_config}.config not found"
            exit 1
        fi
        
        run_compile_test "${dict_dir}" "test/configs/${specific_config}.config"
    else
        # Run all configs
        for TARGET in test/configs/*.config; do
            run_compile_test "${dict_dir}" "${TARGET}"
        done
    fi
    
    make clean
    make distclean
}

run_compile_test()
{
    local dict_dir=$1
    local target=$2
    local config_name=$(basename ${target} .config)
    
    start_test mcu_compile "$target"
    make clean
    make distclean
    unset CC
    cp ${target} .config
    make olddefconfig
    make V=1 -j2
    size out/*.elf
    finish_test mcu_compile "$target"
    cp out/klipper.dict ${dict_dir}/${config_name}.dict
}

export DICTDIR=${DICTDIR:-${BUILD_DIR}/dict}

# Process command line arguments
if [ $# -eq 0 ]; then
    # Default behavior: compile all configs if DICTDIR doesn't exist
    if [ ! -d "${DICTDIR}" ]; then
        mkdir -p ${DICTDIR}
        compile ${DICTDIR} ""
    fi
else
    case "$1" in
        list-configs)
            list_configs
            exit 0
            ;;
        compile)
            mkdir -p ${DICTDIR}
            if [ $# -gt 1 ]; then
                # Compile specific config
                compile ${DICTDIR} "$2"
            else
                # Compile all configs
                compile ${DICTDIR} ""
            fi
            ;;
        *)
            echo "Usage: $0 [list-configs|compile [config_name]]"
            exit 1
            ;;
    esac
fi

######################################################################
# Verify klippy host software
######################################################################

if [ "${1-}" != "list-configs" ] && [ "${1-}" != "compile" ]; then
    start_test klippy "py.test suite"
    py.test
    finish_test klippy "py.test suite"
fi
