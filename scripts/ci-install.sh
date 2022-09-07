#!/bin/bash
# Build setup script for continuous integration testing.
# See ci-build.sh for the actual test steps.

# Stop script early on any error; check variables; be verbose
set -eux

MAIN_DIR=${PWD}
BUILD_DIR=${PWD}/ci_build
mkdir -p ${BUILD_DIR}


######################################################################
# Install system dependencies
######################################################################

echo -e "\n\n=============== Install system dependencies\n\n"
PKGS="python3-dev libffi-dev build-essential"
PKGS="${PKGS} gcc-avr avr-libc"
PKGS="${PKGS} libnewlib-arm-none-eabi gcc-arm-none-eabi binutils-arm-none-eabi"
PKGS="${PKGS} pv libmpfr-dev libgmp-dev libmpc-dev texinfo bison flex"
sudo apt-get install ${PKGS}


######################################################################
# Create python3 virtualenv environment
######################################################################

echo -e "\n\n=============== Install python3 virtualenv\n\n"
cd ${MAIN_DIR}
python3 -m venv ${BUILD_DIR}/python-env
${BUILD_DIR}/python-env/bin/pip install -r ${MAIN_DIR}/scripts/klippy-requirements.txt
