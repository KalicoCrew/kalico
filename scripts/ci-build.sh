#!/bin/bash
# Test script for continuous integration.

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
