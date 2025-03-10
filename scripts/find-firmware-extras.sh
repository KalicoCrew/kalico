#!/bin/bash

if [ ! -d src/extras ] ; then
    echo "Must run from top level source directory"
    exit 1
fi

find src/extras -name Kconfig -depth 2 | sed 's,\(.*\),source "\1",' > src/extras/Kconfig.tmp

if [ -s src/extras/Kconfig.tmp ] ; then
    echo 'menu "Firmware Extras"' > src/extras/Kconfig
    cat src/extras/Kconfig.tmp >> src/extras/Kconfig
    echo 'endmenu' >> src/extras/Kconfig

    find src/extras -name Makefile -depth 2 | sed 's,\(.*\),include \1,' > src/extras/Makefile
else
    echo -n '' > src/extras/Kconfig
    echo -n '' > src/extras/Makefile
fi

rm -f src/extras/Kconfig.tmp

