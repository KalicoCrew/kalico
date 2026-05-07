#!/bin/bash

KDIR="${HOME}/klipper"
KENV="${HOME}/klippy-env"

BKDIR="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

if [ ! -d "$KDIR" ] || [ ! -d "$KENV" ]; then
    echo "beacon: klipper or klippy env doesn't exist"
    exit 1
fi

# install beacon requirements to env
echo "beacon: installing python requirements to env, this may take 10+ minutes."
"${KENV}/bin/pip" install -r "${BKDIR}/requirements.txt"

# update link to beacon.py
echo "beacon: linking klippy to beacon.py."
if [ -e "${KDIR}/klippy/extras/beacon.py" ]; then
    rm "${KDIR}/klippy/extras/beacon.py"
fi
ln -s "${BKDIR}/beacon.py" "${KDIR}/klippy/extras/beacon.py"

# exclude beacon.py from klipper git tracking
if ! grep -q "klippy/extras/beacon.py" "${KDIR}/.git/info/exclude"; then
    echo "klippy/extras/beacon.py" >> "${KDIR}/.git/info/exclude"
fi
echo "beacon: installation successful."

echo "Updating firmware."
"$KENV/bin/python" "$BKDIR/update_firmware.py" update all
