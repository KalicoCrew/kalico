#!/bin/bash
set -euo pipefail

cd /work

echo "=== Starting Renode (dual MCU) ==="
renode --port 3335 --disable-gui \
  -e "include @tools/sim/dual_mcu_docker.resc" \
  -e 'logLevel 3 sysbus' -e 'logLevel 3 rcc' -e 'logLevel 3 nvic' \
  -e 'logLevel 0 usart2' -e 'start' &
RENODE_PID=$!

echo "Waiting for Renode UARTs..."
until nc -z localhost 3334 2>/dev/null; do sleep 0.5; done
until nc -z localhost 3336 2>/dev/null; do sleep 0.5; done
echo "Renode UARTs ready (H723:3334, F446:3336)"

echo "=== Starting socat PTY bridges ==="
socat PTY,link=/tmp/renode_h7_pty,raw,echo=0 TCP:localhost:3334 &
socat PTY,link=/tmp/renode_f4_pty,raw,echo=0 TCP:localhost:3336 &
until [ -e /tmp/renode_h7_pty ] && [ -e /tmp/renode_f4_pty ]; do sleep 0.2; done
echo "PTY bridges ready: H7=$(readlink /tmp/renode_h7_pty) F4=$(readlink /tmp/renode_f4_pty)"

echo "=== Printer config ==="
cat > /tmp/homing_test.cfg << 'CFGEOF'
[mcu]
serial: /tmp/renode_h7_pty

[mcu bottom]
serial: /tmp/renode_f4_pty

[printer]
kinematics: cartesian
max_velocity: 300
max_accel: 3000

[stepper_x]
step_pin: PB5
dir_pin: PB6
enable_pin: !PB7
microsteps: 16
rotation_distance: 40
endstop_pin: ^PC6
position_endstop: 300
position_max: 300
homing_speed: 50
homing_retract_dist: 5
min_home_dist: 15

[stepper_y]
step_pin: PB9
dir_pin: PB10
enable_pin: !PB11
microsteps: 16
rotation_distance: 40
endstop_pin: ^PB12
position_endstop: 0
position_max: 200
homing_speed: 50

[stepper_z]
step_pin: bottom:PB5
dir_pin: bottom:PB6
enable_pin: !bottom:PB7
microsteps: 16
rotation_distance: 8
endstop_pin: ^bottom:PC6
position_endstop: 0
position_max: 200
homing_speed: 5

[input_shaper]
shaper_freq_x: 50
shaper_type_x: smooth_zv
shaper_freq_y: 50
shaper_type_y: smooth_zv

[force_move]
enable_force_move: True
CFGEOF

# Remove stale chelper from host volume mount (wrong arch)
rm -f klippy/chelper/c_helper.so klippy/chelper/*.o

echo "=== Starting klippy ==="
mkdir -p /tmp/logs
python3 klippy/klippy.py /tmp/homing_test.cfg \
  -l /tmp/logs/klippy.log \
  -a /tmp/klippy_api &
KLIPPY_PID=$!

echo "Waiting for klippy to reach ready state..."
for i in $(seq 1 120); do
  if grep -q 'Printer is ready\|Welcome' /tmp/logs/klippy.log 2>/dev/null; then
    echo "Klippy ready after ${i}s"
    break
  fi
  if grep -q 'Printer is halted\|Internal error' /tmp/logs/klippy.log 2>/dev/null; then
    echo "Klippy FAILED after ${i}s:"
    grep 'error\|halted\|Internal\|Traceback' /tmp/logs/klippy.log | tail -10
    exit 1
  fi
  if ! kill -0 $KLIPPY_PID 2>/dev/null; then
    echo "Klippy process died after ${i}s"
    cat /tmp/logs/klippy.log | tail -20
    exit 1
  fi
  sleep 1
done

if ! grep -q 'Printer is ready\|Welcome' /tmp/logs/klippy.log 2>/dev/null; then
  echo "Klippy never reached ready state"
  tail -30 /tmp/logs/klippy.log
  exit 1
fi

echo "=== Running homing test ==="
python3 tools/sim/test_homing_lag.py --api /tmp/klippy_api --renode-monitor localhost:3335
TEST_RC=$?

echo ""
echo "=== klippy.log (homing-relevant lines) ==="
grep -i 'homing\|endstop\|arm\|trigger\|trip\|bridge-trace\|needs rehome\|No trigger\|steps_moved\|note_homing\|_mcu_pending' /tmp/logs/klippy.log 2>/dev/null | tail -30

kill $KLIPPY_PID $SOCAT_PID $RENODE_PID 2>/dev/null
exit $TEST_RC
