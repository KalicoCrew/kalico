#!/bin/bash
set -euo pipefail

cd /work

echo "=== Starting Renode ==="
renode --port 3335 --disable-gui \
  -e "include @tools/sim/h723_sim_docker.resc" \
  -e 'logLevel 3 sysbus' -e 'logLevel 3 rcc' -e 'logLevel 3 nvic' \
  -e 'logLevel 0 usart2' -e 'start' &
RENODE_PID=$!

# Wait for UART port
echo "Waiting for Renode UART..."
until nc -z localhost 3334 2>/dev/null; do sleep 0.5; done
echo "Renode UART ready"

echo "=== Starting socat PTY bridge ==="
socat PTY,link=/tmp/renode_pty,raw,echo=0 TCP:localhost:3334 &
SOCAT_PID=$!
until [ -e /tmp/renode_pty ]; do sleep 0.2; done
echo "PTY bridge ready: $(readlink /tmp/renode_pty)"

echo "=== Printer config ==="
cat > /tmp/homing_test.cfg << 'CFGEOF'
[mcu]
serial: /tmp/renode_pty

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
endstop_pin: ^PB8
position_endstop: 20
position_max: 20
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
step_pin: PB13
dir_pin: PB14
enable_pin: !PB15
microsteps: 16
rotation_distance: 8
endstop_pin: ^PC0
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

echo "=== Starting klippy ==="
mkdir -p /tmp/logs
python3 klippy/klippy.py /tmp/homing_test.cfg \
  -l /tmp/logs/klippy.log \
  -a /tmp/klippy_api &
KLIPPY_PID=$!

# Wait for klippy to connect to MCU
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

echo "=== Done ==="
# Copy logs out
cp /tmp/logs/klippy.log /work/tools/sim/.homing-test-logs/ 2>/dev/null || true

kill $KLIPPY_PID $SOCAT_PID $RENODE_PID 2>/dev/null
