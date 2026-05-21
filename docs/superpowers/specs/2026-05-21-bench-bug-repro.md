# Bench bug repro

```bash
ssh dderg@trident.local
sudo systemctl restart klipper && sleep 15
curl --data-urlencode "script=SET_KINEMATIC_POSITION X=100 Y=100 Z=10" http://localhost:7125/printer/gcode/script
curl --data-urlencode "script=G1 X110 F600" http://localhost:7125/printer/gcode/script
sleep 30
tail -n 3000 /home/dderg/printer_data/logs/klippy.log | grep "mcu 'mcu':" | grep fault_detail | sed -nE "s/.*fault_detail.: ([0-9]+).*/\1/p" | python3 -c "import sys; [print(f'0x{(int(l)>>24)&0xFF:02X}={int(l)&0xFFFFFF}') for l in sys.stdin]" | sort -u
```

Expect on success (motor moves): 0xC8 ≫ 0 (steps pushed), 0xC9 = small non-zero (≈ ±1), 0xCD reconstructs to milliseconds (f32 top-24 ≈ 0x3B000000..0x3D000000 range, decimal payload roughly 3.9M..4.1M).
Actual on bench: 0xC8 = 0, 0xC9 = 0, 0xCD = 4325383 (= top 24 of f32 ≈ 32 seconds). Arm succeeds (0xA1=4, 0xA2=5, 0xA3=3) but per-axis eval emits zero steps.