# Bench bug repro

```bash
ssh dderg@trident.local
sudo systemctl restart klipper && sleep 15
curl --data-urlencode "script=SET_KINEMATIC_POSITION X=100 Y=100 Z=10" http://localhost:7125/printer/gcode/script
curl --data-urlencode "script=G1 X110 F600" http://localhost:7125/printer/gcode/script
sleep 30
tail -n 3000 /home/dderg/printer_data/logs/klippy.log | grep "mcu 'mcu':" | grep fault_detail | sed -nE "s/.*fault_detail.: ([0-9]+).*/\1/p" | python3 -c "import sys; [print(f'0x{(int(l)>>24)&0xFF:02X}={int(l)&0xFFFFFF}') for l in sys.stdin]" | sort -u
```

Expect: arm tags 0xA1=4, 0xA2=5, 0xA3=3 (arm succeeds); but 0xC8=0, 0xCC=0, 0xCD=4325383 (~32s) — eval never emits steps, motor silent.