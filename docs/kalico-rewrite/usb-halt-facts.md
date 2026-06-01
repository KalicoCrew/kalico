# USB-halt / -311 — measured facts ledger

Facts only. Each line is tagged `[commit]` with the commit at which it was
measured (bench) or read (source). No reasoning, no hypotheses — those live
elsewhere. Append new measurements; do not edit old ones.

Bench: Trident, H7 "mcu" (X/Y/E, CoreXY) + F446 "bottom" (Z) + Beacon probe.
Flash both per the flashing-trident-mcus skill. Heaters cold throughout.

## Config (Pi: ~/klipper/.config.h7.bak, .config.f446.test)

- [c4ed8d740] H7: `CONFIG_CLOCK_FREQ=520000000`, `CONFIG_KALICO_MOTION_SAMPLE_RATE_HZ=10000`.
- [c4ed8d740] F446: `CONFIG_CLOCK_FREQ=180000000`, `CONFIG_KALICO_MOTION_SAMPLE_RATE_HZ=20000`.

## Clock tree (source read)

- [c4ed8d740] `stm32h7.c:103` H723 PLL → SYSCLK = CONFIG_CLOCK_FREQ. `:164` D1CFGR = HPRE_DIV2. `:165` D2CFGR = D2PPRE1_DIV2 | D2PPRE2_DIV2. No `D1CPRE` write anywhere.
- [c4ed8d740] `stm32h7.c:21` `get_pclock_frequency` returns `CONFIG_CLOCK_FREQ/4` (= 130 MHz).
- [c4ed8d740] `stm32f4.c:21` `FREQ_PERIPH_DIV = 4` for F446. `:207` CFGR = PPRE1_DIV4 | PPRE2_DIV4. `get_pclock_frequency` returns `CONFIG_CLOCK_FREQ/4` (= 45 MHz).
- [c4ed8d740] TIM5 (h7/f4), TIM3 (h7), TIM2 (f4) all PSC=0.
- [c4ed8d740] `hard_pwm.c:323-326` applies `pclock_div = CONFIG_CLOCK_FREQ/get_pclock_frequency(); if (>1) /=2; // Timers run at twice the normal pclock frequency`.
- [c4ed8d740] `runtime_tick.c:52` `runtime_clock_freq = CONFIG_CLOCK_FREQ`.

## TIM5 inter-arrival (bench, `prior_diag_summary_tim5ia`, DWT cycles)

- [5f4a713cb] ARR from `runtime_clock_freq`. Idle: H7 min/last/max = 102495 / 103996 / 104419, period 52000. F446 = 16995 / 18000 / 18368, period 9000. (= 2.0× period.)
- [a6c172f89] ARR from `motion_timer_clk()` (= CONFIG_CLOCK_FREQ/2). Idle: H7 = 50457 / 52006 / 52288, period 52000. F446 = 7473 / 8966 / 10527, period 9000. (= 1.0× period.)
- [5832b0e9e] Under jog (X 100↔200 @50 mm/s): H7 = 50430 / 52005 / 52683, period 52000. (= 1.0×.)

## -311 TickIntervalExceeded (bench)

- [9c741927d] (priority lift MOTION_NVIC_PRIO=0, pre-clock-fix): jog → -311 fires, `detail=2`, both MCUs.
- [a6c172f89] (clock fix): no -311 observed, idle or under jog.
- [5832b0e9e] (clock fix + co-equal prio + step yield): no -311 under jog (tim5_ia 1.0×).

## USB-CDC halt under sustained motion (bench)

- [5832b0e9e] Jog (X 100↔200 @50 mm/s, ~8 moves) → host `kalico_host_rt::host_io::reactor` logs `port read error: Io(BrokenPipe "Broken pipe"); transitioning to Closed` → `[reactor-spawn] EXIT_ON_FAULT — transport closed via IO error on CRITICAL MCU; aborting klippy` → systemd `klipper.service: Main process exited, code=killed, status=6/ABRT` → `Scheduled restart`. klippy auto-recovers to `ready`. EXIT_ON_FAULT/ABRT count rose 0→3 over one jog.
- [5832b0e9e] MCU did NOT reboot at the halt: no fresh `boot_diag emit` after the abort; `reset_epoch=0xa5a5a5a5` (identical on every attach); `boot_diag ... rcc 335544322` = 0x14000002 = SFTRSTF|PINRSTF (IWDGRSTF bit 0x20000000 NOT set).
- [5832b0e9e] MCU healthy at halt: tim5_ia 1.0× period (above); `tim5_total/tim5_n = 931404835/2112975 = 441 cyc/tick`; at 10 kHz that is ~0.85% of 520 MHz.
- [5832b0e9e] Host piece-flow at halt (`PIECEDIAG`/`transit-diag`): `arrival_lead_us ≈ 1.30e6`; `ring_depth 661`; `room=0`; `PIECEDIAG STALL` ~126 lines/sec; `PIECEDIAG SEND` batch `count` 7 / 19 / 32.
- [5832b0e9e] MCU `prior_diag_summary_block` at jog: `systick 1331 stepout 782 stepout_burst 23566 usb_burst 129017` (DWT cycles). `otg_max_cyc 1151` (idle dumps).
- [5832b0e9e] MCU `prior_diag_tasks` at jog: `out_max_gap 3949284761 in_max_gap 2649850198 drain_max_gap 534756 stat_max_gap 52037838` (timer ticks; 20 ms threshold = ~10.4e6 ticks @520 MHz). (Note: these gaps may include the boot/connect transient — not isolated to the jog.)

## USB-core state at the halt (bench, `prior_diag_summary_usb`)

- [63537cdb9] Jog (X 100↔200 @50 mm/s) → same halt (BrokenPipe → SIGABRT, EXIT_ON_FAULT/ABRT count 0→3). klippy auto-recovers.
- [63537cdb9] H7: `in_busy 40`; F446: `in_busy 75`. (usb_send_bulk_in returned -1 / EPENA-still-set this many times: bulk-IN endpoint backed up — host not draining IN.)
- [63537cdb9] Both MCUs: `gintsts = gintsts_sticky = 0x548C4C38`. Set bits include SOF(0x8), RXFLVL(0x10), NPTXFE(0x20), ESUSP(0x400), USBSUSP(0x800), USBRST(0x1000), ENUMDNE(0x2000), EOPF(0x8000), IEPINT(0x40000), + high bits 0x04000000/0x10000000(SRQINT)/0x40000000. NOTE: GINTSTS USBRST/USBSUSP are W1C and this firmware never clears them (masked: GINTMSK=0x40000=IEPINT only at dump), so their set state does NOT distinguish boot enumeration from a halt-time reset — inconclusive by construction.
- [63537cdb9] At dump time (post-halt): H7 `in_diepctl 0x498100`, F446 `0x488100` — EPENA(0x80000000) CLEAR, USBAEP(0x8000) set. `in_diepint 0x2093` (XFRC|EPDISD|ITTXFE|...). `in_dtxfsts 16` (IN TX FIFO fully free = empty). `out_doepctl 0xB8000` — NAKSTS(0x20000) set, EPENA clear. `out_doepint 0x11`. (i.e. endpoints idle/quiescent at dump — not a stuck-armed endpoint; but this is post-halt + post-reconnect, not the halt instant.)
- [63537cdb9] tim5_ia under jog: H7 min/last/max 50399/51996/52490 period 52000 (1.0×); F446 7970/9034/9440 period 9000 (1.0×).
- [63537cdb9] `prior_diag_summary_block` under jog: H7 `systick 587 stepout 0 stepout_burst 0 usb_burst 129508`; F446 `systick 681 stepout 629 stepout_burst 629 usb_burst 146896`. (H7 stepout=0 this run — H7 step-output timer recorded no fires during an X-only CoreXY jog; unexplained, noted as fact.)

## Host-side reactor timing at the drop (bench, `[usb-drop]` log)

- [36391d0b1] Host `.so` rebuilt with reactor timing instrumentation (no MCU flash; MCU stays 63537cdb9). Jog still halts identically.
- [36391d0b1] `[usb-drop]` samples (silence_ms = since last inbound bytes; since_write_ms = since last successful port write; consec_zero = consecutive Ok(0) reads before the error):
  - F3000: silence 37, since_write 325, consec_zero 0
  - F3000: silence 28, since_write 417, consec_zero 0
  - F1200: silence 108, since_write 271, consec_zero 0
  - F6000: silence 19, since_write 30, consec_zero 0
  - All: `err=Io(BrokenPipe "Broken pipe")`.
- [36391d0b1] consec_zero = 0 in ALL samples (abrupt error, no Ok(0) preamble). silence_ms small (19–108) in all (MCU sending bytes shortly before the drop). since_write_ms varies 30–417 (tracks write cadence/speed, NOT a fixed timeout). Halt reproduces at F1200, F3000, F6000 (all speeds tried).
- [36391d0b1] Host kinematic config (klippy.log `configure_axes`): `step_modes=[1,1,1,1] any_phase_stepping=False phase_motor_count=0` — H7 X/Y use step pulses, NOT phase-stepping.

## PushPieces round-trip / non-blocking (source read)

- [9c016776d] `PushPiecesResponse` carries `result:i32` (0=OK, neg=MCU reject), `arrival_clock:u64`, `front_start_time:u64` (messages.rs ~247-256). Pump (`pump.rs` send_frame ~583-596) gates `pushed`/cursor advance on `result==OK`; arrival_clock/front_start_time are diagnostic-only (`[transit-diag]`); `retired` comes from the StatusHeartbeat (pump.rs ~505-523), NOT the response; `new_head` computed host-side.
- [9c016776d] Wire-level ACK/NAK + UnackedWindow already handle frame delivery / retransmit independently of the app-level response. The only load-bearing role of `PushPiecesResponse` is surfacing an MCU commit-rejection (result != OK). Fire-and-forget PushPieces is feasible host-side but would make a MCU commit-rejection invisible (silent ring divergence) unless the MCU emits an out-of-band FaultEvent on rejection (protocol change).

## Kernel USB view at the drop (bench, `journalctl -k` / dmesg) — DECISIVE

- [36391d0b1] At each halt the Pi kernel logs `usb 3-2: USB disconnect, device number N` immediately followed by `usb 3-2: new full-speed USB device number N+1 ... Product: stm32h723xx`. I.e. the H7 USB device fully DISCONNECTS from the bus and RE-ENUMERATES. NOT a transfer error / babble / -EPROTO / over-current — a clean disconnect+re-enumerate.
- [36391d0b1] The F446 (`stm32f446xx`, usb 3-1) also disconnects+re-enumerates in the same episodes.
- [36391d0b1] MCU does not reboot across this (no boot_diag, motion continuous) — the USB peripheral re-attaches without an MCU reset.
- [36391d0b1] usbotg.c: VBUS sensing is DISABLED — `GOTGCTL = BVALOEN|BVALOVAL` (session forced valid) + `GCCFG |= NOVBUSSENS` (usbotg.c:513,515). So a VBUS/power dip cannot make the OTG see "unplugged".
- [36391d0b1] usbotg.c contains NO soft-disconnect (no `DCTL.SDIS` set anywhere); the only GRSTCTL writes are TX-FIFO flushes + the init AHB-idle wait. The firmware never deliberately disconnects/re-attaches USB during operation. ISR GINTMSK = RXFLVLM|IEPINT (USBRST/USBSUSP not serviced).
- [36391d0b1] => the disconnect is not firmware-initiated and not VBUS-sense: the device drops off the bus electrically (signal/clock/power) during motor stepping, then re-enumerates. Reproduces at all jog speeds (F1200/F3000/F6000); does not occur at idle.

## ROOT CAUSE: MCU freeze -> IWDG reset -> USB re-enumerate (NOT hardware)

User: bench prints fine at full amps over USB on main => not hardware; the MCU
must be freezing. Confirmed:
- [f2f3f94ab] `fg_freeze ... iwdg 1` after one jog: an IWDG (independent
  hardware watchdog) reset fired. The MCU froze >= the ~0.5s IWDG window ->
  reset -> USB re-enumerate (the kernel "USB disconnect + new device ~1s later"
  IS the reset+reboot+re-enum). No prior_fault => a hang, not a crash. No
  prior_diag_ring tag 8 (DIAG_EV_RUST_FAULT) => no runtime fault, shutdown() not
  reached.
- [fe1787c62] With the freeze-detector gate fixed (it had been gated on
  engine_status==RUNNING, which is never set — runtime_tick.c:421): under jog,
  `fg_freeze stall_ticks 10 (H7) / 20 (F446)` = only ~1ms foreground stalls
  caught (exc 0 = thread). The >=0.5s killer freeze is NOT caught => during it
  the TIM5 ISR itself is masked => interrupts OFF (PRIMASK held) for the freeze.
- [13b2b9a33 / f2f3f94ab / fe1787c62] dispatch breadcrumb `last_disp_func`
  resolves (addr2line) to `runtime_drain_event` (src/runtime_tick.c:261) on BOTH
  MCUs — the 1 kHz drain-wake timer. It is the last timer dispatched before
  interrupts were masked.
- Source inference: `runtime_drain_event` returns quickly (no hang). After it,
  `sched_timer_dispatch` reschedules it via `insert_timer` (src/sched.c:113),
  whose `for(;;)` list-walk (pos = pos->next until a later waketime) never
  breaks if the timer list is corrupted/cyclic -> spins forever under the
  SysTick `irq_disable` (armcm_timer.c:278 + timer_dispatch_many) -> PRIMASK
  held -> TIM5 masked -> IWDG. This spin runs BEFORE timer_dispatch_many's
  "Rescheduled timer in the past" guard (armcm_timer.c:256), so that guard never
  fires (consistent with no rsched_past output). => the scheduler timer list is
  being corrupted during motion; insert_timer spinning on it is the freeze.
- Open: WHAT corrupts a timer `.next` during motion (bad/out-of-bounds write).
  insert_timer should also fail loudly (cycle/iteration cap) instead of freezing.

## REVISED ROOT CAUSE: F446 raises -308 PieceStartInPast on Z (NOT a freeze)

Measured at e552d6491 (ISR-phase breadcrumb + walk/monomial split + both MCUs
flashed). Repro: `SET_KINEMATIC_POSITION X=150 Y=150 Z=10` then jog `G1 X100/X200
F3000` x10 (X only, 50 mm/s). Failure reproduced first attempt.

- [e552d6491] klippy.log (rotated `klippy.log.2026-06-01_17-16-17`, the jog run):
  `mcu 'bottom': got {'type':'fault','fault_code':65228,'fault_detail':131072,
  'segment_id':0,'synthesized':False,'#name':'kalico_fault'}` then `MCU 'bottom'
  shutdown: kalico runtime fault`. 65228 = -308 as u16 = `PieceStartInPast`
  (error.rs:124, raised in engine.rs:736 inside `get_piece_for_time`). fault_detail
  131072 = 0x20000 = (axis_idx 2 << 16) per fault_helpers.rs:128 => axis_idx=2=Z.
- [e552d6491] After 'bottom' faulted, klippy commanded the others down:
  `MCU 'mcu' shutdown: Command request`, `MCU 'beacon' shutdown: Emergency stop`.
  The H7 did NOT fault itself; it was shut down by klippy.
- [e552d6491] H7 ('mcu') post-reset prior_diag (its reset = klippy's shutdown ->
  reconnect software reset): `prior_diag_phase walk_max 311 walk_n 10670 mono_max
  358 mono_n 10670 isr_phase 9` (9 = RT_PHASE_ISR_EXIT = ISR completed cleanly;
  freeze NOT in the Rust motion ISR). `prior_diag_summary boot 33718 tim5_n
  1960459 tim5_max_cyc 3131`; `prior_diag_hist_irq b0 1960459` (ALL 1.96M TIM5
  ISRs in bucket 0, <4096 cyc; zero slow ticks). `tim5ia min 50269 max 53354 last
  52006 period 52000` (metronomic, max 1.03x). `fg_freeze stall_ticks 10 exc 0
  iwdg 0` (iwdg counter 0 => NOT an IWDG reset). `boot_diag rcc 0x01420000` (SFTRSTF
  bit24 set; IWDG bit not set).
- [e552d6491] F446 ('bottom') emitted NO `#output` prior_diag lines in any log
  slice (0 of 80 'bottom' lines). The F446 diag burst was not captured (report
  task absent on F4, or `.persistent_diag` did not survive its reset, or scrolled).
  => no F446 isr_phase / walk / tim5ia captured this run.
- [e552d6491] Kernel (`dmesg -T`): F446 (`stm32f446xx`, usb 3-1) USB disconnect
  17:16:17 -> re-enumerate 17:16:18. H7 (`stm32h723xx`, usb 3-2) disconnect
  17:16:29 -> re-enumerate 17:16:30. **F446 dropped first, ~12 s before the H7.**
  The re-enumerations follow the kalico fault + klippy restart (consequence, not
  cause). I jogged X (H7 axis); the Z MCU (F446) faulted.
- [e552d6491] F446 config: SAMPLE_RATE_HZ=20000 (period 9000 cyc @ 180 MHz; -308
  tolerance = 2 ticks = 18000 cyc = 100 us). H7: 10000 Hz (200 us tolerance).
- [e552d6491] fault_detail for -308 carries ONLY axis_idx (bits 16..24); the
  actual deficit (now - start_time) is NOT encoded => magnitude of lateness unknown.

## Host / USB architecture (source read)

- [c4ed8d740] `usbotg.c:513` `GAHBCFG = GINT` only (no DMAEN). `:512` `GINTMSK = RXFLVLM | IEPINT`. ISR (`OTG_FS_IRQHandler` ~:417) sets wake flags only; on RXFLVL it masks RXFLVLM (~:437). All FIFO movement is CPU-copy in foreground DECL_TASKs (`usb_bulk_out_task`/`usb_bulk_in_task`, `fifo_read_packet`/`fifo_write_packet`).
- [c4ed8d740] Bulk-IN TX FIFO = 16 words = 64 B (one max CDC packet). Shared RX FIFO (GRXFSIZ) = 80 words = 320 B.
- [c4ed8d740] Bulk-IN is never firmware-STALLed (`usbotg.c`: DIEPCTL_STALL set only on EP0).
- [c4ed8d740] `kalico_call_on_channel` (mod.rs:841) — PushPieces is BLOCKING request-response: `sync_channel(1)` + `rx.recv_timeout(timeout)`. Pump (pump.rs) blocks per batch on the response.
- [c4ed8d740] Reactor (`reactor.rs:1`) is a single-thread poll-reactor (reads + writes + round-trips on one thread). `READ_TIMEOUT = 1 ms`.
- [c4ed8d740] pump.rs run loop blocks on `rx.recv()`, drains, then `'send` loop; on `StallFull` it logs and `break 'send` back to `recv()` (no busy-spin). `MAX_PER_FRAME = 32` (pump.rs:488).
- [c4ed8d740] On a critical-MCU transport IO error the host reactor aborts the klippy process (SIGABRT); systemd `Restart=always` relaunches it. Per-MCU `is_critical` default true.
