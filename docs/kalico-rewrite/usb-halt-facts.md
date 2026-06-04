# USB-halt / -311 — measured facts ledger

Facts only. Each line is tagged `[commit]` with the commit at which it was
measured (bench) or read (source). No reasoning, no hypotheses — those live
elsewhere. Append new measurements; do not edit old ones.

Bench: Trident, H7 "mcu" (X/Y/E, CoreXY) + F446 "bottom" (Z) + Beacon probe.
Flash both per the flashing-trident-mcus skill. Heaters cold throughout.

## CURRENT (2026-06-02): it is a -308 PieceStartInPast fault, not a freeze

The "USB halt" is a clean kalico runtime fault (-308 PieceStartInPast) that makes
klippy shut down all MCUs and restart — the USB re-enumerate is the consequence,
NOT an MCU freeze, NOT IWDG, NOT a USB-electrical drop. Those early theories
(sections below marked SUPERSEDED) were disproven. The -308 has two run-to-run
variable faces; see "-308 CHARACTERIZED". The -311 was a real, SEPARATE, FIXED
bug (clock-domain, a6c172f89); the title is kept as history.

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

## Kernel USB view at the drop (bench, `journalctl -k` / dmesg)

- [36391d0b1] At each halt the Pi kernel logs `usb 3-2: USB disconnect, device number N` immediately followed by `usb 3-2: new full-speed USB device number N+1 ... Product: stm32h723xx`. I.e. the H7 USB device fully DISCONNECTS from the bus and RE-ENUMERATES. NOT a transfer error / babble / -EPROTO / over-current — a clean disconnect+re-enumerate.
- [36391d0b1] The F446 (`stm32f446xx`, usb 3-1) also disconnects+re-enumerates in the same episodes.
- [36391d0b1] MCU does not reboot across this (no boot_diag, motion continuous) — the USB peripheral re-attaches without an MCU reset.
- [36391d0b1] usbotg.c: VBUS sensing is DISABLED — `GOTGCTL = BVALOEN|BVALOVAL` (session forced valid) + `GCCFG |= NOVBUSSENS` (usbotg.c:513,515). So a VBUS/power dip cannot make the OTG see "unplugged".
- [36391d0b1] usbotg.c contains NO soft-disconnect (no `DCTL.SDIS` set anywhere); the only GRSTCTL writes are TX-FIFO flushes + the init AHB-idle wait. The firmware never deliberately disconnects/re-attaches USB during operation. ISR GINTMSK = RXFLVLM|IEPINT (USBRST/USBSUSP not serviced).
- [36391d0b1] => not firmware-initiated, not VBUS-sense. SUPERSEDED interpretation: originally read as an electrical drop during stepping. Actual cause (b0c3d6565): the re-enumerate is the consequence of the -308 fault -> klippy shuts down all MCUs -> systemd restart -> klippy resets the MCUs on reconnect (software/SFTRST). The 36391d0b1 "no MCU reboot" note predates the clean -308 capture. Reproduces at all jog speeds; not at idle.

## Freeze theory — SUPERSEDED (it is the -308 fault, not a freeze)

The "MCU freeze -> IWDG reset" theory was investigated and DISPROVEN at
e552d6491: the event is a clean -308 PieceStartInPast runtime fault (code +
detail captured; see the -308 sections below), not a hang. Measurements that
still stand:
- [fe1787c62] under jog, `fg_freeze stall_ticks 10 (H7) / 20 (F446)` = ~1ms
  foreground stalls (exc 0 = thread). Now understood as the ~1ms USB-burst
  fence behind Face A of the -308, not a >=0.5s freeze.
- [13b2b9a33] dispatch breadcrumb `last_disp_func` = `runtime_drain_event` (the
  1kHz drain-wake timer) on both MCUs.
Disproven and dropped: the insert_timer-spin / PRIMASK-held / IWDG-reset chain.
Evidence against it: the insert_timer fail-loud guard (c185d1483) never fired;
the isr_phase breadcrumb reads `ISR_EXIT` (the Rust motion ISR completes
cleanly); the actual fault is -308.

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

## -308 is HOST-EARLY + SYSTEMIC (transit-diag + 5 kHz bisect, cb54339ea)

- [cb54339ea] Host bridge log now persists to ~/printer_data/logs/kalico-bridge.log
  (env_logger -> file, default info). transit-diag emits per PushPieces frame.
- [cb54339ea] Jog (X 100<->200 F3000 x10), F446 @ 20 kHz: F446 (mcu=1 axis=2/Z)
  transit-diag `arrival_lead_us` = +244665 .. +16326230 (0.24 .. 16.3 s POSITIVE),
  monotonically GROWING over the jog. ZERO negative-lead / ALERT / WARN lines on
  either MCU. host_front_start_time == mcu_front_start_time (no clock-sync gap).
  => the host delivers Z pieces 0.24-16 s EARLY; the ring net-fills. -308 is NOT
  late delivery; it is the FRONT (oldest) piece being >2 ticks stale at ADOPTION.
- [cb54339ea, F446 rebuilt @ 5 kHz, working .config only; .config.f446.test
  untouched; H7 left at e552d6491] Same jog: -308 STILL fires, but on the H7
  ('mcu') axis_idx=1=Y (fault_detail 0x10000), NOT the F446. klippy then shuts
  the others down. => -308 is systemic across MCUs/axes; the first node to exceed
  its 2-tick tolerance trips. F446@5kHz tolerance=400us (2*36000cyc@180MHz) was
  enough that the H7@10kHz tolerance=200us tripped first instead.
- [cb54339ea] Magnitude bracket implied by the move: F446 stopped tripping when
  its tolerance went 100us->400us, and H7 (200us tol) trips => the adoption
  deficit is order ~100-400 us (a piece-boundary/tick-scale slip), NOT seconds.
  Cross-check: [e552d6491] H7 first-run tim5ia max was 1.03x period (metronomic,
  ~2.5us jitter) => `now` advances smoothly; the staleness is in the adopted
  piece's start_time, not a `now` jump. Direct deficit measurement still pending
  (fault_detail does not yet encode now-start_time).

## Host PRODUCE path (source read + bench, cb54339ea)

- [cb54339ea] Piece start_time is set in motion-bridge/src/enqueue.rs:160:
  `start_time = project(mcu_id, t0 + bp.u_start)`, `duration = u_end - u_start`.
  Curves are ABSOLUTE planner time: emit.rs `restrict_to_domain(axes, t_lo, t_hi)`
  sets the axis u-domain to [t_start, t_end]; state.rs shifts fitted pieces by
  `time_offset = t_dispatched`. So within a contiguous stream pieces tile exactly;
  no segment-local-domain bug.
- [cb54339ea] anchor.rs: `t0 = host_now + 0.25 - seg_t_start` on a FRESH stream
  (DEFAULT_LEAD_SECS=0.25), constant for contiguous segments, re-anchored only on
  a BACKWARD planner-time jump. A forward jog is one stream; fresh only at start.
- [cb54339ea] [seg0-deficit] (router.rs:482) at both jogs, both MCUs: seg0
  `deficit_us = +249998` (~+250 ms, POSITIVE/future = healthy). Stream START is
  fine; the -308 is a later piece.
- [cb54339ea] project = router.rs:443 `host_time_to_mcu_clock` =
  `last_clock + (host_secs - clock_offset) * clock_freq` — a LINEAR extrapolation
  of the live clock-sync estimate (updated periodically by set_clock_est_from_sample
  via spawn_periodic_clock_sync).
- [cb54339ea] Pump flow control is by piece COUNT, not time: pump.rs `room() =
  ring_depth - in_flight` (ring_depth: H7 661, F446 512); StallFull when room==0.
  So the committed time-lead is unbounded for long-duration pieces.
- [cb54339ea] Measured arrival_lead (transit-diag, commit-time front_start_time -
  arrival_clock): H7 (mcu=0, axes 0/1) +0.24 s .. +1.42 s; F446 (mcu=1, axis 2/Z)
  +0.24 s .. +16.3 s, growing. (Z is far deeper because Z-at-0 hold pieces are
  long, so 512 pieces span ~16 s.) NO negative leads at commit on either MCU.

## Host / USB architecture (source read)

- [c4ed8d740] `usbotg.c:513` `GAHBCFG = GINT` only (no DMAEN). `:512` `GINTMSK = RXFLVLM | IEPINT`. ISR (`OTG_FS_IRQHandler` ~:417) sets wake flags only; on RXFLVL it masks RXFLVLM (~:437). All FIFO movement is CPU-copy in foreground DECL_TASKs (`usb_bulk_out_task`/`usb_bulk_in_task`, `fifo_read_packet`/`fifo_write_packet`).
- [c4ed8d740] Bulk-IN TX FIFO = 16 words = 64 B (one max CDC packet). Shared RX FIFO (GRXFSIZ) = 80 words = 320 B.
- [c4ed8d740] Bulk-IN is never firmware-STALLed (`usbotg.c`: DIEPCTL_STALL set only on EP0).
- [c4ed8d740] `kalico_call_on_channel` (mod.rs:841) — PushPieces is BLOCKING request-response: `sync_channel(1)` + `rx.recv_timeout(timeout)`. Pump (pump.rs) blocks per batch on the response.
- [c4ed8d740] Reactor (`reactor.rs:1`) is a single-thread poll-reactor (reads + writes + round-trips on one thread). `READ_TIMEOUT = 1 ms`.
- [c4ed8d740] pump.rs run loop blocks on `rx.recv()`, drains, then `'send` loop; on `StallFull` it logs and `break 'send` back to `recv()` (no busy-spin). `MAX_PER_FRAME = 32` (pump.rs:488).
- [c4ed8d740] On a critical-MCU transport IO error the host reactor aborts the klippy process (SIGABRT); systemd `Restart=always` relaunches it. Per-MCU `is_critical` default true.

## -308 CHARACTERIZED: two faces, deficit-instrumented, A/B-verified

- [ff6a011e7] -308 `fault_detail` now encodes the adoption deficit (now-start_time)
  in microseconds (low 16 bits, saturated 0xFFFF = 65 ms) + axis_idx (bits 16-23).
- [b0c3d6565] Repro: `SET_KINEMATIC_POSITION` + jog X 100<->200 @ F3000 x10 (X only;
  10 forced direction-reversal stops). 10-run A/B (pre vs post comment-cleanup,
  byte-identical behavior) => -308 is RUN-TO-RUN VARIABLE, two faces:
  - **Face A (majority): F446/Z (axis 2), deficit ~0.5-3 ms, mid-motion (~3-18 s).**
    The ~1 ms USB-OTG-ISR-burst fence on the 180 MHz M4 @ 20 kHz (OTG NVIC prio 1,
    above TIM5 prio 2); when TIM5 resumes, `now` has jumped past the front piece.
  - **Face B (minority): H7/X (axis 0), deficit SATURATED >=65 ms, at motion-END /
    idle transition (~16-22 s).** Stale-at-idle / re-anchor — only visible when a
    jog runs to completion (Face A usually kills the run first).
- [cb4492e2b] `MAX_PER_FRAME` is load-bearing (NOT a -311 leftover): reverting the
  32-piece cap to 255 regressed Face-A time-to-fault ~13-17 s -> ~5 s — it bounds
  the F446 USB-burst fence.
- [a6c172f89] tim5_ia under jog = 1.0x period (metronomic) on the H7 => `now`
  advances smoothly; -308 staleness is the adopted piece's start_time vs now, not
  a `now` jump.

## Host produce/idle handling (source read) + FIX IN PROGRESS

- Piece start_time = `project(t0 + u_start)` (enqueue.rs); `project` = linear
  extrapolation of the live clock-sync estimate (router.rs `host_time_to_mcu_clock`).
- MCU `now` free-runs continuously (widened DWT); never stops/resets at idle on the
  real MCUs (the DRAINED->RUNNING reseed path is dormant — Linux-sim only).
- Host idle handling: `advance_idle(sync_instant.elapsed())` fast-forwards the
  planner clock to wall-time on a detected idle gap (planner.rs ~814). It does NOT
  fire at within-jog reversals (planner queued ahead there). `reset(home_pos)`
  zeroes the clock only on SKP / underrun / force_idle / reconnect.
- [23855b56a] FIX (partial): pump commits no piece whose projected start_time is
  > `MAX_LEAD_SECS` (1.0 s) ahead of the MCU's projected clock (`StallAhead` + poll
  resume); -308 tolerance = `MAX_START_IN_PAST_SECS` (200 us drift budget) + 1
  sample period, decoupled from the sample rate. Bench: commit-lead bounded 16 s ->
  1 s, steady-state far-ahead -308 eliminated. Did NOT fix the two faces above.
  `MAX_LEAD_SECS` / `MAX_START_IN_PAST_SECS` are NOT proven / NOT final.

## OPEN

- Face A: stop the USB-OTG burst from fencing the F446 TIM5 (~1 ms) — smaller
  per-MCU frame cap / USB below TIM5 / DMA. NOT a tolerance tweak.
- Face B: the idle/end-transition stale piece (>=65 ms) on the H7 — the anchor /
  `advance_idle` handling at a true motion-end.
