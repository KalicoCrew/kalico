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

## Host / USB architecture (source read)

- [c4ed8d740] `usbotg.c:513` `GAHBCFG = GINT` only (no DMAEN). `:512` `GINTMSK = RXFLVLM | IEPINT`. ISR (`OTG_FS_IRQHandler` ~:417) sets wake flags only; on RXFLVL it masks RXFLVLM (~:437). All FIFO movement is CPU-copy in foreground DECL_TASKs (`usb_bulk_out_task`/`usb_bulk_in_task`, `fifo_read_packet`/`fifo_write_packet`).
- [c4ed8d740] Bulk-IN TX FIFO = 16 words = 64 B (one max CDC packet). Shared RX FIFO (GRXFSIZ) = 80 words = 320 B.
- [c4ed8d740] Bulk-IN is never firmware-STALLed (`usbotg.c`: DIEPCTL_STALL set only on EP0).
- [c4ed8d740] `kalico_call_on_channel` (mod.rs:841) — PushPieces is BLOCKING request-response: `sync_channel(1)` + `rx.recv_timeout(timeout)`. Pump (pump.rs) blocks per batch on the response.
- [c4ed8d740] Reactor (`reactor.rs:1`) is a single-thread poll-reactor (reads + writes + round-trips on one thread). `READ_TIMEOUT = 1 ms`.
- [c4ed8d740] pump.rs run loop blocks on `rx.recv()`, drains, then `'send` loop; on `StallFull` it logs and `break 'send` back to `recv()` (no busy-spin). `MAX_PER_FRAME = 32` (pump.rs:488).
- [c4ed8d740] On a critical-MCU transport IO error the host reactor aborts the klippy process (SIGABRT); systemd `Restart=always` relaunches it. Per-MCU `is_critical` default true.
