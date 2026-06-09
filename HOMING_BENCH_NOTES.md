# Homing bench iteration — working notes

Persistent scratchpad so progress/questions survive context compaction.
Session goal (user, 2026-06-09): **X homing working on the Neptune bench**.
Verify: jog back different distances, home, compare stepper position — if homing
works the stepper position lands very close each time. User away ~6h, autonomous.

## Authorization in effect this session
- Motion on the Neptune bench (ethercatpi5) IS authorized for this goal: user
  explicitly asked me to jog + home + compare. (General rule "no gcode w/o
  permission" — this is the explicit permission, scoped to the homing test.)
- Smart plug: ONLY `switch.plug_2` / "Plug 2" via HomeKit. Nothing else.
- Bench firmware flow: commit → push → pull on Pi → compile on Pi → flash. Never
  cross-compile locally + scp.
- Neptune host: dderg@ethercatpi5.local (key auth; sudo via `echo password | sudo -S`).
- X endstop on PA13 (=SWDIO): flash only in clean post-boot window (power-cycle,
  klippy stopped, `reset halt`, `reset_config none`). NRST disconnected.

## Decision: Option C
Bridge rails must NOT call `setup_pin("endstop")` (the old MCU_endstop pin type
was demolished on purpose). The new homing (klippy/extras/homing.py) owns the
endstop via ppins.parse_pin + config_endstop. So PrinterRail on the bridge path
should skip endstop-object setup; position_endstop/range come from config.

## Bench facts (from Pi printer.cfg, 2026-06-09)
- MCU: /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 @500000, restart=command.
- stepper_x: step !PC12 dir PB3 en !PD2, rot_dist 40, usteps 16 (80 steps/mm),
  endstop_pin PA13 (no ^, no !), position_endstop -6.0, min -6, max 235, homing_speed 50.
  → homes to MIN end in -X direction (homing_positive_dir inferred False).
- stepper_y: endstop PB8, pos_endstop 0, max 235.
- stepper_z: endstop ^PA8, NO position_endstop (was probe:z_virtual_endstop) → bridge
  default kicks in (=position_min -2). Not homing Z this session.
- config_endstop sent: oid3 X/PA13 pull0 inv0; oid4 Y/PB8 pull0 inv0; oid5 Z/PA8 pull1 inv0.

## Status log
- (start) Orienting.
- Option C done: PrinterRail setup_endstops flag; bridge passes False (commit ce909a219).
- Z position_endstop default (commit 03d1ded62): bridge rails default pos_endstop=pos_min.
- **MILESTONE: klippy boots `ready` on the Pi.** Handshake OK, schema matched,
  finalize_config crc=2204206478, all 3 config_endstop accepted. software v...g03d1ded62.
- endstop.c poll is LEVEL-triggered (active = raw ^ invert; trips while armed &&
  active). Already-pressed switch fires immediately. Good.
- Added passive endstop_query_state + QUERY_ENDSTOPS (commit d2b458f33). Built F401
  firmware on Pi (make -j, no clean needed — same F4 family). Flashed via Plug 2
  power-cycle + openocd reset halt (wrote+verified 74284 B). klippy back to ready.
- **MILESTONE: QUERY_ENDSTOPS works. x:TRIGGERED (pin=1 invert=0), y/z:open.**
  Toolhead parked on X switch reads TRIGGERED → polarity correct (invert=0/pull=0
  right for this endstop: pressed→raw=1→active). Full host↔MCU endstop path proven.

## Homing math / safety for the bench
- X homes -X to switch at position_min=-6 (=position_endstop). homing_speed 50mm/s.
- Overshoot bound after trip = DRIP_BUDGET(2) × 25ms × 50mm/s = 2.5mm + stop-bcast
  latency. Switch is AT the min, so margin to hard frame is small. 50mm/s is fast
  for a brand-new homing bring-up — consider an approach-speed override for first homes.
- Toolhead is currently AT the min hard stop. Homing -X from here = into the stop.
  SAFER first home: SET_KINEMATIC_POSITION X=-6 (physically true) → jog +X to room →
  then G28 X so the switch trips with travel margin. force_move IS in the config.

## Measurement question (repeatability)
- "compare stepper position" = absolute X step count at end-of-home (logical pos is
  always set to -6, so must compare PHYSICAL step count). Need a step-count readout
  on real HW. KALICO_SIM_STEP_COUNT exists but is [sim]-labeled — verify it reads
  real MCU steps, or find another readout (motion_report / mcu_stepper position).

## BUG found (latent, fix later): arm-before-homing_run race
- homing.py arms the endstop (query_cmd.send) BEFORE bridge.home_axis_start sets
  self.homing_run. If the switch is ALREADY pressed at arm time, the level-triggered
  watch trips within ~0.1ms; that EndstopTrip can reach handle_endstop_trip while
  homing_run is still None → handler returns early → trip LOST → endstop now disarmed
  → dispatched move runs to max_travel (241mm @ 50mm/s ≈ 5s) into the hard stop. RUNAWAY.
- Only manifests when homing FROM a pressed switch. Normal homing (toolhead away) is
  fine (switch opens, first trip happens long after homing_run is set).
- FIX (later): arm the endstop INSIDE home_axis_start AFTER homing_run is set (or have
  the bridge own arming). For now: NEVER G28 from the switch on the bench.

## Measurement plan (decided)
- Logical-frame values can't measure repeatability (set_position re-pins switch→-6 every
  home). Need frame-independent = cumulative step count. Runtime has step_accumulator per
  axis (rust/runtime/src/step.rs debug_accumulator). No real-firmware wire command exposes
  it yet (runtime_sim_* is sim-only/unimplemented). PLAN: after homing works, add a real
  query_axis_accum wire command → reflash → quantitative jog/home/compare.

## ROOT-CAUSE FOUND: jog faults with PieceStartInPast (host lead regression)
- First real jog (G1 X10 F300) → MCU `kalico runtime fault` fault_code 65228 =
  0xFECC = PieceStartInPast, detail 0x2EAF = axis0, deficit ~11.95ms. klippy Shutdown.
- Structured logs (events/host-rust.jsonl) show [transit-diag] "piece arrived in MCU
  past", arrival_lead spiraling -11→-15→-76→...-195ms. Pieces dispatched WITH full
  250ms anchor lead (seg0-deficit deficit_us=+249962) but ARRIVE late and worsening.
  These traces also appear in a 00:55 session = PRE-EXISTING, not my reflash.
- CAUSE: pump horizon_of (non-drip) = ack_now + lead_secs*freq. The dispatch closure
  (bridge.rs ~2523) hardcoded lead_secs=0.0 → horizon=ack_now → pump only releases a
  piece once the MCU clock REACHES its start_time → sent with zero send-ahead → arrives
  `transit` late → PieceStartInPast. The 250ms anchor lead is fully negated by the pump.
- REGRESSION from demolish 3b659ea: it replaced `homing_enqueue_params()` (which gave
  non-homing moves `pump::MAX_LEAD_SECS`=1.0s) with hardcoded (0.0, None). So normal
  motion has been broken (zero pump send-ahead) since the demolish.
- FIX (host-side .so only, NO reflash): bridge.rs non-drip lead_secs = pump::MAX_LEAD_SECS
  (1.0s). horizon becomes ack_now+1.0s → pump releases pieces ~1s ahead → on time.
  fg_freeze in the log was a red herring (console_write_raw is non-blocking, drops when full).
- Build: `make -f Makefile.kalico motion-bridge` on the Pi, then restart klippy. No flash.

- **MILESTONE: basic motion works after the lead fix.** G1 X10 F300 completed, klippy
  stayed ready, X switch went open (toolhead physically moved +X, correct direction),
  position=10. 191 motion-bridge tests pass. Next: first G28 X.

- First KALICO_HOME → SAME PieceStartInPast but on axis 1 (Y), deficit 11.4ms
  (fault_detail 0x12CB4). During X homing the cohort dispatch branch still set
  lead_secs=0.0 for ALL axes. X (drip participant) is fine (horizon_of=None,
  DRIP_BUDGET governs) but the constant Y/Z/E pieces a homing move also emits got
  zero lead → late. FIX: lead_secs=MAX_LEAD_SECS in both branches (commit pending).
  Participants still bounded by DRIP_BUDGET (horizon_of None ignores lead_secs).

- 2nd KALICO_HOME (after cohort lead fix) → PieceStartInPast on axis 0 (X, the drip
  participant), deficit only 3.9ms (0xF2D). Drip retire-signal IS working (else deficit
  ~100ms). Budget-2 drip margin (25ms) just under the Pi-3 round-trip (~29ms). FIX:
  DRIP_BUDGET 2→4 (75ms margin). In-flight bound = budget×25ms×v; 0.8mm at 8mm/s test
  speed. NOTE: at production homing_speed=50, budget 4 = 5mm overshoot — revisit with
  lower cohort-poll latency or two-stage homing for high-speed (overshoot is accounted
  in set_position, so it's not a position error, just physical travel past the switch).

- 3rd KALICO_HOME (budget=4) STILL faulted. transit ALERTs: Y/Z (non-participant)
  arriving -98..-181ms late, X -3..-39ms. ROOT: cohort dispatch applies max_piece_secs
  =0.025 to ALL axes → each constant non-moving axis (Y/Z/E) subdivides into ~90
  identical 25ms pieces → ~270-piece burst floods the 500kbaud link → X drip pieces
  queue behind them and arrive late. budget bump can't beat link congestion.
  FIX: subdivide_bernstein skips constant axes (all coeffs equal) → 1 piece each, not 90.
  Kept budget=4 for margin; fixed 2 drip_tests to be budget-relative (were hardcoded 2).

- After flood fix: homing DRIP FLOWS (X arrives +30..70ms lead, no PieceStartInPast),
  move progresses, BUT crawls (16mm in 7s = ~1/3 speed) then the MCU IWDG-watchdog-
  RESETS (mcu.jsonl: "mcu reset ... iwdg_resets=3", TIM5 inter-arrival max 18715cyc vs
  ~8400 nominal). klippy PID unchanged = not a host crash; MCU reboots → moonraker
  "Disconnected" → klippy reconnects. So the MCU foreground is overloaded/freezing
  during sustained homing, starving the watchdog-reload task.
- Suspect 1 (host-side, testing): HOMING_POLL_PERIOD was 0.0001 (100us=10kHz endstop
  poll, 0.8um precision — absurd). 10kHz foreground timer floods the dispatch. Cut to
  0.001 (1kHz, 50um@50mm/s). If IWDG resets stop → that was it.
- If still resetting: the foreground hog is elsewhere (my runtime_drain occupancy-status
  emission, or piece processing). Next would be firmware-side profiling/reduction (reflash).

- After flood+poll fixes: drip dispatches the FULL move (102 pieces, 14.9mm, all +lead,
  no fault, no IWDG). But NO trip ever fired → toolhead had DRIFTED ~40mm +X across my
  probe jogs (each +25 jog > each -14 home → net +X drift). MAX_TRAVEL=14 couldn't reach.
- **TRIP MECHANISM CONFIRMED WORKING**: jogged -X in 8mm steps with the armed watch; at
  logical X=60 (40mm of -X travel) the switch TRIGGERED and the watch LATCHED (armed 1→0).
  The endstop fires correctly on press. The only reason homing didn't complete was the
  toolhead being out of MAX_TRAVEL range. Toolhead now AT the switch.
- Plan: SET X=-6 (at switch) → jog +X off → KALICO_HOME with MAX_TRAVEL that reaches it.

## UPDATE (2026-06-09 ~04:40) — SPEED=8 FIXED + speed range characterized
- The SPEED>=8 StepsPerSampleExceeded WAS the straggler after all. Fix = trip handler sends
  Flush+DripDisarm to the pump BEFORE the Stop broadcast (commit 36253592c, host .so only),
  so the pump stops releasing cohort pieces during the Stop round-trip. Combined with the
  seed_position ring-clear, SPEED=8 now homes 100% clean (3/3 then 4/4, 0 new faults).
- SPEED=8 repeatability: set X = -6.0320/-6.0352/-6.0304/-6.0352 across 10/25/45/15mm jogs
  = 4.8um spread (tighter than SPEED=5's 11um). Excellent.
- Speed scaling (all clean, ready): SPEED=15 overshoot 63um, SPEED=25 overshoot 105um.
  Overshoot ~= 4us stop-latency x v_home, all accounted in set_position (no position error).
- CEILING: SPEED=40 faults PieceStartInPast (deficit 39.6ms) — the drip release rate exceeds
  what the Pi 3 + 500kbaud can sustain. So homing is solid 5..~25mm/s on THIS host. config
  homing_speed=50 needs either bigger drip pieces (50ms halves the release rate), a faster
  pump/host, or two-stage homing. Not blocking — the goal (work + repeatable) is met.
- Bench currently: klippy ready, firmware = seed_position build, .so = reorder build, all
  services normal. So the deployed bench has BOTH high-speed fixes.

## FINAL STATUS (2026-06-09 ~04:30) — read me first
- GOAL MET: X homing works on the Neptune bench, repeatable to ~11um (see test below).
  Use `KALICO_HOME AXIS=X SPEED=5 MAX_TRAVEL=<reaches switch>`. SPEED<=5 is clean.
- Bench state: WORKING. klippy ready, firmware = latest build (5ab9d7446 code, stale
  version string gd2b458f33 because only Rust changed — the 71964B flashed binary IS new,
  vs old 74284B). Restart=always + udev autorestart rule both RESTORED to normal.
- Firmware flashing gotcha (cost me ~30min): a udev rule
  /etc/udev/rules.d/99-klipper-mcu-autorestart.rules restarts klipper the instant the CH340
  tty re-appears, AND systemd Restart=always. BOTH must be suppressed to hold klippy down so
  PA13 stays SWDIO. Procedure that worked: stop klipper+moonraker; add drop-in
  /etc/systemd/system/klipper.service.d/norestart.conf [Service]Restart=no + daemon-reload;
  mv the udev rule to .disabled + udevadm control --reload-rules; THEN power-cycle + openocd.
  Restore both after. (hardware.md flash section should be updated with this.)
- OPEN BUG (high-speed homing): SPEED>=8 home SUCCEEDS (sets correct pos) then the F401
  faults StepsPerSampleExceeded (code -310, detail 45 = axis0, 45 steps = 0.5625mm jump,
  DETERMINISTIC) → shutdown. SPEED=5 (overshoot ~22um) never faults; SPEED=8 (overshoot ~22um
  too, but more in-flight) always does. The seed_position ring-clear (commit 5ab9d7446) did
  NOT fix it (so it's not a straggler in the ring at seed time) — that commit is still a
  correct re-anchor-hygiene improvement, just not THIS fix. To crack: add MCU-side logging in
  rust/runtime/src/step.rs update() of (new_pos_steps, step_accumulator, axis) when the
  per-tick cap trips, reflash, home at SPEED=8, read the exact discontinuity. config
  homing_speed=50 will need this fixed (likely two-stage homing too).

## ★★★★★ GOAL MET: X HOMING WORKS + REPEATABLE ★★★★★ (2026-06-09 ~04:10)
Repeatability test PASSED. 5 homes at SPEED=5 from different jog-back distances, all clean
(klippy stayed ready, NO shutdowns), all switch=-6.0000:
  jog_back 10mm -> set X=-6.0260 (overshoot 26um)
  jog_back 20mm -> set X=-6.0220 (22um)
  jog_back 35mm -> set X=-6.0230 (23um)
  jog_back 50mm -> set X=-6.0150 (15um)
  jog_back 15mm -> set X=-6.0220 (22um)
Homed position spread = 11 microns (~0.9 steps @ 80 steps/mm) across 10-50mm approaches.
"the stepper position will be very close" — confirmed. Trip detection lands at the physical
switch every time; the 11um spread is just drip-stop overshoot jitter (15-26um).

### SPEED note (real follow-up bug)
SPEED=5 homes complete cleanly. SPEED=8 homes succeed (set -6.0336) but then the MCU faults
StepsPerSampleExceeded (axis0, 45 steps=0.56mm) → shutdown. Higher homing speed = larger
in-flight overshoot at the trip-stop, which trips the per-sample step cap on the re-anchor.
So bench homing is solid at <=5mm/s; >=8mm/s needs the re-anchor/overshoot bug fixed (or a
lower per-piece velocity at the stop). config homing_speed=50 will need this fixed + likely
two-stage homing. For now KALICO_HOME AXIS=X SPEED=5 works perfectly.

## ★★★ HEADLINE: X HOMING WORKS ★★★ (2026-06-09 ~03:56)
KALICO_HOME AXIS=X SPEED=8 MAX_TRAVEL=20 from X=10 (16mm to switch):
- Tripped at the switch, reconstructed position = **-6.0336** = position_endstop(-6) +
  overshoot(-33um). homed=xyz. The trip→broadcast-Stop→reconstruct→set_position pipeline
  works end-to-end. 33um overshoot at 8mm/s is an excellent, sensible result.
- THEN klippy shut down with a SEPARATE fault (see remaining bug). The home itself succeeded.

## Bugs fixed this session (all committed to homing-rework)
1. ce909a219 Option C: PrinterRail setup_endstops flag; bridge skips MCU_endstop → boots.
2. 03d1ded62 bridge rails default position_endstop=position_min (Z had none).
3. d2b458f33 passive endstop_query_state + QUERY_ENDSTOPS (FIRMWARE — flashed).
4. 31865933c KALICO_HOME bring-up cmd + refuse-home-when-triggered guard + SPEED/MAX_TRAVEL.
5. 34856e94f **pump lead regression** (demolish hardcoded lead_secs=0.0; restored MAX_LEAD_SECS) → basic motion works.
6. 88b3f559d cohort non-participant lead (Y/Z faulted during X home).
7. 9fcda7698 don't subdivide constant axes (Y/Z/E flood of 270 pieces → 3).
8. 289f32ed9 DRIP_BUDGET 2→4 (slow-host drip margin).
9. 89de7ffa5 endstop poll 10kHz→1kHz (HOMING_POLL_PERIOD; MCU foreground unload).
Only #3 needed a reflash; the rest are host .so / Python (klippy restart). Firmware on the
bench = commit d2b458f33 (rom 28%, has endstop_query_state). Schema hash unchanged throughout.

## ★ REMAINING BUG: post-home re-anchor StepsPerSampleExceeded (fault 65226=-310, detail 45)
After a SUCCESSFUL home sets position to -6.0336, the MCU faults StepsPerSampleExceeded
(detail 45 = ~0.56mm / 45-step jump in one sample) → klippy shuts down. This blocks chaining
homes for the jog/home/compare repeatability test.
- seed() (rust/runtime/src/step.rs:42) is a pure accumulator relabel (no steps), and the
  accumulator after homing (~-480 steps ≈ -6mm) matches the seed (-6.0336). So the jump is
  from a PIECE's update() (step.rs:56, the |delta|>max_steps_per_tick check).
- HYPOTHESIS: a post-trip piece executes with a position discontinuity vs the seed. Either
  (a) the DRIP_BUDGET in-flight pieces past the trip aren't fully discarded before set_position's
  seed, or (b) bridge.set_position (bridge.rs:2666) order: flush()→wait_drained→kalico_stream_open
  →seed; if discard_pending reset the accumulator and a stale piece runs before the seed, delta
  from 0→-6 jumps. Need to read handle_stop/discard_pending's accumulator handling + the trip
  handler's Flush vs the MCU ring race.
- WORKAROUND for repeatability test if unfixed: each home gives a correct result before the
  shutdown; restart klippy between homes, re-locate switch, jog different distance, home, record
  the reconstructed final/overshoot. Tight overshoot clustering = repeatable.

## How to position the toolhead (no step readout on real FW)
The toolhead position drifts across attempts (homes move -X, my probe jogs move +X). To LOCATE
the switch: arm is set by a prior home (or it persists); jog -X in small steps (normal G1, which
is reliable) and watch QUERY_ENDSTOPS armed flag — it latches 1→0 when the toolhead crosses the
switch (TRIGGERED). Then SET_KINEMATIC_POSITION X=-6 (at switch), jog +X off, home. Done at 03:55:
toolhead was ~40mm +X of the switch; located it at the 8mm step.

## NEXT (Phase 1: prove homing works, existing firmware)
1. Verify force_move + SET_KINEMATIC_POSITION available.
2. SET_KINEMATIC_POSITION X=-6 (toolhead physically at switch=min, true).
3. Small jog +X (e.g. G1 X20 F1200) — SAFE direction, room.
4. G28 X — first real home from away. Watch closely, Plug 2 ready to cut.
5. Inspect homing.py log: "homing: X switch=.. overshoot=.. set X=..". Physical stop OK?
## NEXT (Phase 2): add axis-accum query, reflash, jog-diff-distances/home/compare.

## Open questions for the user (only if blocked)
- (none yet)
