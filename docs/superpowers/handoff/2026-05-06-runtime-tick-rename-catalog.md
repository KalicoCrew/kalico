# Runtime-tick rename catalog (Task 1)

## C symbols defined or extern-declared (src/)
```
src/generic/armcm_timer.c:45:// 0. Fork to the software CYCCNT (kalico_sim_cyccnt, bumped from the TIM5
src/generic/armcm_timer.c:46:// ISR by cycles-per-tick per fire — see src/stm32/kalico_h7_timer.c) so
src/generic/armcm_timer.c:56:    extern volatile uint32_t kalico_sim_cyccnt;
src/generic/armcm_timer.c:57:    return kalico_sim_cyccnt;
src/generic/usb_cdc.c:104:// contents are not split across calls — caller (kalico_transport_send_frame
src/generic/usb_cdc.c:105:// in src/kalico_dispatch.c) hands a single, complete frame.
src/generic/usb_cdc.c:107:kalico_console_write_raw(const uint8_t *buf, uint16_t len)
src/generic/usb_cdc.c:36:// KALICO_TX_BUF_SIZE = 256 in src/kalico_dispatch.c), so the buffer is sized
src/linux/console.c:151:kalico_console_write_raw(const uint8_t *buf, uint16_t len)
src/linux/console.c:189:        kalico_demux_output_t out = kalico_demux_feed_byte(b);
src/linux/console.c:194:            const uint8_t *kbuf = kalico_demux_klipper_buf();
src/linux/console.c:195:            uint8_t klen = kalico_demux_klipper_len();
src/linux/console.c:201:            kalico_demux_consume();
src/linux/console.c:205:            uint8_t channel = kalico_demux_kalico_channel();
src/linux/console.c:206:            const uint8_t *payload = kalico_demux_kalico_payload();
src/linux/console.c:207:            uint16_t payload_len = kalico_demux_kalico_payload_len();
src/linux/console.c:208:            kalico_dispatch_frame(channel, payload, payload_len);
src/linux/console.c:209:            kalico_demux_consume();
src/linux/console.c:213:            kalico_demux_consume();
src/linux/kalico_host_tick.c:100:        if (!kalico_rt_handle)
src/linux/kalico_host_tick.c:107:        // FFI-driven kalico_sim_endstop_set_pin path isn't clobbered
src/linux/kalico_host_tick.c:110:        kalico_endstop_sample_pins();
src/linux/kalico_host_tick.c:113:        uint32_t cyc = kalico_h7_read_cyccnt();
src/linux/kalico_host_tick.c:114:        kalico_runtime_tick(kalico_rt_handle, cyc);
src/linux/kalico_host_tick.c:119:extern void *kalico_rt_handle;
src/linux/kalico_host_tick.c:122:kalico_h7_timer_init(void)
src/linux/kalico_host_tick.c:134:        fprintf(stderr, "kalico_host_tick: pthread_create failed: %d\n", rc);
src/linux/kalico_host_tick.c:135:        // Mark as not started so a later kalico_h7_timer_init can retry —
src/linux/kalico_host_tick.c:144:kalico_h7_enable_tim5(void)
src/linux/kalico_host_tick.c:157:    if (kalico_rt_handle) {
src/linux/kalico_host_tick.c:160:        kalico_runtime_seed_widen(kalico_rt_handle, baseline);
src/linux/kalico_host_tick.c:166:kalico_h7_disable_tim5(void)
src/linux/kalico_host_tick.c:17:#include "kalico_runtime.h"
src/linux/kalico_host_tick.c:2:// kalico_runtime_tick at 40 kHz, mirroring TIM5_IRQHandler behavior on
src/linux/kalico_host_tick.c:20:extern void *kalico_rt_handle;
src/linux/kalico_host_tick.c:21:extern void kalico_endstop_sample_pins(void); // src/runtime_tick.c
src/linux/kalico_host_tick.c:25:volatile uint32_t kalico_bench_samples_buf[KALICO_BENCH_MAX_SAMPLES];
src/linux/kalico_host_tick.c:26:volatile uint16_t kalico_bench_count = 0;
src/linux/kalico_host_tick.c:27:volatile uint16_t kalico_bench_target = 0;
src/linux/kalico_host_tick.c:28:volatile uint8_t  kalico_bench_isolate = 0;
src/linux/kalico_host_tick.c:3:// the H7 firmware (src/stm32/kalico_h7_timer.c). Used by MACH_LINUX
src/linux/kalico_host_tick.c:33:volatile uint8_t kalico_liveness_ok = 1;
src/linux/kalico_host_tick.c:36:// stm32/kalico_sim_clock.c). The Linux build maps it onto the host
src/linux/kalico_host_tick.c:37:// monotonic-derived counter that kalico_h7_read_cyccnt returns.
src/linux/kalico_host_tick.c:38:volatile uint32_t kalico_sim_cyccnt = 0;
src/linux/kalico_host_tick.c:6:#include "kalico_host_tick.h"
src/linux/kalico_host_tick.c:60:extern void kalico_runtime_seed_widen(void *rt, uint64_t baseline);
src/linux/kalico_host_tick.c:63:kalico_h7_read_cyccnt(void)
src/linux/kalico_host_tick.c:73:// Wrapper exposed for kalico_h7_timer_init's widen-seeding call.
src/linux/kalico_host_tick.c:75:kalico_host_widened_clock_now(void)
src/linux/kalico_host_tick.h:13:extern volatile uint32_t kalico_bench_samples_buf[KALICO_BENCH_MAX_SAMPLES];
src/linux/kalico_host_tick.h:14:extern volatile uint16_t kalico_bench_count;
src/linux/kalico_host_tick.h:15:extern volatile uint16_t kalico_bench_target;
src/linux/kalico_host_tick.h:16:extern volatile uint8_t  kalico_bench_isolate;
src/linux/kalico_host_tick.h:2:// TIM5 ISR (src/stm32/kalico_h7_timer.c). Provides the same symbol
src/linux/kalico_host_tick.h:20:void kalico_h7_timer_init(void);
src/linux/kalico_host_tick.h:21:void kalico_h7_enable_tim5(void);
src/linux/kalico_host_tick.h:22:void kalico_h7_disable_tim5(void);
src/linux/kalico_host_tick.h:23:uint32_t kalico_h7_read_cyccnt(void);
src/runtime_tick.c:1000:    kalico_bench_count = 0;
src/runtime_tick.c:1001:    kalico_bench_target = samples;
src/runtime_tick.c:1002:    kalico_bench_isolate = isolate;
src/runtime_tick.c:1012:    while (kalico_bench_count < kalico_bench_target) {
src/runtime_tick.c:1014:        // get pre-empted by our spin and starve). Spec §5.7 — `kalico_liveness_ok`
src/runtime_tick.c:1020:            kalico_bench_target = 0;  // tell ISR to stop bracketing
src/runtime_tick.c:1021:            sendf("kalico_bench_done count=%hu error=%i",
src/runtime_tick.c:1022:                  kalico_bench_count, KALICO_BENCH_ERR_ISR_TIMEOUT);
src/runtime_tick.c:103:// last engine_status lets us emit a `kalico_fault` async event ONCE on
src/runtime_tick.c:1041:        sendf("kalico_bench_done count=%hu error=%i", 0,
src/runtime_tick.c:1046:    // Emit one Klipper-framed `kalico_bench_sample value=N` response per
src/runtime_tick.c:1051:    // including the trailing kalico_bench_done. Drain by calling the bulk
src/runtime_tick.c:1060:        sendf("kalico_bench_sample value=%u", kalico_bench_samples_buf[i]);
src/runtime_tick.c:1072:    sendf("kalico_bench_done count=%hu error=%i",
src/runtime_tick.c:1079:DECL_COMMAND(command_kalico_bench_run, "kalico_bench_run isolate=%c samples=%hu");
src/runtime_tick.c:118:// gate of test_h723_first_light.py is the `kalico_status` response over
src/runtime_tick.c:139:// timer system is best-effort even with the kalico_sim_cyccnt fork
src/runtime_tick.c:150:volatile uint32_t kalico_sim_drain_counter = 0;
src/runtime_tick.c:155:kalico_sim_isr_wake_drain(void)
src/runtime_tick.c:157:    if (++kalico_sim_drain_counter >= KALICO_SIM_DRAIN_PERIOD_TICKS) {
src/runtime_tick.c:158:        kalico_sim_drain_counter = 0;
src/runtime_tick.c:167:    kalico_rt_handle = kalico_runtime_init();
src/runtime_tick.c:168:    if (!kalico_rt_handle) {
src/runtime_tick.c:17:#include "kalico_runtime.h"
src/runtime_tick.c:173:    last_seen_tick_counter = kalico_runtime_tick_counter(kalico_rt_handle);
src/runtime_tick.c:175:    last_seen_status = kalico_runtime_status(kalico_rt_handle);
src/runtime_tick.c:180:    // that calls kalico_runtime_tick at 40 kHz.
src/runtime_tick.c:181:    extern void kalico_h7_timer_init(void);
src/runtime_tick.c:182:    kalico_h7_timer_init();
src/runtime_tick.c:20:#include "stm32/kalico_h7_timer.h" // kalico_h7_disable_tim5 / enable / read_cyccnt
src/runtime_tick.c:205:volatile uint32_t kalico_sim_drain_calls = 0;
src/runtime_tick.c:211:    if (!kalico_rt_handle) return;
src/runtime_tick.c:215:    kalico_sim_drain_calls++;
src/runtime_tick.c:223:    // (same FgState consumer); the order matters — `kalico_runtime_drain_trace`
src/runtime_tick.c:225:    // `kalico_runtime_drain_and_reclaim` consumes any remaining samples for
src/runtime_tick.c:23:// still calls kalico_h7_enable_tim5/disable_tim5/read_cyccnt across the
src/runtime_tick.c:230:    uint32_t n = kalico_runtime_drain_trace(
src/runtime_tick.c:231:        kalico_rt_handle, (struct TraceSample*)batch_buf, KALICO_TRACE_BATCH,
src/runtime_tick.c:245:        // (different msgid for `kalico_trace`) anyway.
src/runtime_tick.c:248:        output("kalico_trace count=%u data=%*s", n, n * 40, batch_buf);
src/runtime_tick.c:25:// src/linux/kalico_host_tick.c.
src/runtime_tick.c:253:    // Returns a packed status word — see kalico_runtime_drain_and_reclaim
src/runtime_tick.c:256:    // Closure-review fix: `kalico_credit_freed` MUST OR the trace leg's
src/runtime_tick.c:26:#include "linux/kalico_host_tick.h"
src/runtime_tick.c:262:    uint32_t reclaim_status = kalico_runtime_drain_and_reclaim(
src/runtime_tick.c:263:        kalico_rt_handle, KALICO_TRACE_BATCH);
src/runtime_tick.c:268:    // §10.4: emit one `kalico_credit_freed` async event per drain cycle that
src/runtime_tick.c:275:        uint32_t retired = kalico_runtime_retired_through_segment_id(kalico_rt_handle);
src/runtime_tick.c:276:        uint8_t depth = kalico_runtime_queue_depth(kalico_rt_handle);
src/runtime_tick.c:279:        kalico_native_emit_credit_freed(retired, free_slots);
src/runtime_tick.c:282:    // §13.1: a fresh trace-overflow latch is reported via the `kalico_fault`
src/runtime_tick.c:285:    // the Rust side); the periodic `kalico_status_v6` frame echoes it on
src/runtime_tick.c:289:        int32_t fault_code = kalico_runtime_last_error(kalico_rt_handle);
src/runtime_tick.c:290:        uint32_t fault_detail = kalico_runtime_fault_detail(kalico_rt_handle);
src/runtime_tick.c:291:        uint32_t cur_seg = kalico_runtime_current_segment_id(kalico_rt_handle);
src/runtime_tick.c:292:        kalico_native_emit_fault_event((uint16_t)fault_code, fault_detail, cur_seg);
src/runtime_tick.c:301:    uint32_t cur_counter = kalico_runtime_tick_counter(kalico_rt_handle);
src/runtime_tick.c:303:    uint8_t cur_status = kalico_runtime_status(kalico_rt_handle);
src/runtime_tick.c:310:            kalico_liveness_ok = 0;
src/runtime_tick.c:317:    // FAULT → also block kicks. Emit one-shot kalico_fault event if the
src/runtime_tick.c:321:        kalico_liveness_ok = 0;
src/runtime_tick.c:323:            int32_t fault_code = kalico_runtime_last_error(kalico_rt_handle);
src/runtime_tick.c:324:            uint32_t fault_detail = kalico_runtime_fault_detail(kalico_rt_handle);
src/runtime_tick.c:325:            uint32_t cur_seg = kalico_runtime_current_segment_id(kalico_rt_handle);
src/runtime_tick.c:326:            kalico_native_emit_fault_event((uint16_t)fault_code, fault_detail, cur_seg);
src/runtime_tick.c:335:    // re-enables TIM5 on the next kalico_runtime_push_segment call when
src/runtime_tick.c:341:        kalico_h7_disable_tim5();
src/runtime_tick.c:353:// Phase 11 Task 11.1 §5.3 periodic 10 Hz `kalico_status_v6` frame.
src/runtime_tick.c:361:    if (!kalico_rt_handle) return;
src/runtime_tick.c:371:    uint8_t status = kalico_runtime_status(kalico_rt_handle);
src/runtime_tick.c:372:    int32_t last_err = kalico_runtime_last_error(kalico_rt_handle);
src/runtime_tick.c:373:    uint32_t cur_seg = kalico_runtime_current_segment_id(kalico_rt_handle);
src/runtime_tick.c:374:    uint8_t depth = kalico_runtime_queue_depth(kalico_rt_handle);
src/runtime_tick.c:375:    uint32_t fault_detail = kalico_runtime_fault_detail(kalico_rt_handle);
src/runtime_tick.c:377:    // Phase C: replace the legacy `kalico_status_v6` Klipper-protocol output
src/runtime_tick.c:38:// Exposed to Rust via `extern "C" { static kalico_clock_freq: u32; }`.
src/runtime_tick.c:380:    kalico_native_emit_status_event(status, depth, cur_seg, last_err, fault_detail);
src/runtime_tick.c:386:    int32_t c0 = kalico_runtime_get_stepper_count(kalico_rt_handle, 0);
src/runtime_tick.c:387:    int32_t c1 = kalico_runtime_get_stepper_count(kalico_rt_handle, 1);
src/runtime_tick.c:388:    int32_t c2 = kalico_runtime_get_stepper_count(kalico_rt_handle, 2);
src/runtime_tick.c:40:const uint32_t kalico_clock_freq __attribute__((used, externally_visible))
src/runtime_tick.c:422:// Non-static so kalico_dispatch.c's LoadCurve handler can reuse the same
src/runtime_tick.c:424:float kalico_aligned_cps[CONFIG_RUNTIME_MAX_CONTROL_POINTS];
src/runtime_tick.c:425:float kalico_aligned_knots[CONFIG_RUNTIME_MAX_KNOT_VECTOR_LEN];
src/runtime_tick.c:43:extern volatile uint8_t kalico_liveness_ok;  // defined in src/stm32/watchdog.c
src/runtime_tick.c:430:// kalico_push_segment command. Curve uploads and segment pushes now arrive
src/runtime_tick.c:431:// as native kalico frames; see src/kalico_dispatch.c handlers.
src/runtime_tick.c:435:command_kalico_query_status(uint32_t *args)
src/runtime_tick.c:437:    if (!kalico_rt_handle) {
src/runtime_tick.c:438:        sendf("kalico_status status=%c last_err=%i", (uint8_t)255, -7);
src/runtime_tick.c:441:    uint8_t status = kalico_runtime_status(kalico_rt_handle);
src/runtime_tick.c:442:    int32_t last_err = kalico_runtime_last_error(kalico_rt_handle);
src/runtime_tick.c:443:    sendf("kalico_status status=%c last_err=%i", status, last_err);
src/runtime_tick.c:445:DECL_COMMAND(command_kalico_query_status, "kalico_query_status");
src/runtime_tick.c:450:command_kalico_set_homed(uint32_t *args)
src/runtime_tick.c:453:    if (!kalico_rt_handle) {
src/runtime_tick.c:454:        sendf("kalico_set_homed_response result=%i", -7);
src/runtime_tick.c:457:    int32_t r = kalico_set_homed(kalico_rt_handle);
src/runtime_tick.c:458:    sendf("kalico_set_homed_response result=%i", r);
src/runtime_tick.c:460:DECL_COMMAND(command_kalico_set_homed, "kalico_set_homed");
src/runtime_tick.c:463:// no-arg kalico_set_homed (preserved for backward compat), letting the
src/runtime_tick.c:466:command_kalico_set_homed_state(uint32_t *args)
src/runtime_tick.c:468:    if (!kalico_rt_handle) {
src/runtime_tick.c:469:        sendf("kalico_set_homed_response result=%i", -7);
src/runtime_tick.c:473:    int32_t r = kalico_set_homed_state(kalico_rt_handle, homed);
src/runtime_tick.c:474:    sendf("kalico_set_homed_response result=%i", r);
src/runtime_tick.c:476:DECL_COMMAND(command_kalico_set_homed_state, "kalico_set_homed_state homed=%c");
src/runtime_tick.c:484:// through `kalico_endstop_set_pin_level` before `kalico_runtime_tick`
src/runtime_tick.c:496:extern int32_t kalico_endstop_set_pin_level(uint16_t gpio, uint8_t level);
src/runtime_tick.c:498:// Called from TIM5_IRQHandler immediately before kalico_runtime_tick.
src/runtime_tick.c:501:kalico_endstop_sample_pins(void)
src/runtime_tick.c:507:        (void)kalico_endstop_set_pin_level(endstop_pin_table[i].gpio_id, level);
src/runtime_tick.c:519:// rust/kalico-c-api/src/runtime_ffi.rs::kalico_endstop_arm decode (record
src/runtime_tick.c:544:command_kalico_arm_endstop(uint32_t *args)
src/runtime_tick.c:559:    (void)kalico_endstop_arm(arm_id, arm_clock_lo, arm_clock_hi,
src/runtime_tick.c:568:    sendf("kalico_arm_endstop_response arm_id=%u status=%c", arm_id, status);
src/runtime_tick.c:570:DECL_COMMAND(command_kalico_arm_endstop,
src/runtime_tick.c:571:    "kalico_arm_endstop arm_id=%u arm_clock_lo=%u arm_clock_hi=%u "
src/runtime_tick.c:576:command_kalico_disarm_endstop(uint32_t *args)
src/runtime_tick.c:580:    (void)kalico_endstop_disarm(arm_id, &status);
src/runtime_tick.c:585:    sendf("kalico_disarm_endstop_response arm_id=%u status=%c", arm_id, status);
src/runtime_tick.c:587:DECL_COMMAND(command_kalico_disarm_endstop, "kalico_disarm_endstop arm_id=%u");
src/runtime_tick.c:590:         "kalico_endstop_tripped arm_id=%u "
src/runtime_tick.c:596:// `kalico_endstop_tripped` outputs. Modeled on `runtime_status_drain` —
src/runtime_tick.c:60:kalico_host_now_us(void)
src/runtime_tick.c:603:    if (!kalico_rt_handle) return;
src/runtime_tick.c:606:    int32_t r = kalico_endstop_poll_trip(buf, sizeof(buf), &actual);
src/runtime_tick.c:619:    output("kalico_endstop_tripped arm_id=%u "
src/runtime_tick.c:630:command_kalico_configure_axes(uint32_t *args)
src/runtime_tick.c:632:    if (!kalico_rt_handle) {
src/runtime_tick.c:633:        sendf("kalico_configure_axes_response result=%i", -7);
src/runtime_tick.c:637:    int32_t r = kalico_configure_axes(kalico_rt_handle, kinematics);
src/runtime_tick.c:638:    sendf("kalico_configure_axes_response result=%i", r);
src/runtime_tick.c:640:DECL_COMMAND(command_kalico_configure_axes, "kalico_configure_axes kinematics=%c");
src/runtime_tick.c:648:command_kalico_stream_open(uint32_t *args)
src/runtime_tick.c:650:    if (!kalico_rt_handle) {
src/runtime_tick.c:651:        sendf("kalico_stream_open_response result=%i credit_epoch=%u", -7, 0);
src/runtime_tick.c:656:    int32_t r = kalico_runtime_stream_open(
src/runtime_tick.c:657:        kalico_rt_handle, stream_id, &credit_epoch);
src/runtime_tick.c:658:    sendf("kalico_stream_open_response result=%i credit_epoch=%u",
src/runtime_tick.c:661:DECL_COMMAND(command_kalico_stream_open, "kalico_stream_open stream_id=%u");
src/runtime_tick.c:664:command_kalico_stream_arm(uint32_t *args)
src/runtime_tick.c:666:    if (!kalico_rt_handle) {
src/runtime_tick.c:668:            "kalico_stream_arm_response result=%i armed_t_start_lo=%u armed_t_start_hi=%u",
src/runtime_tick.c:675:    int32_t r = kalico_runtime_stream_arm(
src/runtime_tick.c:676:        kalico_rt_handle, t_start_t0, arm_lead_cycles, &armed_t_start);
src/runtime_tick.c:678:        "kalico_stream_arm_response result=%i armed_t_start_lo=%u armed_t_start_hi=%u",
src/runtime_tick.c:681:DECL_COMMAND(command_kalico_stream_arm,
src/runtime_tick.c:682:    "kalico_stream_arm t_start_t0_lo=%u t_start_t0_hi=%u arm_lead_cycles=%u");
src/runtime_tick.c:685:command_kalico_stream_terminal(uint32_t *args)
src/runtime_tick.c:687:    if (!kalico_rt_handle) {
src/runtime_tick.c:688:        sendf("kalico_stream_terminal_response result=%i", -7);
src/runtime_tick.c:692:    int32_t r = kalico_runtime_stream_terminal(kalico_rt_handle, segment_id);
src/runtime_tick.c:693:    sendf("kalico_stream_terminal_response result=%i", r);
src/runtime_tick.c:695:DECL_COMMAND(command_kalico_stream_terminal,
src/runtime_tick.c:696:    "kalico_stream_terminal segment_id=%u");
src/runtime_tick.c:699:command_kalico_stream_flush(uint32_t *args)
src/runtime_tick.c:702:    if (!kalico_rt_handle) {
src/runtime_tick.c:703:        sendf("kalico_stream_flush_response result=%i credit_epoch=%u", -7, 0);
src/runtime_tick.c:707:    int32_t r = kalico_runtime_stream_flush(kalico_rt_handle, &credit_epoch);
src/runtime_tick.c:708:    sendf("kalico_stream_flush_response result=%i credit_epoch=%u",
src/runtime_tick.c:711:DECL_COMMAND(command_kalico_stream_flush, "kalico_stream_flush");
src/runtime_tick.c:715:command_kalico_clock_sync_request(uint32_t *args)
src/runtime_tick.c:717:    if (!kalico_rt_handle) {
src/runtime_tick.c:719:            "kalico_clock_sync_response request_id=%u mcu_clock_lo=%u mcu_clock_hi=%u",
src/runtime_tick.c:727:    kalico_runtime_clock_sync_request(
src/runtime_tick.c:728:        kalico_rt_handle, request_id,
src/runtime_tick.c:732:        "kalico_clock_sync_response request_id=%u mcu_clock_lo=%u mcu_clock_hi=%u",
src/runtime_tick.c:735:DECL_COMMAND(command_kalico_clock_sync_request,
src/runtime_tick.c:736:    "kalico_clock_sync_request request_id=%u "
src/runtime_tick.c:743:command_kalico_query_pool_state(uint32_t *args)
src/runtime_tick.c:745:    if (!kalico_rt_handle) {
src/runtime_tick.c:747:            "kalico_pool_state_response result=%i slot_idx=%hu current_gen=%hu last_retired_gen=%hu",
src/runtime_tick.c:75:// inline the body at every callsite. The kalico_c_api.a archive then
src/runtime_tick.c:754:    int32_t r = kalico_runtime_query_pool_state(
src/runtime_tick.c:755:        kalico_rt_handle, slot, &current_gen, &last_retired_gen);
src/runtime_tick.c:757:        "kalico_pool_state_response result=%i slot_idx=%hu current_gen=%hu last_retired_gen=%hu",
src/runtime_tick.c:760:DECL_COMMAND(command_kalico_query_pool_state,
src/runtime_tick.c:761:    "kalico_query_pool_state slot=%hu");
src/runtime_tick.c:764:// `kalico_credit_freed` and `kalico_fault` are MCU-emitted async events
src/runtime_tick.c:772:         "kalico_trace count=%u data=%*s");
src/runtime_tick.c:773:// kalico_credit_freed / kalico_fault / kalico_status_v6 retired Phase C —
src/runtime_tick.c:775:// kalico_native_emit_credit_freed / _fault_event / _status_event in
src/runtime_tick.c:776:// src/kalico_dispatch.c.
src/runtime_tick.c:779:         "kalico_sim_gpio_sample sample_id=%u pin=%c value=%c");
src/runtime_tick.c:78:// Solution: provide thin wrappers `kalico_irq_save` / `kalico_irq_restore`
src/runtime_tick.c:783:extern volatile uint32_t kalico_sim_drain_calls;
src/runtime_tick.c:784:extern volatile uint32_t kalico_sim_cyccnt;
src/runtime_tick.c:785:extern volatile uint32_t kalico_sim_drain_counter;
src/runtime_tick.c:788:command_kalico_sim_diag(uint32_t *args)
src/runtime_tick.c:790:    uint8_t status = kalico_rt_handle ? kalico_runtime_status(kalico_rt_handle) : 255;
src/runtime_tick.c:791:    int32_t last_err = kalico_rt_handle ? kalico_runtime_last_error(kalico_rt_handle) : 0;
src/runtime_tick.c:792:    uint32_t tick_counter = kalico_rt_handle ? kalico_runtime_tick_counter(kalico_rt_handle) : 0;
src/runtime_tick.c:794:        "kalico_sim_diag_response drain_calls=%u cyccnt=%u drain_counter=%u "
src/runtime_tick.c:796:        kalico_sim_drain_calls, kalico_sim_cyccnt, kalico_sim_drain_counter,
src/runtime_tick.c:799:DECL_COMMAND(command_kalico_sim_diag, "kalico_sim_diag");
src/runtime_tick.c:802:command_kalico_sim_gpio_sample(uint32_t *args)
src/runtime_tick.c:810:    sendf("kalico_sim_gpio_sample_response sample_id=%u pin=%c value=%c",
src/runtime_tick.c:812:    output("kalico_sim_gpio_sample sample_id=%u pin=%c value=%c",
src/runtime_tick.c:815:DECL_COMMAND(command_kalico_sim_gpio_sample,
src/runtime_tick.c:816:    "kalico_sim_gpio_sample sample_id=%u pin=%c pull_up=%c");
src/runtime_tick.c:822:command_kalico_sim_stepper_count_query(uint32_t *args)
src/runtime_tick.c:825:    int32_t count = kalico_rt_handle
src/runtime_tick.c:826:        ? kalico_runtime_get_stepper_count(kalico_rt_handle, oid)
src/runtime_tick.c:828:    sendf("kalico_sim_stepper_count_response oid=%c count=%i", oid, count);
src/runtime_tick.c:830:DECL_COMMAND(command_kalico_sim_stepper_count_query,
src/runtime_tick.c:831:    "kalico_sim_stepper_count_query oid=%c");
src/runtime_tick.c:837:command_kalico_sim_axis_steps_query(uint32_t *args)
src/runtime_tick.c:840:    float spm = kalico_rt_handle
src/runtime_tick.c:841:        ? kalico_runtime_get_axis_steps_per_mm(kalico_rt_handle, oid)
src/runtime_tick.c:846:    sendf("kalico_sim_axis_steps_response oid=%c milli_spm=%i", oid, milli);
src/runtime_tick.c:848:DECL_COMMAND(command_kalico_sim_axis_steps_query,
src/runtime_tick.c:849:    "kalico_sim_axis_steps_query oid=%c");
src/runtime_tick.c:85:kalico_irq_save(void)
src/runtime_tick.c:852:command_kalico_sim_axis_accum_query(uint32_t *args)
src/runtime_tick.c:855:    double a = kalico_rt_handle
src/runtime_tick.c:856:        ? kalico_runtime_get_axis_accumulator(kalico_rt_handle, oid)
src/runtime_tick.c:859:    sendf("kalico_sim_axis_accum_response oid=%c milli=%i", oid, milli);
src/runtime_tick.c:861:DECL_COMMAND(command_kalico_sim_axis_accum_query,
src/runtime_tick.c:862:    "kalico_sim_axis_accum_query oid=%c");
src/runtime_tick.c:872:extern int32_t kalico_runtime_load_fixture(
src/runtime_tick.c:876:command_kalico_load_fixture_curve(uint32_t *args)
src/runtime_tick.c:878:    if (!kalico_rt_handle) {
src/runtime_tick.c:879:        sendf("kalico_load_fixture_response result=%i curve_handle_packed=%u",
src/runtime_tick.c:886:    int32_t r = kalico_runtime_load_fixture(
src/runtime_tick.c:887:        kalico_rt_handle, slot, fixture_id, &handle_packed);
src/runtime_tick.c:888:    sendf("kalico_load_fixture_response result=%i curve_handle_packed=%u",
src/runtime_tick.c:891:DECL_COMMAND(command_kalico_load_fixture_curve,
src/runtime_tick.c:892:    "kalico_load_fixture_curve slot=%hu fixture_id=%hu");
src/runtime_tick.c:900:extern int32_t kalico_endstop_set_pin_level(uint16_t gpio, uint8_t level);
src/runtime_tick.c:903:command_kalico_sim_endstop_set_pin(uint32_t *args)
src/runtime_tick.c:907:    int32_t r = kalico_endstop_set_pin_level(gpio, level);
src/runtime_tick.c:908:    sendf("kalico_sim_endstop_set_pin_response gpio=%hu level=%c result=%i",
src/runtime_tick.c:911:DECL_COMMAND(command_kalico_sim_endstop_set_pin,
src/runtime_tick.c:912:    "kalico_sim_endstop_set_pin gpio=%hu level=%c");
src/runtime_tick.c:916:// (`kalico_h7_timer_init` only configures it; `kalico_h7_enable_tim5`
src/runtime_tick.c:92:kalico_irq_restore(uint32_t flags)
src/runtime_tick.c:920:// fires. This sim-only shim drives `kalico_h7_enable_tim5` directly so
src/runtime_tick.c:922:extern void kalico_h7_enable_tim5(void);
src/runtime_tick.c:925:command_kalico_sim_engine_tick_start(uint32_t *args)
src/runtime_tick.c:928:    kalico_h7_enable_tim5();
src/runtime_tick.c:929:    sendf("kalico_sim_engine_tick_start_response result=%i", 0);
src/runtime_tick.c:931:DECL_COMMAND(command_kalico_sim_engine_tick_start,
src/runtime_tick.c:932:    "kalico_sim_engine_tick_start");
src/runtime_tick.c:937:// Surface-C only. Captures DWT->CYCCNT around `kalico_runtime_tick` over N
src/runtime_tick.c:938:// samples and replies with one `kalico_bench_sample value=<cycles>` response
src/runtime_tick.c:939:// per measurement (after the warmup skip) and a final `kalico_bench_done
src/runtime_tick.c:942:// via klippy/msgproto.py wrapped by tools/kalico_host_io.py.
src/runtime_tick.c:949:// KALICO_BENCH_MAX_SAMPLES is declared in `src/stm32/kalico_h7_timer.h`
src/runtime_tick.c:951:// `kalico_h7_timer.c` see the same value.
src/runtime_tick.c:952:extern volatile uint32_t kalico_bench_samples_buf[KALICO_BENCH_MAX_SAMPLES];
src/runtime_tick.c:953:extern volatile uint16_t kalico_bench_count;
src/runtime_tick.c:954:extern volatile uint16_t kalico_bench_target;
src/runtime_tick.c:955:extern volatile uint8_t kalico_bench_isolate;
src/runtime_tick.c:958:// `kalico_bench_done count=%hu error=%i` per Klipper's one-format-per-message
src/runtime_tick.c:968:command_kalico_bench_run(uint32_t *args)
src/runtime_tick.c:97:void* kalico_rt_handle = 0;            // exposed (non-static) for kalico_h7_timer.c
src/runtime_tick.c:970:    if (!kalico_rt_handle) {
src/runtime_tick.c:971:        sendf("kalico_bench_done count=%hu error=%i", 0, KALICO_BENCH_ERR_NOT_INIT);
src/runtime_tick.c:978:    if (!kalico_liveness_ok) {
src/runtime_tick.c:979:        sendf("kalico_bench_done count=%hu error=%i", 0, KALICO_BENCH_ERR_LIVENESS);
src/stm32/kalico_h7_timer.c:1:// src/stm32/kalico_h7_timer.c
src/stm32/kalico_h7_timer.c:101:// Cycle-count bench buffer storage. Declared `extern` in kalico_h7_timer.h
src/stm32/kalico_h7_timer.c:104:volatile uint32_t kalico_bench_samples_buf[KALICO_BENCH_MAX_SAMPLES];
src/stm32/kalico_h7_timer.c:105:volatile uint16_t kalico_bench_count = 0;
src/stm32/kalico_h7_timer.c:106:volatile uint16_t kalico_bench_target = 0;
src/stm32/kalico_h7_timer.c:107:volatile uint8_t  kalico_bench_isolate = 0;
src/stm32/kalico_h7_timer.c:119:    extern volatile uint32_t kalico_sim_cyccnt;
src/stm32/kalico_h7_timer.c:120:    kalico_sim_cyccnt += (kalico_clock_freq / 40000U);
src/stm32/kalico_h7_timer.c:124:    extern void kalico_sim_isr_wake_drain(void);
src/stm32/kalico_h7_timer.c:125:    kalico_sim_isr_wake_drain();
src/stm32/kalico_h7_timer.c:13:extern const uint32_t kalico_clock_freq;
src/stm32/kalico_h7_timer.c:138:    // via `command_kalico_sim_endstop_set_pin`, and a real-GPIO sample
src/stm32/kalico_h7_timer.c:141:    extern void kalico_endstop_sample_pins(void);
src/stm32/kalico_h7_timer.c:142:    kalico_endstop_sample_pins();
src/stm32/kalico_h7_timer.c:145:    uint32_t before = kalico_h7_read_cyccnt();
src/stm32/kalico_h7_timer.c:146:    if (kalico_rt_handle) {
src/stm32/kalico_h7_timer.c:147:        kalico_runtime_tick(kalico_rt_handle, before);
src/stm32/kalico_h7_timer.c:149:    uint32_t after = kalico_h7_read_cyccnt();
src/stm32/kalico_h7_timer.c:15:extern void* kalico_rt_handle;   // exposed in src/runtime_tick.c
src/stm32/kalico_h7_timer.c:152:    if (kalico_bench_count < kalico_bench_target) {
src/stm32/kalico_h7_timer.c:153:        kalico_bench_samples_buf[kalico_bench_count] = after - before;
src/stm32/kalico_h7_timer.c:154:        kalico_bench_count++;
src/stm32/kalico_h7_timer.c:22:// --gc-sections, mirroring kalico_clock_freq / kalico_liveness_ok.
src/stm32/kalico_h7_timer.c:25:kalico_h7_disable_tim5(void)
src/stm32/kalico_h7_timer.c:35:// widening loop in sim. Fork to a software counter (kalico_sim_cyccnt) bumped
src/stm32/kalico_h7_timer.c:41:kalico_h7_read_cyccnt(void)
src/stm32/kalico_h7_timer.c:44:    extern volatile uint32_t kalico_sim_cyccnt;
src/stm32/kalico_h7_timer.c:45:    return kalico_sim_cyccnt;
src/stm32/kalico_h7_timer.c:53:kalico_h7_enable_tim5(void)
src/stm32/kalico_h7_timer.c:61:kalico_h7_timer_init(void)
src/stm32/kalico_h7_timer.c:8:#include "kalico_runtime.h"
src/stm32/kalico_h7_timer.c:82:    TIM5->ARR = (kalico_clock_freq / 40000U) - 1U;
src/stm32/kalico_h7_timer.c:9:#include "kalico_h7_timer.h"   // shared bench buffer + helper sigs
src/stm32/kalico_h7_timer.c:98:    // kalico_h7_enable_tim5() via the producer protocol.
src/stm32/kalico_h7_timer.h:1:// src/stm32/kalico_h7_timer.h
src/stm32/kalico_h7_timer.h:18:extern volatile uint32_t kalico_bench_samples_buf[KALICO_BENCH_MAX_SAMPLES];
src/stm32/kalico_h7_timer.h:19:extern volatile uint16_t kalico_bench_count;
src/stm32/kalico_h7_timer.h:20:extern volatile uint16_t kalico_bench_target;
src/stm32/kalico_h7_timer.h:21:extern volatile uint8_t  kalico_bench_isolate;
src/stm32/kalico_h7_timer.h:23:void kalico_h7_timer_init(void);
src/stm32/kalico_h7_timer.h:24:void kalico_h7_enable_tim5(void);
src/stm32/kalico_h7_timer.h:25:void kalico_h7_disable_tim5(void);
src/stm32/kalico_h7_timer.h:26:uint32_t kalico_h7_read_cyccnt(void);
src/stm32/kalico_h7_timer.h:4:// both src/stm32/kalico_h7_timer.c (defines the storage) and src/runtime_tick.c
src/stm32/kalico_sim_clock.c:1:// src/stm32/kalico_sim_clock.c
src/stm32/kalico_sim_clock.c:17:// Bumped by TIM5 ISR (kalico_h7_timer.c) once per tick.
src/stm32/kalico_sim_clock.c:19:volatile uint32_t kalico_sim_cyccnt = 0;
src/stm32/watchdog.c:19:volatile uint8_t kalico_liveness_ok __attribute__((used, externally_visible))
src/stm32/watchdog.c:30:    if (!kalico_liveness_ok) return;   // kalico runtime detected liveness fault
```

## Rust symbols (rust/)
```
rust/motion-bridge/src/bridge.rs:101:        .kalico_call(MessageKind::QueryRuntimeCaps, Vec::new(), timeout)
rust/motion-bridge/src/bridge.rs:102:        .map_err(|e| format!("kalico_call QueryRuntimeCaps: {e:?}"))?;
rust/motion-bridge/src/bridge.rs:1080:        //     no real `kalico_credit_freed` accounting yet),
rust/motion-bridge/src/bridge.rs:1152:        // bind the `kalico_credit_freed.retired_through_segment_id`
rust/motion-bridge/src/bridge.rs:1315:                                 awaiting kalico_credit_freed retirement events",
rust/motion-bridge/src/bridge.rs:1359:                // `kalico_credit_freed`-driven retirement can release them.
rust/motion-bridge/src/bridge.rs:136:fn router_err(e: kalico_host_rt::passthrough_queue::RouterError) -> PyErr {
rust/motion-bridge/src/bridge.rs:1462:    // and handles async `kalico_endstop_tripped` events via the existing
rust/motion-bridge/src/bridge.rs:1465:    /// Send `kalico_arm_endstop` and wait for the synchronous response.
rust/motion-bridge/src/bridge.rs:1480:        use kalico_host_rt::endstop;
rust/motion-bridge/src/bridge.rs:1523:    /// Send `kalico_disarm_endstop` and wait for the response. Returns the
rust/motion-bridge/src/bridge.rs:1533:        use kalico_host_rt::endstop;
rust/motion-bridge/src/bridge.rs:1542:    /// Send `kalico_set_homed_state homed=%c`. Spec §8.
rust/motion-bridge/src/bridge.rs:1551:        use kalico_host_rt::endstop;
rust/motion-bridge/src/bridge.rs:16:use kalico_host_rt::clock::RealClock;
rust/motion-bridge/src/bridge.rs:1642:    /// Drive the bridge with a `kalico_credit_freed` event.
rust/motion-bridge/src/bridge.rs:1658:    /// `kalico_credit_freed` over its existing serial loop and is
rust/motion-bridge/src/bridge.rs:17:use kalico_host_rt::credit::CreditCounter;
rust/motion-bridge/src/bridge.rs:1770:    //! glue that forwards `kalico_credit_freed` MCU events into the
rust/motion-bridge/src/bridge.rs:18:use kalico_host_rt::host_io::parser::{DataDictionary, MsgProtoParser};
rust/motion-bridge/src/bridge.rs:20:use kalico_host_rt::passthrough_queue::{
rust/motion-bridge/src/bridge.rs:23:use kalico_host_rt::producer;
rust/motion-bridge/src/bridge.rs:235:        let clock: Arc<dyn kalico_host_rt::clock::Clock + Send + Sync> = Arc::new(RealClock);
rust/motion-bridge/src/bridge.rs:35:/// `kalico_credit_freed` events into [`CreditCounter::on_credit_freed`] via
rust/motion-bridge/src/bridge.rs:57:    runtime_rx: Option<Receiver<kalico_host_rt::host_io::runtime_events::RuntimeEvent>>,
rust/motion-bridge/src/bridge.rs:594:        match host_io.kalico_identify(std::time::Duration::from_secs(5)) {
rust/motion-bridge/src/bridge.rs:603:                    "attach_serial: kalico_identify failed for {serial_path}: {e}"
rust/motion-bridge/src/bridge.rs:63:    runtime_caps: Option<kalico_protocol::messages::RuntimeCapsResponse>,
rust/motion-bridge/src/bridge.rs:69:const FALLBACK_RUNTIME_CAPS: kalico_protocol::messages::RuntimeCapsResponse =
rust/motion-bridge/src/bridge.rs:695:            io.kalico_call(
rust/motion-bridge/src/bridge.rs:696:                kalico_protocol::MessageKind::ConfigureAxes,
rust/motion-bridge/src/bridge.rs:70:    kalico_protocol::messages::RuntimeCapsResponse {
rust/motion-bridge/src/bridge.rs:79:/// reactor + serial port (the actual `kalico_call` round-trip is exercised
rust/motion-bridge/src/bridge.rs:790:            use kalico_host_rt::transport::Transport;
rust/motion-bridge/src/bridge.rs:797:            use kalico_host_rt::transport::MessageValue;
rust/motion-bridge/src/bridge.rs:816:    ///   - `"status"`: kalico_status_v6 heartbeat — keys: `engine_status`,
rust/motion-bridge/src/bridge.rs:818:    ///   - `"credit_freed"`: kalico_credit_freed — keys: `retired_through_segment_id`,
rust/motion-bridge/src/bridge.rs:820:    ///   - `"fault"`: kalico_fault — keys: `fault_code`, `fault_detail`,
rust/motion-bridge/src/bridge.rs:823:    ///   - `"endstop_tripped"`: kalico_endstop_tripped — keys: `arm_id`,
rust/motion-bridge/src/bridge.rs:826:        use kalico_host_rt::host_io::runtime_events::RuntimeEvent;
rust/motion-bridge/src/bridge.rs:83:) -> Result<kalico_protocol::messages::RuntimeCapsResponse, String> {
rust/motion-bridge/src/bridge.rs:84:    use kalico_protocol::codec::{Cursor, Decode};
rust/motion-bridge/src/bridge.rs:85:    use kalico_protocol::messages::RuntimeCapsResponse;
rust/motion-bridge/src/bridge.rs:98:) -> Result<kalico_protocol::messages::RuntimeCapsResponse, String> {
rust/motion-bridge/src/bridge.rs:99:    use kalico_protocol::MessageKind;
rust/motion-bridge/src/cap_check.rs:6:use kalico_host_rt::producer::CurveLoadParams;
rust/motion-bridge/src/dispatch.rs:60:impl From<kalico_protocol::messages::RuntimeCapsResponse> for McuCaps {
rust/motion-bridge/src/dispatch.rs:61:    fn from(r: kalico_protocol::messages::RuntimeCapsResponse) -> Self {
rust/motion-bridge/src/dispatch.rs:9:use kalico_host_rt::producer::{CurveLoadParams, SegmentPushParams};
rust/motion-bridge/src/router_transport.rs:1://! `RouterTransport` — a `kalico_host_rt::transport::Transport` impl backed
rust/motion-bridge/src/router_transport.rs:167:        // sentinel — see `kalico_host_rt::passthrough_queue::router`.
rust/motion-bridge/src/router_transport.rs:182:    use kalico_host_rt::clock::RealClock;
rust/motion-bridge/src/router_transport.rs:185:        let clock: Arc<dyn kalico_host_rt::clock::Clock + Send + Sync> = Arc::new(RealClock);
rust/motion-bridge/src/router_transport.rs:201:        match t.call("kalico_load_curve", "kalico_load_curve_response", Duration::from_millis(10)) {
rust/motion-bridge/src/router_transport.rs:22:use kalico_host_rt::host_io::parser::{FieldValue, MsgProtoParser};
rust/motion-bridge/src/router_transport.rs:23:use kalico_host_rt::passthrough_queue::{
rust/motion-bridge/src/router_transport.rs:26:use kalico_host_rt::transport::{MessageParams, Transport, TransportError};
rust/motion-bridge/src/router_transport.rs:73:    /// `kalico_host_rt::passthrough_queue::router::PassthroughRouter::dispatch_response`.
rust/motion-bridge/src/slot_pool.rs:127:    /// per-slot retirement signal (e.g. a future `kalico_curve_freed`
rust/motion-bridge/src/slot_pool.rs:138:    /// `<= retired_through`. Driven by `kalico_credit_freed`'s
rust/motion-bridge/src/slot_pool.rs:19://!   `retired_through_segment_id` (in `kalico_credit_freed`).
rust/motion-bridge/src/slot_pool.rs:26://! `kalico_load_curve_response.curve_handle_packed`. The host increments
rust/motion-bridge/src/slot_pool.rs:3://! Backs the `kalico_load_curve` `slot: u16` field. The firmware-side curve
rust/motion-bridge/src/slot_pool.rs:41://! `EventDispatcher` that lifts `kalico_credit_freed` lives in the
rust/motion-bridge/src/slot_pool.rs:97:    /// caller must ship `slot_idx` in the `kalico_load_curve` request and
rust/motion-bridge/src/types.rs:6:use kalico_host_rt::passthrough_queue::{CommandQueueId, McuHandle, PassthroughStats};
rust/motion-bridge/tests/runtime_caps.rs:14:use kalico_protocol::codec::{Cursor, Decode, Encode};
rust/motion-bridge/tests/runtime_caps.rs:15:use kalico_protocol::messages::{RuntimeCapsResponse, RUNTIME_CAPS_RESPONSE_BODY_LEN};
rust/motion-bridge/tests/sim_motion.rs:100:/// `kalico_load_curve_chunk` fire-and-forget frames and consumed by
rust/motion-bridge/tests/sim_motion.rs:101:/// `kalico_load_curve_finalize` to assemble a `LoadCurveCapture`.
rust/motion-bridge/tests/sim_motion.rs:1256:/// reuse is gated on `kalico_credit_freed` retirement events.
rust/motion-bridge/tests/sim_motion.rs:1262:///   2. After each flush, simulates a `kalico_credit_freed` event with
rust/motion-bridge/tests/sim_motion.rs:1285:        // retires the lot. (Equivalent to a real `kalico_credit_freed`
rust/motion-bridge/tests/sim_motion.rs:1311:/// `kalico_load_curve_response { result != 0 }` once slot >= 64.)
rust/motion-bridge/tests/sim_motion.rs:1374:    let pushes = octopus.sent_starting_with("kalico_push_segment");
rust/motion-bridge/tests/sim_motion.rs:1375:    let loads = octopus.sent_starting_with("kalico_load_curve");
rust/motion-bridge/tests/sim_motion.rs:155:    /// All captured `kalico_load_curve` payloads, in submission order.
rust/motion-bridge/tests/sim_motion.rs:179:            if record.cmd.starts_with("kalico_push_segment") {
rust/motion-bridge/tests/sim_motion.rs:228:            "kalico_load_curve_response" => {
rust/motion-bridge/tests/sim_motion.rs:239:            "kalico_push_response" => {
rust/motion-bridge/tests/sim_motion.rs:269:            "kalico_load_curve_finalize_response" => {
rust/motion-bridge/tests/sim_motion.rs:292:            "kalico_push_response" => {
rust/motion-bridge/tests/sim_motion.rs:32://!     submit `submit_move(10, 0, 0, 0, 100)`, wait. Assert: `kalico_load_curve`
rust/motion-bridge/tests/sim_motion.rs:325:            "kalico_load_curve_begin" => {
rust/motion-bridge/tests/sim_motion.rs:33://!     fired for the X axis on the Octopus, `kalico_push_segment` fired with
rust/motion-bridge/tests/sim_motion.rs:342:            "kalico_load_curve_chunk" => {
rust/motion-bridge/tests/sim_motion.rs:37://!     `kalico_load_curve` + `kalico_push_segment` only on F446, nothing on
rust/motion-bridge/tests/sim_motion.rs:391:    /// to tests so they can simulate `kalico_credit_freed` retirement
rust/motion-bridge/tests/sim_motion.rs:56:use kalico_host_rt::credit::CreditCounter;
rust/motion-bridge/tests/sim_motion.rs:57:use kalico_host_rt::host_io::parser::FieldValue;
rust/motion-bridge/tests/sim_motion.rs:58:use kalico_host_rt::producer::{self, DEFAULT_LOAD_CURVE_TIMEOUT};
rust/motion-bridge/tests/sim_motion.rs:59:use kalico_host_rt::transport::{
rust/motion-bridge/tests/sim_motion.rs:596:    /// Simulate a `kalico_credit_freed` event for the given MCU. Releases
rust/motion-bridge/tests/sim_motion.rs:738:    let load_curves = octopus.sent_starting_with("kalico_load_curve");
rust/motion-bridge/tests/sim_motion.rs:739:    let pushes = octopus.sent_starting_with("kalico_push_segment");
rust/motion-bridge/tests/sim_motion.rs:742:        "expected kalico_load_curve on Octopus, saw none"
rust/motion-bridge/tests/sim_motion.rs:746:        "expected kalico_push_segment on Octopus, saw none"
rust/motion-bridge/tests/sim_motion.rs:779:    let f446_loads = f446.sent_starting_with("kalico_load_curve");
rust/motion-bridge/tests/sim_motion.rs:780:    let f446_pushes = f446.sent_starting_with("kalico_push_segment");
rust/motion-bridge/tests/sim_motion.rs:783:        "expected kalico_load_curve on F446 (Z), saw none"
rust/motion-bridge/tests/sim_motion.rs:787:        "expected kalico_push_segment on F446 (Z), saw none"
rust/motion-bridge/tests/sim_motion.rs:80:// RecordingTransport — synchronous recording stub for `kalico_load_curve` and
rust/motion-bridge/tests/sim_motion.rs:81:// `kalico_push_segment`. Returns canned successful responses.
rust/motion-bridge/tests/sim_motion.rs:843:    assert!(octopus.sent_starting_with("kalico_load_curve").is_empty());
rust/motion-bridge/tests/sim_motion.rs:844:    assert!(octopus.sent_starting_with("kalico_push_segment").is_empty());
rust/motion-bridge/tests/sim_motion.rs:87:    /// args. Populated for `kalico_load_curve` calls, `None` otherwise.
rust/motion-bridge/tests/sim_motion.rs:99:/// In-flight curve-upload state, populated by `kalico_load_curve_begin` +
rust/runtime/src/clock.rs:199:        // Helper is parametric over kalico_clock_freq. Sanity-check at the
rust/runtime/src/clock.rs:32:    /// `kalico_h7_disable_tim5()`, and passes that u64 value back at re-enable.
rust/runtime/src/engine.rs:137:    /// `kalico_clock_freq` static is read once at FFI init time and the value
rust/runtime/src/error.rs:176:    /// Cast to u16 for the `kalico_status` and `kalico_fault` wire formats
rust/runtime/src/reclaim.rs:120:/// fresh latch transition (so callers can emit a `kalico_fault` frame),
rust/runtime/src/sim_fixtures.rs:5://! firmware — the production `kalico_runtime_load_curve` path validates the
rust/runtime/src/state.rs:139:    /// the periodic 10 Hz `kalico_status_v6` frame and the async
rust/runtime/src/state.rs:140:    /// `kalico_fault` event so the host can decode the fault context
rust/runtime/src/state.rs:147:    // to the segment id from `kalico_stream_terminal`; the ISR retire path
rust/runtime/src/state.rs:16:// foreign symbol declarations for `kalico_clock_freq` / `irq_save` /
rust/runtime/src/state.rs:253:    static kalico_clock_freq: u32;
rust/runtime/src/state.rs:261:    /// `AtomicBool` guard on `kalico_runtime_init`). This function writes
rust/runtime/src/state.rs:291:            // Engine::new_production both see the same value. `kalico_clock_freq`
rust/runtime/src/state.rs:293:            let freq = core::ptr::read_volatile(core::ptr::addr_of!(kalico_clock_freq));
rust/runtime/src/state.rs:60:// `kalico_irq_save` / `kalico_irq_restore` defined in `src/runtime_tick.c`,
rust/runtime/src/state.rs:69:    pub fn kalico_irq_save() -> u32;
rust/runtime/src/state.rs:70:    pub fn kalico_irq_restore(flags: u32);
rust/runtime/src/state.rs:74:/// (`kalico_runtime_push_segment`, `kalico_runtime_load_curve`,
rust/runtime/src/state.rs:75:/// `kalico_runtime_drain_trace`, …).
rust/runtime/src/state.rs:89:    /// Set by §8.3 `kalico_stream_terminal` handler; consumed by the ISR
rust/runtime/src/stream.rs:113:/// `kalico_stream_arm` handler. §6.3 / §6.4 / §8.3 / §8.5.
rust/runtime/src/stream.rs:159:/// `kalico_stream_terminal` handler. §8.3 / §8.5.
rust/runtime/src/stream.rs:215:/// `kalico_clock_sync_request` handler. §12.1.
rust/runtime/src/stream.rs:256:/// `kalico_runtime_init`.
rust/runtime/src/stream.rs:26:/// payload so the host's `kalico_runtime_fault_detail` accessor (and the
rust/runtime/src/stream.rs:27:/// periodic `kalico_status_v6` frame's `fault_detail` column) can carry
rust/runtime/src/stream.rs:278:    fg.flush_start_tick = Some(unsafe { kalico_host_now_us() });
rust/runtime/src/stream.rs:281:    // timeout. Use `kalico_host_now_us` (Klipper's `timer_read_time` µs)
rust/runtime/src/stream.rs:285:    let deadline_us = unsafe { kalico_host_now_us() }.saturating_add(1000);
rust/runtime/src/stream.rs:288:        let now_us = unsafe { kalico_host_now_us() };
rust/runtime/src/stream.rs:317:    let irq_flags = unsafe { kalico_irq_save() };
rust/runtime/src/stream.rs:319:        // SAFETY: kalico_irq_save() above pins the ISR off; we transiently
rust/runtime/src/stream.rs:334:    unsafe { kalico_irq_restore(irq_flags) };
rust/runtime/src/stream.rs:61:    /// `kalico_clock_freq / 1_000_000`. Not ISR-safe in spirit (the
rust/runtime/src/stream.rs:64:    fn kalico_host_now_us() -> u64;
rust/runtime/src/stream.rs:67:// `kalico_irq_save` / `kalico_irq_restore` are declared in `state.rs` —
rust/runtime/src/stream.rs:70:use crate::state::{kalico_irq_restore, kalico_irq_save};
rust/runtime/src/stream.rs:74:/// `kalico_stream_open` handler. §8.3 / §8.5.
rust/runtime/tests/flush_basic.rs:25:// flush() imports `kalico_host_now_us` and `irq_save`/`irq_restore` from C.
rust/runtime/tests/flush_basic.rs:28:pub static kalico_clock_freq: u32 = 520_000_000;
rust/runtime/tests/flush_basic.rs:37:pub extern "C" fn kalico_host_now_us() -> u64 {
rust/runtime/tests/flush_basic.rs:42:pub extern "C" fn kalico_irq_save() -> u32 {
rust/runtime/tests/flush_basic.rs:47:pub extern "C" fn kalico_irq_restore(_flags: u32) {}
rust/runtime/tests/flush_drains_queue.rs:27:pub static kalico_clock_freq: u32 = 520_000_000;
rust/runtime/tests/flush_drains_queue.rs:32:pub extern "C" fn kalico_host_now_us() -> u64 {
rust/runtime/tests/flush_drains_queue.rs:37:pub extern "C" fn kalico_irq_save() -> u32 {
rust/runtime/tests/flush_drains_queue.rs:42:pub extern "C" fn kalico_irq_restore(_flags: u32) {}
rust/runtime/tests/flush_timeout.rs:25:pub static kalico_clock_freq: u32 = 520_000_000;
rust/runtime/tests/flush_timeout.rs:27:// Each call to `kalico_host_now_us` advances the counter by 100 µs. With
rust/runtime/tests/flush_timeout.rs:33:pub extern "C" fn kalico_host_now_us() -> u64 {
rust/runtime/tests/flush_timeout.rs:38:pub extern "C" fn kalico_irq_save() -> u32 {
rust/runtime/tests/flush_timeout.rs:43:pub extern "C" fn kalico_irq_restore(_flags: u32) {}
rust/runtime/tests/flush_timeout.rs:62:    // boundary set against `kalico_host_now_us`.
rust/runtime/tests/stream_lifecycle.rs:26:// stream::flush imports `kalico_host_now_us` (foreign symbol from
rust/runtime/tests/stream_lifecycle.rs:32:pub extern "C" fn kalico_host_now_us() -> u64 {
rust/runtime/tests/stream_lifecycle.rs:37:pub extern "C" fn kalico_irq_save() -> u32 {
rust/runtime/tests/stream_lifecycle.rs:42:pub extern "C" fn kalico_irq_restore(_flags: u32) {}
```

## DECL_COMMAND text strings (src/runtime_tick.c)
```
445:DECL_COMMAND(command_kalico_query_status, "kalico_query_status");
460:DECL_COMMAND(command_kalico_set_homed, "kalico_set_homed");
476:DECL_COMMAND(command_kalico_set_homed_state, "kalico_set_homed_state homed=%c");
587:DECL_COMMAND(command_kalico_disarm_endstop, "kalico_disarm_endstop arm_id=%u");
640:DECL_COMMAND(command_kalico_configure_axes, "kalico_configure_axes kinematics=%c");
661:DECL_COMMAND(command_kalico_stream_open, "kalico_stream_open stream_id=%u");
711:DECL_COMMAND(command_kalico_stream_flush, "kalico_stream_flush");
799:DECL_COMMAND(command_kalico_sim_diag, "kalico_sim_diag");
1079:DECL_COMMAND(command_kalico_bench_run, "kalico_bench_run isolate=%c samples=%hu");
```

## Klippy host-side references
```
klippy/extras/force_move.py:218:        # that kalico_runtime_push_segment is accepted (homed gate §7-B).
klippy/extras/homing.py:504:                    "kalico_set_homed_state failed on %s: %s", mcu_name, e
klippy/motion_bridge.py:160:        """Forward an MCU `kalico_credit_freed` event into the slot pool.
klippy/motion_bridge.py:358:        # the kalico_arm_endstop wire format expects (spec §3.1).
klippy/motion_bridge.py:398:        # Register an async handler for kalico_endstop_tripped before
klippy/motion_bridge.py:402:                self._on_trip_message, "kalico_endstop_tripped"
klippy/motion_bridge.py:434:                "kalico_arm_endstop rejected (status=%d)" % status
klippy/motion_toolhead.py:498:                "kalico_sim_stepper_count_query oid=%d" % oid,
klippy/motion_toolhead.py:499:                "kalico_sim_stepper_count_response",
klippy/motion_toolhead.py:520:                "kalico_sim_axis_steps_query oid=%d" % oid,
klippy/motion_toolhead.py:521:                "kalico_sim_axis_steps_response",
klippy/motion_toolhead.py:542:                "kalico_sim_axis_accum_query oid=%d" % oid,
klippy/motion_toolhead.py:543:                "kalico_sim_axis_accum_response",
klippy/motion_toolhead.py:558:        `command_kalico_sim_endstop_set_pin` injects a level into the
klippy/motion_toolhead.py:571:                "kalico_sim_endstop_set_pin gpio=%d level=%d" % (gpio, level),
klippy/motion_toolhead.py:572:                "kalico_sim_endstop_set_pin_response",
klippy/motion_toolhead.py:654:            # Register the kalico_credit_freed handler on every bridge-MCU's
klippy/motion_toolhead.py:757:        """Register a kalico_credit_freed handler on each bridge-attached MCU.
klippy/motion_toolhead.py:761:            MCU emits `kalico_credit_freed` (src/runtime_tick.c)
klippy/motion_toolhead.py:765:                 "kalico_credit_freed" and dispatches via self.handlers
klippy/motion_toolhead.py:779:                    "kalico_credit_freed handler not registered",
klippy/motion_toolhead.py:805:            serial.register_response(_on_credit_freed, "kalico_credit_freed")
klippy/motion_toolhead.py:807:                "MotionToolhead: registered kalico_credit_freed handler for "
klippy/serialhdl.py:101:                name = "kalico_status_v6"
klippy/serialhdl.py:109:                        "%s[bridge-async] kalico_status_v6 frame #%d engine_status=%s",
klippy/serialhdl.py:115:                name = "kalico_credit_freed"
klippy/serialhdl.py:117:                name = "kalico_fault"
klippy/serialhdl.py:119:                name = "kalico_endstop_tripped"
klippy/serialhdl.py:369:            # This is the inbound async path for kalico_status_v6 etc.
klippy/stepper.py:285:        # authoritative MCU step counter snapshot (from a kalico_endstop
scripts/ci-local.sh:151:        "grep -qF 'kalico_liveness_ok' '$ROOT/src/stm32/watchdog.c' && grep -qF 'CONFIG_KALICO_RUNTIME' '$ROOT/src/stm32/watchdog.c'"
tools/sim_klippy/printer.cfg:19:# by kalico_endstop_sample_pins; we drive its level via the
tools/sim_klippy/printer.cfg:20:# kalico_sim_endstop_set_pin shim from the test driver.
tools/sim_klippy/run.py:179:            if "kalico_status_v6" in line:
tools/sim_klippy/test_phase4_steps.py:7:Uses the bridge's bridge_call() to query kalico_sim_stepper_count_query oid=0
```

## External klipper-sim corpus (if reachable)
```
/Users/daniladergachev/Developer/klipper-sim/simulate_gcode.py:100:            "See scripts/kalico_quintic_patch.py for the known Kalico f6360e50 workaround."
/Users/daniladergachev/Developer/klipper-sim/klipsim/runner.py:72:    see scripts/ for known patches (e.g. scripts/kalico_quintic_patch.py).
/Users/daniladergachev/Developer/klipper-sim/scripts/kalico_quintic_patch.py:14:    python3 scripts/kalico_quintic_patch.py /path/to/klipper-root
/Users/daniladergachev/Developer/klipper-sim/scripts/kalico_quintic_patch.py:24:    python3 simulate_gcode.py ... --klippy-patch scripts/kalico_quintic_patch.py
```
