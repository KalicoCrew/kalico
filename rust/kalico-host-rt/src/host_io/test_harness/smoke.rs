use super::*;

#[test]
fn empty_tick_changes_nothing() {
    let mut h = ReactorHarness::new();
    let outcome = h.tick();
    assert_eq!(outcome, TickOutcome::Continue);
    assert_eq!(h.unacked_depth(), 0);
    assert_eq!(h.awaiting_depth(), 0);
    assert!(h.tx_log().is_empty());
}

#[test]
fn reactor_first_bridge_call_after_identify_succeeds_with_nonzero_initial_seq() {
    // Spec §3.3, §5.2 — H7 regression. Pre-refactor the reactor hardcoded
    // send_seq:1 / receive_seq:1, so any post-identify state where the
    // host had already burned ≥1 sequences would put a stale seq=1 on the
    // wire. Firmware that already advanced past seq=1 ignores it; first
    // bridge_call hangs until host-side timeout.
    //
    // With IdentifySeqState plumbing, the reactor adopts the post-identify
    // counters and the next outbound frame carries the correct seq nibble.
    let mut h = ReactorHarness::new_with_seq_state(IdentifySeqState {
        next_send_seq_abs: 5,
        mcu_receive_seq_abs: 5,
    });

    // Sanity: the public send_seq accessor reflects the adopted state
    // *before* any frame goes out.
    assert_eq!(
        h.send_seq(),
        5,
        "reactor must adopt next_send_seq_abs from identify"
    );

    let deadline = Instant::now() + Duration::from_secs(1);
    let _completion = h.submit_via_dispatch(42, vec![0x01], "noop", deadline);

    let written = h.tx_log();
    assert!(!written.is_empty(), "reactor should have written a frame");
    // Frame layout (see wire::build_frame): [len][seq|DEST][payload..][crc_hi][crc_lo][SYNC]
    let seq_byte = written[1];
    let wire_seq = seq_byte & 0x0F;
    assert_eq!(
        wire_seq, 5,
        "first frame after identify must carry seq=5 (= next_send_seq_abs mod 16), not seq=1",
    );
    // And send_seq must have advanced past it.
    assert_eq!(h.send_seq(), 6, "send_seq must increment after dispatch");
}

#[test]
fn clock_advance_is_visible_to_reactor() {
    let h = ReactorHarness::new();
    let t0 = h.reactor.clock.now();
    h.advance_clock(Duration::from_secs(1));
    let t1 = h.reactor.clock.now();
    assert_eq!(t1 - t0, Duration::from_secs(1));
}
