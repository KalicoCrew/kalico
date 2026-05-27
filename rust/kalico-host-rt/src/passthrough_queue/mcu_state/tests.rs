use super::*;
use crate::passthrough_queue::entry::NotifyId;

fn entry(min_clock: u64, req_clock: u64) -> PassthroughEntry {
    PassthroughEntry::new(vec![0x01], min_clock, req_clock, NotifyId::none())
}

#[test]
fn allocates_distinct_command_queue_ids() {
    let mut state = McuState::new();
    let a = state.alloc_command_queue();
    let b = state.alloc_command_queue();
    assert_ne!(a, b);
}

#[test]
fn pop_picks_lowest_req_clock_across_queues() {
    let mut state = McuState::new();
    let qa = state.alloc_command_queue();
    let qb = state.alloc_command_queue();

    state.push(qa, entry(0, 200)).unwrap();
    state.push(qb, entry(0, 100)).unwrap();
    state.push(qa, entry(0, 150)).unwrap();

    assert_eq!(state.pop_next().unwrap().req_clock(), 100);
    assert_eq!(state.pop_next().unwrap().req_clock(), 150);
    assert_eq!(state.pop_next().unwrap().req_clock(), 200);
    assert!(state.pop_next().is_none());
}

#[test]
fn promote_runs_across_all_queues() {
    let mut state = McuState::new();
    let qa = state.alloc_command_queue();
    let qb = state.alloc_command_queue();

    state.push(qa, entry(10, 50)).unwrap();
    state.push(qb, entry(20, 40)).unwrap();

    // Nothing ready yet
    assert!(state.pop_next().is_none());

    state.promote_all(10);
    // Only qa's entry is ready
    assert_eq!(state.pop_next().unwrap().req_clock(), 50);
    assert!(state.pop_next().is_none());

    state.promote_all(20);
    assert_eq!(state.pop_next().unwrap().req_clock(), 40);
}

#[test]
fn push_to_unknown_queue_returns_error() {
    let mut state = McuState::new();
    let bogus = CommandQueueId(999);
    assert!(state.push(bogus, entry(0, 0)).is_err());
}

#[test]
fn background_entries_only_emitted_when_no_non_background_exist() {
    let mut state = McuState::new();
    let qa = state.alloc_command_queue();
    let qb = state.alloc_command_queue();

    // qa has a normal entry, qb has a background entry
    state.push(qa, entry(0, 200)).unwrap();
    state.push(qb, entry(0, BACKGROUND_PRIORITY_CLOCK)).unwrap();

    // Normal entry from qa should come first despite qb also having a
    // ready entry.
    assert_eq!(state.pop_next().unwrap().req_clock(), 200);

    // Now only the background entry remains — it should be emitted.
    let bg = state.pop_next().unwrap();
    assert!(bg.is_background_priority());
    assert!(state.pop_next().is_none());
}

#[test]
fn mixed_queues_normal_preferred_over_background() {
    let mut state = McuState::new();
    let qa = state.alloc_command_queue();
    let qb = state.alloc_command_queue();

    // qa: background, qb: normal
    state.push(qa, entry(0, BACKGROUND_PRIORITY_CLOCK)).unwrap();
    state.push(qb, entry(0, 100)).unwrap();
    state.push(qb, entry(0, 300)).unwrap();

    // Non-background entries first, then background.
    assert_eq!(state.pop_next().unwrap().req_clock(), 100);
    assert_eq!(state.pop_next().unwrap().req_clock(), 300);
    assert_eq!(
        state.pop_next().unwrap().req_clock(),
        BACKGROUND_PRIORITY_CLOCK
    );
}

#[test]
fn peek_next_req_clock_ignores_background_while_normal_exist() {
    let mut state = McuState::new();
    let qa = state.alloc_command_queue();
    let qb = state.alloc_command_queue();

    state.push(qa, entry(0, BACKGROUND_PRIORITY_CLOCK)).unwrap();
    state.push(qb, entry(0, 500)).unwrap();

    // peek should return the non-background value
    assert_eq!(state.peek_next_req_clock(), Some(500));
}
