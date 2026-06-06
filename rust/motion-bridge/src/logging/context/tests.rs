use super::*;
use crate::logging::CONTEXT_TEST_LOCK;

#[test]
fn defaults_to_unbound_sentinel() {
    let c = SessionContext::default();
    assert_eq!(c.session_id, "__unbound__");
    assert_eq!(c.print_id, "");
}

#[test]
fn set_load_and_clear_sequence() {
    let _guard = CONTEXT_TEST_LOCK.lock().unwrap();

    set_context(
        "k-1748700131-4412".to_string(),
        "print-1748700500".to_string(),
    );
    let c = load_context();
    assert_eq!(c.session_id, "k-1748700131-4412");
    assert_eq!(c.print_id, "print-1748700500");

    set_context("k-1".to_string(), "print-x".to_string());
    set_context("k-1".to_string(), String::new());
    assert_eq!(load_context().print_id, "");
}

#[test]
fn arc_swap_concurrent_coherence() {
    let _guard = CONTEXT_TEST_LOCK.lock().unwrap();

    const WRITER_ITERS: usize = 50_000;
    const READER_ITERS: usize = 100_000;

    let writer = std::thread::spawn(|| {
        for i in 0..WRITER_ITERS {
            if i % 2 == 0 {
                set_context("k-AAA".to_string(), "print-AAA".to_string());
            } else {
                set_context("k-BBB".to_string(), "print-BBB".to_string());
            }
        }
    });

    for _ in 0..READER_ITERS {
        let ctx = load_context();
        let coherent = (ctx.session_id == "k-AAA" && ctx.print_id == "print-AAA")
            || (ctx.session_id == "k-BBB" && ctx.print_id == "print-BBB")
            || ctx.session_id != "k-AAA" && ctx.session_id != "k-BBB";
        assert!(
            coherent,
            "torn read detected: session_id={:?} print_id={:?}",
            ctx.session_id, ctx.print_id
        );
    }

    writer.join().expect("writer thread must not panic");
}
