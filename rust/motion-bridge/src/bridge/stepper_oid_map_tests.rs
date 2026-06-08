use super::*;

#[test]
fn register_and_resolve_oid_slot() {
    let b = PyMotionBridge::new_for_test();
    b.register_stepper_slot(7, 12, 1).unwrap();
    let map = b.stepper_oid_map.lock().unwrap_or_else(|p| p.into_inner());
    assert_eq!(map.get(&(7u32, 12u8)).copied(), Some(1u8));
}
