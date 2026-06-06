use super::*;

#[test]
fn double_init_is_idempotent() {
    let dir = std::env::temp_dir().join(format!("kalico-init-test-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let first = init_logging(&dir);
    let second = init_logging(&dir);
    assert!(matches!(second, Ok(())));
    let _ = first;
}
