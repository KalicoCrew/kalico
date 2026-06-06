use super::result_codes;

#[test]
fn result_codes_are_stable() {
    assert_eq!(result_codes::OK, 0);
    assert_eq!(result_codes::RING_FULL, -309); // KALICO_ERR_RING_FULL (error.rs:126)
    assert_eq!(result_codes::INVALID_ARG, -26); // KALICO_ERR_INVALID_ARG (error.rs:92)
    assert_ne!(result_codes::RING_FULL, result_codes::INVALID_ARG);
}
