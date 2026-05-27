use super::*;

#[test]
fn pack_unpack_round_trips() {
    let cfg = PhaseConfig {
        spi_bus_id: 0x03,
        cs_pin_id: 0x42,
    };
    assert_eq!(cfg.pack(), 0x0342);
    assert_eq!(PhaseConfig::unpack(0x0342), Some(cfg));
}

#[test]
fn sentinel_unpacks_to_none() {
    assert_eq!(PhaseConfig::unpack(NONE_SENTINEL), None);
}

#[test]
fn pack_distinct_from_sentinel_for_realistic_inputs() {
    // bus_id 0xFF is reserved as the sentinel marker; any legitimate
    // (bus, cs) where bus != 0xFF must pack to a non-sentinel value.
    let cfg = PhaseConfig {
        spi_bus_id: 0,
        cs_pin_id: 0xFF,
    };
    assert_ne!(cfg.pack(), NONE_SENTINEL);
    assert_eq!(PhaseConfig::unpack(cfg.pack()), Some(cfg));
}

#[test]
fn store_load_round_trip() {
    let slot = AtomicU16::new(NONE_SENTINEL);
    assert_eq!(load(&slot), None);
    let cfg = PhaseConfig {
        spi_bus_id: 1,
        cs_pin_id: 0x10,
    };
    store(&slot, Some(cfg));
    assert_eq!(load(&slot), Some(cfg));
    store(&slot, None);
    assert_eq!(load(&slot), None);
}
