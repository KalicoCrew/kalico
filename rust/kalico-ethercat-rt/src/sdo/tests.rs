use super::*;
use kalico_protocol::messages::{
    SdoRead, SdoWrite, ERR_SDO_UNSUPPORTED_SIZE, ERR_SDO_VALUE_RANGE, ERR_SDO_VERIFY_MISMATCH,
    SDO_SIZE_PROBE,
};

fn test_dict() -> DictSdoBus {
    DictSdoBus::new(vec![
        (
            (0x2002, 0),
            DictObject {
                size: 2,
                value: [100, 0, 0, 0],
                read_only: false,
                unsigned_clamp_max: None,
            },
        ),
        (
            (0x2003, 0),
            DictObject {
                size: 2,
                value: [0, 0, 0, 0],
                read_only: false,
                unsigned_clamp_max: Some(500),
            },
        ),
        (
            (0x2010, 1),
            DictObject {
                size: 4,
                value: [0; 4],
                read_only: false,
                unsigned_clamp_max: None,
            },
        ),
        (
            (0x6041, 0),
            DictObject {
                size: 2,
                value: [0x37, 0x02, 0, 0],
                read_only: true,
                unsigned_clamp_max: None,
            },
        ),
    ])
}

#[test]
fn read_returns_size_and_bytes() {
    let mut bus = test_dict();
    let resp = execute_sdo_read(
        &mut bus,
        &SdoRead {
            index: 0x2002,
            subindex: 0,
        },
    );
    assert_eq!(resp.result, 0);
    assert_eq!(resp.size, 2);
    assert_eq!(resp.data, [100, 0, 0, 0]);
}

#[test]
fn read_unknown_object_returns_abort_code() {
    let mut bus = test_dict();
    let resp = execute_sdo_read(
        &mut bus,
        &SdoRead {
            index: 0x7777,
            subindex: 0,
        },
    );
    assert_eq!(resp.result, COE_ABORT_NOT_FOUND);
}

#[test]
fn probed_write_discovers_size_writes_and_verifies() {
    let mut bus = test_dict();
    let resp = execute_sdo_write(
        &mut bus,
        &SdoWrite {
            index: 0x2002,
            subindex: 0,
            size: SDO_SIZE_PROBE,
            value: 250,
        },
    );
    assert_eq!(resp.result, 0);
    assert_eq!(resp.readback_size, 2);
    assert_eq!(resp.readback_data, [250, 0, 0, 0]);
    assert_eq!(bus.read_count, 2, "probe + verify");
}

#[test]
fn typed_write_skips_probe() {
    let mut bus = test_dict();
    let resp = execute_sdo_write(
        &mut bus,
        &SdoWrite {
            index: 0x2002,
            subindex: 0,
            size: 2,
            value: 250,
        },
    );
    assert_eq!(resp.result, 0);
    assert_eq!(bus.read_count, 1, "verify only — no probe");
}

#[test]
fn negative_value_encodes_twos_complement() {
    let mut bus = test_dict();
    let resp = execute_sdo_write(
        &mut bus,
        &SdoWrite {
            index: 0x2010,
            subindex: 1,
            size: 4,
            value: -4096,
        },
    );
    assert_eq!(resp.result, 0);
    assert_eq!(resp.readback_data, (-4096i32).to_le_bytes());
}

#[test]
fn value_exceeding_discovered_width_is_rejected_before_writing() {
    let mut bus = test_dict();
    let resp = execute_sdo_write(
        &mut bus,
        &SdoWrite {
            index: 0x2002,
            subindex: 0,
            size: SDO_SIZE_PROBE,
            value: 70_000,
        },
    );
    assert_eq!(resp.result, ERR_SDO_VALUE_RANGE);
    let after = execute_sdo_read(
        &mut bus,
        &SdoRead {
            index: 0x2002,
            subindex: 0,
        },
    );
    assert_eq!(after.data, [100, 0, 0, 0], "object must be untouched");
}

#[test]
fn clamped_write_reports_verify_mismatch_with_settled_bytes() {
    let mut bus = test_dict();
    let resp = execute_sdo_write(
        &mut bus,
        &SdoWrite {
            index: 0x2003,
            subindex: 0,
            size: 2,
            value: 600,
        },
    );
    assert_eq!(resp.result, ERR_SDO_VERIFY_MISMATCH);
    assert_eq!(resp.readback_size, 2);
    assert_eq!(
        resp.readback_data,
        [0xF4, 0x01, 0, 0],
        "drive settled on 500"
    );
}

#[test]
fn read_only_object_write_surfaces_abort_code() {
    let mut bus = test_dict();
    let resp = execute_sdo_write(
        &mut bus,
        &SdoWrite {
            index: 0x6041,
            subindex: 0,
            size: 2,
            value: 1,
        },
    );
    assert_eq!(resp.result, COE_ABORT_READ_ONLY);
}

#[test]
fn probe_reporting_oversized_object_is_rejected() {
    struct BigObjectBus;
    impl SdoBus for BigObjectBus {
        fn read(&mut self, _index: u16, _subindex: u8) -> Result<(u8, [u8; 4]), i32> {
            Ok((8, [0; 4]))
        }
        fn write(&mut self, _index: u16, _subindex: u8, _bytes: &[u8]) -> Result<(), i32> {
            panic!("must not write an unsupported-size object");
        }
    }
    let resp = execute_sdo_write(
        &mut BigObjectBus,
        &SdoWrite {
            index: 0x1008,
            subindex: 0,
            size: SDO_SIZE_PROBE,
            value: 1,
        },
    );
    assert_eq!(resp.result, ERR_SDO_UNSUPPORTED_SIZE);
}

#[test]
fn typed_write_to_unknown_object_surfaces_abort_code() {
    let mut bus = test_dict();
    let resp = execute_sdo_write(
        &mut bus,
        &SdoWrite {
            index: 0x7777,
            subindex: 0,
            size: 2,
            value: 1,
        },
    );
    assert_eq!(resp.result, COE_ABORT_NOT_FOUND);
    assert_eq!(resp.readback_size, 0);
    assert_eq!(resp.readback_data, [0; 4]);
}

#[test]
fn verify_read_failure_surfaces_its_code() {
    struct WriteOkReadFailBus;
    impl SdoBus for WriteOkReadFailBus {
        fn read(&mut self, _index: u16, _subindex: u8) -> Result<(u8, [u8; 4]), i32> {
            Err(-999)
        }
        fn write(&mut self, _index: u16, _subindex: u8, _bytes: &[u8]) -> Result<(), i32> {
            Ok(())
        }
    }
    let resp = execute_sdo_write(
        &mut WriteOkReadFailBus,
        &SdoWrite {
            index: 0x2002,
            subindex: 0,
            size: 2,
            value: 1,
        },
    );
    assert_eq!(resp.result, -999);
    assert_eq!(resp.readback_size, 0);
    assert_eq!(resp.readback_data, [0; 4]);
}
