use runtime::stepping_state::{StepperBindingRust, TMC_CS_OID_NONE};

#[test]
fn binding_size_is_four() {
    assert_eq!(core::mem::size_of::<StepperBindingRust>(), 4);
}

#[test]
fn tmc_cs_oid_none_sentinel() {
    let b = StepperBindingRust { tmc_cs_oid: TMC_CS_OID_NONE, _pad: [0; 3] };
    assert_eq!(b.tmc_cs_oid, 0xFF);
}

#[test]
fn tmc_cs_oid_zero_is_valid() {
    // OID 0 is a real SPI device OID and must NOT be treated as "no TMC."
    let b = StepperBindingRust { tmc_cs_oid: 0, _pad: [0; 3] };
    assert_ne!(b.tmc_cs_oid, TMC_CS_OID_NONE);
}
