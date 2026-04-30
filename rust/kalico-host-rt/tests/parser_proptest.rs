//! Parser encode/decode round-trip property test. Spec §4.13. Always-on.

use proptest::prelude::*;
use kalico_host_rt::host_io::parser::{encode_vlq, decode_vlq};

proptest! {
    #[test]
    fn vlq_round_trip(v in i32::MIN..=i32::MAX) {
        let mut buf = Vec::new();
        encode_vlq(&mut buf, i64::from(v)).unwrap();
        let (decoded, _) = decode_vlq(&buf).unwrap();
        prop_assert_eq!(decoded as i32, v);
    }

    #[test]
    fn u32_round_trip(v in 0u32..=u32::MAX) {
        let mut buf = Vec::new();
        encode_vlq(&mut buf, i64::from(v)).unwrap();
        let (decoded, _) = decode_vlq(&buf).unwrap();
        prop_assert_eq!(decoded as u32, v);
    }
}
