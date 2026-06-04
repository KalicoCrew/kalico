use kalico_host_rt::host_io::parser::{decode_vlq, encode_vlq};
use proptest::prelude::*;

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
