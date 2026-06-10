use std::collections::BTreeMap;

use kalico_protocol::messages::{
    SdoRead, SdoReadResponse, SdoWrite, SdoWriteResponse, ERR_SDO_UNSUPPORTED_SIZE,
    ERR_SDO_VALUE_RANGE, ERR_SDO_VERIFY_MISMATCH, SDO_SIZE_PROBE,
};

pub const MAX_SDO_BYTES: u8 = 4;
pub const COE_ABORT_READ_ONLY: i32 = 0x0601_0002_u32 as i32;
pub const COE_ABORT_NOT_FOUND: i32 = 0x0602_0000_u32 as i32;
pub const COE_ABORT_LENGTH_MISMATCH: i32 = 0x0607_0010_u32 as i32;

/// Errors are result codes: > 0 CoE abort code, < 0 local ERR_SDO_* constant.
/// `read` must only report sizes 1..=4 packed little-endian into the array
/// (a larger reported size means the object is unsupported, not truncated data).
pub trait SdoBus {
    fn read(&mut self, index: u16, subindex: u8) -> Result<(u8, [u8; 4]), i32>;
    fn write(&mut self, index: u16, subindex: u8, bytes: &[u8]) -> Result<(), i32>;
}

pub fn execute_sdo_read(bus: &mut dyn SdoBus, msg: &SdoRead) -> SdoReadResponse {
    match bus.read(msg.index, msg.subindex) {
        Ok((size, data)) => SdoReadResponse {
            result: 0,
            size,
            data,
        },
        Err(code) => SdoReadResponse {
            result: code,
            size: 0,
            data: [0; 4],
        },
    }
}

fn value_fits(value: i64, size: u8) -> bool {
    let bits = u32::from(size) * 8;
    let min = -(1i64 << (bits - 1));
    let max = (1i64 << bits) - 1;
    (min..=max).contains(&value)
}

fn encode_value(value: i64, size: u8) -> [u8; 4] {
    let le = value.to_le_bytes();
    let mut out = [0u8; 4];
    out[..usize::from(size)].copy_from_slice(&le[..usize::from(size)]);
    out
}

pub fn execute_sdo_write(bus: &mut dyn SdoBus, msg: &SdoWrite) -> SdoWriteResponse {
    let fail = |result| SdoWriteResponse {
        result,
        readback_size: 0,
        readback_data: [0; 4],
    };
    let size = if msg.size == SDO_SIZE_PROBE {
        match bus.read(msg.index, msg.subindex) {
            Ok((probed, _)) => probed,
            Err(code) => return fail(code),
        }
    } else {
        msg.size
    };
    if size == 0 || size > MAX_SDO_BYTES {
        return fail(ERR_SDO_UNSUPPORTED_SIZE);
    }
    if !value_fits(msg.value, size) {
        return fail(ERR_SDO_VALUE_RANGE);
    }
    let bytes = encode_value(msg.value, size);
    if let Err(code) = bus.write(msg.index, msg.subindex, &bytes[..usize::from(size)]) {
        return fail(code);
    }
    match bus.read(msg.index, msg.subindex) {
        Ok((rb_size, rb_data)) => {
            if rb_size == size && rb_data == bytes {
                SdoWriteResponse {
                    result: 0,
                    readback_size: size,
                    readback_data: bytes,
                }
            } else {
                SdoWriteResponse {
                    result: ERR_SDO_VERIFY_MISMATCH,
                    readback_size: rb_size,
                    readback_data: rb_data,
                }
            }
        }
        Err(code) => fail(code),
    }
}

pub struct DictObject {
    pub size: u8,
    pub value: [u8; 4],
    pub read_only: bool,
    pub clamp_max: Option<u32>,
}

/// In-memory object dictionary: the stub endpoint's fake drive, and the
/// unit-test bus. `read_count` exposes probe/verify traffic for assertions.
pub struct DictSdoBus {
    objects: BTreeMap<(u16, u8), DictObject>,
    pub read_count: u32,
}

impl DictSdoBus {
    pub fn new(objects: Vec<((u16, u8), DictObject)>) -> Self {
        Self {
            objects: objects.into_iter().collect(),
            read_count: 0,
        }
    }
}

impl SdoBus for DictSdoBus {
    fn read(&mut self, index: u16, subindex: u8) -> Result<(u8, [u8; 4]), i32> {
        self.read_count += 1;
        match self.objects.get(&(index, subindex)) {
            Some(o) => Ok((o.size, o.value)),
            None => Err(COE_ABORT_NOT_FOUND),
        }
    }

    fn write(&mut self, index: u16, subindex: u8, bytes: &[u8]) -> Result<(), i32> {
        let o = self
            .objects
            .get_mut(&(index, subindex))
            .ok_or(COE_ABORT_NOT_FOUND)?;
        if o.read_only {
            return Err(COE_ABORT_READ_ONLY);
        }
        if bytes.len() != usize::from(o.size) {
            return Err(COE_ABORT_LENGTH_MISMATCH);
        }
        let mut v = [0u8; 4];
        v[..bytes.len()].copy_from_slice(bytes);
        if let Some(max) = o.clamp_max {
            if u32::from_le_bytes(v) > max {
                v = max.to_le_bytes();
            }
        }
        o.value = v;
        Ok(())
    }
}

#[cfg(test)]
mod tests;
