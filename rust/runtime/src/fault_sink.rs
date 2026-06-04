/// Sink for hard motion faults raised by the shared walker on the real-time path.
///
/// Implementations run on the real-time path (MCU ISR or EtherCAT DC loop)
/// and MUST be allocation-free and non-blocking. The detail word must be written
/// with `Release` semantics before the fault code word so that a foreground reader
/// observing a non-zero `last_error` is guaranteed to see the associated
/// `fault_detail`.
pub trait FaultSink {
    fn piece_start_in_past(&self, axis_idx: usize, deficit_us: u32);
}
