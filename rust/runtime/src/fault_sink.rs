/// Sink for hard motion faults raised by the shared walker on the real-time
/// path.
///
/// # Contract
///
/// Implementations run on the real-time path (MCU ISR or EtherCAT DC loop)
/// and MUST be allocation-free and non-blocking. Implementations MUST
/// preserve the detail-before-code store ordering documented on
/// `raise_piece_start_in_past`: the fault detail word must be written with
/// `Release` semantics before the fault code word, so that a foreground
/// reader observing a non-zero `last_error` is guaranteed to see the
/// associated `fault_detail`.
pub trait FaultSink {
    /// A piece was adopted whose start is >2 ticks in the past.
    ///
    /// `deficit_us` is how many microseconds the adopted piece's start is
    /// behind `now` at the moment of adoption. The MCU sink saturates large
    /// values to 0xFFFF internally; sinks without a detail channel may ignore
    /// it.
    ///
    /// Implementors MUST preserve the detail-before-code store ordering
    /// documented on `raise_piece_start_in_past`.
    fn piece_start_in_past(&self, axis_idx: usize, deficit_us: u32);
}
