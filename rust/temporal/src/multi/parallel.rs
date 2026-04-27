//! 3-thread fan-out for re-solving dirty segments. Per spec §2.6.
//!
//! TODO(task-8): real implementation lands in Task 8. This is a
//! stub that returns Ok(()) without doing any work; only used by
//! Task 6's joining-loop wiring + Task 6's unit tests, which don't
//! exercise the re-solve path.

use crate::multi::joining::SegmentState;
use crate::multi::{BatchError, SegmentInput};
use crate::GridConfig;

#[allow(dead_code, clippy::needless_pass_by_ref_mut, clippy::unnecessary_wraps)]
// TODO(task-8): replace stub with real implementation.
pub(crate) fn fan_out_solves(
    _inputs: &[SegmentInput<'_>],
    _states: &mut [SegmentState],
    _grids: &[GridConfig],
    _n_threads: usize,
) -> Result<(), BatchError> {
    Ok(())
}
