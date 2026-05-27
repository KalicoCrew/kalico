//! C-backed segment SPSC queue stub — Task 5 placeholder.
//!
//! The real C-backed SPSC has been removed. These stubs are zero-sized
//! markers so `FgState` and `IsrState` compile until Task 6.

use core::marker::PhantomData;

/// Reset the C-side queue. Stub no-op.
#[allow(clippy::unused_self)]
pub fn reset() {}

/// Producer side (zero-sized marker).
#[derive(Debug)]
pub struct Producer<T>(PhantomData<T>);

impl<T> Producer<T> {
    pub fn new() -> Self {
        Self(PhantomData)
    }

    /// Enqueue — always returns `Err(item)` (stub; real queue is gone).
    #[allow(clippy::unused_self)]
    pub fn enqueue(&mut self, item: T) -> Result<(), T> {
        Err(item)
    }
}

impl<T> Default for Producer<T> {
    fn default() -> Self {
        Self::new()
    }
}

/// Consumer side (zero-sized marker).
#[derive(Debug)]
pub struct Consumer<T>(PhantomData<T>);

impl<T> Consumer<T> {
    pub fn new() -> Self {
        Self(PhantomData)
    }

    /// Dequeue — always returns `None` (stub; real queue is gone).
    #[allow(clippy::unused_self)]
    pub fn dequeue(&mut self) -> Option<T> {
        None
    }

    /// Length — always returns 0 (stub; real queue is gone).
    #[allow(clippy::unused_self)]
    pub fn len(&self) -> usize {
        0
    }
}

impl<T> Default for Consumer<T> {
    fn default() -> Self {
        Self::new()
    }
}
