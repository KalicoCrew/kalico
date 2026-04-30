//! Integration test for subscriber API. Requires firmware sim.
//! Gated behind the `hardware-test` feature — concrete impl expanded during
//! Phase F soak testing.
#![cfg(feature = "hardware-test")]

// Sketch:
// 1. Open KalicoHostIo against sim.
// 2. subscribe_fault() -> Ok(receiver)
// 3. Trigger fault via test command (sim emulates fault).
// 4. Assert receiver.recv() gets the FaultEvent.
// 5. Verify second subscribe_fault() returns AlreadySubscribed.
