//! `retired` is the MCU's raw cumulative counter (monotonic per boot); `sent`
//! is per-stream. `reset()` snapshots `retired` into `baseline` so drained
//! comparisons (`retired - baseline == sent`) survive multi-stream sessions.

use std::collections::HashMap;
use std::sync::{Condvar, Mutex};
use std::time::{Duration, Instant};

type AxisKey = (u32, u8);

#[derive(Default)]
struct Counts {
    sent: HashMap<AxisKey, u32>,
    retired: HashMap<AxisKey, u32>,
    baseline: HashMap<AxisKey, u32>,
}

pub struct DrainSync {
    counts: Mutex<Counts>,
    cv: Condvar,
}

impl DrainSync {
    pub fn new() -> Self {
        Self {
            counts: Mutex::new(Counts::default()),
            cv: Condvar::new(),
        }
    }

    pub fn add_sent(&self, mcu: u32, axis: u8, n: u32) {
        let mut c = self.counts.lock().unwrap_or_else(|p| p.into_inner());
        let e = c.sent.entry((mcu, axis)).or_insert(0);
        *e = e.wrapping_add(n);
    }

    pub fn set_retired(&self, mcu: u32, axis: u8, retired: u32) {
        let mut c = self.counts.lock().unwrap_or_else(|p| p.into_inner());
        c.retired.insert((mcu, axis), retired);
        drop(c);
        self.cv.notify_all();
    }

    pub fn reset(&self) {
        let mut c = self.counts.lock().unwrap_or_else(|p| p.into_inner());
        c.sent.clear();
        let snapshot: Vec<(AxisKey, u32)> = c.retired.iter().map(|(&k, &v)| (k, v)).collect();
        for (k, v) in snapshot {
            c.baseline.insert(k, v);
        }
        drop(c);
        self.cv.notify_all();
    }

    fn is_drained(c: &Counts) -> bool {
        c.sent.iter().all(|(k, &s)| {
            let r = c.retired.get(k).copied().unwrap_or(0);
            let b = c.baseline.get(k).copied().unwrap_or(0);
            r.saturating_sub(b) == s
        })
    }

    pub fn wait_drained(&self, timeout: Duration) -> Result<(), String> {
        let deadline = Instant::now() + timeout;
        let mut c = self.counts.lock().unwrap_or_else(|p| p.into_inner());
        while !Self::is_drained(&c) {
            let now = Instant::now();
            if now >= deadline {
                let lagging: Vec<String> = c
                    .sent
                    .iter()
                    .filter(|(k, s)| {
                        let r = c.retired.get(*k).copied().unwrap_or(0);
                        let b = c.baseline.get(*k).copied().unwrap_or(0);
                        r.saturating_sub(b) != **s
                    })
                    .map(|(k, s)| {
                        let r = c.retired.get(k).copied().unwrap_or(0);
                        let b = c.baseline.get(k).copied().unwrap_or(0);
                        format!(
                            "mcu{} axis{}: retired {} baseline {} delta {} / sent {}",
                            k.0, k.1, r, b, r.saturating_sub(b), s
                        )
                    })
                    .collect();
                return Err(format!(
                    "motion drain timed out after {:?}; not finished: [{}]",
                    timeout,
                    lagging.join(", ")
                ));
            }
            let (guard, _) = self
                .cv
                .wait_timeout(c, deadline - now)
                .unwrap_or_else(|p| p.into_inner());
            c = guard;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn drained_when_retired_equals_sent() {
        let d = DrainSync::new();
        d.add_sent(1, 0, 3);
        d.add_sent(1, 1, 2);
        assert!(d.wait_drained(Duration::from_millis(20)).is_err());
        d.set_retired(1, 0, 3);
        d.set_retired(1, 1, 2);
        assert!(d.wait_drained(Duration::from_millis(20)).is_ok());
    }

    #[test]
    fn no_sent_is_trivially_drained() {
        let d = DrainSync::new();
        assert!(d.wait_drained(Duration::from_millis(20)).is_ok());
    }

    /// After reset, `sent` is cleared, so even with no new activity the stream
    /// is trivially drained — even when the heartbeat keeps delivering the old
    /// cumulative retired value.
    #[test]
    fn reset_clears_sent_cumulative_retired_does_not_break_drain() {
        let d = DrainSync::new();
        d.add_sent(1, 0, 5);
        d.set_retired(1, 0, 5);
        assert!(d.wait_drained(Duration::from_millis(20)).is_ok());
        d.reset();
        // Heartbeat arrives again with the same cumulative value; no new sent.
        d.set_retired(1, 0, 5);
        assert!(d.wait_drained(Duration::from_millis(20)).is_ok());
    }

    /// Regression: exact bench numbers from the failing session.
    ///
    /// Stream 1: 8737 sent, retired reaches 8737 → drained.
    /// reset() → baseline is set to 8737.
    /// Heartbeat fires with cumulative 8737 (no new MCU work yet) → still trivially drained.
    /// Stream 2: 8737 more sent; MCU retires them: cumulative reaches 17474 → drained.
    #[test]
    fn regression_cumulative_retired_across_two_streams() {
        let d = DrainSync::new();

        // Stream 1.
        d.add_sent(0, 0, 8737);
        d.set_retired(0, 0, 8737);
        assert!(
            d.wait_drained(Duration::from_millis(20)).is_ok(),
            "stream 1 should be drained"
        );

        d.reset();

        // Heartbeat repeats the cumulative value before any new pieces are sent.
        d.set_retired(0, 0, 8737);
        assert!(
            d.wait_drained(Duration::from_millis(20)).is_ok(),
            "after reset with no new sent, trivially drained"
        );

        // Stream 2: same number of pieces again.
        d.add_sent(0, 0, 8737);
        // MCU hasn't retired them yet — not drained.
        assert!(
            d.wait_drained(Duration::from_millis(20)).is_err(),
            "stream 2 not yet drained before MCU catches up"
        );
        // MCU retires stream 2: cumulative becomes 17474.
        d.set_retired(0, 0, 17474);
        assert!(
            d.wait_drained(Duration::from_millis(20)).is_ok(),
            "stream 2 should be drained at cumulative 17474"
        );
    }

    /// After reset, a cumulative retired value that is ahead of baseline must
    /// NOT satisfy a new stream's sent unless the delta actually covers it.
    #[test]
    fn post_reset_partial_retired_not_drained() {
        let d = DrainSync::new();

        // Stream 1: drain cleanly.
        d.add_sent(0, 1, 86);
        d.set_retired(0, 1, 86);
        assert!(d.wait_drained(Duration::from_millis(20)).is_ok());

        d.reset();

        // Stream 2: send 100 more; MCU heartbeat still at old cumulative 86.
        d.add_sent(0, 1, 100);
        d.set_retired(0, 1, 86); // delta = 0 — not drained
        assert!(
            d.wait_drained(Duration::from_millis(20)).is_err(),
            "delta 0 against sent 100 must not be drained"
        );
        // MCU retires 50 of the 100 new pieces: cumulative = 136.
        d.set_retired(0, 1, 136); // delta = 50 — still not drained
        assert!(
            d.wait_drained(Duration::from_millis(20)).is_err(),
            "delta 50 against sent 100 must not be drained"
        );
        // Full 100 retired: cumulative = 186.
        d.set_retired(0, 1, 186); // delta = 100 — drained
        assert!(
            d.wait_drained(Duration::from_millis(20)).is_ok(),
            "delta 100 against sent 100 must be drained"
        );
    }

    /// Multi-axis: all axes must be drained before wait_drained returns Ok.
    #[test]
    fn multi_axis_all_must_drain() {
        let d = DrainSync::new();
        d.add_sent(0, 0, 17474);
        d.add_sent(1, 2, 86);

        d.set_retired(0, 0, 17474);
        assert!(
            d.wait_drained(Duration::from_millis(20)).is_err(),
            "axis (1,2) still pending"
        );
        d.set_retired(1, 2, 86);
        assert!(
            d.wait_drained(Duration::from_millis(20)).is_ok(),
            "all axes drained"
        );
    }
}
