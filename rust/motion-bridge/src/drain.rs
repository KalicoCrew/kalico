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

    pub fn unsend(&self, mcu: u32, axis: u8, n: u32) {
        let mut c = self.counts.lock().unwrap_or_else(|p| p.into_inner());
        let e = c.sent.entry((mcu, axis)).or_insert(0);
        *e = e.wrapping_sub(n);
        drop(c);
        self.cv.notify_all();
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

    pub fn is_drained_now(&self) -> bool {
        let c = self.counts.lock().unwrap_or_else(|p| p.into_inner());
        Self::is_drained(&c)
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
                            k.0,
                            k.1,
                            r,
                            b,
                            r.saturating_sub(b),
                            s
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
mod tests;
