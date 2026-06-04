use std::collections::HashMap;
use std::sync::{Condvar, Mutex};
use std::time::{Duration, Instant};

type AxisKey = (u32, u8);

#[derive(Default)]
struct Counts {
    sent: HashMap<AxisKey, u32>,
    retired: HashMap<AxisKey, u32>,
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
        c.retired.clear();
        drop(c);
        self.cv.notify_all();
    }

    fn is_drained(c: &Counts) -> bool {
        c.sent
            .iter()
            .all(|(k, &s)| c.retired.get(k).copied().unwrap_or(0) == s)
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
                    .filter(|(k, s)| c.retired.get(*k).copied().unwrap_or(0) != **s)
                    .map(|(k, s)| {
                        let r = c.retired.get(k).copied().unwrap_or(0);
                        format!("mcu{} axis{}: retired {} / sent {}", k.0, k.1, r, s)
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

    #[test]
    fn reset_clears_both_sides() {
        let d = DrainSync::new();
        d.add_sent(1, 0, 5);
        d.reset();
        assert!(d.wait_drained(Duration::from_millis(20)).is_ok());
    }
}
