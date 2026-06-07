pub struct Participant {
    pub last_status_time: f64,
    pub expire_time: f64,
}

pub struct ExtensionEngine {
    participants: Vec<Participant>,
    expire_secs: f64,
    min_extend_secs: f64,
}

impl ExtensionEngine {
    pub fn new(
        participants: Vec<Participant>,
        expire_secs: f64,
        min_extend_secs: f64,
    ) -> Self {
        Self { participants, expire_secs, min_extend_secs }
    }

    /// Record a `can_trigger=1` report from `idx` at host time `status_time`
    /// and return the `(participant_idx, new_expire_time)` set_timeout sends
    /// due. Mirrors trdispatch: each participant's anchor is the minimum
    /// status time among the OTHERS (the minimum-holder anchors to the
    /// second minimum), so no participant ever extends itself. Sends are
    /// suppressed unless the expire advances by at least `min_extend_secs`.
    /// Every sent participant's `expire_time` is persisted on send (C-faithful
    /// hysteresis dedup: a silent MCU is not re-spammed with the same expire).
    pub fn on_report(&mut self, idx: usize, status_time: f64) -> Vec<(usize, f64)> {
        self.participants[idx].last_status_time = status_time;

        let mut min_time = f64::INFINITY;
        let mut next_min_time = f64::INFINITY;
        let mut min_idx = usize::MAX;
        for (i, p) in self.participants.iter().enumerate() {
            let t = p.last_status_time;
            if t < next_min_time {
                next_min_time = t;
                if t < min_time {
                    next_min_time = min_time;
                    min_time = t;
                    min_idx = i;
                }
            }
        }
        if next_min_time == f64::INFINITY {
            next_min_time = min_time;
        }

        let mut sends = Vec::new();
        for (i, p) in self.participants.iter_mut().enumerate() {
            let anchor = if i == min_idx { next_min_time } else { min_time };
            let expire = anchor + self.expire_secs;
            if expire - p.expire_time >= self.min_extend_secs && expire > p.expire_time {
                p.expire_time = expire;
                sends.push((i, expire));
            }
        }
        sends
    }
}

/// Reconstruct a full 64-bit clock from a 32-bit report value, anchored to
/// the projected 64-bit now (mainline `clock_from_clock32`).
pub fn clock32_to_64(now64: u64, clock32: u32) -> u64 {
    let delta = clock32.wrapping_sub(now64 as u32) as i32;
    now64.wrapping_add(delta as i64 as u64)
}

pub fn ticks_to_host_time(ticks: u64, now_ticks: u64, host_now: f64, freq: f64) -> f64 {
    host_now + (ticks as i64 - now_ticks as i64) as f64 / freq
}

pub fn host_time_to_ticks(t: f64, now_ticks: u64, host_now: f64, freq: f64) -> u64 {
    let delta = (t - host_now) * freq;
    (now_ticks as i64 + delta.round() as i64) as u64
}
