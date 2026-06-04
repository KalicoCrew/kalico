pub const BACKGROUND_PRIORITY_CLOCK: u64 = u64::MAX;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct NotifyId(u64);

impl NotifyId {
    pub const fn none() -> Self {
        Self(0)
    }

    pub const fn new(id: u64) -> Self {
        Self(id)
    }

    pub fn is_none(&self) -> bool {
        self.0 == 0
    }

    pub fn raw(&self) -> u64 {
        self.0
    }
}

#[derive(Debug, Clone)]
pub struct PassthroughEntry {
    bytes: Vec<u8>,
    min_clock: u64,
    req_clock: u64,
    notify_id: NotifyId,
}

impl PassthroughEntry {
    pub fn new(bytes: Vec<u8>, min_clock: u64, req_clock: u64, notify_id: NotifyId) -> Self {
        Self {
            bytes,
            min_clock,
            req_clock,
            notify_id,
        }
    }

    pub fn bytes(&self) -> &[u8] {
        &self.bytes
    }

    pub fn min_clock(&self) -> u64 {
        self.min_clock
    }

    pub fn req_clock(&self) -> u64 {
        self.req_clock
    }

    pub fn notify_id(&self) -> NotifyId {
        self.notify_id
    }

    pub fn is_background_priority(&self) -> bool {
        self.req_clock == BACKGROUND_PRIORITY_CLOCK
    }
}

#[cfg(test)]
mod tests;
