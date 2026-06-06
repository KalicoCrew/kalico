use std::sync::Arc;

use arc_swap::ArcSwap;

pub const UNBOUND_SESSION: &str = "__unbound__";

#[derive(Debug, Clone)]
pub struct SessionContext {
    pub session_id: String,
    pub print_id: String,
}

impl Default for SessionContext {
    fn default() -> Self {
        SessionContext {
            session_id: UNBOUND_SESSION.to_string(),
            print_id: String::new(),
        }
    }
}

fn global() -> &'static ArcSwap<SessionContext> {
    use std::sync::OnceLock;
    static CTX: OnceLock<ArcSwap<SessionContext>> = OnceLock::new();
    CTX.get_or_init(|| ArcSwap::from_pointee(SessionContext::default()))
}

pub fn set_context(session_id: String, print_id: String) {
    global().store(Arc::new(SessionContext {
        session_id,
        print_id,
    }));
}

pub fn load_context() -> Arc<SessionContext> {
    global().load_full()
}

#[cfg(test)]
mod tests;
