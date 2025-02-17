use std::sync::{
    atomic::{AtomicBool, AtomicU64, Ordering},
    Arc,
};
use tokio::sync::{Barrier, Mutex, Notify};

pub struct WorkerSync {
    pub notify: Arc<Notify>,
    pub clock: Arc<AtomicU64>,
    pub start_barrier: Arc<Barrier>,
    pub end_barrier: Arc<Barrier>,
    pub kill: Arc<AtomicBool>,
}
