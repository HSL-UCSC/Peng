use std::sync::{
    Arc,
    atomic::{AtomicBool, AtomicU64},
};
use tokio::sync::Barrier;

pub struct WorkerSync {
    pub clock: Arc<AtomicU64>,
    pub start_barrier: Arc<Barrier>,
    pub end_barrier: Arc<Barrier>,
    pub kill: Arc<AtomicBool>,
}
