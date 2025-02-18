use std::sync::{
    atomic::{AtomicBool, AtomicU64},
    Arc,
};
use tokio::sync::Barrier;

pub struct WorkerSync {
    pub clock: Arc<AtomicU64>,
    pub start_barrier: Arc<Barrier>,
    pub end_barrier: Arc<Barrier>,
    pub kill: Arc<AtomicBool>,
}
