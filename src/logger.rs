use colored::Colorize;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;
use std::time::{SystemTime, UNIX_EPOCH};

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub enum LogLevel {
    Error,
    Warn,
    Info,
    Debug,
}

pub struct PrintLogger {
    level: Arc<AtomicUsize>,
}

impl PrintLogger {
    pub fn new(level: LogLevel) -> Self {
        Self {
            level: Arc::new(AtomicUsize::new(level as usize)),
        }
    }

    pub fn set_level(&self, level: LogLevel) {
        self.level.store(level as usize, Ordering::SeqCst);
    }

    pub fn log(&self, level: LogLevel, message: &str) {
        if level.clone() as usize <= self.level.load(Ordering::SeqCst) {
            let timestamp = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .expect("Time went backwards")
                .as_secs();
            let string = format!("[{:?}] [{}] {}", level, timestamp, message);
            match level {
                LogLevel::Error => println!("{}", string.red()),
                LogLevel::Warn => println!("{}", string.yellow()),
                LogLevel::Info => println!("{}", string.blue()),
                LogLevel::Debug => println!("{}", string.cyan()),
            };
        }
    }
}

// Macros for logging
#[macro_export]
macro_rules! error {
    ($logger:expr, $($arg:tt)*) => {
        $logger.log(logger::LogLevel::Error, &format!($($arg)*));
    };
}

#[macro_export]
macro_rules! warn {
    ($logger:expr, $($arg:tt)*) => {
        $logger.log(logger::LogLevel::Warn, &format!($($arg)*));
    };
}

#[macro_export]
macro_rules! info {
    ($logger:expr, $($arg:tt)*) => {
        $logger.log(logger::LogLevel::Info, &format!($($arg)*));
    };
}

#[macro_export]
macro_rules! debug {
    ($logger:expr, $($arg:tt)*) => {
        $logger.log(LogLevel::Debug, &format!($($arg)*));
    };
}
