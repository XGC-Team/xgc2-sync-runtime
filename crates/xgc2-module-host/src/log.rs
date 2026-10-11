//! Line logger on stderr. Modules log through `HostApi::log`; the host logs lifecycle
//! events, never individual steps.

use std::sync::atomic::{AtomicU8, Ordering};
use std::time::{SystemTime, UNIX_EPOCH};

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Level {
    Debug = 0,
    Info = 1,
    Warn = 2,
    Error = 3,
}

impl Level {
    pub fn from_abi(level: i32) -> Level {
        match level {
            i32::MIN..=0 => Level::Debug,
            1 => Level::Info,
            2 => Level::Warn,
            _ => Level::Error,
        }
    }

    pub fn parse(text: &str) -> Option<Level> {
        match text {
            "debug" => Some(Level::Debug),
            "info" => Some(Level::Info),
            "warn" => Some(Level::Warn),
            "error" => Some(Level::Error),
            _ => None,
        }
    }

    fn label(self) -> &'static str {
        match self {
            Level::Debug => "DEBUG",
            Level::Info => "INFO",
            Level::Warn => "WARN",
            Level::Error => "ERROR",
        }
    }
}

static THRESHOLD: AtomicU8 = AtomicU8::new(Level::Info as u8);

pub fn set_level(level: Level) {
    THRESHOLD.store(level as u8, Ordering::Relaxed);
}

pub fn enabled(level: Level) -> bool {
    level as u8 >= THRESHOLD.load(Ordering::Relaxed)
}

/// Days since 1970-01-01 to (year, month, day), proleptic Gregorian (Hinnant's algorithm).
fn civil(days: i64) -> (i64, u32, u32) {
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let month = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    (yoe + era * 400 + i64::from(month <= 2), month, day)
}

fn timestamp() -> String {
    let now = SystemTime::now().duration_since(UNIX_EPOCH).unwrap_or_default();
    let secs = now.as_secs() as i64;
    let (year, month, day) = civil(secs.div_euclid(86_400));
    let rest = secs.rem_euclid(86_400);
    format!("{year:04}-{month:02}-{day:02}T{:02}:{:02}:{:02}.{:03}Z", rest / 3600, rest % 3600 / 60, rest % 60, now.subsec_millis())
}

pub fn emit(level: Level, target: &str, message: &str) {
    if enabled(level) {
        eprintln!("{} {:<5} {}: {}", timestamp(), level.label(), target, message);
    }
}

#[macro_export]
macro_rules! log_at {
    ($level:expr, $target:expr, $($arg:tt)*) => {
        if $crate::log::enabled($level) {
            $crate::log::emit($level, $target, &format!($($arg)*));
        }
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn civil_dates() {
        assert_eq!(civil(0), (1970, 1, 1));
        assert_eq!(civil(11_017), (2000, 3, 1));
        assert_eq!(civil(20_736), (2026, 10, 10));
    }

    #[test]
    fn levels_map_from_the_abi() {
        assert_eq!(Level::from_abi(-4), Level::Debug);
        assert_eq!(Level::from_abi(1), Level::Info);
        assert_eq!(Level::from_abi(2), Level::Warn);
        assert_eq!(Level::from_abi(3), Level::Error);
        assert_eq!(Level::from_abi(99), Level::Error);
        assert!(Level::Error > Level::Warn);
    }
}
