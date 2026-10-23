//! JSON-lines logging, in the style of oasa-link: one object per line with
//! `ts` and `event`. Packet contents are never passed to anything here.

use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

/// `2026-10-23T12:34:56.789Z`
pub fn rfc3339(t: SystemTime) -> String {
    let d = t.duration_since(UNIX_EPOCH).unwrap_or_default();
    let secs = d.as_secs();
    let (days, rem) = ((secs / 86_400) as i64, secs % 86_400);
    // Days since 1970-01-01 to a calendar date (Howard Hinnant's algorithm).
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = yoe + era * 400 + i64::from(month <= 2);
    format!(
        "{year:04}-{month:02}-{day:02}T{:02}:{:02}:{:02}.{:03}Z",
        rem / 3600,
        rem % 3600 / 60,
        rem % 60,
        d.subsec_millis()
    )
}

/// A JSON string literal, quotes included.
pub fn json_str(s: &str) -> String {
    let mut o = String::with_capacity(s.len() + 2);
    o.push('"');
    for c in s.chars() {
        match c {
            '"' => o.push_str("\\\""),
            '\\' => o.push_str("\\\\"),
            '\n' => o.push_str("\\n"),
            '\r' => o.push_str("\\r"),
            '\t' => o.push_str("\\t"),
            c if (c as u32) < 0x20 => o.push_str(&format!("\\u{:04x}", c as u32)),
            c => o.push(c),
        }
    }
    o.push('"');
    o
}

/// Limits drop lines to 10 a second, then one summary line a second.
/// Counting is not affected: the stats stay exact.
pub struct DropLog {
    window: Instant,
    printed: u32,
    suppressed: u64,
}

pub const DROP_LINES_PER_SECOND: u32 = 10;

impl DropLog {
    pub fn new(now: Instant) -> Self {
        Self {
            window: now,
            printed: 0,
            suppressed: 0,
        }
    }

    /// Start a new second if one has passed. Returns how many lines the
    /// second that ended held back.
    pub fn roll(&mut self, now: Instant) -> u64 {
        if now.saturating_duration_since(self.window) < Duration::from_secs(1) {
            return 0;
        }
        self.window = now;
        self.printed = 0;
        std::mem::take(&mut self.suppressed)
    }

    /// May one more drop line be printed now? (Call `roll` first.)
    pub fn allow(&mut self) -> bool {
        if self.printed < DROP_LINES_PER_SECOND {
            self.printed += 1;
            true
        } else {
            self.suppressed += 1;
            false
        }
    }

    /// Take the held-back count now (at exit).
    pub fn take_suppressed(&mut self) -> u64 {
        std::mem::take(&mut self.suppressed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at(secs: u64, ms: u32) -> SystemTime {
        UNIX_EPOCH + Duration::new(secs, ms * 1_000_000)
    }

    #[test]
    fn timestamps_are_rfc3339_utc() {
        assert_eq!(rfc3339(at(0, 0)), "1970-01-01T00:00:00.000Z");
        assert_eq!(rfc3339(at(951_782_400, 5)), "2000-02-29T00:00:00.005Z");
        assert_eq!(rfc3339(at(1_700_000_000, 999)), "2023-11-14T22:13:20.999Z");
        assert_eq!(rfc3339(at(4_107_542_400, 0)), "2100-03-01T00:00:00.000Z");
    }

    #[test]
    fn strings_are_escaped() {
        assert_eq!(json_str("a\"b\\c\n\u{1}é"), "\"a\\\"b\\\\c\\n\\u0001é\"");
    }

    #[test]
    fn drop_lines_are_limited_to_ten_a_second_with_a_summary() {
        let t0 = Instant::now();
        let mut log = DropLog::new(t0);
        assert_eq!((0..1000).filter(|_| log.allow()).count(), 10);
        assert_eq!(log.roll(t0 + Duration::from_millis(900)), 0);
        assert_eq!(log.roll(t0 + Duration::from_millis(1000)), 990);
        assert_eq!(log.roll(t0 + Duration::from_millis(1100)), 0);
        assert_eq!((0..50).filter(|_| log.allow()).count(), 10);
        assert_eq!(log.take_suppressed(), 40);
    }
}
