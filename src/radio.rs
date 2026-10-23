//! The `Radio` trait: what the bridge needs from any radio.
//!
//! A radio moves opaque packets. It never looks inside them, never logs their
//! contents (sizes and counts only) and never encrypts: the packets are
//! already encrypted by oasa-link.

use std::collections::BTreeMap;
use std::fmt;
use std::time::Duration;

/// Why a radio call failed. Dropped packets are not errors: they are counted.
#[derive(Debug)]
pub enum RadioError {
    /// The device or socket failed (unplugged, closed, permission).
    Io(std::io::Error),
    /// A setting is wrong, found while opening.
    Config(String),
}

impl fmt::Display for RadioError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            RadioError::Io(e) => write!(f, "{e}"),
            RadioError::Config(s) => write!(f, "{s}"),
        }
    }
}

impl std::error::Error for RadioError {}

impl From<std::io::Error> for RadioError {
    fn from(e: std::io::Error) -> Self {
        RadioError::Io(e)
    }
}

/// What a radio is, for `oasa-radio info` and the start log line.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RadioInfo {
    /// Adapter kind: `udp`, `serial`.
    pub kind: &'static str,
    /// Where it is connected: a device path or an address.
    pub target: String,
    /// Largest packet it carries.
    pub mtu: usize,
    /// Nominal line rate in bits per second, if known. For serial this is the
    /// UART rate; the air rate of the radio behind it may be lower.
    pub bitrate: Option<u32>,
}

/// Counts since the radio was opened. Packet sizes only, never contents.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Counters {
    pub sent: u64,
    pub sent_bytes: u64,
    pub received: u64,
    pub received_bytes: u64,
    /// Drops by reason (`bad_frame`, `oversize`, ...).
    pub drops: BTreeMap<&'static str, u64>,
}

impl Counters {
    pub fn drop_total(&self) -> u64 {
        self.drops.values().sum()
    }

    pub fn add_drop(&mut self, reason: &'static str, n: u64) {
        if n > 0 {
            *self.drops.entry(reason).or_insert(0) += n;
        }
    }
}

/// One radio. Adapters implement this; the bridge only uses this.
pub trait Radio {
    /// Facts about this radio.
    fn info(&self) -> RadioInfo;

    /// Short adapter name, as in `--radio`.
    fn name(&self) -> &'static str {
        self.info().kind
    }

    /// Largest packet `send` accepts.
    fn mtu(&self) -> usize {
        self.info().mtu
    }

    /// Nominal bitrate in bits per second, if known.
    fn bitrate(&self) -> Option<u32> {
        self.info().bitrate
    }

    /// Send one packet. A packet that can't be sent (empty, over the MTU) is
    /// counted in `counters().drops` and returns `Ok`; `Err` means the radio failed.
    fn send(&mut self, packet: &[u8]) -> Result<(), RadioError>;

    /// Wait up to `timeout` for the next packet. `Ok(None)` means none came.
    fn recv(&mut self, timeout: Duration) -> Result<Option<Vec<u8>>, RadioError>;

    /// Counts so far.
    fn counters(&self) -> &Counters;
}
