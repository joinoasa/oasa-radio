//! Radio adapters. Add one by implementing [`crate::radio::Radio`]; see the README.
//!
//! LoRa is not an adapter here: our modem firmware (`firmware/heltec-v3`)
//! speaks the serial framing, so the `serial` adapter carries it.
//!
//! Planned, not built yet (no code on purpose):
//!
//! - `sdr`: our own waveforms on software-defined radios (R-COM2 phase 2,
//!   T-RF1). The packet goes to a modem; the adapter is the interface to it.

pub mod serial;
pub mod udp;

pub use serial::{SerialConfig, SerialRadio};
pub use udp::UdpRadio;
