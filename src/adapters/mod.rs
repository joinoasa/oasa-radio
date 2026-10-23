//! Radio adapters. Add one by implementing [`crate::radio::Radio`]; see the README.
//!
//! Planned, not built yet (no code on purpose):
//!
//! - `lora`: LoRa boards through RNode (KISS over USB serial) or our own
//!   firmware. A LoRa packet is small (about 200 bytes at best, far less on
//!   slow settings) and the air time is long, so it needs the MTU and
//!   duty-cycle limits from the radio settings. It would use KISS framing, not
//!   the serial framing here.
//! - `sdr`: our own waveforms on software-defined radios (R-COM2 phase 2,
//!   T-RF1). The packet goes to a modem; the adapter is the interface to it.

pub mod serial;
pub mod udp;

pub use serial::{SerialConfig, SerialRadio};
pub use udp::UdpRadio;
