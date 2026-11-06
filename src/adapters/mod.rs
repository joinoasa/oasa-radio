//! Radio adapters. Add one by implementing [`crate::radio::Radio`]; see the README.
//!
//! LoRa is not an adapter here: our modem firmware (`firmware/heltec-v3`)
//! speaks the serial framing, so the `serial` adapter carries it.
//!
//! `sdr` carries packets as frame v1 of OASA's own waveform over a
//! software-defined radio, through two continuous IQ streams.

pub mod sdr;
pub mod serial;
pub mod udp;

pub use sdr::{SdrConfig, SdrRadio};
pub use serial::{SerialConfig, SerialRadio};
pub use udp::UdpRadio;
