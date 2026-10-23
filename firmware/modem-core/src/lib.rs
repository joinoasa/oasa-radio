//! OASA LoRa modem core (tech tree node R-COM1, LoRa side).
//!
//! Everything here is `no_std` and has no hardware in it, so `cargo test`
//! checks the logic on a PC. The firmware crate (`firmware/heltec-v3`) adds
//! pins, the clock and the USB serial port.
//!
//! The modem never encrypts and never reads a payload. It carries the bytes
//! oasa-link already encrypted: one framed serial packet becomes one LoRa
//! packet, and the reverse.
//!
//! - `frame`: the serial framing (COBS, length, CRC-16) and the control and
//!   air frame kinds. Format: `docs/serial-framing.md`.
//! - `control`: requests and replies on the control channel.
//! - `settings`: radio settings, limits and air time.
//! - `stats`: counters.
//! - `queue`: the fixed TX queue.
//! - `modem`: the engine that ties them together.
//! - `sx126x`: the SX1262 driver.
//! - `sim` (feature `sim`): a simulated radio and air for tests.

#![no_std]
#![forbid(unsafe_code)]

#[cfg(any(test, feature = "sim"))]
extern crate std;

pub mod cobs;
pub mod control;
pub mod crc;
pub mod frame;
pub mod modem;
pub mod queue;
pub mod settings;
pub mod stats;
pub mod sx126x;

#[cfg(any(test, feature = "sim"))]
pub mod sim;

/// Control protocol version, returned by `VERSION`.
pub const PROTOCOL_VERSION: u8 = 1;
