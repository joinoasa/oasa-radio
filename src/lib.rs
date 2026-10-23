//! OASA radio adapters (tech tree node R-COM1).
//!
//! Adapters carry packets that oasa-link has already encrypted. Nothing in
//! this crate encrypts, decrypts, inspects or logs packet contents.
//!
//! - [`radio::Radio`]: the trait every adapter implements.
//! - [`adapters`]: `udp` and `serial`.
//! - [`framing`]: how packets are framed on a byte stream.
//! - [`lora`]: control of the OASA LoRa modem firmware (`oasa-radio lora`).
//! - [`bridge`]: UDP on one side, a radio on the other.
//! - [`relay`], [`rng`]: test tools (pty pairs, a lossy serial line, a seeded PRNG).

#![forbid(unsafe_code)]

pub mod adapters;
pub mod bridge;
pub mod cli;
pub mod framing;
pub mod log;
pub mod lora;
pub mod radio;
pub mod relay;
pub mod rng;
