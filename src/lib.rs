//! OASA radio adapters (tech tree node R-COM1).
//!
//! Adapters carry packets that oasa-link has already encrypted. Nothing in
//! this crate encrypts, decrypts, inspects or logs packet contents.

#![forbid(unsafe_code)]

pub mod adapters;
pub mod framing;
pub mod log;
pub mod radio;
pub mod relay;
pub mod rng;
