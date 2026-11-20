//! OASA USB network adapter test logic (tech tree node T-MSG1, phone side).
//!
//! Everything here is `no_std`, has no hardware in it and no dependencies,
//! so `cargo test` checks it on a PC. The board crate
//! (`firmware/rak4631-usbnet`) adds USB, the network stack and the sockets.
//!
//! - `ncm`: answers to the CDC-NCM setup requests the USB library does not
//!   handle the way phones need (the packet filter and input size requests).
//! - `dhcp`: a DHCP server for one client. No router, no name server.
//! - `http`: one test page.
//! - `net`: the addresses and MAC addresses.

#![no_std]
#![forbid(unsafe_code)]

#[cfg(test)]
extern crate std;

pub mod dhcp;
pub mod http;
pub mod ncm;
pub mod net;
