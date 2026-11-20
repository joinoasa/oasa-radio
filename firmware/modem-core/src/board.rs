//! Small pieces every board crate needs, kept here so they are tested once.
//!
//! - [`OutFrame`]: an owned copy of one frame for the host, so a board can
//!   queue it for a separate serial writer task.
//! - [`fw_version!`](crate::fw_version): the firmware version from the board
//!   crate's `Cargo.toml`, as the three bytes `VERSION` returns.
//! - [`hex_id`]: a chip ID as text, for a USB serial number.

use crate::modem::MAX_OUT_FRAME;

/// One frame for the host, copied out of the engine.
#[derive(Clone)]
pub struct OutFrame {
    len: usize,
    buf: [u8; MAX_OUT_FRAME],
}

impl OutFrame {
    /// Copy `wire`. `None` if it is longer than [`MAX_OUT_FRAME`].
    pub fn new(wire: &[u8]) -> Option<Self> {
        if wire.len() > MAX_OUT_FRAME {
            return None;
        }
        let mut buf = [0; MAX_OUT_FRAME];
        buf[..wire.len()].copy_from_slice(wire);
        Some(Self {
            len: wire.len(),
            buf,
        })
    }

    pub fn bytes(&self) -> &[u8] {
        &self.buf[..self.len]
    }
}

/// One part of a version string ("0", "12") as a byte. Above 255 gives 255.
/// Only digits are allowed: anything else fails the build (it is `const`).
pub const fn version_part(s: &str) -> u8 {
    let b = s.as_bytes();
    let (mut i, mut n) = (0, 0u32);
    while i < b.len() {
        n = n * 10 + (b[i] - b'0') as u32;
        if n > 255 {
            n = 256;
        }
        i += 1;
    }
    if n > 255 {
        255
    } else {
        n as u8
    }
}

/// `id` as 16 upper case hex digits, most significant first.
pub fn hex_id(id: u64, out: &mut [u8; 16]) -> &str {
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    for (i, b) in out.iter_mut().enumerate() {
        *b = HEX[((id >> (60 - 4 * i)) & 0xF) as usize];
    }
    // Only ASCII hex digits were written.
    core::str::from_utf8(out).unwrap_or("")
}

/// The calling crate's version (`CARGO_PKG_VERSION_*`) as `[major, minor, patch]`.
#[macro_export]
macro_rules! fw_version {
    () => {
        [
            $crate::board::version_part(env!("CARGO_PKG_VERSION_MAJOR")),
            $crate::board::version_part(env!("CARGO_PKG_VERSION_MINOR")),
            $crate::board::version_part(env!("CARGO_PKG_VERSION_PATCH")),
        ]
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn out_frame_copies_and_refuses_long_frames() {
        let f = OutFrame::new(&[1, 2, 3]).unwrap();
        assert_eq!(f.bytes(), &[1, 2, 3]);
        assert_eq!(OutFrame::new(&[]).unwrap().bytes(), &[] as &[u8]);
        assert_eq!(
            OutFrame::new(&[7; MAX_OUT_FRAME]).unwrap().bytes().len(),
            MAX_OUT_FRAME
        );
        assert!(OutFrame::new(&[0; MAX_OUT_FRAME + 1]).is_none());
    }

    #[test]
    fn version_parts() {
        assert_eq!(version_part("0"), 0);
        assert_eq!(version_part("1"), 1);
        assert_eq!(version_part("42"), 42);
        assert_eq!(version_part("255"), 255);
        assert_eq!(version_part("256"), 255);
        assert_eq!(version_part("99999999999"), 255, "no overflow");
        assert_eq!(version_part(""), 0);
    }

    #[test]
    fn hex_ids() {
        let mut b = [0; 16];
        assert_eq!(hex_id(0x0123_4567_89AB_CDEF, &mut b), "0123456789ABCDEF");
        assert_eq!(hex_id(0, &mut b), "0000000000000000");
        assert_eq!(hex_id(u64::MAX, &mut b), "FFFFFFFFFFFFFFFF");
    }

    #[test]
    fn fw_version_is_this_crates_version() {
        const V: [u8; 3] = crate::fw_version!();
        assert_eq!(V, [0, 1, 0]); // oasa-modem-core 0.1.0
    }
}
