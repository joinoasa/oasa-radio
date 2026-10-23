//! Serial framing. Format: `docs/serial-framing.md`.
//!
//! A wire frame is `00 | COBS(inner) | 00`. Two kinds of inner:
//!
//! - **Data** (the original format): `len16 | packet | crc16`, `len` 1 or more.
//! - **Extended**: `00 00 | kind | body_len16 | body | crc16`. A zero length can never be a
//!   data frame, so the two cannot be confused. Older hosts drop extended
//!   frames as `bad_length` and carry on.
//!
//! The CRC covers everything before it. Neither is security: the link above
//! authenticates every packet.

use crate::cobs;
use crate::crc::crc16;

/// Largest LoRa payload, and so the largest packet the modem sends.
pub const MAX_LORA_PAYLOAD: usize = 255;
/// Largest packet the decoder reads. Bigger than LoRa on purpose, so a packet
/// of 256 to 512 bytes is read and refused with its own counter.
pub const MAX_PACKET: usize = 512;
/// Extended frame kind: host to modem, a control request.
pub const KIND_REQUEST: u8 = 0x01;
/// Extended frame kind: modem to host, the reply to a request.
pub const KIND_RESPONSE: u8 = 0x02;
/// Extended frame kind: modem to host, a frame heard on the air (air mode).
pub const KIND_AIR: u8 = 0x03;

/// Bytes the frame adds inside COBS around a data packet: 2 length, 2 CRC.
pub const DATA_OVERHEAD: usize = 4;
/// Bytes an extended frame adds around its body: 2 zero, 1 kind, 2 length, 2 CRC.
pub const EXT_OVERHEAD: usize = 7;
/// Largest inner part (before COBS).
pub const MAX_INNER: usize = MAX_PACKET + EXT_OVERHEAD;
/// Largest COBS part.
pub const MAX_BODY: usize = cobs::max_len(MAX_INNER);
/// Largest whole wire frame, delimiters included.
pub const MAX_WIRE: usize = MAX_BODY + 2;

/// A decoded frame, borrowed from the decoder.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Frame<'a> {
    /// A packet for the radio (or from it).
    Data(&'a [u8]),
    /// A control or air frame.
    Ext { kind: u8, body: &'a [u8] },
}

/// Why a frame could not be built.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EncodeError {
    /// Empty packets are not carried.
    Empty,
    /// Longer than [`MAX_PACKET`].
    TooLarge,
    /// The output buffer is shorter than [`MAX_WIRE`] needs.
    OutTooSmall,
}

fn finish(inner: &[u8], out: &mut [u8]) -> Result<usize, EncodeError> {
    if out.len() < cobs::max_len(inner.len()) + 2 {
        return Err(EncodeError::OutTooSmall);
    }
    out[0] = 0;
    let n = cobs::encode(inner, &mut out[1..]).ok_or(EncodeError::OutTooSmall)?;
    out[1 + n] = 0;
    Ok(n + 2)
}

/// Build the wire frame for a data packet. Returns its length.
pub fn encode_data(packet: &[u8], out: &mut [u8]) -> Result<usize, EncodeError> {
    if packet.is_empty() {
        return Err(EncodeError::Empty);
    }
    if packet.len() > MAX_PACKET {
        return Err(EncodeError::TooLarge);
    }
    let mut inner = [0u8; MAX_PACKET + DATA_OVERHEAD];
    let n = packet.len();
    // Fits: n <= 512.
    inner[..2].copy_from_slice(&(n as u16).to_be_bytes());
    inner[2..2 + n].copy_from_slice(packet);
    let crc = crc16(&inner[..2 + n]);
    inner[2 + n..4 + n].copy_from_slice(&crc.to_be_bytes());
    finish(&inner[..4 + n], out)
}

/// Build the wire frame for an extended frame. Returns its length.
pub fn encode_ext(kind: u8, body: &[u8], out: &mut [u8]) -> Result<usize, EncodeError> {
    if body.len() > MAX_PACKET {
        return Err(EncodeError::TooLarge);
    }
    let mut inner = [0u8; MAX_INNER];
    let n = body.len();
    inner[2] = kind;
    // Fits: n <= 512. Without this length, a stray zero after the CRC would
    // pass for a changed frame (CRC-16 of data plus its own CRC is a fixed value).
    inner[3..5].copy_from_slice(&(n as u16).to_be_bytes());
    inner[5..5 + n].copy_from_slice(body);
    let crc = crc16(&inner[..5 + n]);
    inner[5 + n..7 + n].copy_from_slice(&crc.to_be_bytes());
    finish(&inner[..7 + n], out)
}

/// Frames rejected, by reason.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct DecoderStats {
    /// Good frames, both kinds.
    pub frames: u32,
    /// Not valid COBS.
    pub bad_cobs: u32,
    /// Too short, or the length disagrees with the frame size.
    pub bad_length: u32,
    /// CRC mismatch.
    pub bad_crc: u32,
    /// Too long for [`MAX_PACKET`].
    pub oversize: u32,
}

impl DecoderStats {
    /// All rejected frames.
    pub fn bad_total(&self) -> u32 {
        self.bad_cobs
            .wrapping_add(self.bad_length)
            .wrapping_add(self.bad_crc)
            .wrapping_add(self.oversize)
    }
}

enum Bad {
    Cobs,
    Length,
    Crc,
}

/// Cuts a serial byte stream into frames, one byte at a time.
///
/// Memory is fixed. A long run of bytes with no `00` is thrown away as it
/// arrives and counted as `oversize` at the next `00`.
pub struct Decoder {
    buf: [u8; MAX_BODY],
    used: usize,
    overflow: bool,
    raw: [u8; MAX_INNER],
    stats: DecoderStats,
}

impl Default for Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl Decoder {
    pub const fn new() -> Self {
        Self {
            buf: [0; MAX_BODY],
            used: 0,
            overflow: false,
            raw: [0; MAX_INNER],
            stats: DecoderStats {
                frames: 0,
                bad_cobs: 0,
                bad_length: 0,
                bad_crc: 0,
                oversize: 0,
            },
        }
    }

    pub fn stats(&self) -> DecoderStats {
        self.stats
    }

    /// Feed one byte. Returns a frame when this byte closed a good one.
    /// The frame lasts until the next call.
    pub fn push_byte(&mut self, b: u8) -> Option<Frame<'_>> {
        if b != 0 {
            if self.overflow {
                return None;
            }
            if self.used >= MAX_BODY {
                self.overflow = true;
                self.used = 0;
                return None;
            }
            self.buf[self.used] = b;
            self.used += 1;
            return None;
        }
        // Delimiter.
        let used = self.used;
        self.used = 0;
        if self.overflow {
            self.overflow = false;
            self.stats.oversize = self.stats.oversize.wrapping_add(1);
            return None;
        }
        if used == 0 {
            return None;
        }
        match self.decode(used) {
            Ok(n) => {
                self.stats.frames = self.stats.frames.wrapping_add(1);
                Some(Self::parse(&self.raw[..n]))
            }
            Err(r) => {
                let s = &mut self.stats;
                match r {
                    Bad::Cobs => s.bad_cobs = s.bad_cobs.wrapping_add(1),
                    Bad::Length => s.bad_length = s.bad_length.wrapping_add(1),
                    Bad::Crc => s.bad_crc = s.bad_crc.wrapping_add(1),
                }
                None
            }
        }
    }

    /// Check the frame in `buf[..used]`; on success it is in `raw[..n]`.
    fn decode(&mut self, used: usize) -> Result<usize, Bad> {
        let n = cobs::decode(&self.buf[..used], &mut self.raw).ok_or(Bad::Cobs)?;
        let raw = &self.raw[..n];
        if n < DATA_OVERHEAD + 1 {
            return Err(Bad::Length);
        }
        let len = usize::from(u16::from_be_bytes([raw[0], raw[1]]));
        if len != 0 && len + DATA_OVERHEAD != n {
            return Err(Bad::Length);
        }
        if len == 0
            && (n < EXT_OVERHEAD
                || usize::from(u16::from_be_bytes([raw[3], raw[4]])) + EXT_OVERHEAD != n)
        {
            return Err(Bad::Length);
        }
        let (body, crc) = raw.split_at(n - 2);
        if crc16(body) != u16::from_be_bytes([crc[0], crc[1]]) {
            return Err(Bad::Crc);
        }
        Ok(n)
    }

    fn parse(raw: &[u8]) -> Frame<'_> {
        let len = usize::from(u16::from_be_bytes([raw[0], raw[1]]));
        if len == 0 {
            Frame::Ext {
                kind: raw[2],
                body: &raw[5..raw.len() - 2],
            }
        } else {
            Frame::Data(&raw[2..2 + len])
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec::Vec;

    fn data_wire(p: &[u8]) -> Vec<u8> {
        let mut out = std::vec![0u8; MAX_WIRE];
        let n = encode_data(p, &mut out).unwrap();
        out.truncate(n);
        out
    }

    fn ext_wire(kind: u8, body: &[u8]) -> Vec<u8> {
        let mut out = std::vec![0u8; MAX_WIRE];
        let n = encode_ext(kind, body, &mut out).unwrap();
        out.truncate(n);
        out
    }

    /// Everything the decoder yields, as owned values.
    #[derive(Debug, PartialEq, Eq, Clone)]
    enum Got {
        Data(Vec<u8>),
        Ext(u8, Vec<u8>),
    }

    fn run(d: &mut Decoder, bytes: &[u8]) -> Vec<Got> {
        let mut v = Vec::new();
        for &b in bytes {
            match d.push_byte(b) {
                Some(Frame::Data(p)) => v.push(Got::Data(p.to_vec())),
                Some(Frame::Ext { kind, body }) => v.push(Got::Ext(kind, body.to_vec())),
                None => {}
            }
        }
        v
    }

    /// Small seeded generator so the tests need no crates.
    struct Rng(u64);
    impl Rng {
        fn next(&mut self) -> u64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            self.0
        }
        fn below(&mut self, n: u64) -> u64 {
            self.next() % n
        }
    }

    #[test]
    fn data_frame_bytes_match_the_host_format() {
        // Same fixed bytes as src/framing.rs frame_layout_is_exact.
        assert_eq!(
            data_wire(&[1, 2, 3]),
            [0x00, 0x01, 0x07, 0x03, 0x01, 0x02, 0x03, 0xEB, 0xE1, 0x00]
        );
    }

    #[test]
    fn ext_frame_layout_is_exact() {
        // inner = 00 00 | kind | len | body | crc
        let w = ext_wire(KIND_REQUEST, &[0x07, 0x02]);
        let crc = crc16(&[0, 0, 1, 0, 2, 7, 2]).to_be_bytes();
        let inner = [0, 0, 1, 0, 2, 7, 2, crc[0], crc[1]];
        let mut coded = std::vec![0u8; 24];
        let n = cobs::encode(&inner, &mut coded).unwrap();
        let mut want = std::vec![0u8];
        want.extend_from_slice(&coded[..n]);
        want.push(0);
        assert_eq!(w, want);
        let mut d = Decoder::new();
        assert_eq!(run(&mut d, &w), [Got::Ext(KIND_REQUEST, std::vec![7, 2])]);
    }

    #[test]
    fn data_round_trips_at_every_length() {
        let mut rng = Rng(1);
        let mut d = Decoder::new();
        for len in 1..=MAX_PACKET {
            let p: Vec<u8> = (0..len)
                .map(|_| {
                    if rng.below(4) == 0 {
                        0
                    } else {
                        rng.next() as u8
                    }
                })
                .collect();
            let w = data_wire(&p);
            assert!(w.len() <= MAX_WIRE);
            assert_eq!(run(&mut d, &w), [Got::Data(p)]);
        }
        assert_eq!(d.stats().bad_total(), 0);
        assert_eq!(d.stats().frames, MAX_PACKET as u32);
    }

    #[test]
    fn ext_round_trips_at_every_length_including_empty() {
        let mut rng = Rng(2);
        let mut d = Decoder::new();
        for len in 0..=MAX_PACKET {
            let body: Vec<u8> = (0..len).map(|_| rng.next() as u8).collect();
            let w = ext_wire(KIND_AIR, &body);
            assert_eq!(run(&mut d, &w), [Got::Ext(KIND_AIR, body)]);
        }
        assert_eq!(d.stats().bad_total(), 0);
    }

    #[test]
    fn oversize_input_is_refused() {
        let mut out = std::vec![0u8; MAX_WIRE];
        assert_eq!(
            encode_data(&[1; MAX_PACKET + 1], &mut out),
            Err(EncodeError::TooLarge)
        );
        assert_eq!(encode_data(&[], &mut out), Err(EncodeError::Empty));
        assert_eq!(
            encode_ext(1, &[0; MAX_PACKET + 1], &mut out),
            Err(EncodeError::TooLarge)
        );
        let mut small = [0u8; 8];
        assert_eq!(
            encode_data(&[1; 20], &mut small),
            Err(EncodeError::OutTooSmall)
        );
    }

    #[test]
    fn a_data_frame_of_length_zero_does_not_exist() {
        // Hand-built: len 0 means "extended", so 00 00 + 3 bytes + crc is kind+body.
        let mut d = Decoder::new();
        let w = ext_wire(9, &[]);
        assert_eq!(run(&mut d, &w), [Got::Ext(9, Vec::new())]);
    }

    #[test]
    fn data_and_ext_frames_mix_in_one_stream() {
        let mut d = Decoder::new();
        let mut s = Vec::new();
        s.extend(data_wire(&[1, 2, 3]));
        s.extend(ext_wire(KIND_REQUEST, &[1, 1]));
        s.extend(data_wire(&[0, 0, 0, 9]));
        s.extend(ext_wire(KIND_AIR, &[0xFF; 40]));
        let got = run(&mut d, &s);
        assert_eq!(
            got,
            [
                Got::Data(std::vec![1, 2, 3]),
                Got::Ext(KIND_REQUEST, std::vec![1, 1]),
                Got::Data(std::vec![0, 0, 0, 9]),
                Got::Ext(KIND_AIR, std::vec![0xFF; 40]),
            ]
        );
    }

    #[test]
    fn noise_between_frames_costs_only_a_count() {
        let mut rng = Rng(3);
        let mut d = Decoder::new();
        let mut want = Vec::new();
        let mut s = Vec::new();
        for i in 0..1500 {
            let n = rng.below(300) as usize;
            s.extend((0..n).map(|_| rng.next() as u8));
            let len = 1 + rng.below(255) as usize;
            let p: Vec<u8> = (0..len).map(|_| rng.next() as u8).collect();
            if i % 3 == 0 {
                s.extend(ext_wire(KIND_RESPONSE, &p));
                want.push(Got::Ext(KIND_RESPONSE, p));
            } else {
                s.extend(data_wire(&p));
                want.push(Got::Data(p));
            }
        }
        assert_eq!(run(&mut d, &s), want);
        assert!(d.stats().bad_total() > 0);
    }

    #[test]
    fn every_single_bit_flip_is_caught_and_the_stream_recovers() {
        let after = std::vec![1u8, 2, 3, 0, 4];
        let wa = data_wire(&after);
        for w in [
            data_wire(&[0x42]),
            data_wire(&[0, 0, 0]),
            data_wire(&(0..60).collect::<Vec<u8>>()),
            ext_wire(KIND_REQUEST, &[1, 2, 3]),
            ext_wire(KIND_AIR, &[0xFF; 100]),
        ] {
            let mut good = Decoder::new();
            let orig = run(&mut good, &w);
            assert_eq!(orig.len(), 1);
            for bit in 0..w.len() * 8 {
                let mut s = w.clone();
                s[bit / 8] ^= 1 << (bit % 8);
                s.extend_from_slice(&wa);
                let mut d = Decoder::new();
                let got = run(&mut d, &s);
                // A flip can split or merge frames. Never a changed frame, and
                // the frame after the damage always arrives.
                assert_eq!(got.last(), Some(&Got::Data(after.clone())), "bit {bit}");
                assert!(
                    got.len() == 1 || got[0] == orig[0],
                    "bit {bit} w={w:02x?} got={got:02x?}"
                );
            }
        }
    }

    #[test]
    fn every_truncation_is_safe() {
        let next = std::vec![9u8, 0, 9];
        let wn = data_wire(&next);
        for w in [data_wire(&[5; 50]), ext_wire(KIND_RESPONSE, &[1, 2, 3, 4])] {
            for cut in 0..w.len() {
                let mut d = Decoder::new();
                let mut s = w[..cut].to_vec();
                s.extend_from_slice(&wn);
                let got = run(&mut d, &s);
                assert_eq!(got.last(), Some(&Got::Data(next.clone())), "cut {cut}");
                let mut d = Decoder::new();
                assert!(run(&mut d, &w[..cut]).is_empty(), "alone {cut}");
            }
        }
    }

    #[test]
    fn long_garbage_is_bounded_and_counted() {
        let mut d = Decoder::new();
        assert!(run(&mut d, &std::vec![0xAA; 100_000]).is_empty());
        assert!(run(&mut d, &[0]).is_empty());
        assert_eq!(d.stats().oversize, 1);
        assert_eq!(
            run(&mut d, &data_wire(&[1, 2, 3])),
            [Got::Data(std::vec![1, 2, 3])]
        );
    }

    #[test]
    fn host_style_old_decoder_would_call_ext_frames_bad_length() {
        // The old rule: a length that does not match the frame size is bad.
        // Here len is 0 and the frame has 5 or more bytes, so it never matches.
        for body_len in 0..40usize {
            let w = ext_wire(KIND_AIR, &std::vec![7; body_len]);
            let mut coded = std::vec![0u8; w.len()];
            let n = cobs::decode(&w[1..w.len() - 1], &mut coded).unwrap();
            let raw = &coded[..n];
            let len = usize::from(u16::from_be_bytes([raw[0], raw[1]]));
            assert_eq!(len, 0);
            assert_ne!(len + DATA_OVERHEAD, raw.len());
        }
    }

    #[test]
    fn byte_at_a_time_matches_chunks() {
        let mut rng = Rng(5);
        let mut s = Vec::new();
        let mut want = Vec::new();
        for _ in 0..100 {
            let len = 1 + rng.below(255) as usize;
            let p: Vec<u8> = (0..len).map(|_| rng.next() as u8).collect();
            s.extend(data_wire(&p));
            want.push(Got::Data(p));
        }
        let mut d = Decoder::new();
        assert_eq!(run(&mut d, &s), want);
    }
}
