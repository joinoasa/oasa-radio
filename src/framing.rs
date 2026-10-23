//! Packet framing for byte-stream radios (serial). Format: `docs/serial-framing.md`.
//!
//! A wire frame is `00 | COBS(len16 | packet | crc16) | 00`. COBS removes every
//! zero from the body, so a zero byte always marks a frame edge. The length and
//! CRC only detect line errors and let the receiver resync. They are not
//! security: the link above authenticates every packet.
//!
//! A length of zero marks an **extended** frame, `00 00 | kind | n16 | body |
//! crc16`, used by the LoRa modem for its control channel and air reports. A
//! data frame never has length zero, so the two cannot be mixed up.

/// Default largest packet (SiK radios carry 252 bytes per radio packet).
pub const DEFAULT_MTU: usize = 252;
/// Largest `--mtu` the 16-bit length field allows.
pub const MAX_MTU: usize = 65_535;
/// Bytes added inside the COBS body: 2 length + 2 CRC.
pub const INNER_OVERHEAD: usize = 4;
/// Bytes an extended frame adds around its body: 2 zero, 1 kind, 2 length, 2 CRC.
pub const EXT_OVERHEAD: usize = 7;
/// Largest extended frame body the receiver reads.
pub const EXT_MAX_BODY: usize = 512;
/// Extended frame kinds (see `oasa_modem_core::frame`).
pub use oasa_modem_core::frame::{KIND_AIR, KIND_REQUEST, KIND_RESPONSE};

/// A frame taken off the line.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Frame {
    /// A packet.
    Data(Vec<u8>),
    /// A control or air frame.
    Ext { kind: u8, body: Vec<u8> },
}

/// CRC-16/CCITT-FALSE: polynomial 0x1021, init 0xFFFF, no reflection, no final XOR.
/// Check value: `crc16(b"123456789") == 0x29B1`.
pub fn crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in data {
        crc ^= u16::from(b) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

/// Longest COBS encoding of `n` input bytes.
pub fn cobs_max_len(n: usize) -> usize {
    n + n / 254 + 1
}

/// Append the COBS encoding of `input` to `out`. The output has no zero bytes.
pub fn cobs_encode(input: &[u8], out: &mut Vec<u8>) {
    let mut code_at = out.len();
    out.push(0);
    let mut code: u8 = 1;
    for &b in input {
        if b == 0 {
            out[code_at] = code;
            code_at = out.len();
            out.push(0);
            code = 1;
        } else {
            out.push(b);
            code += 1;
            if code == 0xFF {
                out[code_at] = code;
                code_at = out.len();
                out.push(0);
                code = 1;
            }
        }
    }
    out[code_at] = code;
}

/// Decode COBS data. `None` if it holds a zero byte or a block runs past the end.
pub fn cobs_decode(input: &[u8]) -> Option<Vec<u8>> {
    let mut out = Vec::with_capacity(input.len());
    let mut i = 0;
    while i < input.len() {
        let code = usize::from(input[i]);
        if code == 0 {
            return None;
        }
        i += 1;
        let end = i + code - 1;
        let block = input.get(i..end)?;
        if block.contains(&0) {
            return None;
        }
        out.extend_from_slice(block);
        i = end;
        if code < 0xFF && i < input.len() {
            out.push(0);
        }
    }
    Some(out)
}

/// Why a packet can't be framed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameError {
    /// Empty packets are not carried.
    Empty,
    /// Longer than the MTU.
    TooLarge { len: usize, mtu: usize },
}

impl std::fmt::Display for FrameError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FrameError::Empty => write!(f, "empty packet"),
            FrameError::TooLarge { len, mtu } => {
                write!(f, "packet of {len} bytes is over the MTU of {mtu}")
            }
        }
    }
}

impl std::error::Error for FrameError {}

/// Frames received or rejected. `bad_frame` is the sum of the four reasons.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct FrameStats {
    /// Good data frames delivered.
    pub frames: u64,
    /// Good extended frames (control and air). Not counted in `bad_frame`.
    pub ext_frames: u64,
    /// All rejected frames (noise between frames counts too).
    pub bad_frame: u64,
    /// Not valid COBS.
    pub bad_cobs: u64,
    /// Too short, or the length field disagrees with the frame size.
    pub bad_length: u64,
    /// CRC mismatch.
    pub bad_crc: u64,
    /// Longer than the MTU (or too long to be a frame at all).
    pub oversize: u64,
}

/// Cuts a byte stream into packets and builds frames for sending.
#[derive(Debug, Clone)]
pub struct Framer {
    mtu: usize,
    max_body: usize,
    buf: Vec<u8>,
    overflow: bool,
    stats: FrameStats,
}

impl Framer {
    /// `mtu` is the largest packet, 1 to [`MAX_MTU`].
    pub fn new(mtu: usize) -> Result<Self, String> {
        if !(1..=MAX_MTU).contains(&mtu) {
            return Err(format!("mtu must be 1 to {MAX_MTU}"));
        }
        Ok(Self {
            mtu,
            max_body: cobs_max_len((mtu + INNER_OVERHEAD).max(EXT_MAX_BODY + EXT_OVERHEAD)),
            buf: Vec::new(),
            overflow: false,
            stats: FrameStats::default(),
        })
    }

    pub fn mtu(&self) -> usize {
        self.mtu
    }

    pub fn stats(&self) -> FrameStats {
        self.stats
    }

    /// Most bytes one frame takes on the wire, delimiters included.
    pub fn max_wire_len(&self) -> usize {
        self.max_body + 2
    }

    /// Append the wire frame for `packet` to `out` (nothing is added on error).
    pub fn encode(&self, packet: &[u8], out: &mut Vec<u8>) -> Result<(), FrameError> {
        if packet.is_empty() {
            return Err(FrameError::Empty);
        }
        if packet.len() > self.mtu {
            return Err(FrameError::TooLarge {
                len: packet.len(),
                mtu: self.mtu,
            });
        }
        let mut inner = Vec::with_capacity(packet.len() + INNER_OVERHEAD);
        // Fits: packet.len() <= mtu <= 65535.
        inner.extend_from_slice(&(packet.len() as u16).to_be_bytes());
        inner.extend_from_slice(packet);
        let crc = crc16(&inner);
        inner.extend_from_slice(&crc.to_be_bytes());
        out.push(0);
        cobs_encode(&inner, out);
        out.push(0);
        Ok(())
    }

    /// Append the wire frame for an extended frame to `out`.
    pub fn encode_ext(&self, kind: u8, body: &[u8], out: &mut Vec<u8>) -> Result<(), FrameError> {
        if body.len() > EXT_MAX_BODY {
            return Err(FrameError::TooLarge {
                len: body.len(),
                mtu: EXT_MAX_BODY,
            });
        }
        let mut inner = Vec::with_capacity(body.len() + EXT_OVERHEAD);
        inner.extend_from_slice(&[0, 0, kind]);
        // Fits: body.len() <= 512.
        inner.extend_from_slice(&(body.len() as u16).to_be_bytes());
        inner.extend_from_slice(body);
        let crc = crc16(&inner);
        inner.extend_from_slice(&crc.to_be_bytes());
        out.push(0);
        cobs_encode(&inner, out);
        out.push(0);
        Ok(())
    }

    /// Feed bytes from the line. Returns every data packet they complete;
    /// extended frames are counted and skipped (see [`Framer::push_frames`]).
    /// Bad frames are dropped and counted; the next zero byte resyncs.
    pub fn push(&mut self, bytes: &[u8]) -> Vec<Vec<u8>> {
        self.push_frames(bytes)
            .into_iter()
            .filter_map(|f| match f {
                Frame::Data(p) => Some(p),
                Frame::Ext { .. } => None,
            })
            .collect()
    }

    /// Feed bytes from the line. Returns every frame they complete, in order.
    pub fn push_frames(&mut self, bytes: &[u8]) -> Vec<Frame> {
        let mut done = Vec::new();
        for &b in bytes {
            if b != 0 {
                if self.overflow {
                    continue;
                }
                if self.buf.len() >= self.max_body {
                    self.overflow = true;
                    self.buf.clear();
                    continue;
                }
                self.buf.push(b);
                continue;
            }
            // Delimiter.
            if self.overflow {
                self.overflow = false;
                self.stats.bad_frame += 1;
                self.stats.oversize += 1;
            } else if !self.buf.is_empty() {
                match self.decode_body() {
                    Ok(f) => {
                        match f {
                            Frame::Data(_) => self.stats.frames += 1,
                            Frame::Ext { .. } => self.stats.ext_frames += 1,
                        }
                        done.push(f);
                    }
                    Err(r) => {
                        self.stats.bad_frame += 1;
                        match r {
                            Bad::Cobs => self.stats.bad_cobs += 1,
                            Bad::Length => self.stats.bad_length += 1,
                            Bad::Crc => self.stats.bad_crc += 1,
                            Bad::Oversize => self.stats.oversize += 1,
                        }
                    }
                }
            }
            self.buf.clear();
        }
        done
    }

    fn decode_body(&self) -> Result<Frame, Bad> {
        let raw = cobs_decode(&self.buf).ok_or(Bad::Cobs)?;
        if raw.len() < INNER_OVERHEAD + 1 {
            return Err(Bad::Length);
        }
        let len = usize::from(u16::from_be_bytes([raw[0], raw[1]]));
        let ext = len == 0;
        if ext {
            let n = raw.len();
            if n < EXT_OVERHEAD
                || usize::from(u16::from_be_bytes([raw[3], raw[4]])) + EXT_OVERHEAD != n
            {
                return Err(Bad::Length);
            }
        } else if len + INNER_OVERHEAD != raw.len() {
            return Err(Bad::Length);
        }
        let (body, crc) = raw.split_at(raw.len() - 2);
        if crc16(body) != u16::from_be_bytes([crc[0], crc[1]]) {
            return Err(Bad::Crc);
        }
        if ext {
            return Ok(Frame::Ext {
                kind: raw[2],
                body: raw[5..raw.len() - 2].to_vec(),
            });
        }
        if len > self.mtu {
            return Err(Bad::Oversize);
        }
        Ok(Frame::Data(raw[2..2 + len].to_vec()))
    }
}

enum Bad {
    Cobs,
    Length,
    Crc,
    Oversize,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rng::Rng;

    fn frame(f: &Framer, p: &[u8]) -> Vec<u8> {
        let mut v = Vec::new();
        f.encode(p, &mut v).unwrap();
        v
    }

    /// Random packet; half the time with many zeros.
    fn packet(rng: &mut Rng, max: usize) -> Vec<u8> {
        let len = 1 + rng.below(max as u64) as usize;
        let zeros = rng.below(2) == 0;
        (0..len)
            .map(|_| {
                if zeros && rng.below(3) == 0 {
                    0
                } else {
                    rng.next_u64() as u8
                }
            })
            .collect()
    }

    fn packet_of(rng: &mut Rng, len: usize) -> Vec<u8> {
        (0..len)
            .map(|_| {
                if rng.below(4) == 0 {
                    0
                } else {
                    rng.next_u64() as u8
                }
            })
            .collect()
    }

    #[test]
    fn crc_check_value() {
        assert_eq!(crc16(b"123456789"), 0x29B1);
        assert_eq!(crc16(b""), 0xFFFF);
    }

    #[test]
    fn cobs_known_vectors() {
        let cases: &[(&[u8], &[u8])] = &[
            (&[], &[0x01]),
            (&[0x00], &[0x01, 0x01]),
            (&[0x00, 0x00], &[0x01, 0x01, 0x01]),
            (&[0x11, 0x22, 0x00, 0x33], &[0x03, 0x11, 0x22, 0x02, 0x33]),
            (&[0x11, 0x22, 0x33, 0x44], &[0x05, 0x11, 0x22, 0x33, 0x44]),
            (&[0x11, 0x00, 0x00, 0x00], &[0x02, 0x11, 0x01, 0x01, 0x01]),
        ];
        for (plain, coded) in cases {
            let mut out = Vec::new();
            cobs_encode(plain, &mut out);
            assert_eq!(&out, coded, "encode {plain:02x?}");
            assert_eq!(&cobs_decode(coded).unwrap(), plain, "decode {coded:02x?}");
        }
        // Block limits: 254 non-zero bytes fill one block (plus a closing 01).
        let p254: Vec<u8> = (1..=254).map(|i| i as u8).collect();
        let mut out = Vec::new();
        cobs_encode(&p254, &mut out);
        assert_eq!(out.len(), 256);
        assert_eq!((out[0], out[255]), (0xFF, 0x01));
        assert_eq!(cobs_decode(&out).unwrap(), p254);
        let p255: Vec<u8> = (0..255).map(|i| (i % 255 + 1) as u8).collect();
        out.clear();
        cobs_encode(&p255, &mut out);
        assert_eq!(out.len(), 257);
        assert_eq!((out[0], out[255], out[256]), (0xFF, 0x02, p255[254]));
        assert_eq!(cobs_decode(&out).unwrap(), p255);
    }

    #[test]
    fn cobs_rejects_bad_input() {
        assert!(cobs_decode(&[0x00]).is_none());
        assert!(cobs_decode(&[0x03, 0x11]).is_none()); // block runs past the end
        assert!(cobs_decode(&[0x03, 0x11, 0x00]).is_none()); // zero inside
        assert!(cobs_decode(&[0x02, 0x11, 0x00]).is_none());
    }

    #[test]
    fn cobs_round_trips_and_never_emits_zero() {
        let mut rng = Rng::new(1);
        for _ in 0..3000 {
            let p = packet(&mut rng, 600);
            let mut out = Vec::new();
            cobs_encode(&p, &mut out);
            assert!(!out.contains(&0));
            assert!(out.len() <= cobs_max_len(p.len()));
            assert_eq!(cobs_decode(&out).unwrap(), p);
        }
        for n in [0usize, 253, 254, 255, 507, 508, 509] {
            for fill in [0u8, 1] {
                let p = vec![fill; n];
                let mut out = Vec::new();
                cobs_encode(&p, &mut out);
                assert_eq!(cobs_decode(&out).unwrap(), p, "n={n} fill={fill}");
                assert!(out.len() <= cobs_max_len(n));
            }
        }
    }

    #[test]
    fn frame_round_trip_every_length() {
        let mut f = Framer::new(300).unwrap();
        let mut rng = Rng::new(2);
        for len in 1..=300 {
            let p: Vec<u8> = (0..len).map(|_| rng.next_u64() as u8).collect();
            let wire = frame(&f, &p);
            assert_eq!((wire[0], *wire.last().unwrap()), (0, 0));
            assert!(!wire[1..wire.len() - 1].contains(&0));
            assert!(wire.len() <= f.max_wire_len());
            assert_eq!(f.push(&wire), vec![p]);
        }
        assert_eq!(f.stats().bad_frame, 0);
        assert_eq!(f.stats().frames, 300);
    }

    #[test]
    fn frame_layout_is_exact() {
        // Packet 01 02 03: inner = 00 03 | 01 02 03 | CRC, then COBS, then delimiters.
        let f = Framer::new(10).unwrap();
        let crc = crc16(&[0x00, 0x03, 1, 2, 3]).to_be_bytes();
        let inner = [0x00, 0x03, 1, 2, 3, crc[0], crc[1]];
        let mut want = vec![0];
        cobs_encode(&inner, &mut want);
        want.push(0);
        assert_eq!(frame(&f, &[1, 2, 3]), want);
        // First two bytes inside COBS are the big-endian length.
        let w = frame(&f, &[0xAB; 7]);
        let body = cobs_decode(&w[1..w.len() - 1]).unwrap();
        assert_eq!(&body[..2], &[0, 7]);
        assert_eq!(body.len(), 2 + 7 + 2);
        // Fixed bytes (made with an independent Python script) so the format
        // can't change by accident: CRC 0xEBE1 over 00 03 01 02 03.
        assert_eq!(
            frame(&f, &[1, 2, 3]),
            [0x00, 0x01, 0x07, 0x03, 0x01, 0x02, 0x03, 0xEB, 0xE1, 0x00]
        );
    }

    #[test]
    fn max_size_passes_and_oversize_is_refused() {
        let mut f = Framer::new(DEFAULT_MTU).unwrap();
        for fill in [0u8, 0xFF, 0x55] {
            let p = vec![fill; DEFAULT_MTU];
            let wire = frame(&f, &p);
            assert!(wire.len() <= f.max_wire_len());
            assert_eq!(f.push(&wire), vec![p]);
        }
        let mut out = vec![9];
        assert_eq!(
            f.encode(&vec![1; DEFAULT_MTU + 1], &mut out),
            Err(FrameError::TooLarge {
                len: DEFAULT_MTU + 1,
                mtu: DEFAULT_MTU
            })
        );
        assert_eq!(f.encode(&[], &mut out), Err(FrameError::Empty));
        assert_eq!(out, vec![9], "nothing written on error");
        // A valid frame from a peer with a bigger MTU is rejected and counted.
        let big = Framer::new(1000).unwrap();
        assert!(f.push(&frame(&big, &vec![7; 400])).is_empty());
        assert_eq!(f.stats().oversize, 1);
        // One byte over is rejected too.
        assert!(f.push(&frame(&big, &vec![7; DEFAULT_MTU + 1])).is_empty());
        assert_eq!(f.stats().oversize, 2);
        assert_eq!(f.stats().bad_frame, 2);
        // The framer still works afterwards.
        let ok = frame(&big, &[5, 5]);
        assert_eq!(f.push(&ok), vec![vec![5, 5]]);
    }

    #[test]
    fn long_garbage_without_delimiter_is_bounded() {
        let mut f = Framer::new(50).unwrap();
        assert!(f.push(&vec![0xAAu8; 100_000]).is_empty());
        assert!(f.buf.len() <= f.max_body);
        assert!(f.push(&[0]).is_empty());
        assert_eq!(f.stats().oversize, 1);
        let ok = frame(&f, &[1, 2, 3]);
        assert_eq!(f.push(&ok), vec![vec![1, 2, 3]]);
    }

    #[test]
    fn back_to_back_frames_and_shared_delimiters() {
        let mut f = Framer::new(252).unwrap();
        let mut rng = Rng::new(3);
        let pk: Vec<Vec<u8>> = (0..50).map(|_| packet(&mut rng, 252)).collect();
        let mut wire = Vec::new();
        for p in &pk {
            f.encode(p, &mut wire).unwrap();
        }
        assert_eq!(f.push(&wire), pk);
        // Frames that share one delimiter also work.
        let mut shared = vec![0];
        for p in &pk {
            shared.extend_from_slice(&frame(&f, p)[1..]);
        }
        assert_eq!(f.push(&shared), pk);
        assert_eq!(f.stats().bad_frame, 0);
    }

    #[test]
    fn byte_at_a_time_and_random_chunks() {
        let mut rng = Rng::new(4);
        let pk: Vec<Vec<u8>> = (0..200).map(|_| packet(&mut rng, 252)).collect();
        let mut f = Framer::new(252).unwrap();
        let mut wire = Vec::new();
        for p in &pk {
            f.encode(p, &mut wire).unwrap();
        }
        let mut got = Vec::new();
        for b in &wire {
            got.extend(f.push(&[*b]));
        }
        assert_eq!(got, pk);
        let mut f = Framer::new(252).unwrap();
        let mut got = Vec::new();
        let mut i = 0;
        while i < wire.len() {
            let n = (1 + rng.below(700) as usize).min(wire.len() - i);
            got.extend(f.push(&wire[i..i + n]));
            i += n;
        }
        assert_eq!(got, pk);
    }

    #[test]
    fn every_truncation_is_safe_and_the_next_frame_survives() {
        let f = Framer::new(252).unwrap();
        let mut rng = Rng::new(5);
        for len in [1usize, 2, 3, 10, 100, 252] {
            let p = packet_of(&mut rng, len);
            let next = vec![0xEE, 0, 0x01, 0, 0x7F];
            let w = frame(&f, &p);
            let wn = frame(&f, &next);
            for cut in 0..w.len() {
                // End cut off: the frame is lost; the next one still arrives.
                let mut r = Framer::new(252).unwrap();
                let mut stream = w[..cut].to_vec();
                stream.extend_from_slice(&wn);
                // Losing only the closing delimiter is harmless: the next
                // frame's leading zero closes it.
                let mut want = vec![next.clone()];
                if cut == w.len() - 1 {
                    want.insert(0, p.clone());
                }
                assert_eq!(r.push(&stream), want, "tail cut at {cut}");
                // Alone, a truncated frame delivers nothing.
                let mut r = Framer::new(252).unwrap();
                assert!(r.push(&w[..cut]).is_empty(), "alone {cut}");
                // Start cut off: nothing wrong is delivered, the next frame arrives.
                let mut r = Framer::new(252).unwrap();
                let mut stream = w[cut..].to_vec();
                stream.extend_from_slice(&wn);
                let got = r.push(&stream);
                assert_eq!(got.last(), Some(&next), "head cut at {cut}");
                assert!(got.len() == 1 || got[0] == p, "head cut at {cut}");
            }
        }
    }

    #[test]
    fn noise_between_frames_never_hides_the_next_frame() {
        let mut rng = Rng::new(6);
        let mut f = Framer::new(252).unwrap();
        let mut sent = Vec::new();
        let mut wire = Vec::new();
        for _ in 0..2000 {
            let n = rng.below(400) as usize;
            wire.extend((0..n).map(|_| rng.next_u64() as u8));
            let p = packet(&mut rng, 252);
            f.encode(&p, &mut wire).unwrap();
            sent.push(p);
        }
        let got = f.push(&wire);
        assert_eq!(got, sent, "every frame arrives, none is invented");
        assert!(f.stats().bad_frame > 0, "noise was counted");
    }

    #[test]
    fn every_single_bit_flip_is_caught_and_the_stream_recovers() {
        let f = Framer::new(252).unwrap();
        let after = vec![1u8, 2, 3, 0, 4];
        let wa = frame(&f, &after);
        for p in [
            vec![0x42u8],
            vec![0, 0, 0],
            (0..60).collect::<Vec<u8>>(),
            vec![0xFF; 252],
        ] {
            let w = frame(&f, &p);
            for bit in 0..w.len() * 8 {
                let mut stream = w.clone();
                stream[bit / 8] ^= 1 << (bit % 8);
                stream.extend_from_slice(&wa);
                let mut r = Framer::new(252).unwrap();
                let got = r.push(&stream);
                // A flip can split or merge frames. What arrives is never a
                // changed packet, and the frame after the damage always does.
                assert_eq!(got.last(), Some(&after), "bit {bit}");
                assert!(got.len() == 1 || got[0] == p, "bit {bit}");
            }
        }
    }

    #[test]
    fn random_damage_is_never_delivered_changed() {
        // CRC-16 misses about 1 in 65,536 random changes, so this run is seeded:
        // the same seed gives the same result every time.
        let mut rng = Rng::new(7);
        let f = Framer::new(252).unwrap();
        let after = vec![9u8; 20];
        let wa = frame(&f, &after);
        let (mut wrong, mut detected) = (0, 0);
        for _ in 0..3000 {
            let p = packet(&mut rng, 252);
            let mut w = frame(&f, &p);
            for _ in 0..1 + rng.below(6) {
                let i = rng.below(w.len() as u64) as usize;
                match rng.below(3) {
                    0 => w[i] ^= 1 << rng.below(8),
                    1 => w[i] = rng.next_u64() as u8,
                    _ => {
                        w.remove(i);
                        if w.is_empty() {
                            w.push(0);
                        }
                    }
                }
            }
            w.extend_from_slice(&wa);
            let mut r = Framer::new(252).unwrap();
            let got = r.push(&w);
            assert_eq!(got.last(), Some(&after));
            wrong += got[..got.len() - 1].iter().filter(|g| **g != p).count();
            if r.stats().bad_frame > 0 {
                detected += 1;
            }
        }
        assert_eq!(wrong, 0, "a damaged frame was delivered changed");
        assert!(detected > 2500, "detected {detected}");
    }

    #[test]
    fn mtu_bounds() {
        assert!(Framer::new(0).is_err());
        assert!(Framer::new(MAX_MTU + 1).is_err());
        let mut f = Framer::new(MAX_MTU).unwrap();
        let p = vec![0xA5; MAX_MTU];
        let w = frame(&f, &p);
        assert!(w.len() <= f.max_wire_len());
        assert_eq!(f.push(&w), vec![p]);
    }

    // ---- extended frames, and agreement with the firmware's codec ----

    fn ext_frame(f: &Framer, kind: u8, body: &[u8]) -> Vec<u8> {
        let mut v = Vec::new();
        f.encode_ext(kind, body, &mut v).unwrap();
        v
    }

    #[test]
    fn extended_frames_round_trip_and_push_skips_them() {
        let mut f = Framer::new(255).unwrap();
        let mut rng = Rng::new(21);
        let mut wire = Vec::new();
        let mut want = Vec::new();
        for i in 0..300 {
            let body: Vec<u8> = (0..rng.below(300) as usize)
                .map(|_| rng.next_u64() as u8)
                .collect();
            if i % 2 == 0 {
                f.encode_ext(KIND_AIR, &body, &mut wire).unwrap();
                want.push(Frame::Ext {
                    kind: KIND_AIR,
                    body,
                });
            } else if !body.is_empty() {
                f.encode(&body, &mut wire).unwrap_or(());
                if body.len() <= 255 {
                    want.push(Frame::Data(body));
                }
            }
        }
        assert_eq!(f.push_frames(&wire), want);
        assert_eq!(f.stats().bad_frame, 0);
        let datas = want.iter().filter(|x| matches!(x, Frame::Data(_))).count();
        assert_eq!(f.stats().frames as usize, datas);
        assert_eq!(f.stats().ext_frames as usize, want.len() - datas);
        // `push` gives only the data packets.
        let mut g = Framer::new(255).unwrap();
        let only: Vec<Vec<u8>> = want
            .iter()
            .filter_map(|x| match x {
                Frame::Data(p) => Some(p.clone()),
                _ => None,
            })
            .collect();
        assert_eq!(g.push(&wire), only);
        assert_eq!(g.stats().bad_frame, 0);
        assert!(f
            .encode_ext(1, &vec![0; EXT_MAX_BODY + 1], &mut Vec::new())
            .is_err());
    }

    #[test]
    fn a_stray_zero_after_an_extended_frame_is_not_a_changed_frame() {
        // CRC-16 of data followed by its own CRC is a constant, so without the
        // length field a trailing zero would pass as a changed frame. Flip the
        // closing delimiter to 01 and check nothing wrong is delivered.
        let f = Framer::new(252).unwrap();
        let w = ext_frame(&f, KIND_REQUEST, &[1, 2, 3]);
        let mut bad = w.clone();
        *bad.last_mut().unwrap() = 1;
        bad.extend(frame(&f, &[9]));
        let mut r = Framer::new(252).unwrap();
        assert_eq!(r.push_frames(&bad), vec![Frame::Data(vec![9])]);
    }

    #[test]
    fn extended_bit_flips_are_caught() {
        let f = Framer::new(252).unwrap();
        let after = frame(&f, &[1, 2, 3, 0, 4]);
        for w in [
            ext_frame(&f, KIND_REQUEST, &[3, 2]),
            ext_frame(&f, KIND_AIR, &[0xFF; 100]),
        ] {
            let mut good = Framer::new(252).unwrap();
            let orig = good.push_frames(&w);
            assert_eq!(orig.len(), 1);
            for bit in 0..w.len() * 8 {
                let mut s = w.clone();
                s[bit / 8] ^= 1 << (bit % 8);
                s.extend_from_slice(&after);
                let mut r = Framer::new(252).unwrap();
                let got = r.push_frames(&s);
                assert_eq!(
                    got.last(),
                    Some(&Frame::Data(vec![1, 2, 3, 0, 4])),
                    "bit {bit}"
                );
                assert!(got.len() == 1 || got[0] == orig[0], "bit {bit}");
            }
        }
    }

    #[test]
    fn host_and_firmware_codecs_agree_byte_for_byte() {
        use oasa_modem_core::frame as core;
        let f = Framer::new(255).unwrap();
        let mut rng = Rng::new(22);
        let mut out = vec![0u8; core::MAX_WIRE];
        for _ in 0..500 {
            let p = packet(&mut rng, 255);
            let n = core::encode_data(&p, &mut out).unwrap();
            assert_eq!(&out[..n], &frame(&f, &p)[..], "data {p:02x?}");
            let kind = rng.below(255) as u8;
            let n = core::encode_ext(kind, &p, &mut out).unwrap();
            assert_eq!(&out[..n], &ext_frame(&f, kind, &p)[..], "ext {p:02x?}");
        }
        // And each reads what the other wrote.
        let mut dec = core::Decoder::new();
        let mut host = Framer::new(255).unwrap();
        let mut wire = Vec::new();
        let mut want = Vec::new();
        for i in 0..200 {
            let p = packet(&mut rng, 255);
            if i % 2 == 0 {
                f.encode(&p, &mut wire).unwrap();
                want.push(Frame::Data(p));
            } else {
                f.encode_ext(7, &p, &mut wire).unwrap();
                want.push(Frame::Ext { kind: 7, body: p });
            }
        }
        let mut seen = Vec::new();
        for &b in &wire {
            match dec.push_byte(b) {
                Some(core::Frame::Data(p)) => seen.push(Frame::Data(p.to_vec())),
                Some(core::Frame::Ext { kind, body }) => seen.push(Frame::Ext {
                    kind,
                    body: body.to_vec(),
                }),
                None => {}
            }
        }
        assert_eq!(seen, want);
        assert_eq!(host.push_frames(&wire), want);
        // The firmware encoder's output is read by the host.
        let mut wire = Vec::new();
        for fr in &want {
            let n = match fr {
                Frame::Data(p) => core::encode_data(p, &mut out).unwrap(),
                Frame::Ext { kind, body } => core::encode_ext(*kind, body, &mut out).unwrap(),
            };
            wire.extend_from_slice(&out[..n]);
        }
        let mut host = Framer::new(255).unwrap();
        assert_eq!(host.push_frames(&wire), want);
    }
}
