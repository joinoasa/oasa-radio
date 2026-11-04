//! The frame formats and their small pieces: CRC-8 and CRC-16, whitening, the preamble sequence,
//! the v1 header, bit packing.
//!
//! **W0** (the default):
//!
//! ```text
//! | preamble: 63 chips | sync word: 16 bits | length: 8 | length complement: 8 | payload | CRC-16: 16 |
//!   one bit per symbol (BPSK or FSK)          |<-------- whitened, then sent with the payload modulation -------->|
//! ```
//!
//! The CRC covers the length byte and the payload. Bits go out most significant bit first.
//!
//! **V1** (protected header, see `docs/frame-v1.md`):
//!
//! ```text
//! | preamble: 63 | sync word: 16 | header: 152 symbols                 | payload | CRC-16: 16 |
//!   one bit per symbol            | one bit per symbol                  | whitened, payload modulation |
//!
//! header fields, 24 bits, most significant bit first:
//! | version 2 | extension 1 | modulation 2 | payload_code 3 | pilots 1 | length 8 | channel_plan 4 | coexistence 3 |
//!
//! header, 152 symbols: fields (24) + CRC-8 (8) = 32 bits
//!   -> whiten (PN9) -> convolutional code K=7 171/133, 6 tail bits = 76 coded bits (A)
//!   -> send A, then A xor PN9 (76 bits) = 152 symbols
//! ```
//!
//! The CRC-16 of v1 covers the three header field bytes and the payload.

use crate::fec::{coded_len, conv_encode, viterbi_decode};

/// Which frame format a modem sends and expects.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum FrameFormat {
    /// Length byte and its complement, unprotected. The original W0 format.
    #[default]
    W0,
    /// Protected header: fields, CRC-8, convolutional code, sent twice.
    V1,
}

impl FrameFormat {
    pub fn name(self) -> &'static str {
        match self {
            FrameFormat::W0 => "w0",
            FrameFormat::V1 => "v1",
        }
    }

    pub fn parse(s: &str) -> Option<Self> {
        match s {
            "w0" => Some(FrameFormat::W0),
            "v1" => Some(FrameFormat::V1),
            _ => None,
        }
    }
}

/// Longest payload a frame carries (the length field is one byte).
pub const MAX_PAYLOAD: usize = 255;
/// Length of the preamble m-sequence, in chips (one per symbol).
pub const PREAMBLE_LEN: usize = 63;
/// Default sync word. Any 16-bit value works; two networks using different words ignore each other.
pub const DEFAULT_SYNC_WORD: u16 = 0x2DD4;

/// CRC-16/CCITT-FALSE: polynomial 0x1021, initial value 0xFFFF, no reflection, no final xor.
/// Check value: `crc16(b"123456789") == 0x29B1`.
pub fn crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in data {
        crc ^= (b as u16) << 8;
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

/// CRC-8: polynomial 0x07, initial value 0xFF, no reflection, no final xor.
/// Check value: `crc8(b"123456789") == 0xFB`.
pub fn crc8(data: &[u8]) -> u8 {
    let mut crc: u8 = 0xFF;
    for &b in data {
        crc ^= b;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 {
                (crc << 1) ^ 0x07
            } else {
                crc << 1
            };
        }
    }
    crc
}

/// The preamble: a 63-chip maximal-length sequence (LFSR `a[n+6] = a[n+1] xor a[n]`, from `x^6 + x + 1`),
/// as bits. Its periodic autocorrelation is 63 at zero shift and -1 elsewhere, so a correlator has a
/// sharp, unambiguous peak.
pub fn preamble_bits() -> [u8; PREAMBLE_LEN] {
    let mut a = [0u8; PREAMBLE_LEN + 6];
    a[0] = 1;
    for n in 0..PREAMBLE_LEN {
        a[n + 6] = a[n + 1] ^ a[n];
    }
    let mut out = [0u8; PREAMBLE_LEN];
    out.copy_from_slice(&a[..PREAMBLE_LEN]);
    out
}

/// Whitening: xor with the PN9 sequence (`x^9 + x^5 + 1`, all ones at the start). Doing it twice
/// gives the data back. It keeps long runs of equal bits (all-zero payloads) from putting a
/// tone in the spectrum or starving a clock recovery loop.
pub fn whiten(bits: &mut [u8]) {
    let mut s: u16 = 0x1FF;
    for b in bits.iter_mut() {
        *b ^= (s & 1) as u8;
        let fb = (s ^ (s >> 4)) & 1;
        s = (s >> 1) | (fb << 8);
    }
}

/// Bytes to bits, most significant bit first. Each bit is 0 or 1.
pub fn bytes_to_bits(bytes: &[u8]) -> Vec<u8> {
    bytes
        .iter()
        .flat_map(|&b| (0..8).rev().map(move |i| (b >> i) & 1))
        .collect()
}

/// Bits (0 or 1, most significant first) to bytes. A trailing partial byte is dropped.
pub fn bits_to_bytes(bits: &[u8]) -> Vec<u8> {
    bits.chunks_exact(8)
        .map(|c| c.iter().fold(0u8, |a, &b| (a << 1) | (b & 1)))
        .collect()
}

/// The sync word as 16 bits.
pub fn sync_bits(word: u16) -> Vec<u8> {
    bytes_to_bits(&word.to_be_bytes())
}

/// The whitened bits that follow the sync word: length, its complement, payload, CRC.
/// `payload.len()` must be at most [`MAX_PAYLOAD`].
pub fn body_bits(payload: &[u8]) -> Vec<u8> {
    assert!(payload.len() <= MAX_PAYLOAD, "payload too long");
    let len = payload.len() as u8;
    let mut bytes = vec![len, !len];
    bytes.extend_from_slice(payload);
    let mut crc_in = vec![len];
    crc_in.extend_from_slice(payload);
    bytes.extend_from_slice(&crc16(&crc_in).to_be_bytes());
    let mut bits = bytes_to_bits(&bytes);
    whiten(&mut bits);
    bits
}

/// Number of bits [`body_bits`] makes for a payload of `n` bytes.
pub fn body_len_bits(n: usize) -> usize {
    (2 + n + 2) * 8
}

/// Parse whitened header bits (the first 16 body bits): the payload length, or `None` if the
/// complement byte does not match.
pub fn parse_header(header_bits: &[u8]) -> Option<usize> {
    let mut b = header_bits[..16].to_vec();
    whiten(&mut b);
    let bytes = bits_to_bytes(&b);
    (bytes[0] == !bytes[1]).then_some(bytes[0] as usize)
}

/// Take whitened body bits (header, payload, CRC; `body_len_bits(n)` of them) apart.
/// Returns the payload and whether the CRC matched.
pub fn parse_body(bits: &[u8], n: usize) -> (Vec<u8>, bool) {
    let mut b = bits[..body_len_bits(n)].to_vec();
    whiten(&mut b);
    let bytes = bits_to_bytes(&b);
    let payload = bytes[2..2 + n].to_vec();
    let got = u16::from_be_bytes([bytes[2 + n], bytes[3 + n]]);
    let mut crc_in = vec![n as u8];
    crc_in.extend_from_slice(&payload);
    (payload, crc16(&crc_in) == got)
}

/// Header field bytes, then CRC-8, as bits: 32 bits.
pub const HEADER_BLOCK_BITS: usize = 32;
/// Coded bits of one copy of the v1 header (32 bits plus 6 tail bits, rate 1/2).
pub const HEADER_CODED_BITS: usize = 76;
/// Symbols the v1 header takes: the coded bits, then the coded bits xored with PN9.
pub const HEADER_SYMBOLS: usize = 2 * HEADER_CODED_BITS;

/// The v1 header fields (24 bits). Values are taken as they are; each must fit its width
/// (2, 1, 2, 3, 1, 8, 4, 3 bits), higher bits are dropped when sent.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct HeaderFields {
    /// This format is 0.
    pub version: u8,
    /// 1: more header follows. Not defined yet; v1 receivers reject it.
    pub extension: u8,
    /// 0 BPSK, 1 QPSK, 2 2-FSK, 3 reserved: the modulation of the body.
    pub modulation: u8,
    /// 0: none. 1 to 7 are reserved for error-correction tiers; v1 receivers reject them.
    pub payload_code: u8,
    /// Not defined yet; v1 receivers reject 1.
    pub pilots: u8,
    /// Payload bytes, 0 to 255.
    pub length: u8,
    /// Reserved, sent as 0, ignored and reported by the receiver.
    pub channel_plan: u8,
    /// Reserved flags, sent as 0, ignored and reported by the receiver.
    pub coexistence: u8,
}

impl HeaderFields {
    /// The fields of an ordinary v1 frame: `modulation` code and payload length, the rest 0.
    pub fn new(modulation: u8, length: u8) -> Self {
        Self {
            modulation,
            length,
            ..Self::default()
        }
    }

    /// The 24 bits as three bytes, first field in the top bits of the first byte.
    pub fn to_bytes(&self) -> [u8; 3] {
        let v = ((self.version as u32 & 3) << 22)
            | ((self.extension as u32 & 1) << 21)
            | ((self.modulation as u32 & 3) << 19)
            | ((self.payload_code as u32 & 7) << 16)
            | ((self.pilots as u32 & 1) << 15)
            | ((self.length as u32) << 7)
            | ((self.channel_plan as u32 & 15) << 3)
            | (self.coexistence as u32 & 7);
        [(v >> 16) as u8, (v >> 8) as u8, v as u8]
    }

    pub fn from_bytes(b: [u8; 3]) -> Self {
        let v = (b[0] as u32) << 16 | (b[1] as u32) << 8 | b[2] as u32;
        Self {
            version: (v >> 22) as u8 & 3,
            extension: (v >> 21) as u8 & 1,
            modulation: (v >> 19) as u8 & 3,
            payload_code: (v >> 16) as u8 & 7,
            pilots: (v >> 15) as u8 & 1,
            length: (v >> 7) as u8,
            channel_plan: (v >> 3) as u8 & 15,
            coexistence: v as u8 & 7,
        }
    }

    /// The rules a v1 receiver applies: version 0, no extension, no pilots, no payload code, and
    /// the modulation it is set up for (`modulation_code`).
    pub fn acceptable(&self, modulation_code: u8) -> bool {
        self.version == 0
            && self.extension == 0
            && self.pilots == 0
            && self.payload_code == 0
            && self.modulation == modulation_code
    }
}

/// PN9 xor sequence for the second copy of the coded header: `whiten` applied to zeros.
fn second_copy_mask() -> [u8; HEADER_CODED_BITS] {
    let mut m = [0u8; HEADER_CODED_BITS];
    whiten(&mut m);
    m
}

/// The 152 header symbols (bits) of a v1 frame: fields and CRC-8, whitened, coded, then the coded
/// bits again xored with a fixed sequence.
pub fn header_symbols(h: &HeaderFields) -> Vec<u8> {
    let f = h.to_bytes();
    header_symbols_raw([f[0], f[1], f[2], crc8(&f)])
}

/// [`header_symbols`] for a 32-bit block (three field bytes and a CRC-8) given as it is, so a
/// test can send a block with a wrong CRC-8.
pub fn header_symbols_raw(block: [u8; 4]) -> Vec<u8> {
    let mut bits = bytes_to_bits(&block);
    whiten(&mut bits);
    let a = conv_encode(&bits);
    debug_assert_eq!(a.len(), coded_len(HEADER_BLOCK_BITS));
    let mask = second_copy_mask();
    let mut out = a.clone();
    out.extend(a.iter().zip(&mask).map(|(&x, &m)| x ^ m));
    out
}

/// Decode the 152 header symbols from soft values (positive: bit 0; see [`crate::fec`]): undo the
/// xor of the second copy, add the two soft values of each coded bit, Viterbi decode, unwhiten,
/// check the CRC-8. `None` if the CRC-8 does not match. The field rules are not applied here.
pub fn parse_header_v1(soft: &[f32]) -> Option<HeaderFields> {
    assert_eq!(
        soft.len(),
        HEADER_SYMBOLS,
        "need {HEADER_SYMBOLS} soft values"
    );
    let mask = second_copy_mask();
    let combined: Vec<f32> = (0..HEADER_CODED_BITS)
        .map(|i| {
            let b = soft[HEADER_CODED_BITS + i];
            soft[i] + if mask[i] == 1 { -b } else { b }
        })
        .collect();
    let mut bits = viterbi_decode(&combined, HEADER_BLOCK_BITS);
    whiten(&mut bits);
    let bytes = bits_to_bytes(&bits);
    (crc8(&bytes[..3]) == bytes[3])
        .then(|| HeaderFields::from_bytes([bytes[0], bytes[1], bytes[2]]))
}

/// The whitened body bits of a v1 frame: payload, then CRC-16 over the three header bytes and the
/// payload. `payload.len()` must be at most [`MAX_PAYLOAD`].
pub fn body_bits_v1(h: &HeaderFields, payload: &[u8]) -> Vec<u8> {
    assert!(payload.len() <= MAX_PAYLOAD, "payload too long");
    let mut crc_in = h.to_bytes().to_vec();
    crc_in.extend_from_slice(payload);
    let mut bytes = payload.to_vec();
    bytes.extend_from_slice(&crc16(&crc_in).to_be_bytes());
    let mut bits = bytes_to_bits(&bytes);
    whiten(&mut bits);
    bits
}

/// Number of bits [`body_bits_v1`] makes for a payload of `n` bytes.
pub fn body_len_bits_v1(n: usize) -> usize {
    (n + 2) * 8
}

/// Take whitened v1 body bits apart (`body_len_bits_v1(n)` of them). Returns the payload and
/// whether the CRC-16 matched, given the header the receiver decoded.
pub fn parse_body_v1(bits: &[u8], h: &HeaderFields, n: usize) -> (Vec<u8>, bool) {
    let mut b = bits[..body_len_bits_v1(n)].to_vec();
    whiten(&mut b);
    let bytes = bits_to_bytes(&b);
    let payload = bytes[..n].to_vec();
    let got = u16::from_be_bytes([bytes[n], bytes[n + 1]]);
    let mut crc_in = h.to_bytes().to_vec();
    crc_in.extend_from_slice(&payload);
    (payload, crc16(&crc_in) == got)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc16_known_answer() {
        assert_eq!(crc16(b"123456789"), 0x29B1);
        assert_eq!(crc16(b""), 0xFFFF);
    }

    #[test]
    fn preamble_is_a_maximal_length_sequence() {
        let p = preamble_bits();
        assert_eq!(p.iter().map(|&b| b as usize).sum::<usize>(), 32);
        let s: Vec<i32> = p.iter().map(|&b| 1 - 2 * b as i32).collect();
        // periodic autocorrelation: 63 at 0, -1 elsewhere
        for lag in 0..63usize {
            let c: i32 = (0..63).map(|i| s[i] * s[(i + lag) % 63]).sum();
            assert_eq!(c, if lag == 0 { 63 } else { -1 }, "lag {lag}");
        }
        // aperiodic sidelobes (what a correlator on a stream sees) stay small
        let worst = (1..63usize)
            .map(|lag| (0..63 - lag).map(|i| s[i] * s[i + lag]).sum::<i32>().abs())
            .max()
            .unwrap();
        assert!(worst <= 9, "worst aperiodic sidelobe {worst}");
    }

    #[test]
    fn whitening_is_its_own_inverse_and_balances_zeros() {
        let mut z = vec![0u8; 4096];
        whiten(&mut z);
        let ones = z.iter().map(|&b| b as usize).sum::<usize>();
        assert!((ones as i64 - 2048).abs() < 100, "ones {ones}");
        whiten(&mut z);
        assert!(z.iter().all(|&b| b == 0));
    }

    #[test]
    fn bits_bytes_round_trip() {
        let v = vec![0x00, 0xFF, 0xA5, 0x3C];
        assert_eq!(bits_to_bytes(&bytes_to_bits(&v)), v);
        assert_eq!(bytes_to_bits(&[0x80, 0x01])[..8], [1, 0, 0, 0, 0, 0, 0, 0]);
        assert_eq!(sync_bits(0x2DD4).len(), 16);
    }

    #[test]
    fn body_round_trip_and_crc_catches_every_single_bit_flip() {
        let payload: Vec<u8> = (0..20u8).map(|i| i.wrapping_mul(37)).collect();
        let body = body_bits(&payload);
        assert_eq!(body.len(), body_len_bits(20));
        assert_eq!(parse_header(&body), Some(20));
        assert_eq!(parse_body(&body, 20), (payload.clone(), true));
        for i in 0..body.len() {
            let mut bad = body.clone();
            bad[i] ^= 1;
            let (_, ok) = parse_body(&bad, 20);
            assert!(!ok || i < 16, "flip {i} went unseen");
        }
    }

    #[test]
    fn header_complement_rejects_a_damaged_length() {
        let mut body = body_bits(&[1, 2, 3]);
        body[3] ^= 1;
        assert_eq!(parse_header(&body), None);
    }

    #[test]
    fn crc8_known_answers() {
        assert_eq!(crc8(b"123456789"), 0xFB);
        assert_eq!(crc8(&[0, 0, 0]), 0x2B);
        assert_eq!(crc8(b""), 0xFF);
    }

    #[test]
    fn header_fields_layout() {
        let h = HeaderFields {
            version: 1,
            extension: 1,
            modulation: 2,
            payload_code: 5,
            pilots: 1,
            length: 0xA5,
            channel_plan: 0xA,
            coexistence: 5,
        };
        // 01 1 10 101 | 1 1010010 | 1 1010 101
        assert_eq!(h.to_bytes(), [0b0111_0101, 0b1101_0010, 0b1101_0101]);
        assert_eq!(HeaderFields::from_bytes(h.to_bytes()), h);
        assert_eq!(
            HeaderFields::new(1, 7).to_bytes(),
            [0b0000_1000, 0b0000_0011, 0b1000_0000]
        );
        // every field alone lands in its own bits
        for v in 0..=255u8 {
            let h = HeaderFields::new(0, v);
            assert_eq!(HeaderFields::from_bytes(h.to_bytes()).length, v);
        }
    }

    #[test]
    fn header_symbols_round_trip_and_the_two_copies_differ() {
        let h = HeaderFields {
            channel_plan: 9,
            coexistence: 3,
            ..HeaderFields::new(1, 200)
        };
        let sym = header_symbols(&h);
        assert_eq!(sym.len(), HEADER_SYMBOLS);
        assert_ne!(sym[..76], sym[76..]);
        let soft: Vec<f32> = sym.iter().map(|&b| 1.0 - 2.0 * b as f32).collect();
        assert_eq!(parse_header_v1(&soft), Some(h));
        // an all-zero header is not a run of equal symbols
        let z = header_symbols(&HeaderFields::default());
        let ones = z.iter().map(|&b| b as usize).sum::<usize>();
        assert!((40..112).contains(&ones), "ones {ones}");
    }

    #[test]
    fn header_crc8_rejects_damage_the_code_cannot_fix() {
        // Flip far more symbols than the code can correct: it decodes to something else, and the
        // CRC-8 (or, rarely, a lucky match) decides. Over many tries almost all are caught.
        let h = HeaderFields::new(0, 64);
        let soft: Vec<f32> = header_symbols(&h)
            .iter()
            .map(|&b| 1.0 - 2.0 * b as f32)
            .collect();
        let mut rng = crate::rng::Rng::new(4);
        let (mut caught, mut wrong) = (0, 0);
        for _ in 0..400 {
            let mut s = soft.clone();
            for x in s.iter_mut() {
                if rng.below(2) == 0 {
                    *x = -*x;
                }
            }
            match parse_header_v1(&s) {
                None => caught += 1,
                Some(g) if g != h => wrong += 1,
                Some(_) => {}
            }
        }
        assert!(caught > 380, "caught {caught}, accepted wrong {wrong}");
    }

    #[test]
    fn header_rules() {
        let ok = HeaderFields::new(0, 5);
        assert!(ok.acceptable(0));
        assert!(!ok.acceptable(1));
        for bad in [
            HeaderFields { version: 1, ..ok },
            HeaderFields { extension: 1, ..ok },
            HeaderFields { pilots: 1, ..ok },
            HeaderFields {
                payload_code: 3,
                ..ok
            },
        ] {
            assert!(!bad.acceptable(0), "{bad:?}");
        }
        let reserved = HeaderFields {
            channel_plan: 15,
            coexistence: 7,
            ..ok
        };
        assert!(reserved.acceptable(0));
    }

    #[test]
    fn v1_body_crc_covers_the_header_bytes() {
        let payload: Vec<u8> = (0..20u8).collect();
        let h = HeaderFields::new(0, 20);
        let body = body_bits_v1(&h, &payload);
        assert_eq!(body.len(), body_len_bits_v1(20));
        assert_eq!(parse_body_v1(&body, &h, 20), (payload.clone(), true));
        // a header that miscorrected to another length (or any other field) fails the CRC-16
        for other in [
            HeaderFields::new(0, 21),
            HeaderFields::new(0, 4),
            HeaderFields {
                channel_plan: 1,
                ..h
            },
        ] {
            assert!(!parse_body_v1(&body, &other, 20).1, "{other:?}");
        }
        for i in 0..body.len() {
            let mut bad = body.clone();
            bad[i] ^= 1;
            assert!(!parse_body_v1(&bad, &h, 20).1, "flip {i}");
        }
    }
}
