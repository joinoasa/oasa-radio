//! The frame format and its small pieces: CRC-16, whitening, the preamble sequence, bit packing.
//!
//! ```text
//! | preamble: 63 chips | sync word: 16 bits | length: 8 | length complement: 8 | payload | CRC-16: 16 |
//!   one bit per symbol (BPSK or FSK)          |<-------- whitened, then sent with the payload modulation -------->|
//! ```
//!
//! The CRC covers the length byte and the payload. Bits go out most significant bit first.

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
}
