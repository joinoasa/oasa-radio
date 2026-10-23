//! Consistent Overhead Byte Stuffing, into caller buffers (no allocation).

/// Longest COBS encoding of `n` input bytes.
pub const fn max_len(n: usize) -> usize {
    n + n / 254 + 1
}

/// Encode `input` into `out`. The output has no zero bytes. Returns the
/// length, or `None` if `out` is shorter than [`max_len`].
pub fn encode(input: &[u8], out: &mut [u8]) -> Option<usize> {
    if out.len() < max_len(input.len()) {
        return None;
    }
    let mut o = 1;
    let mut code_at = 0;
    let mut code: u8 = 1;
    for &b in input {
        if b == 0 {
            out[code_at] = code;
            code_at = o;
            o += 1;
            code = 1;
        } else {
            out[o] = b;
            o += 1;
            code += 1;
            if code == 0xFF {
                out[code_at] = code;
                code_at = o;
                o += 1;
                code = 1;
            }
        }
    }
    out[code_at] = code;
    Some(o)
}

/// Decode `input` into `out`. `None` if the input holds a zero byte, a block
/// runs past the end, or `out` is too small.
pub fn decode(input: &[u8], out: &mut [u8]) -> Option<usize> {
    let mut o = 0;
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
        let dst = out.get_mut(o..o + block.len())?;
        dst.copy_from_slice(block);
        o += block.len();
        i = end;
        if code < 0xFF && i < input.len() {
            *out.get_mut(o)? = 0;
            o += 1;
        }
    }
    Some(o)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec::Vec;

    fn enc(p: &[u8]) -> Vec<u8> {
        let mut out = std::vec![0u8; max_len(p.len())];
        let n = encode(p, &mut out).unwrap();
        out.truncate(n);
        out
    }

    fn dec(c: &[u8]) -> Option<Vec<u8>> {
        let mut out = std::vec![0u8; c.len() + 1];
        let n = decode(c, &mut out)?;
        out.truncate(n);
        Some(out)
    }

    #[test]
    fn known_vectors_match_the_host_tests() {
        let cases: &[(&[u8], &[u8])] = &[
            (&[], &[0x01]),
            (&[0x00], &[0x01, 0x01]),
            (&[0x00, 0x00], &[0x01, 0x01, 0x01]),
            (&[0x11, 0x22, 0x00, 0x33], &[0x03, 0x11, 0x22, 0x02, 0x33]),
            (&[0x11, 0x22, 0x33, 0x44], &[0x05, 0x11, 0x22, 0x33, 0x44]),
            (&[0x11, 0x00, 0x00, 0x00], &[0x02, 0x11, 0x01, 0x01, 0x01]),
        ];
        for (plain, coded) in cases {
            assert_eq!(&enc(plain), coded, "encode {plain:02x?}");
            assert_eq!(&dec(coded).unwrap(), plain, "decode {coded:02x?}");
        }
        let p254: Vec<u8> = (1..=254).map(|i| i as u8).collect();
        let e = enc(&p254);
        assert_eq!((e.len(), e[0], e[255]), (256, 0xFF, 0x01));
        assert_eq!(dec(&e).unwrap(), p254);
        let p255: Vec<u8> = (0..255).map(|i| (i % 255 + 1) as u8).collect();
        let e = enc(&p255);
        assert_eq!(
            (e.len(), e[0], e[255], e[256]),
            (257, 0xFF, 0x02, p255[254])
        );
        assert_eq!(dec(&e).unwrap(), p255);
    }

    #[test]
    fn rejects_bad_input() {
        assert!(dec(&[0x00]).is_none());
        assert!(dec(&[0x03, 0x11]).is_none());
        assert!(dec(&[0x03, 0x11, 0x00]).is_none());
        assert!(dec(&[0x02, 0x11, 0x00]).is_none());
    }

    #[test]
    fn small_buffers_are_refused_not_overrun() {
        let mut tiny = [0u8; 3];
        assert!(encode(&[1, 2, 3], &mut tiny).is_none());
        let mut small = [0u8; 2];
        assert!(decode(&[0x04, 1, 2, 3], &mut small).is_none());
        assert!(decode(&[0x02, 1, 0x02, 2], &mut small).is_none());
    }

    #[test]
    fn round_trips_every_length_around_the_block_edges() {
        for n in [0usize, 1, 2, 253, 254, 255, 256, 507, 508, 509, 600] {
            for fill in [0u8, 1, 7] {
                let p = std::vec![fill; n];
                let e = enc(&p);
                assert!(!e.contains(&0));
                assert!(e.len() <= max_len(n));
                assert_eq!(dec(&e).unwrap(), p, "n={n} fill={fill}");
            }
        }
    }
}
