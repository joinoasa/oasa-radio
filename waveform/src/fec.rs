//! Convolutional code, K = 7, rate 1/2, generators 171 and 133 (octal), and a soft-decision
//! Viterbi decoder. The frame v1 header uses it (see `docs/frame-v1.md`); the payload error
//! correction tier will reuse it.
//!
//! Encoder: for each input bit `u`, the register is `u` followed by the six previous inputs (newest
//! on the left, the generator's most significant bit). Two output bits per input, in the order
//! 171, 133. The input is followed by six zero tail bits that bring the register back to zero, so a
//! block of `n` bits makes `2 * (n + 6)` coded bits and the decoder knows where it ends.
//!
//! **Soft value convention:** a soft value is positive for a coded bit 0 and negative for a 1; its
//! size is the confidence. For BPSK it is the matched filter output as is (bit 0 is the positive
//! symbol). The decoder maximizes the correlation between the soft values and the code word, which
//! is the maximum likelihood choice in Gaussian noise. A value of 0 is an erasure.

/// Constraint length.
pub const K: usize = 7;
/// Zero bits that end a block.
pub const TAIL_BITS: usize = K - 1;
/// Generator polynomials, in the order the two output bits are sent.
pub const GENERATORS: [u8; 2] = [0o171, 0o133];

const STATES: usize = 1 << (K - 1);

fn parity(x: u8) -> u8 {
    (x.count_ones() & 1) as u8
}

/// The two output bits for input `u` from register state `state` (previous inputs, newest in bit 5).
fn outputs(state: usize, u: u8) -> [u8; 2] {
    let reg = ((u as usize) << (K - 1)) | state;
    [
        parity(reg as u8 & GENERATORS[0]),
        parity(reg as u8 & GENERATORS[1]),
    ]
}

/// Number of coded bits for `n_info` information bits.
pub fn coded_len(n_info: usize) -> usize {
    2 * (n_info + TAIL_BITS)
}

/// Encode `bits` (each 0 or 1), terminated with six zero tail bits. Makes `coded_len(bits.len())` bits.
pub fn conv_encode(bits: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(coded_len(bits.len()));
    let mut state = 0usize;
    for &u in bits.iter().chain(&[0u8; TAIL_BITS]) {
        let u = u & 1;
        out.extend(outputs(state, u));
        state = (state >> 1) | ((u as usize) << (K - 2));
    }
    out
}

/// Soft-decision Viterbi decoding of a terminated block: `soft` holds `coded_len(n_info)` values
/// (positive means bit 0, see the module docs). Returns the `n_info` information bits.
///
/// Panics if `soft` has the wrong length.
pub fn viterbi_decode(soft: &[f32], n_info: usize) -> Vec<u8> {
    assert_eq!(soft.len(), coded_len(n_info), "wrong number of soft values");
    let steps = n_info + TAIL_BITS;
    // the best way into each state: the encoder starts in state 0
    let mut metric = [f32::NEG_INFINITY; STATES];
    metric[0] = 0.0;
    // for each step and state, the bit that fell out of the register on the best path
    let mut dropped = vec![[0u8; STATES]; steps];
    for (t, pair) in soft.chunks_exact(2).enumerate() {
        let mut next = [f32::NEG_INFINITY; STATES];
        for (ns, slot) in next.iter_mut().enumerate() {
            let u = (ns >> (K - 2)) as u8;
            for b in 0..2usize {
                let ps = ((ns & (STATES / 2 - 1)) << 1) | b;
                let o = outputs(ps, u);
                let bm: f32 = o
                    .iter()
                    .zip(pair)
                    .map(|(&bit, &s)| if bit == 0 { s } else { -s })
                    .sum();
                let cand = metric[ps] + bm;
                if cand > *slot || (b == 0 && cand == *slot) {
                    *slot = cand;
                    dropped[t][ns] = b as u8;
                }
            }
        }
        metric = next;
    }
    // the tail ends the block in state 0
    let mut state = 0usize;
    let mut bits = vec![0u8; steps];
    for t in (0..steps).rev() {
        bits[t] = (state >> (K - 2)) as u8;
        state = ((state & (STATES / 2 - 1)) << 1) | dropped[t][state] as usize;
    }
    bits.truncate(n_info);
    bits
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rng::Rng;

    fn s(bits: &str) -> Vec<u8> {
        bits.bytes().map(|c| c - b'0').collect()
    }

    fn hard(coded: &[u8]) -> Vec<f32> {
        coded.iter().map(|&b| 1.0 - 2.0 * b as f32).collect()
    }

    fn random_bits(rng: &mut Rng, n: usize) -> Vec<u8> {
        (0..n).map(|_| (rng.next_u64() & 1) as u8).collect()
    }

    #[test]
    fn encoder_known_answers() {
        // A single 1 followed by zeros gives the generators themselves, one bit pair at a time:
        // 171 = 1111001 and 133 = 1011011 read from the most significant bit.
        assert_eq!(conv_encode(&[1]), s("11101111000111"));
        // Checked against an independent Python implementation (a shift register held as a list).
        let v = [1, 0, 1, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0, 0, 1, 1];
        assert_eq!(
            conv_encode(&v),
            s("11100010010111111001101111100111100011011011")
        );
        assert!(conv_encode(&[0; 10]).iter().all(|&b| b == 0));
        assert_eq!(conv_encode(&v).len(), coded_len(v.len()));
    }

    #[test]
    fn round_trip_without_noise() {
        let mut rng = Rng::new(1);
        for n in [0usize, 1, 7, 32, 100] {
            let bits = random_bits(&mut rng, n);
            assert_eq!(viterbi_decode(&hard(&conv_encode(&bits)), n), bits, "n {n}");
        }
    }

    #[test]
    fn corrects_two_errors_anywhere_in_a_32_bit_block() {
        let mut rng = Rng::new(2);
        for _ in 0..300 {
            let bits = random_bits(&mut rng, 32);
            let mut soft = hard(&conv_encode(&bits));
            let a = rng.below(soft.len() as u64) as usize;
            let b = rng.below(soft.len() as u64) as usize;
            soft[a] = -soft[a];
            soft[b] = -soft[b];
            assert_eq!(viterbi_decode(&soft, 32), bits);
        }
        // free distance 10: any 4 errors are corrected too
        for _ in 0..300 {
            let bits = random_bits(&mut rng, 32);
            let mut soft = hard(&conv_encode(&bits));
            for _ in 0..4 {
                let a = rng.below(soft.len() as u64) as usize;
                soft[a] = -soft[a];
            }
            assert_eq!(viterbi_decode(&soft, 32), bits);
        }
    }

    #[test]
    fn soft_values_beat_hard_decisions() {
        // Eight coded bits are flipped, but weakly (a small value of the wrong sign); the rest are
        // strong. A decoder that uses the sizes sees through them; one given only the signs does not.
        let mut rng = Rng::new(3);
        let (mut soft_wins, mut hard_wins) = (0, 0);
        for _ in 0..200 {
            let bits = random_bits(&mut rng, 32);
            let coded = conv_encode(&bits);
            let mut soft = hard(&coded);
            for _ in 0..8 {
                let a = rng.below(soft.len() as u64) as usize;
                soft[a] = -0.1 * soft[a].signum();
            }
            let hardened: Vec<f32> = soft.iter().map(|x| x.signum()).collect();
            soft_wins += (viterbi_decode(&soft, 32) == bits) as u32;
            hard_wins += (viterbi_decode(&hardened, 32) == bits) as u32;
        }
        assert_eq!(soft_wins, 200, "soft decoding failed on weak errors");
        assert!(hard_wins < soft_wins, "hard {hard_wins} soft {soft_wins}");
    }

    #[test]
    fn all_erasures_still_return_the_right_length() {
        let out = viterbi_decode(&vec![0.0; coded_len(32)], 32);
        assert_eq!(out.len(), 32);
    }
}
