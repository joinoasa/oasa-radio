//! A small seeded generator, so every run repeats exactly from its seed. Not for anything secret.
//!
//! xoshiro256** (public domain, Blackman and Vigna), seeded through splitmix64. Gaussian numbers
//! come from the Box-Muller transform.

#[derive(Debug, Clone)]
pub struct Rng {
    s: [u64; 4],
    spare: Option<f64>,
}

fn splitmix64(state: &mut u64) -> u64 {
    *state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut z = *state;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

impl Rng {
    pub fn new(seed: u64) -> Self {
        let mut sm = seed;
        let s = [
            splitmix64(&mut sm),
            splitmix64(&mut sm),
            splitmix64(&mut sm),
            splitmix64(&mut sm),
        ];
        Self { s, spare: None }
    }

    pub fn next_u64(&mut self) -> u64 {
        let r = self.s[1].wrapping_mul(5).rotate_left(7).wrapping_mul(9);
        let t = self.s[1] << 17;
        self.s[2] ^= self.s[0];
        self.s[3] ^= self.s[1];
        self.s[1] ^= self.s[2];
        self.s[0] ^= self.s[3];
        self.s[2] ^= t;
        self.s[3] = self.s[3].rotate_left(45);
        r
    }

    /// Uniform in `[0, 1)`, 53 bits.
    pub fn uniform(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// 0 up to but not including `n` (`n` of 0 gives 0).
    pub fn below(&mut self, n: u64) -> u64 {
        if n == 0 {
            0
        } else {
            self.next_u64() % n
        }
    }

    pub fn bytes(&mut self, n: usize) -> Vec<u8> {
        (0..n).map(|_| (self.next_u64() >> 56) as u8).collect()
    }

    /// A standard normal number (mean 0, variance 1), Box-Muller.
    pub fn gaussian(&mut self) -> f64 {
        if let Some(g) = self.spare.take() {
            return g;
        }
        // 1 - u is in (0, 1], so the logarithm is finite.
        let u1 = 1.0 - self.uniform();
        let u2 = self.uniform();
        let r = (-2.0 * u1.ln()).sqrt();
        let (s, c) = (2.0 * std::f64::consts::PI * u2).sin_cos();
        self.spare = Some(r * s);
        r * c
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn same_seed_same_numbers() {
        let mut a = Rng::new(7);
        let mut b = Rng::new(7);
        let mut c = Rng::new(8);
        let va: Vec<u64> = (0..8).map(|_| a.next_u64()).collect();
        let vb: Vec<u64> = (0..8).map(|_| b.next_u64()).collect();
        let vc: Vec<u64> = (0..8).map(|_| c.next_u64()).collect();
        assert_eq!(va, vb);
        assert_ne!(va, vc);
    }

    #[test]
    fn gaussian_has_unit_variance_and_right_tails() {
        let mut r = Rng::new(1);
        let n = 400_000;
        let (mut s, mut s2, mut tail) = (0.0, 0.0, 0usize);
        for _ in 0..n {
            let g = r.gaussian();
            s += g;
            s2 += g * g;
            if g > 2.0 {
                tail += 1;
            }
        }
        let mean = s / n as f64;
        let var = s2 / n as f64 - mean * mean;
        assert!(mean.abs() < 0.01, "mean {mean}");
        assert!((var - 1.0).abs() < 0.01, "var {var}");
        // P(g > 2) = 0.02275
        let p = tail as f64 / n as f64;
        assert!((p - 0.02275).abs() < 0.0015, "tail {p}");
    }
}
