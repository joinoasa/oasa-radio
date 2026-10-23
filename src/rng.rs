//! A small seeded generator for tests and the test relay. Not for anything
//! secret: it is predictable on purpose, so test runs repeat exactly.

/// Splitmix64.
#[derive(Debug, Clone)]
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        Self(seed)
    }

    pub fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// 0 up to but not including `n` (`n` of 0 gives 0).
    pub fn below(&mut self, n: u64) -> u64 {
        if n == 0 {
            0
        } else {
            self.next_u64() % n
        }
    }

    /// True with probability `rate` (0.0 to 1.0).
    pub fn chance(&mut self, rate: f64) -> bool {
        rate > 0.0 && (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64 <= rate
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rng_repeats_and_spreads() {
        let (mut a, mut b) = (Rng::new(9), Rng::new(9));
        assert!((0..100).all(|_| a.next_u64() == b.next_u64()));
        let mut r = Rng::new(1);
        assert!((0..1000).all(|_| r.below(7) < 7));
        assert_eq!(r.below(0), 0);
        let hits = (0..10_000).filter(|_| r.chance(0.1)).count();
        assert!((800..1200).contains(&hits), "{hits}");
        assert!(!(0..100).any(|_| r.chance(0.0)));
        assert!((0..100).all(|_| r.chance(1.0)));
    }
}
