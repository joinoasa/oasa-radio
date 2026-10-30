//! A small radix-2 FFT for the receiver's coarse frequency search. Forward transform only:
//! `X[k] = sum_n x[n] exp(-j 2 pi k n / N)`, no scaling. Sizes are powers of two.

use crate::iq::Complex32;
use std::f64::consts::PI;

#[derive(Debug, Clone)]
pub struct Fft {
    n: usize,
    /// `exp(-j 2 pi k / n)` for `k < n / 2`.
    twiddle: Vec<Complex32>,
    /// Bit-reversed index of every position.
    rev: Vec<u32>,
}

impl Fft {
    /// A plan for size `n`, a power of two of at least 2.
    pub fn new(n: usize) -> Self {
        assert!(
            n >= 2 && n.is_power_of_two(),
            "FFT size must be a power of two"
        );
        let bits = n.trailing_zeros();
        let twiddle = (0..n / 2)
            .map(|k| {
                let (s, c) = (-2.0 * PI * k as f64 / n as f64).sin_cos();
                Complex32::new(c as f32, s as f32)
            })
            .collect();
        let rev = (0..n as u32)
            .map(|i| i.reverse_bits() >> (32 - bits))
            .collect();
        Self { n, twiddle, rev }
    }

    pub fn len(&self) -> usize {
        self.n
    }

    pub fn is_empty(&self) -> bool {
        false
    }

    /// Transform `x` (length `n`) in place.
    pub fn forward(&self, x: &mut [Complex32]) {
        assert_eq!(x.len(), self.n);
        for i in 0..self.n {
            let j = self.rev[i] as usize;
            if i < j {
                x.swap(i, j);
            }
        }
        let mut half = 1;
        while half < self.n {
            let stride = self.n / (2 * half);
            for start in (0..self.n).step_by(2 * half) {
                for k in 0..half {
                    let w = self.twiddle[k * stride];
                    let a = x[start + k];
                    let b = x[start + k + half] * w;
                    x[start + k] = a + b;
                    x[start + k + half] = a - b;
                }
            }
            half *= 2;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rng::Rng;

    fn dft(x: &[Complex32]) -> Vec<(f64, f64)> {
        let n = x.len();
        (0..n)
            .map(|k| {
                let (mut re, mut im) = (0.0f64, 0.0f64);
                for (i, s) in x.iter().enumerate() {
                    let (sn, cs) = (-2.0 * PI * (k * i % n) as f64 / n as f64).sin_cos();
                    re += s.re as f64 * cs - s.im as f64 * sn;
                    im += s.re as f64 * sn + s.im as f64 * cs;
                }
                (re, im)
            })
            .collect()
    }

    #[test]
    fn matches_a_direct_dft() {
        let mut rng = Rng::new(5);
        for n in [2usize, 4, 8, 64, 1024] {
            let x: Vec<Complex32> = (0..n)
                .map(|_| Complex32::new(rng.gaussian() as f32, rng.gaussian() as f32))
                .collect();
            let want = dft(&x);
            let mut got = x.clone();
            Fft::new(n).forward(&mut got);
            let scale = (n as f64).sqrt();
            for (k, (g, w)) in got.iter().zip(&want).enumerate() {
                let e = ((g.re as f64 - w.0).powi(2) + (g.im as f64 - w.1).powi(2)).sqrt();
                assert!(e < 1e-4 * scale, "n {n} bin {k}: error {e}");
            }
        }
    }

    #[test]
    fn a_tone_lands_in_its_bin() {
        let n = 256;
        let mut x: Vec<Complex32> = (0..n)
            .map(|i| Complex32::from_polar(1.0, (2.0 * PI * 37.0 * i as f64 / n as f64) as f32))
            .collect();
        Fft::new(n).forward(&mut x);
        assert!((x[37].re - n as f32).abs() < 1e-2 && x[37].im.abs() < 1e-2);
        assert!(x
            .iter()
            .enumerate()
            .all(|(k, v)| k == 37 || v.norm_sqr() < 1e-4));
    }
}
