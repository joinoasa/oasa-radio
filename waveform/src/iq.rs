//! The sample type and its byte form (SigMF `cf32_le`: f32 real, f32 imaginary, little endian).

use std::ops::{Add, AddAssign, Mul, Sub};

/// A complex baseband sample, 32-bit float parts. Same layout as an SDR's `cf32`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Complex32 {
    pub re: f32,
    pub im: f32,
}

impl Complex32 {
    pub const ZERO: Complex32 = Complex32 { re: 0.0, im: 0.0 };

    pub const fn new(re: f32, im: f32) -> Self {
        Self { re, im }
    }

    /// `r * e^(j*theta)`.
    pub fn from_polar(r: f32, theta: f32) -> Self {
        Self::new(r * theta.cos(), r * theta.sin())
    }

    pub fn conj(self) -> Self {
        Self::new(self.re, -self.im)
    }

    /// `|x|^2`.
    pub fn norm_sqr(self) -> f32 {
        self.re * self.re + self.im * self.im
    }

    /// Phase angle in radians, in `(-pi, pi]`.
    pub fn arg(self) -> f32 {
        self.im.atan2(self.re)
    }
}

impl Add for Complex32 {
    type Output = Self;
    fn add(self, o: Self) -> Self {
        Self::new(self.re + o.re, self.im + o.im)
    }
}

impl AddAssign for Complex32 {
    fn add_assign(&mut self, o: Self) {
        self.re += o.re;
        self.im += o.im;
    }
}

impl Sub for Complex32 {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        Self::new(self.re - o.re, self.im - o.im)
    }
}

impl Mul for Complex32 {
    type Output = Self;
    fn mul(self, o: Self) -> Self {
        Self::new(
            self.re * o.re - self.im * o.im,
            self.re * o.im + self.im * o.re,
        )
    }
}

impl Mul<f32> for Complex32 {
    type Output = Self;
    fn mul(self, k: f32) -> Self {
        Self::new(self.re * k, self.im * k)
    }
}

/// Samples as SigMF `cf32_le` bytes (8 bytes per sample).
pub fn to_cf32_le(samples: &[Complex32]) -> Vec<u8> {
    let mut out = Vec::with_capacity(samples.len() * 8);
    for s in samples {
        out.extend_from_slice(&s.re.to_le_bytes());
        out.extend_from_slice(&s.im.to_le_bytes());
    }
    out
}

/// The inverse of [`to_cf32_le`]. A trailing partial sample is ignored.
pub fn from_cf32_le(bytes: &[u8]) -> Vec<Complex32> {
    bytes
        .chunks_exact(8)
        .map(|c| {
            Complex32::new(
                f32::from_le_bytes([c[0], c[1], c[2], c[3]]),
                f32::from_le_bytes([c[4], c[5], c[6], c[7]]),
            )
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cf32_round_trip_and_layout() {
        let v = vec![Complex32::new(1.5, -2.0), Complex32::new(0.0, 3.25)];
        let b = to_cf32_le(&v);
        assert_eq!(b.len(), 16);
        assert_eq!(&b[0..4], &1.5f32.to_le_bytes());
        assert_eq!(&b[4..8], &(-2.0f32).to_le_bytes());
        assert_eq!(from_cf32_le(&b), v);
        assert_eq!(from_cf32_le(&b[..13]).len(), 1);
    }

    #[test]
    fn complex_arithmetic() {
        let a = Complex32::new(1.0, 2.0);
        let b = Complex32::new(3.0, -1.0);
        assert_eq!(a * b, Complex32::new(5.0, 5.0));
        assert_eq!(a.conj(), Complex32::new(1.0, -2.0));
        assert_eq!(a.norm_sqr(), 5.0);
        let p = Complex32::from_polar(2.0, std::f32::consts::FRAC_PI_2);
        assert!(p.re.abs() < 1e-6 && (p.im - 2.0).abs() < 1e-6);
    }
}
