//! Fading channel models for the simulation (test plan B9): a two-ray multipath channel and flat
//! Rayleigh fading with a Clarke (Jakes) Doppler spectrum. Applied to the transmitted samples before
//! the rest of the channel (carrier offset, noise; see [`crate::channel`]).
//!
//! Power: every model has a mean power gain of 1, so the Eb/N0 given to the simulation is the mean
//! Eb/N0 over the fading.
//!
//! - [`Fading::TwoRay`]: `y[n] = a h0(n) x[n] + b h1(n) x[n - d]` with `a^2 + b^2 = 1` and `b/a` the second
//!   ray's relative gain. `d` may be fractional. With no Doppler, `h0 = 1` and `h1 = e^(j theta)` with a
//!   random `theta` per frame (each frame sees a different notch in frequency); with Doppler, `h0` and
//!   `h1` are independent Rayleigh processes (two-ray Rayleigh, a frequency-selective fading channel).
//! - [`Fading::Rayleigh`]: `y[n] = h(n) x[n]`, one Rayleigh process.
//!
//! A Rayleigh process is the sum of sinusoids of Zheng and Xiao ("Simulation models with correct
//! statistical properties for Rayleigh fading channels", IEEE Trans. Commun. 51(6), 2003): unit mean power,
//! autocorrelation close to `J0(2 pi fd tau)`, seeded and deterministic. It is evaluated every
//! [`GRID`] samples and interpolated linearly between (at most a 1 kHz Doppler at 1 MS/s, the error
//! is far below the noise).
//!
//! Time runs on across frames: [`Fader::apply`] moves its clock on by the frame interval after each
//! frame, so successive frames see the fading as a radio sending at that rate would.

use crate::channel::fractional_delay;
use crate::iq::Complex32;
use crate::rng::Rng;
use std::f64::consts::PI;

/// Sinusoids per Rayleigh process.
pub const PATHS: usize = 16;
/// Samples between evaluations of a Rayleigh process (linear interpolation between).
pub const GRID: usize = 16;

/// A fading model. Delays in samples, Doppler in cycles per sample.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum Fading {
    /// No fading (AWGN only): the samples pass unchanged.
    #[default]
    None,
    /// Two rays: the second `delay` samples later and `gain_db` (negative) relative to the first.
    TwoRay {
        delay: f64,
        gain_db: f64,
        doppler: f64,
    },
    /// Flat Rayleigh fading.
    Rayleigh { doppler: f64 },
}

impl Fading {
    pub fn name(&self) -> &'static str {
        match self {
            Fading::None => "awgn",
            Fading::TwoRay { .. } => "tworay",
            Fading::Rayleigh { .. } => "rayleigh",
        }
    }
}

/// One Rayleigh fading process of unit mean power.
#[derive(Debug, Clone)]
pub struct RayleighProcess {
    fi: [f64; PATHS],
    fq: [f64; PATHS],
    pi_: [f64; PATHS],
    pq: [f64; PATHS],
}

impl RayleighProcess {
    /// `doppler` in cycles per sample.
    pub fn new(doppler: f64, rng: &mut Rng) -> Self {
        let mut p = Self {
            fi: [0.0; PATHS],
            fq: [0.0; PATHS],
            pi_: [0.0; PATHS],
            pq: [0.0; PATHS],
        };
        let theta = (2.0 * rng.uniform() - 1.0) * PI;
        for m in 0..PATHS {
            let alpha = (2.0 * PI * (m + 1) as f64 - PI + theta) / (4.0 * PATHS as f64);
            p.fi[m] = 2.0 * PI * doppler * alpha.cos();
            p.fq[m] = 2.0 * PI * doppler * alpha.sin();
            p.pi_[m] = (2.0 * rng.uniform() - 1.0) * PI;
            p.pq[m] = (2.0 * rng.uniform() - 1.0) * PI;
        }
        p
    }

    /// The gain at time `t` (samples).
    pub fn at(&self, t: f64) -> Complex32 {
        let (mut re, mut im) = (0.0f64, 0.0f64);
        for m in 0..PATHS {
            re += (self.fi[m] * t + self.pi_[m]).cos();
            im += (self.fq[m] * t + self.pq[m]).cos();
        }
        // sqrt(2/M) per quadrature gives unit power each; 1/sqrt(2) more for a unit complex power
        let s = (1.0 / PATHS as f64).sqrt();
        Complex32::new((re * s) as f32, (im * s) as f32)
    }

    /// Gains for times `t0, t0 + 1, ..., t0 + n - 1`, on a [`GRID`] with linear interpolation.
    pub fn fill(&self, t0: f64, n: usize) -> Vec<Complex32> {
        let mut out = Vec::with_capacity(n);
        let mut k = 0;
        while k < n {
            let a = self.at(t0 + k as f64);
            let b = self.at(t0 + (k + GRID) as f64);
            for j in 0..GRID.min(n - k) {
                let w = j as f32 / GRID as f32;
                out.push(a * (1.0 - w) + b * w);
            }
            k += GRID;
        }
        out
    }
}

/// Applies a [`Fading`] model frame after frame, keeping time.
#[derive(Debug, Clone)]
pub struct Fader {
    model: Fading,
    /// Samples between the starts of successive frames.
    interval: f64,
    t: f64,
    rng: Rng,
    rays: Vec<RayleighProcess>,
}

impl Fader {
    /// `interval`: samples from one frame's start to the next (time moves on by this after each frame).
    pub fn new(model: Fading, interval: f64, seed: u64) -> Self {
        let mut rng = Rng::new(seed);
        let rays = match model {
            Fading::Rayleigh { doppler } => vec![RayleighProcess::new(doppler, &mut rng)],
            Fading::TwoRay { doppler, .. } if doppler > 0.0 => vec![
                RayleighProcess::new(doppler, &mut rng),
                RayleighProcess::new(doppler, &mut rng),
            ],
            _ => vec![],
        };
        Self {
            model,
            interval,
            t: 0.0,
            rng,
            rays,
        }
    }

    pub fn is_none(&self) -> bool {
        self.model == Fading::None
    }

    /// The faded frame (longer than `x` by the second ray's delay for two rays). Time moves on by the interval.
    pub fn apply(&mut self, x: &[Complex32]) -> Vec<Complex32> {
        let y = match self.model {
            Fading::None => x.to_vec(),
            Fading::Rayleigh { .. } => {
                let h = self.rays[0].fill(self.t, x.len());
                x.iter().zip(&h).map(|(s, g)| *s * *g).collect()
            }
            Fading::TwoRay { delay, gain_db, .. } => {
                let g = 10f64.powf(gain_db / 20.0);
                let a = (1.0 / (1.0 + g * g)).sqrt() as f32;
                let b = (g / (1.0 + g * g).sqrt()) as f32;
                let d = delay.floor() as usize;
                let mu = delay - d as f64;
                let late: Vec<Complex32> = if mu > 0.0 {
                    fractional_delay(x, mu)
                } else {
                    x.to_vec()
                };
                let n = (late.len() + d).max(x.len());
                let (h0, h1) = if self.rays.len() == 2 {
                    (self.rays[0].fill(self.t, n), self.rays[1].fill(self.t, n))
                } else {
                    let th = 2.0 * PI * self.rng.uniform();
                    let r = Complex32::new(th.cos() as f32, th.sin() as f32);
                    (vec![Complex32::new(1.0, 0.0); n], vec![r; n])
                };
                let mut y = vec![Complex32::ZERO; n];
                for (k, s) in x.iter().enumerate() {
                    y[k] += *s * h0[k] * a;
                }
                for (k, s) in late.iter().enumerate() {
                    y[k + d] += *s * h1[k + d] * b;
                }
                y
            }
        };
        self.t += self.interval;
        y
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bessel_j0(x: f64) -> f64 {
        // series; fine for |x| < 10
        let (mut sum, mut term) = (1.0f64, 1.0f64);
        for k in 1..40 {
            term *= -(x * x / 4.0) / (k as f64 * k as f64);
            sum += term;
        }
        sum
    }

    #[test]
    fn rayleigh_has_unit_power_and_jakes_autocorrelation() {
        let fd = 1e-3; // cycles per sample
        let lags = [0.0, 50.0, 100.0, 200.0, 383.0, 600.0];
        let mut power = 0.0;
        let mut corr = vec![0.0f64; lags.len()];
        let (runs, times) = (400, 50);
        for seed in 0..runs {
            let p = RayleighProcess::new(fd, &mut Rng::new(seed));
            for k in 0..times {
                let t = k as f64 * 997.0;
                let h0 = p.at(t);
                power += h0.norm_sqr() as f64;
                for (i, &lag) in lags.iter().enumerate() {
                    let h1 = p.at(t + lag);
                    corr[i] += (h0.re * h1.re + h0.im * h1.im) as f64;
                }
            }
        }
        let n = (runs * times) as f64;
        let power = power / n;
        assert!((power - 1.0).abs() < 0.05, "mean power {power}");
        for (i, &lag) in lags.iter().enumerate() {
            let got = corr[i] / n / power;
            let want = bessel_j0(2.0 * PI * fd * lag);
            assert!((got - want).abs() < 0.06, "lag {lag}: {got} vs J0 {want}");
        }
    }

    #[test]
    fn rayleigh_envelope_is_rayleigh() {
        // P(|h|^2 < x) = 1 - exp(-x) for unit power
        let mut below = 0usize;
        let mut total = 0usize;
        for seed in 0..300 {
            let p = RayleighProcess::new(2e-3, &mut Rng::new(seed));
            for k in 0..40 {
                total += 1;
                below += (p.at(k as f64 * 1013.0).norm_sqr() < 0.1) as usize;
            }
        }
        let frac = below as f64 / total as f64;
        let want = 1.0 - (-0.1f64).exp(); // 0.095: a 10 dB fade
        assert!((frac - want).abs() < 0.02, "deep fades {frac} vs {want}");
    }

    #[test]
    fn fill_matches_direct_evaluation() {
        let p = RayleighProcess::new(2e-4, &mut Rng::new(9));
        let v = p.fill(1234.0, 100);
        assert_eq!(v.len(), 100);
        for (k, g) in v.iter().enumerate() {
            let e = (*g - p.at(1234.0 + k as f64)).norm_sqr().sqrt();
            assert!(e < 1e-4, "sample {k}: {e}");
        }
    }

    #[test]
    fn two_ray_impulse_response() {
        // an impulse through 2 rays, 3 samples apart, -6 dB: taps a at 0, b e^(j theta) at 3
        let mut x = vec![Complex32::ZERO; 8];
        x[0] = Complex32::new(1.0, 0.0);
        let model = Fading::TwoRay {
            delay: 3.0,
            gain_db: -6.0,
            doppler: 0.0,
        };
        let y = Fader::new(model, 100.0, 4).apply(&x);
        assert_eq!(y.len(), 11);
        let g = 10f64.powf(-6.0 / 20.0);
        let a = (1.0 / (1.0 + g * g)).sqrt();
        let b = g * a;
        assert!((y[0].norm_sqr().sqrt() as f64 - a).abs() < 1e-6);
        assert!((y[3].norm_sqr().sqrt() as f64 - b).abs() < 1e-6);
        let rest: f32 = y
            .iter()
            .enumerate()
            .filter(|(k, _)| *k != 0 && *k != 3)
            .map(|(_, s)| s.norm_sqr())
            .sum();
        assert!(rest < 1e-12);
        // unit power gain: a^2 + b^2 = 1
        assert!((a * a + b * b - 1.0).abs() < 1e-12);
    }

    #[test]
    fn fading_repeats_from_the_seed_and_none_is_a_copy() {
        let x: Vec<Complex32> = (0..300)
            .map(|k| Complex32::new((k as f32 * 0.1).cos(), 0.5))
            .collect();
        for model in [
            Fading::Rayleigh { doppler: 1e-4 },
            Fading::TwoRay {
                delay: 2.5,
                gain_db: -3.0,
                doppler: 1e-4,
            },
            Fading::TwoRay {
                delay: 1.0,
                gain_db: -3.0,
                doppler: 0.0,
            },
        ] {
            let mut f1 = Fader::new(model, 1000.0, 7);
            let mut f2 = Fader::new(model, 1000.0, 7);
            let mut f3 = Fader::new(model, 1000.0, 8);
            let (a, b, c) = (f1.apply(&x), f2.apply(&x), f3.apply(&x));
            assert_eq!(a, b, "{model:?}");
            assert_ne!(a, c, "{model:?}");
            // time moves on: the next frame differs
            assert_ne!(f1.apply(&x), a, "{model:?}");
        }
        assert_eq!(Fader::new(Fading::None, 1.0, 1).apply(&x), x);
    }
}
