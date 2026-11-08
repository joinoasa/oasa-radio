//! A simulated cable between two `sdr` radios: reads one radio's TX stream, writes the other's RX
//! stream (cf32, little endian), with the loss, clock offset and receiver noise of a real bench. For
//! tests and the lab's simulated bench (`oasa-radio waveform cable`). Not part of any radio path.
//!
//! ```text
//!   radio A --tx FIFO--> cable (gain, offset, noise) --rx FIFO--> radio B
//! ```
//!
//! The TX stream is paced by its writer (`sdr` never runs more than `tx_lead`, 20 ms by default, ahead of the wall clock),
//! so the cable needs no clock of its own: it passes samples on as they come.

use std::io::{Read, Write};

use oasa_waveform::fading::{Fading, RayleighProcess};
use oasa_waveform::iq::Complex32;
use oasa_waveform::rng::Rng;

#[derive(Debug, Clone, PartialEq)]
pub struct CableConfig {
    /// Gain of the path, dB (a 60 dB cable is -60, plus what the radios add).
    pub gain_db: f64,
    /// Noise added at the receiver, dB full scale per complex sample (power).
    pub noise_dbfs: f64,
    /// Frequency offset between the two radios' clocks, Hz.
    pub cfo_hz: f64,
    pub sample_rate: f64,
    pub seed: u64,
    /// Fading before the gain, offset and noise (B9; delays and Doppler in samples and cycles per
    /// sample). A second ray's delay is rounded to whole samples here.
    pub fading: Fading,
}

impl Default for CableConfig {
    fn default() -> Self {
        Self {
            gain_db: 0.0,
            noise_dbfs: -60.0,
            cfo_hz: 0.0,
            sample_rate: 1e6,
            seed: 1,
            fading: Fading::None,
        }
    }
}

/// The fading of a continuous stream, sample by sample.
struct StreamFader {
    h: Vec<RayleighProcess>,
    /// Two rays: the two amplitudes and the fixed phase of the second ray (no Doppler); the delay is
    /// the length of `hist`.
    a: f32,
    b: f32,
    fixed: Complex32,
    hist: std::collections::VecDeque<Complex32>,
    t: u64,
    rayleigh: bool,
}

impl StreamFader {
    fn new(f: Fading, seed: u64) -> Option<Self> {
        let mut rng = Rng::new(seed ^ 0x0FAD_E5EE_D000_0002);
        match f {
            Fading::None => None,
            Fading::Rayleigh { doppler } => Some(Self {
                h: vec![RayleighProcess::new(doppler, &mut rng)],
                a: 1.0,
                b: 0.0,
                fixed: Complex32::ZERO,
                hist: Default::default(),
                t: 0,
                rayleigh: true,
            }),
            Fading::TwoRay {
                delay,
                gain_db,
                doppler,
            } => {
                let g = 10f64.powf(gain_db / 20.0);
                let h = if doppler > 0.0 {
                    vec![
                        RayleighProcess::new(doppler, &mut rng),
                        RayleighProcess::new(doppler, &mut rng),
                    ]
                } else {
                    vec![]
                };
                let th = 2.0 * std::f64::consts::PI * rng.uniform();
                let d = delay.round() as usize;
                Some(Self {
                    h,
                    a: (1.0 / (1.0 + g * g)).sqrt() as f32,
                    b: (g / (1.0 + g * g).sqrt()) as f32,
                    fixed: Complex32::new(th.cos() as f32, th.sin() as f32),
                    hist: std::iter::repeat_n(Complex32::ZERO, d).collect(),
                    t: 0,
                    rayleigh: false,
                })
            }
        }
    }

    fn apply(&mut self, x: &mut [Complex32]) {
        let t0 = self.t as f64;
        self.t += x.len() as u64;
        if self.rayleigh {
            let h = self.h[0].fill(t0, x.len());
            for (s, g) in x.iter_mut().zip(&h) {
                *s = *s * *g;
            }
            return;
        }
        let (h0, h1) = if self.h.len() == 2 {
            (self.h[0].fill(t0, x.len()), self.h[1].fill(t0, x.len()))
        } else {
            (
                vec![Complex32::new(1.0, 0.0); x.len()],
                vec![self.fixed; x.len()],
            )
        };
        for (k, s) in x.iter_mut().enumerate() {
            self.hist.push_back(*s);
            let late = self.hist.pop_front().unwrap_or(Complex32::ZERO);
            *s = *s * h0[k] * self.a + late * h1[k] * self.b;
        }
    }
}

/// Pass samples from `input` to `output` until `input` ends or `output` closes. Returns the
/// number of samples passed.
pub fn run(
    cfg: &CableConfig,
    input: &mut dyn Read,
    output: &mut dyn Write,
) -> std::io::Result<u64> {
    let g = 10f64.powf(cfg.gain_db / 20.0);
    let sigma = (10f64.powf(cfg.noise_dbfs / 10.0) / 2.0).sqrt();
    let w = 2.0 * std::f64::consts::PI * cfg.cfo_hz / cfg.sample_rate;
    let mut rng = Rng::new(cfg.seed);
    let mut fader = StreamFader::new(cfg.fading, cfg.seed);
    let mut x: Vec<Complex32> = Vec::new();
    let mut phase = 0.0f64;
    let mut buf = vec![0u8; 8 * 4096];
    let mut have = 0usize;
    let mut out = Vec::with_capacity(buf.len());
    let mut n: u64 = 0;
    loop {
        let k = match input.read(&mut buf[have..]) {
            Ok(0) => return Ok(n),
            Ok(k) => k,
            Err(e) if e.kind() == std::io::ErrorKind::Interrupted => continue,
            Err(e) => return Err(e),
        };
        have += k;
        let whole = have / 8 * 8;
        out.clear();
        x.clear();
        x.extend(buf[..whole].chunks_exact(8).map(|c| {
            Complex32::new(
                f32::from_le_bytes([c[0], c[1], c[2], c[3]]),
                f32::from_le_bytes([c[4], c[5], c[6], c[7]]),
            )
        }));
        if let Some(f) = fader.as_mut() {
            f.apply(&mut x);
        }
        for c in &x {
            let (re, im) = (f64::from(c.re), f64::from(c.im));
            let (s, co) = phase.sin_cos();
            let yr = g * (re * co - im * s) + sigma * rng.gaussian();
            let yi = g * (re * s + im * co) + sigma * rng.gaussian();
            phase = (phase + w).rem_euclid(2.0 * std::f64::consts::PI);
            out.extend_from_slice(&(yr as f32).to_le_bytes());
            out.extend_from_slice(&(yi as f32).to_le_bytes());
            n += 1;
        }
        buf.copy_within(whole..have, 0);
        have -= whole;
        output.write_all(&out)?;
        output.flush()?;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pass(cfg: &CableConfig, x: &[Complex32]) -> Vec<Complex32> {
        let bytes = oasa_waveform::iq::to_cf32_le(x);
        let mut out = Vec::new();
        run(cfg, &mut &bytes[..], &mut out).unwrap();
        oasa_waveform::iq::from_cf32_le(&out)
    }

    #[test]
    fn two_rays_in_a_stream() {
        let mut x = vec![Complex32::ZERO; 64];
        x[5] = Complex32::new(1.0, 0.0);
        let cfg = CableConfig {
            noise_dbfs: -200.0,
            fading: Fading::TwoRay {
                delay: 3.0,
                gain_db: -6.0,
                doppler: 0.0,
            },
            ..CableConfig::default()
        };
        let y = pass(&cfg, &x);
        let g = 10f32.powf(-6.0 / 20.0);
        let a = 1.0 / (1.0 + g * g).sqrt();
        assert!((y[5].norm_sqr().sqrt() - a).abs() < 1e-5);
        assert!((y[8].norm_sqr().sqrt() - g * a).abs() < 1e-5);
        let rest: f32 = y
            .iter()
            .enumerate()
            .filter(|(k, _)| *k != 5 && *k != 8)
            .map(|(_, s)| s.norm_sqr())
            .sum();
        assert!(rest < 1e-10);
    }

    #[test]
    fn rayleigh_stream_keeps_mean_power_and_awgn_is_unchanged() {
        let x = vec![Complex32::new(1.0, 0.0); 400_000];
        let cfg = CableConfig {
            noise_dbfs: -200.0,
            fading: Fading::Rayleigh { doppler: 2e-3 },
            ..CableConfig::default()
        };
        let y = pass(&cfg, &x);
        let p: f64 = y.iter().map(|s| s.norm_sqr() as f64).sum::<f64>() / y.len() as f64;
        assert!((p - 1.0).abs() < 0.25, "mean power {p}");
        // no fading: the same as before this option existed (gain 1, no offset, noise from the seed)
        let plain = CableConfig::default();
        let y1 = pass(&plain, &x[..1000]);
        let y2 = pass(&plain, &x[..1000]);
        assert_eq!(y1, y2);
        assert!((y1[10].re - 1.0).abs() < 0.01);
    }
}
