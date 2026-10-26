//! The simulated channel: AWGN at a given Eb/N0, plus optional phase offset, carrier frequency
//! offset and sample delay. It knows nothing about the modem; it works on any `&[Complex32]`.
//!
//! # Noise and the Es/N0, Eb/N0 and per-sample SNR conventions
//!
//! Energy is counted as the sum of `|x|^2` over samples (a sample is one unit of time).
//! Then, for complex white noise with variance `sigma2 = E|n|^2` per sample (half in I, half in Q):
//!
//! * `N0 = sigma2`. The noise fills the whole sampling band, and a matched filter of unit energy
//!   sees a noise sample of variance `N0` whatever the oversampling.
//! * `Es/N0 = Es / sigma2`, with `Es` the energy of one symbol; `Eb/N0 = Es/N0 / bits_per_symbol`.
//! * The SNR per sample is `P_signal / sigma2` with `P_signal = Es / sps`. So
//!   `SNR_sample = Es/N0 / sps = Eb/N0 * bits_per_symbol / sps`. At 8 samples per symbol and
//!   BPSK it is 9 dB below Eb/N0. Using the per-sample SNR as if it were Eb/N0 is the classic bug.
//!
//! [`n0_for_ebn0`] turns a target Eb/N0 and the modem's energy per bit into `sigma2`.

use crate::iq::Complex32;
use crate::rng::Rng;
use std::f64::consts::PI;

/// Noise variance per complex sample for a target `Eb/N0` (dB), given the energy of one information bit.
pub fn n0_for_ebn0(ebn0_db: f64, energy_per_bit: f64) -> f64 {
    energy_per_bit / 10f64.powf(ebn0_db / 10.0)
}

/// `Es/N0` in dB from `Eb/N0` in dB.
pub fn esn0_db(ebn0_db: f64, bits_per_symbol: usize) -> f64 {
    ebn0_db + 10.0 * (bits_per_symbol as f64).log10()
}

/// Signal power over noise power per sample, in dB, for an oversampling of `sps` samples per symbol.
pub fn snr_per_sample_db(ebn0_db: f64, bits_per_symbol: usize, sps: usize) -> f64 {
    esn0_db(ebn0_db, bits_per_symbol) - 10.0 * (sps as f64).log10()
}

/// Delay a signal by a fraction of a sample, `mu` in `[0, 1)`: the result at index `n` is the band-limited
/// interpolation of `x` at time `n - mu`. A Blackman-windowed sinc of 49 taps; accurate for signals that stay
/// below about 0.4 of the sampling rate (the modem's do, at 4 or more samples per symbol and
/// roll-off 0.35: its band is 0.17 of the sampling rate at 8 samples per symbol). The output is
/// `x.len() + 24` long, so the delayed tail is kept. Samples before the start are taken as zero.
pub fn fractional_delay(x: &[Complex32], mu: f64) -> Vec<Complex32> {
    const HALF: isize = 24;
    let taps: Vec<f32> = (-HALF..=HALF)
        .map(|k| {
            let t = k as f64 - mu;
            let sinc = if t.abs() < 1e-12 {
                1.0
            } else {
                (PI * t).sin() / (PI * t)
            };
            let w = 0.42
                + 0.5 * (PI * t / (HALF as f64 + 1.0)).cos()
                + 0.08 * (2.0 * PI * t / (HALF as f64 + 1.0)).cos();
            (sinc * w) as f32
        })
        .collect();
    let mut out = vec![Complex32::ZERO; x.len() + HALF as usize];
    for (n, o) in out.iter_mut().enumerate() {
        let mut acc = Complex32::ZERO;
        for (i, &t) in taps.iter().enumerate() {
            // tap i is for offset k = i - HALF: x[n - k]
            let m = n as isize - (i as isize - HALF);
            if m >= 0 && (m as usize) < x.len() {
                acc += x[m as usize] * t;
            }
        }
        *o = acc;
    }
    out
}

/// What the channel does to a signal, in this order: delay by `frac_delay` samples, rotate (phase and frequency), pad with
/// `lead` and `tail` empty samples, add noise to every sample including the padding.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Impairments {
    /// Noise variance per complex sample (`N0`). 0 means no noise.
    pub n0: f64,
    /// Carrier phase offset, radians.
    pub phase_rad: f64,
    /// Carrier frequency offset, cycles per sample.
    pub cfo_cycles_per_sample: f64,
    /// A delay of less than one sample on top of `lead`, `[0, 1)`. See [`fractional_delay`].
    pub frac_delay: f64,
    /// Samples of silence (noise only) before the signal: an integer sample delay.
    pub lead: usize,
    /// Samples of silence after the signal.
    pub tail: usize,
}

#[derive(Debug, Clone)]
pub struct Channel {
    rng: Rng,
}

impl Channel {
    pub fn new(seed: u64) -> Self {
        Self {
            rng: Rng::new(seed),
        }
    }

    pub fn rng(&mut self) -> &mut Rng {
        &mut self.rng
    }

    /// Add complex white Gaussian noise of variance `n0` per sample.
    pub fn add_awgn(&mut self, x: &mut [Complex32], n0: f64) {
        if n0 <= 0.0 {
            return;
        }
        let sd = (n0 / 2.0).sqrt();
        for s in x.iter_mut() {
            s.re += (sd * self.rng.gaussian()) as f32;
            s.im += (sd * self.rng.gaussian()) as f32;
        }
    }

    /// Noise only, `n` samples of variance `n0`.
    pub fn noise(&mut self, n: usize, n0: f64) -> Vec<Complex32> {
        let mut v = vec![Complex32::ZERO; n];
        self.add_awgn(&mut v, n0);
        v
    }

    /// Apply the impairments to `tx` and return what a receiver would see.
    pub fn apply(&mut self, tx: &[Complex32], imp: &Impairments) -> Vec<Complex32> {
        let delayed;
        let tx = if imp.frac_delay != 0.0 {
            delayed = fractional_delay(tx, imp.frac_delay);
            &delayed[..]
        } else {
            tx
        };
        let mut out = vec![Complex32::ZERO; imp.lead + tx.len() + imp.tail];
        for (n, s) in tx.iter().enumerate() {
            // rotation by the carrier offset, counted from the first sample of the signal
            let ph = imp.phase_rad + 2.0 * PI * imp.cfo_cycles_per_sample * n as f64;
            let (si, co) = ph.sin_cos();
            out[imp.lead + n] = *s * Complex32::new(co as f32, si as f32);
        }
        self.add_awgn(&mut out, imp.n0);
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn noise_power_is_n0_and_split_evenly() {
        let mut ch = Channel::new(3);
        let v = ch.noise(200_000, 0.5);
        let p: f64 = v.iter().map(|s| s.norm_sqr() as f64).sum::<f64>() / v.len() as f64;
        let pi: f64 = v.iter().map(|s| (s.re as f64).powi(2)).sum::<f64>() / v.len() as f64;
        assert!((p - 0.5).abs() < 0.01, "power {p}");
        assert!((pi - 0.25).abs() < 0.006, "I power {pi}");
    }

    #[test]
    fn conventions() {
        // 10 dB Eb/N0, 1 bit per unit symbol energy: N0 = 0.1
        assert!((n0_for_ebn0(10.0, 1.0) - 0.1).abs() < 1e-12);
        // QPSK: Eb = 1/2, same Eb/N0 -> half the noise
        assert!((n0_for_ebn0(10.0, 0.5) - 0.05).abs() < 1e-12);
        assert!((esn0_db(4.0, 2) - 7.0103).abs() < 1e-3);
        // BPSK at 8 samples per symbol: SNR per sample is 9.03 dB below Eb/N0
        assert!((snr_per_sample_db(10.0, 1, 8) - (10.0 - 9.0309)).abs() < 1e-3);
    }

    #[test]
    fn rotate_pad_and_repeat() {
        let tx = vec![Complex32::new(1.0, 0.0); 4];
        let imp = Impairments {
            phase_rad: std::f64::consts::FRAC_PI_2,
            lead: 3,
            tail: 2,
            ..Default::default()
        };
        let mut a = Channel::new(1);
        let y = a.apply(&tx, &imp);
        assert_eq!(y.len(), 9);
        assert!(y[..3].iter().all(|s| *s == Complex32::ZERO));
        assert!(y[3].re.abs() < 1e-6 && (y[3].im - 1.0).abs() < 1e-6);
        // noise repeats from the seed
        let imp = Impairments { n0: 0.1, ..imp };
        let y1 = Channel::new(5).apply(&tx, &imp);
        let y2 = Channel::new(5).apply(&tx, &imp);
        let y3 = Channel::new(6).apply(&tx, &imp);
        assert_eq!(y1, y2);
        assert_ne!(y1, y3);
    }

    #[test]
    fn fractional_delay_shifts_a_slow_tone() {
        // a tone at 0.05 cycles per sample, delayed by 0.37 sample: phase lags by 2 pi 0.05 0.37
        let f = 0.05f64;
        let x: Vec<Complex32> = (0..400)
            .map(|n| Complex32::from_polar(1.0, (2.0 * PI * f * n as f64) as f32))
            .collect();
        let y = fractional_delay(&x, 0.37);
        for (n, got) in y.iter().enumerate().take(300).skip(60) {
            let want = Complex32::from_polar(1.0, (2.0 * PI * f * (n as f64 - 0.37)) as f32);
            let e = (*got - want).norm_sqr().sqrt();
            assert!(e < 2e-3, "sample {n}: error {e}");
        }
        assert_eq!(fractional_delay(&x, 0.0)[100].re, x[100].re);
    }

    #[test]
    fn frequency_offset_rotates_progressively() {
        let tx = vec![Complex32::new(1.0, 0.0); 5];
        let imp = Impairments {
            cfo_cycles_per_sample: 0.25,
            ..Default::default()
        };
        let y = Channel::new(1).apply(&tx, &imp);
        assert!((y[1].im - 1.0).abs() < 1e-6 && y[1].re.abs() < 1e-6);
        assert!((y[2].re + 1.0).abs() < 1e-6);
    }
}
