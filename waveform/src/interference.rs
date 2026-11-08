//! Interferers and a radio front end for the simulation (test plan B8).
//!
//! - [`Interferer`]: a CW tone, band-limited Gaussian noise or a swept tone, at an offset from the
//!   centre of the band. [`InterfererGen`] makes it as a continuous stream of samples (phase and
//!   noise carry on from one call to the next), with a given mean power per sample.
//! - [`FrontEnd`]: what a direct-conversion radio (AD9361 class) does to the band before the modem
//!   sees it: an ADC at `ratio` times the modem's sample rate that clips I and Q at full scale, then a
//!   decimating low-pass filter down to the modem's rate. The interferer is added at the high rate,
//!   so one that sits outside the modem's band (the next channel, 5 MHz away) is filtered, or aliased,
//!   as a radio would, and a strong one clips the ADC.
//!
//! Offsets and widths are in cycles per sample of the rate the generator runs at.
//!
//! Noise is made in the frequency domain: blocks of `N` samples with complex Gaussian bins inside the
//! band and nothing outside (a flat, brick-wall spectrum), joined by a half-block crossfade with
//! `cos` and `sin` weights (so the variance stays constant and the band stays where it is). The
//! crossfade blurs the band edges by a few bins of the block (0.2 % of the width) and puts the noise
//! more than 60 dB down 10 % of the width outside the band.

use crate::fft::Fft;
use crate::iq::Complex32;
use crate::rng::Rng;
use std::f64::consts::PI;

/// An interferer. Frequencies in cycles per sample.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Interferer {
    /// A carrier (CW) at `offset`.
    Tone { offset: f64 },
    /// Gaussian noise, flat over `offset - width / 2 .. offset + width / 2`.
    Noise { offset: f64, width: f64 },
    /// A tone sweeping from `offset - span / 2` up to `offset + span / 2` over `period` samples, then
    /// starting again (a sawtooth, phase continuous).
    Sweep { offset: f64, span: f64, period: f64 },
}

impl Interferer {
    pub fn name(&self) -> &'static str {
        match self {
            Interferer::Tone { .. } => "tone",
            Interferer::Noise { .. } => "noise",
            Interferer::Sweep { .. } => "sweep",
        }
    }

    /// The same interferer seen at a sample rate `k` times higher (offsets and widths divided by `k`,
    /// periods multiplied).
    pub fn at_rate(self, k: f64) -> Self {
        match self {
            Interferer::Tone { offset } => Interferer::Tone { offset: offset / k },
            Interferer::Noise { offset, width } => Interferer::Noise {
                offset: offset / k,
                width: width / k,
            },
            Interferer::Sweep {
                offset,
                span,
                period,
            } => Interferer::Sweep {
                offset: offset / k,
                span: span / k,
                period: period * k,
            },
        }
    }

    /// The highest absolute frequency it reaches, cycles per sample.
    pub fn reach(&self) -> f64 {
        match *self {
            Interferer::Tone { offset } => offset.abs(),
            Interferer::Noise { offset, width } => offset.abs() + width / 2.0,
            Interferer::Sweep { offset, span, .. } => offset.abs() + span / 2.0,
        }
    }
}

/// Band-limited noise of unit variance, block by block (see the module notes).
#[derive(Debug, Clone)]
struct NoiseBlocks {
    fft: Fft,
    /// FFT bins in the band.
    bins: Vec<usize>,
    prev: Vec<Complex32>,
    cur: Vec<Complex32>,
    /// Position in the current half-block crossfade, `0 .. n / 2`.
    pos: usize,
    rng: Rng,
}

impl NoiseBlocks {
    fn new(offset: f64, width: f64, rng: Rng) -> Self {
        let width = width.clamp(0.0, 1.0);
        // about 1000 bins in the band (the crossfade spreads each bin over about 2: the edges stay
        // sharp to 0.2 % of the width)
        let n = ((1024.0 / width.max(1e-9)) as usize)
            .next_power_of_two()
            .clamp(4096, 65536);
        let (lo, hi) = (offset - width / 2.0, offset + width / 2.0);
        let mut bins: Vec<usize> = (0..n)
            .filter(|&k| {
                // bin k is at k / n, or k / n - 1 above the middle: test both images
                let f = k as f64 / n as f64;
                (lo..hi).contains(&f)
                    || (lo..hi).contains(&(f - 1.0))
                    || (lo..hi).contains(&(f + 1.0))
            })
            .collect();
        if bins.is_empty() {
            bins.push(((offset.rem_euclid(1.0)) * n as f64).round() as usize % n);
        }
        let mut s = Self {
            fft: Fft::new(n),
            bins,
            prev: Vec::new(),
            cur: Vec::new(),
            pos: 0,
            rng,
        };
        s.prev = s.block();
        s.cur = s.block();
        s
    }

    /// One periodic block of unit variance with its spectrum on `bins`.
    fn block(&mut self) -> Vec<Complex32> {
        let n = self.fft.len();
        let sd = (0.5 / self.bins.len() as f64).sqrt();
        let mut y = vec![Complex32::ZERO; n];
        // x = sum_k X[k] e^(+j 2 pi k n / N) = conj(FFT(conj(X))); X circular Gaussian, so put
        // conj(X) at the bins, transform, and conjugate
        for &k in &self.bins {
            y[k] = Complex32::new(
                (sd * self.rng.gaussian()) as f32,
                -(sd * self.rng.gaussian()) as f32,
            );
        }
        self.fft.forward(&mut y);
        for v in &mut y {
            *v = v.conj();
        }
        y
    }

    fn next(&mut self) -> Complex32 {
        let half = self.fft.len() / 2;
        let th = 0.5 * PI * self.pos as f64 / half as f64;
        let v = self.prev[self.pos + half] * th.cos() as f32 + self.cur[self.pos] * th.sin() as f32;
        self.pos += 1;
        if self.pos == half {
            self.pos = 0;
            self.prev = std::mem::take(&mut self.cur);
            self.cur = self.block();
        }
        v
    }
}

/// An [`Interferer`] as a continuous stream of samples of mean power `power` (`E|i|^2`).
#[derive(Debug, Clone)]
pub struct InterfererGen {
    model: Interferer,
    amp: f64,
    phase: f64,
    /// Samples made so far.
    t: u64,
    noise: Option<NoiseBlocks>,
}

impl InterfererGen {
    pub fn new(model: Interferer, power: f64, seed: u64) -> Self {
        let mut rng = Rng::new(seed);
        let phase = 2.0 * PI * rng.uniform();
        let noise = match model {
            Interferer::Noise { offset, width } => Some(NoiseBlocks::new(offset, width, rng)),
            _ => None,
        };
        Self {
            model,
            amp: power.max(0.0).sqrt(),
            phase,
            t: 0,
            noise,
        }
    }

    pub fn model(&self) -> Interferer {
        self.model
    }

    /// Change the power from the next sample on.
    pub fn set_power(&mut self, power: f64) {
        self.amp = power.max(0.0).sqrt();
    }

    /// The next sample.
    pub fn sample(&mut self) -> Complex32 {
        let v = match self.model {
            Interferer::Tone { offset } => {
                let v = Complex32::new(self.phase.cos() as f32, self.phase.sin() as f32);
                self.phase = (self.phase + 2.0 * PI * offset).rem_euclid(2.0 * PI);
                v
            }
            Interferer::Sweep {
                offset,
                span,
                period,
            } => {
                let v = Complex32::new(self.phase.cos() as f32, self.phase.sin() as f32);
                let frac = (self.t as f64 / period.max(1.0)).fract();
                let f = offset - span / 2.0 + span * frac;
                self.phase = (self.phase + 2.0 * PI * f).rem_euclid(2.0 * PI);
                v
            }
            Interferer::Noise { offset, .. } => {
                // the bins already sit at the offset
                let _ = offset;
                self.noise.as_mut().map_or(Complex32::ZERO, |n| n.next())
            }
        };
        self.t += 1;
        v * self.amp as f32
    }

    /// Add the next `x.len()` samples to `x`.
    pub fn add(&mut self, x: &mut [Complex32]) {
        for s in x.iter_mut() {
            *s += self.sample();
        }
    }

    /// The next `n` samples.
    pub fn take(&mut self, n: usize) -> Vec<Complex32> {
        (0..n).map(|_| self.sample()).collect()
    }
}

/// Clip I and Q each to `+-level` (an ADC at full scale). Returns the samples clipped.
pub fn clip(x: &mut [Complex32], level: f32) -> usize {
    let mut n = 0;
    for s in x.iter_mut() {
        if s.re.abs() > level || s.im.abs() > level {
            n += 1;
        }
        s.re = s.re.clamp(-level, level);
        s.im = s.im.clamp(-level, level);
    }
    n
}

fn bessel_i0(x: f64) -> f64 {
    let (mut sum, mut term) = (1.0f64, 1.0f64);
    for k in 1..50 {
        term *= (x / 2.0) * (x / 2.0) / (k as f64 * k as f64);
        sum += term;
    }
    sum
}

/// A radio's sampling and decimation, `ratio` times the modem's rate: see the module notes.
///
/// The filter is a Kaiser-windowed sinc (beta 8, about 80 dB stopband), `32 ratio + 1` taps, cut
/// off at half the modem's rate: at 1 MS/s out it passes +-0.42 MHz, is 6 dB down at +-0.5 MHz and
/// stops from +-0.58 MHz, so nothing aliases onto the band the modem uses (+-0.42 MHz). A real
/// radio's filters differ; the bench measures them. Both directions are zero phase (no delay).
#[derive(Debug, Clone)]
pub struct FrontEnd {
    ratio: usize,
    /// Centred taps, sum 1.
    taps: Vec<f32>,
}

impl FrontEnd {
    pub fn new(ratio: usize) -> Self {
        let ratio = ratio.max(1);
        let len = 32 * ratio + 1;
        let half = (len / 2) as f64;
        let fc = 0.5 / ratio as f64;
        let beta = 8.0;
        let mut taps: Vec<f32> = (0..len)
            .map(|i| {
                let t = i as f64 - half;
                let sinc = if t == 0.0 {
                    2.0 * fc
                } else {
                    (2.0 * PI * fc * t).sin() / (PI * t)
                };
                let r = t / half;
                let w = bessel_i0(beta * (1.0 - r * r).max(0.0).sqrt()) / bessel_i0(beta);
                (sinc * w) as f32
            })
            .collect();
        let sum: f32 = taps.iter().sum();
        for t in &mut taps {
            *t /= sum;
        }
        Self { ratio, taps }
    }

    pub fn ratio(&self) -> usize {
        self.ratio
    }

    /// The modem-rate samples `x` at `ratio` times the rate (zero stuffing and the filter; the
    /// amplitude is kept). `x.len() * ratio` samples, sample `n * ratio` lines up with `x[n]`.
    pub fn upsample(&self, x: &[Complex32]) -> Vec<Complex32> {
        let r = self.ratio;
        if r == 1 {
            return x.to_vec();
        }
        let half = self.taps.len() / 2;
        let n = x.len() * r;
        let mut y = vec![Complex32::ZERO; n];
        let g = r as f32;
        for (i, s) in x.iter().enumerate() {
            let c = i * r;
            let lo = c.saturating_sub(half);
            let hi = (c + half + 1).min(n);
            for (m, out) in y.iter_mut().enumerate().take(hi).skip(lo) {
                *out += *s * (self.taps[m + half - c] * g);
            }
        }
        y
    }

    /// Filter and keep every `ratio`-th sample: `x.len() / ratio` samples, output `k` lines up with `x[k * ratio]`.
    pub fn downsample(&self, x: &[Complex32]) -> Vec<Complex32> {
        let r = self.ratio;
        if r == 1 {
            return x.to_vec();
        }
        let half = self.taps.len() / 2;
        (0..x.len() / r)
            .map(|k| {
                let c = k * r;
                let mut acc = Complex32::ZERO;
                let lo = c.saturating_sub(half);
                let hi = (c + half + 1).min(x.len());
                for (m, v) in x.iter().enumerate().take(hi).skip(lo) {
                    acc += *v * self.taps[m + half - c];
                }
                acc
            })
            .collect()
    }
}

/// A channel filter for the receiver (opt-in receiver improvement, B8): a low-pass FIR that keeps the
/// modem's band and removes what sits beside it before the detector sees it. The detector normalizes
/// by the power in its window, so a strong signal next to the band, which the matched filter would
/// ignore, still hides the preamble. Streaming: history is kept across calls, and the output is
/// delayed by [`ChannelFilter::delay`] samples.
#[derive(Debug, Clone)]
pub struct ChannelFilter {
    taps: Vec<f32>,
    hist: Vec<Complex32>,
}

impl ChannelFilter {
    /// Cut-off (6 dB) at `cutoff` cycles per sample, `taps` taps (odd), Kaiser beta 7 (about 70 dB).
    pub fn new(cutoff: f64, taps: usize) -> Self {
        let n = taps | 1;
        let half = (n / 2) as f64;
        let beta = 7.0;
        let mut h: Vec<f32> = (0..n)
            .map(|i| {
                let t = i as f64 - half;
                let sinc = if t == 0.0 {
                    2.0 * cutoff
                } else {
                    (2.0 * PI * cutoff * t).sin() / (PI * t)
                };
                let r = t / half.max(1.0);
                (sinc * bessel_i0(beta * (1.0 - r * r).max(0.0).sqrt()) / bessel_i0(beta)) as f32
            })
            .collect();
        let sum: f32 = h.iter().sum();
        for v in &mut h {
            *v /= sum;
        }
        Self {
            hist: vec![Complex32::ZERO; n - 1],
            taps: h,
        }
    }

    /// The modem's band: QPSK or BPSK with an RRC roll-off `rolloff` at `sps` samples per symbol,
    /// cut off at `(1 + rolloff) / 2` symbol rates plus a quarter of the symbol rate, 63 taps.
    pub fn for_modem(sps: usize, rolloff: f64) -> Self {
        Self::new(((1.0 + rolloff) / 2.0 + 0.25) / sps as f64, 63)
    }

    pub fn delay(&self) -> usize {
        self.taps.len() / 2
    }

    /// Filter `x` in place.
    pub fn run(&mut self, x: &mut [Complex32]) {
        let n = self.taps.len();
        self.hist.extend_from_slice(x);
        for (k, out) in x.iter_mut().enumerate() {
            let w = &self.hist[k..k + n];
            let (mut re, mut im) = (0.0f32, 0.0f32);
            // taps are symmetric: no reversal needed
            for (v, t) in w.iter().zip(&self.taps) {
                re += v.re * t;
                im += v.im * t;
            }
            *out = Complex32::new(re, im);
        }
        let keep = self.hist.len() - (n - 1);
        self.hist.drain(..keep);
    }
}

/// A notch for a strong carrier (opt-in receiver improvement, B8): estimates the strongest tone in
/// a block from an FFT and subtracts it (least squares amplitude and phase at that frequency,
/// refined between bins). Removes a CW of any power at the cost of the signal energy at that one
/// frequency. Does nothing when no bin stands `min_db` above the mean bin (a tone of power `P`
/// in a block of `n` samples of total power `T` stands `10 log10(n P / T)` dB above it; noise and
/// the modem's own signal stay below about 16 dB, so 25 is a safe threshold).
pub fn cancel_tone(x: &mut [Complex32], min_db: f64) -> Option<f64> {
    let n = x.len();
    if n < 64 {
        return None;
    }
    let m = (n.next_power_of_two() * 2).min(1 << 16);
    let fft = Fft::new(m);
    let mut y = vec![Complex32::ZERO; m];
    y[..n.min(m)].copy_from_slice(&x[..n.min(m)]);
    fft.forward(&mut y);
    let p: Vec<f32> = y.iter().map(|v| v.norm_sqr()).collect();
    let (k, &peak) = p.iter().enumerate().max_by(|a, b| a.1.total_cmp(b.1))?;
    let mean = (p.iter().map(|v| *v as f64).sum::<f64>() / m as f64).max(1e-30);
    if 10.0 * (peak as f64 / mean).log10() < min_db {
        return None;
    }
    // parabolic refinement on the log magnitude
    let at = |j: isize| p[j.rem_euclid(m as isize) as usize].max(1e-30).ln() as f64;
    let (a, b, c) = (at(k as isize - 1), at(k as isize), at(k as isize + 1));
    let d = if a - 2.0 * b + c != 0.0 {
        (0.5 * (a - c) / (a - 2.0 * b + c)).clamp(-0.5, 0.5)
    } else {
        0.0
    };
    let mut f = (k as f64 + d) / m as f64;
    if f > 0.5 {
        f -= 1.0;
    }
    // least squares amplitude at f, then a finer frequency from the phase drift between halves.
    // The phasor runs by recurrence in f64 (no sin/cos per sample).
    let fit = |f: f64, s: &[Complex32], off: usize| {
        let (si, co) = (-2.0 * PI * f * off as f64).sin_cos();
        let (ws, wc) = (-2.0 * PI * f).sin_cos();
        let (mut pr, mut pi) = (co, si);
        let (mut re, mut im) = (0.0f64, 0.0f64);
        for v in s {
            let (vr, vi) = (v.re as f64, v.im as f64);
            re += vr * pr - vi * pi;
            im += vr * pi + vi * pr;
            (pr, pi) = (pr * wc - pi * ws, pr * ws + pi * wc);
        }
        (re / s.len() as f64, im / s.len() as f64)
    };
    let h = n / 2;
    let (r1, i1) = fit(f, &x[..h], 0);
    let (r2, i2) = fit(f, &x[h..], h);
    let dphi = (i2 * r1 - r2 * i1).atan2(r2 * r1 + i2 * i1);
    f += dphi / (2.0 * PI * h as f64);
    let (re, im) = fit(f, x, 0);
    let (ws, wc) = (2.0 * PI * f).sin_cos();
    let (mut pr, mut pi) = (re, im);
    for v in x.iter_mut() {
        v.re -= pr as f32;
        v.im -= pi as f32;
        (pr, pi) = (pr * wc - pi * ws, pr * ws + pi * wc);
    }
    Some(f)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn power(x: &[Complex32]) -> f64 {
        x.iter().map(|v| v.norm_sqr() as f64).sum::<f64>() / x.len() as f64
    }

    /// Power in `lo .. hi` cycles per sample over the total, from an FFT of `x` (length a power of two).
    fn fraction_in(x: &[Complex32], lo: f64, hi: f64) -> f64 {
        // Hann window, so the measurement's own leakage stays far below the bounds tested
        let n = x.len();
        let mut y: Vec<Complex32> = x
            .iter()
            .enumerate()
            .map(|(i, v)| *v * (0.5 - 0.5 * (2.0 * PI * i as f64 / n as f64).cos()) as f32)
            .collect();
        Fft::new(n).forward(&mut y);
        let (mut inb, mut all) = (0.0f64, 0.0f64);
        for (k, v) in y.iter().enumerate() {
            let mut f = k as f64 / n as f64;
            if f >= 0.5 {
                f -= 1.0;
            }
            let p = v.norm_sqr() as f64;
            all += p;
            if (lo..hi).contains(&f) {
                inb += p;
            }
        }
        inb / all
    }

    #[test]
    fn tone_has_its_power_and_frequency() {
        let mut g = InterfererGen::new(Interferer::Tone { offset: 0.125 }, 0.01, 1);
        let x = g.take(4096);
        assert!((power(&x) - 0.01).abs() < 1e-6);
        assert!(fraction_in(&x, 0.124, 0.126) > 0.999);
        // continuous across calls: phase steps by 2 pi 0.125
        let a = g.sample();
        let b = g.sample();
        let step = (b * a.conj()).arg() as f64;
        assert!((step - 2.0 * PI * 0.125).abs() < 1e-4, "{step}");
    }

    #[test]
    fn noise_is_flat_in_its_band_and_nowhere_else() {
        let model = Interferer::Noise {
            offset: -0.2,
            width: 0.1,
        };
        let mut g = InterfererGen::new(model, 2.0, 7);
        let x = g.take(1 << 16);
        let p = power(&x);
        assert!((p - 2.0).abs() < 0.05, "power {p}");
        let inb = fraction_in(&x, -0.2502, -0.1498);
        assert!(inb > 0.9999, "in band {inb}");
        // 60 dB down 0.01 away from the band
        let near = 1.0 - fraction_in(&x, -0.26, -0.14);
        assert!(near < 1e-6, "outside {near}");
        // flat: each half of the band has half the power
        let a = fraction_in(&x, -0.25, -0.2);
        assert!((a - 0.5).abs() < 0.02, "lower half {a}");
        // Gaussian: about 4.6 % of |i|^2 above 3 times the mean (exp(-3) = 5 %)
        let above =
            x.iter().filter(|v| v.norm_sqr() as f64 > 3.0 * p).count() as f64 / x.len() as f64;
        assert!((above - (-3.0f64).exp()).abs() < 0.005, "{above}");
        // the seed repeats; another seed differs
        let y = InterfererGen::new(model, 2.0, 7).take(100);
        assert_eq!(&x[..100], &y[..]);
        assert_ne!(InterfererGen::new(model, 2.0, 8).take(100), y);
    }

    #[test]
    fn sweep_covers_its_span_and_repeats() {
        let model = Interferer::Sweep {
            offset: 0.1,
            span: 0.1,
            period: 4096.0,
        };
        let mut g = InterfererGen::new(model, 1.0, 3);
        let x = g.take(1 << 15);
        assert!((power(&x) - 1.0).abs() < 1e-4);
        assert!(fraction_in(&x, 0.045, 0.155) > 0.99);
        // about even over the span
        let lo = fraction_in(&x, 0.05, 0.1);
        assert!((lo - 0.5).abs() < 0.05, "{lo}");
        // instantaneous frequency near the start of a period is the bottom of the span
        let f0 = (x[4097] * x[4096].conj()).arg() as f64 / (2.0 * PI);
        assert!((f0 - 0.05).abs() < 1e-3, "{f0}");
    }

    #[test]
    fn front_end_passes_the_band_and_stops_the_rest() {
        let fe = FrontEnd::new(4);
        // a tone at 0.3 of the modem rate (0.075 of the high rate) comes through at its amplitude
        let n = 4096;
        let lo: Vec<Complex32> = (0..n)
            .map(|i| Complex32::from_polar(1.0, (2.0 * PI * 0.3 * i as f64) as f32))
            .collect();
        let hi = fe.upsample(&lo);
        assert_eq!(hi.len(), 4 * n);
        let back = fe.downsample(&hi);
        assert_eq!(back.len(), n);
        for i in 200..n - 200 {
            let e = (back[i] - lo[i]).norm_sqr().sqrt();
            assert!(e < 2e-3, "sample {i}: {e}");
        }
        // a tone at 0.7 of the modem rate (would alias to -0.3) is stopped by 75 dB or more
        let far: Vec<Complex32> = (0..4 * n)
            .map(|i| Complex32::from_polar(1.0, (2.0 * PI * 0.7 / 4.0 * i as f64) as f32))
            .collect();
        let d = fe.downsample(&far);
        let p = power(&d[200..n - 200]);
        assert!(p < 10f64.powf(-7.5), "{} dB", 10.0 * p.log10());
        assert_eq!(FrontEnd::new(1).downsample(&lo), lo);
    }

    #[test]
    fn clip_limits_i_and_q() {
        let mut x = vec![Complex32::new(2.0, -0.5), Complex32::new(0.1, -3.0)];
        assert_eq!(clip(&mut x, 1.0), 2);
        assert_eq!(
            x,
            vec![Complex32::new(1.0, -0.5), Complex32::new(0.1, -1.0)]
        );
    }

    #[test]
    fn channel_filter_keeps_the_band_and_streams() {
        let tone = |f: f64, n: usize| -> Vec<Complex32> {
            (0..n)
                .map(|i| Complex32::from_polar(1.0, (2.0 * PI * f * i as f64) as f32))
                .collect()
        };
        // 4 samples per symbol, roll-off 0.35: cut off at 0.23 cycles per sample
        let mut cf = ChannelFilter::for_modem(4, 0.35);
        let d = cf.delay();
        assert_eq!(d, 31);
        // in the band (0.15: 150 kHz at 1 MS/s): through within 0.1 dB, delayed by `delay`
        let x = tone(0.15, 2000);
        let mut y = x.clone();
        cf.run(&mut y);
        for i in 100..2000 {
            assert!((y[i] - x[i - d]).norm_sqr().sqrt() < 0.012, "sample {i}");
        }
        // the next channel (0.42 away from the signal's centre): 60 dB down or more
        let mut z = tone(0.42, 2000);
        ChannelFilter::for_modem(4, 0.35).run(&mut z);
        assert!(power(&z[100..]) < 1e-6, "{}", power(&z[100..]));
        // in blocks: the same as at once
        let mut a = tone(0.3, 1000);
        let b = a.clone();
        ChannelFilter::for_modem(4, 0.35).run(&mut a);
        let mut f = ChannelFilter::for_modem(4, 0.35);
        let mut c = Vec::new();
        for chunk in b.chunks(77) {
            let mut v = chunk.to_vec();
            f.run(&mut v);
            c.extend(v);
        }
        assert_eq!(a, c);
    }

    #[test]
    fn cancel_tone_removes_a_strong_carrier_and_leaves_noise() {
        let mut rng = Rng::new(11);
        let n = 2000;
        let noise: Vec<Complex32> = (0..n)
            .map(|_| Complex32::new(rng.gaussian() as f32, rng.gaussian() as f32) * 0.1)
            .collect();
        let mut g = InterfererGen::new(Interferer::Tone { offset: 0.1234 }, 1e4, 2);
        let mut x = noise.clone();
        g.add(&mut x);
        let f = cancel_tone(&mut x, 25.0).expect("found");
        assert!((f - 0.1234).abs() < 1e-5, "{f}");
        // what is left is the noise, within a few %, from a carrier 50 dB above it
        let resid: Vec<Complex32> = x.iter().zip(&noise).map(|(a, b)| *a - *b).collect();
        assert!(power(&resid) < 0.05 * power(&noise), "{}", power(&resid));
        // noise alone: nothing to cancel
        let mut y = noise.clone();
        assert_eq!(cancel_tone(&mut y, 25.0), None);
        assert_eq!(y, noise);
    }
}
