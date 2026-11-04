//! Two receive channels: their phase difference, and the direction of arrival it gives (T-RF3).
//!
//! The measurement is `r = mean(x2 * conj(x1))` over a stretch of samples where a signal is present
//! (research/t-rf3-array/TEST-PLAN.md). Its angle is the phase of channel 2 relative to channel 1. A
//! carrier offset common to both channels cancels in the product, and so does the modulation, so the
//! source can be anything: a tone, a frame, a LoRa chirp. Only the two channels must be sampled together
//! (one multi-channel stream).
//!
//! Sign: a positive phase means channel 2 leads. For two antennas `d` wavelengths apart, a source at angle
//! `theta` from broadside, toward the channel 2 side, gives phase `360 * d * sin(theta)` degrees plus the
//! receiver's own offset `eps`, which calibration measures ([`swap_calibration`]). On a cable, a longer
//! cable to channel 2 makes it lag: [`cable_phase_deg`].
//!
//! Everything is accumulated in `f64`.

use crate::iq::Complex32;

/// Speed of light, m/s.
pub const C: f64 = 299_792_458.0;

/// One stretch of samples, measured.
#[derive(Debug, Clone, PartialEq)]
pub struct Segment {
    /// First sample, counted from the start of the capture.
    pub start: usize,
    pub len: usize,
    /// Phase of channel 2 relative to channel 1, degrees in (-180, 180].
    pub phase_deg: f64,
    /// `|r| / sqrt(p1 p2)`, 0 to 1: 1 is the same signal on both channels with no noise.
    pub coherence: f64,
    /// Power of channel 2 over channel 1, dB (signal plus noise).
    pub ratio_db: f64,
    /// Mean power per sample on each channel (signal plus noise).
    pub p1: f64,
    pub p2: f64,
    /// Per-sample SNR estimate of each channel against the given noise power, dB (None without one).
    pub snr1_db: Option<f64>,
    pub snr2_db: Option<f64>,
}

/// `(sum x2 conj(x1), sum |x1|^2, sum |x2|^2)` over the two slices (the shorter length).
pub fn cross_sums(x1: &[Complex32], x2: &[Complex32]) -> ((f64, f64), f64, f64) {
    let (mut re, mut im, mut p1, mut p2) = (0.0f64, 0.0f64, 0.0f64, 0.0f64);
    for (a, b) in x1.iter().zip(x2) {
        let (ar, ai, br, bi) = (a.re as f64, a.im as f64, b.re as f64, b.im as f64);
        // b * conj(a)
        re += br * ar + bi * ai;
        im += bi * ar - br * ai;
        p1 += ar * ar + ai * ai;
        p2 += br * br + bi * bi;
    }
    ((re, im), p1, p2)
}

/// Measure `len` samples from `start`. `noise` is each channel's noise power per sample, if known.
pub fn measure(
    x1: &[Complex32],
    x2: &[Complex32],
    start: usize,
    len: usize,
    noise: Option<(f64, f64)>,
) -> Segment {
    let end = (start + len).min(x1.len()).min(x2.len());
    let start = start.min(end);
    let n = (end - start).max(1) as f64;
    let ((re, im), s1, s2) = cross_sums(&x1[start..end], &x2[start..end]);
    let (p1, p2) = (s1 / n, s2 / n);
    let coherence = if s1 > 0.0 && s2 > 0.0 {
        (re.hypot(im) / (s1 * s2).sqrt()).min(1.0)
    } else {
        0.0
    };
    let snr = |p: f64, n0: f64| {
        if n0 > 0.0 && p > n0 {
            Some(10.0 * ((p - n0) / n0).log10())
        } else {
            None
        }
    };
    Segment {
        start,
        len: end - start,
        phase_deg: wrap_deg(im.atan2(re).to_degrees()),
        coherence,
        ratio_db: if p1 > 0.0 && p2 > 0.0 {
            10.0 * (p2 / p1).log10()
        } else {
            0.0
        },
        p1,
        p2,
        snr1_db: noise.and_then(|(n1, _)| snr(p1, n1)),
        snr2_db: noise.and_then(|(_, n2)| snr(p2, n2)),
    }
}

/// Mean power per sample in consecutive windows of `win` samples (the last partial window dropped).
pub fn window_powers(x: &[Complex32], win: usize) -> Vec<f64> {
    let win = win.max(1);
    x.chunks_exact(win)
        .map(|c| c.iter().map(|s| s.norm_sqr() as f64).sum::<f64>() / win as f64)
        .collect()
}

/// Noise power per sample: the 10th percentile of the window powers. Right when at least about a
/// tenth of the capture is quiet (between bursts); for a capture that is all signal it is too high.
pub fn noise_floor(x: &[Complex32], win: usize) -> f64 {
    let mut p = window_powers(x, win);
    if p.is_empty() {
        return 0.0;
    }
    p.sort_by(|a, b| a.total_cmp(b));
    p[p.len() / 10]
}

/// The bursts in a two-channel capture: runs of windows whose summed power is at least `above_db` over
/// the noise floor. One window is trimmed from each end of a run (the edges ramp), and runs shorter than
/// `min_len` samples after trimming are dropped. Returns `(start, len)` pairs.
pub fn find_bursts(
    x1: &[Complex32],
    x2: &[Complex32],
    win: usize,
    above_db: f64,
    min_len: usize,
) -> Vec<(usize, usize)> {
    let win = win.max(1);
    let p1 = window_powers(x1, win);
    let p2 = window_powers(x2, win);
    let p: Vec<f64> = p1.iter().zip(&p2).map(|(a, b)| a + b).collect();
    if p.is_empty() {
        return Vec::new();
    }
    let mut sorted = p.clone();
    sorted.sort_by(|a, b| a.total_cmp(b));
    let floor = sorted[sorted.len() / 10];
    let level = floor * 10f64.powf(above_db / 10.0);
    let mut out = Vec::new();
    let mut i = 0;
    while i < p.len() {
        if p[i] < level || floor <= 0.0 && p[i] == 0.0 {
            i += 1;
            continue;
        }
        let first = i;
        while i < p.len() && p[i] >= level {
            i += 1;
        }
        // windows first..i are loud; trim one from each end
        if i - first >= 3 {
            let (s, e) = ((first + 1) * win, (i - 1) * win);
            if e - s >= min_len {
                out.push((s, e - s));
            }
        }
    }
    out
}

/// Consecutive blocks of `block` samples over `n` samples (for a continuous source such as a tone).
pub fn fixed_blocks(n: usize, block: usize) -> Vec<(usize, usize)> {
    let block = block.max(1);
    (0..n / block).map(|i| (i * block, block)).collect()
}

/// An angle in degrees, wrapped to (-180, 180].
pub fn wrap_deg(x: f64) -> f64 {
    let y = (x + 180.0).rem_euclid(360.0) - 180.0;
    if y == -180.0 {
        180.0
    } else {
        y
    }
}

/// Circular mean and circular standard deviation (`sqrt(-2 ln R)`, in degrees) of angles in degrees.
/// For small spread the standard deviation is the ordinary one. `None` for no angles.
pub fn circ_stats_deg(a: &[f64]) -> Option<(f64, f64)> {
    if a.is_empty() {
        return None;
    }
    let (s, c) = a.iter().fold((0.0, 0.0), |(s, c), x| {
        let r = x.to_radians();
        (s + r.sin(), c + r.cos())
    });
    let n = a.len() as f64;
    let rbar = (s.hypot(c) / n).clamp(1e-300, 1.0);
    Some((
        wrap_deg(s.atan2(c).to_degrees()),
        (-2.0 * rbar.ln()).sqrt().to_degrees(),
    ))
}

/// Calibration by swapping the two cables at the receiver (TEST-PLAN step 1): `psi_a` measured one way,
/// `psi_b` with the cables swapped. The cables' own difference flips sign and the receiver's does not, so
/// the receiver offset is their circular mean and the cables' difference is half of `psi_a - psi_b`.
/// Returns `(receiver_offset, cable_difference)`, degrees.
pub fn swap_calibration(psi_a: f64, psi_b: f64) -> (f64, f64) {
    let half = wrap_deg(psi_a - psi_b) / 2.0;
    (wrap_deg(psi_b + half), half)
}

/// Direction of arrival in degrees from broadside (positive toward channel 2) for a measured phase,
/// a receiver offset `eps` and an antenna spacing in wavelengths. `None` when no direction fits (the
/// corrected phase is beyond `360 d`, which noise can do near endfire).
pub fn doa_deg(phase_deg: f64, eps_deg: f64, spacing_wl: f64) -> Option<f64> {
    let s = wrap_deg(phase_deg - eps_deg) / (360.0 * spacing_wl);
    (s.abs() <= 1.0).then(|| s.asin().to_degrees())
}

/// The phase in degrees a direction gives: `360 d sin(theta)` (no receiver offset), wrapped.
pub fn phase_for_doa_deg(theta_deg: f64, spacing_wl: f64) -> f64 {
    wrap_deg(360.0 * spacing_wl * theta_deg.to_radians().sin())
}

/// The phase of channel 2 relative to channel 1 when channel 2's cable is longer by `extra_m` metres,
/// at `freq_hz`, for a cable with velocity factor `vf` (0.66 for solid PE such as RG-58, about 0.7 for
/// RG-316, 0.85 for foam): `-360 f extra / (vf c)`, wrapped.
pub fn cable_phase_deg(extra_m: f64, freq_hz: f64, vf: f64) -> f64 {
    wrap_deg(-360.0 * freq_hz * extra_m / (vf * C))
}

/// The direction of arrival that a cable difference imitates: what a source at that angle would read,
/// for antennas `spacing_wl` apart. `None` if the cable phase is beyond what any direction gives.
pub fn cable_as_doa_deg(extra_m: f64, freq_hz: f64, vf: f64, spacing_wl: f64) -> Option<f64> {
    doa_deg(cable_phase_deg(extra_m, freq_hz, vf), 0.0, spacing_wl)
}

/// Approximate standard deviation of the measured phase, degrees, for `n` independent samples at
/// per-sample SNR `snr` (linear, equal on both channels): `var(arg r) = (2/snr + 1/snr^2) / (2n)`
/// (large-`n` result for the angle of a sample cross-correlation). Used to judge whether scatter is
/// noise or something else.
pub fn phase_std_deg(snr: f64, n: usize) -> f64 {
    ((2.0 / snr + 1.0 / (snr * snr)) / (2.0 * n.max(1) as f64))
        .sqrt()
        .to_degrees()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::channel::Channel;

    fn tone(n: usize, f: f64, phase_deg: f64, amp: f32) -> Vec<Complex32> {
        (0..n)
            .map(|i| {
                let t = std::f64::consts::TAU * f * i as f64 + phase_deg.to_radians();
                Complex32::from_polar(amp, t as f32)
            })
            .collect()
    }

    #[test]
    fn wraps() {
        assert_eq!(wrap_deg(180.0), 180.0);
        assert_eq!(wrap_deg(-180.0), 180.0);
        assert!((wrap_deg(190.0) + 170.0).abs() < 1e-12);
        assert!((wrap_deg(-725.0) + 5.0).abs() < 1e-12);
    }

    #[test]
    fn clean_phase_offset_and_cfo_cancel() {
        // channel 2 leads by 37 degrees, both share a frequency offset of 0.013 cycles/sample
        let a = tone(4096, 0.013, 10.0, 0.5);
        let b = tone(4096, 0.013, 47.0, 0.25);
        let s = measure(&a, &b, 0, 4096, None);
        assert!((s.phase_deg - 37.0).abs() < 1e-3, "{}", s.phase_deg);
        assert!((s.coherence - 1.0).abs() < 1e-6);
        assert!((s.ratio_db + 6.0206).abs() < 1e-3);
    }

    #[test]
    fn noisy_phase_scatter_matches_the_approximation() {
        let mut ch = Channel::new(7);
        let (n, snr_db, truth) = (1024usize, 0.0, -120.0);
        let n0 = 10f64.powf(-snr_db / 10.0);
        let mut phases = Vec::new();
        for k in 0..400 {
            let mut a = tone(n, 0.001 * k as f64, 0.0, 1.0);
            let mut b = tone(n, 0.001 * k as f64, truth, 1.0);
            ch.add_awgn(&mut a, n0);
            ch.add_awgn(&mut b, n0);
            phases.push(measure(&a, &b, 0, n, None).phase_deg);
        }
        let (m, sd) = circ_stats_deg(&phases).unwrap();
        let want = phase_std_deg(1.0, n);
        assert!(wrap_deg(m - truth).abs() < 0.5, "mean {m}");
        assert!((sd / want - 1.0).abs() < 0.15, "std {sd} vs {want}");
    }

    #[test]
    fn swap_calibration_separates_receiver_and_cables() {
        for (eps, cable) in [(12.0, 30.0), (-170.0, 25.0), (175.0, -40.0), (0.0, 0.0)] {
            let a = wrap_deg(eps + cable);
            let b = wrap_deg(eps - cable);
            let (e, c) = swap_calibration(a, b);
            assert!(wrap_deg(e - eps).abs() < 1e-9, "{eps} {cable}: {e}");
            assert!((c - cable).abs() < 1e-9, "{eps} {cable}: {c}");
        }
    }

    #[test]
    fn doa_round_trip_and_limits() {
        for th in [-80.0, -60.0, -30.0, 0.0, 15.0, 45.0, 89.0] {
            let p = wrap_deg(phase_for_doa_deg(th, 0.5) + 23.0);
            let got = doa_deg(p, 23.0, 0.5).unwrap();
            assert!((got - th).abs() < 1e-9, "{th}: {got}");
        }
        assert_eq!(doa_deg(170.0, 0.0, 0.4), None); // 0.4 wl reaches only 144 degrees
        assert!((doa_deg(180.0, 0.0, 0.5).unwrap() - 90.0).abs() < 1e-9);
    }

    #[test]
    fn cable_phase() {
        // 10 cm of RG-316 at 915 MHz: 360 * 915e6 * 0.1 / (0.7 c) = 156.97 degrees lag
        let p = cable_phase_deg(0.10, 915e6, 0.7);
        assert!((p + 156.97).abs() < 0.01, "{p}");
        // one wavelength in the cable is a full turn
        let wl = 0.7 * C / 915e6;
        assert!(cable_phase_deg(wl, 915e6, 0.7).abs() < 1e-6);
        // 3 cm imitates a source off broadside toward channel 1 (channel 2 lags)
        let th = cable_as_doa_deg(0.03, 915e6, 0.7, 0.5).unwrap();
        assert!(th < 0.0 && th > -20.0, "{th}");
    }

    #[test]
    fn bursts_are_found_and_trimmed() {
        let mut ch = Channel::new(3);
        let n0 = 1e-4;
        let mut a = ch.noise(40_000, n0);
        let mut b = ch.noise(40_000, n0);
        for (s, len) in [(5_000usize, 8_000usize), (20_000, 12_000), (36_000, 600)] {
            let t1 = tone(len, 0.05, 0.0, 0.3);
            let t2 = tone(len, 0.05, -64.0, 0.3);
            for i in 0..len {
                a[s + i] += t1[i];
                b[s + i] += t2[i];
            }
        }
        let bursts = find_bursts(&a, &b, 256, 10.0, 2048);
        assert_eq!(bursts.len(), 2, "{bursts:?}");
        for &(s, len) in &bursts {
            let seg = measure(&a, &b, s, len, Some((n0, n0)));
            assert!((seg.phase_deg + 64.0).abs() < 0.2, "{seg:?}");
            assert!(seg.coherence > 0.99);
            let snr = seg.snr1_db.unwrap();
            assert!((snr - 29.5).abs() < 0.5, "{snr}"); // 0.09 / 1e-4
        }
        assert!(bursts[0].0 >= 5_000 && bursts[0].0 + bursts[0].1 <= 13_000);
        assert!((noise_floor(&a, 256) / n0 - 1.0).abs() < 0.3);
    }

    #[test]
    fn circ_stats_across_the_wrap() {
        let (m, sd) = circ_stats_deg(&[179.0, -179.0, 178.0, -178.0]).unwrap();
        assert!((m.abs() - 180.0).abs() < 1e-6, "{m}");
        assert!(sd < 2.0);
        assert_eq!(circ_stats_deg(&[]), None);
    }
}
