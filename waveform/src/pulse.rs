//! The root-raised-cosine pulse.

use std::f64::consts::PI;

/// Root-raised-cosine taps for `sps` samples per symbol, `span` symbols long (so `span * sps + 1`
/// taps, centred), roll-off `beta` in `(0, 1]`. Scaled to unit energy: the sum of squares is 1.
///
/// With unit energy, a symbol of amplitude `a` sent through this pulse has energy `|a|^2`
/// (the sum of `|x|^2` over its samples), and the same taps used as the matched filter give the
/// symbol back at amplitude `a` with noise variance `N0` per complex sample.
pub fn rrc(sps: usize, span: usize, beta: f64) -> Vec<f32> {
    rrc_shifted(sps, span, beta, 0.0)
}

/// The same pulse, its centre moved `shift` samples to the right (a fractional delay), still
/// `span * sps + 1` taps and scaled to unit energy. Used by the receiver for sub-sample timing.
pub fn rrc_shifted(sps: usize, span: usize, beta: f64, shift: f64) -> Vec<f32> {
    let n = span * sps + 1;
    let centre = (n - 1) as f64 / 2.0 + shift;
    let mut h: Vec<f64> = (0..n)
        .map(|i| rrc_at((i as f64 - centre) / sps as f64, beta))
        .collect();
    let e: f64 = h.iter().map(|x| x * x).sum();
    let k = 1.0 / e.sqrt();
    h.iter_mut().for_each(|x| *x *= k);
    h.into_iter().map(|x| x as f32).collect()
}

/// The continuous RRC pulse (unnormalized) at time `t` in symbol periods.
fn rrc_at(t: f64, beta: f64) -> f64 {
    if t.abs() < 1e-12 {
        1.0 - beta + 4.0 * beta / PI
    } else if ((4.0 * beta * t).abs() - 1.0).abs() < 1e-9 {
        let a = PI / (4.0 * beta);
        beta / 2f64.sqrt() * ((1.0 + 2.0 / PI) * a.sin() + (1.0 - 2.0 / PI) * a.cos())
    } else {
        let num = (PI * t * (1.0 - beta)).sin() + 4.0 * beta * t * (PI * t * (1.0 + beta)).cos();
        let den = PI * t * (1.0 - (4.0 * beta * t).powi(2));
        num / den
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unit_energy_symmetric_and_odd_length() {
        let h = rrc(8, 10, 0.35);
        assert_eq!(h.len(), 81);
        let e: f64 = h.iter().map(|&x| (x as f64).powi(2)).sum();
        assert!((e - 1.0).abs() < 1e-6);
        for i in 0..h.len() {
            assert!((h[i] - h[h.len() - 1 - i]).abs() < 1e-7);
        }
    }

    #[test]
    fn transmit_times_receive_is_nearly_nyquist() {
        // h * h sampled at the symbol rate: 1 at the centre, ~0 at the other symbol instants.
        for (sps, beta) in [(8usize, 0.35f64), (4, 0.5), (16, 0.25)] {
            let h = rrc(sps, 12, beta);
            let n = h.len();
            let acf = |lag: isize| -> f64 {
                (0..n as isize)
                    .filter(|&i| i + lag >= 0 && i + lag < n as isize)
                    .map(|i| h[i as usize] as f64 * h[(i + lag) as usize] as f64)
                    .sum()
            };
            assert!((acf(0) - 1.0).abs() < 1e-6);
            for k in 1..=4isize {
                let isi = acf(k * sps as isize).abs();
                assert!(isi < 0.01, "sps {sps} beta {beta} lag {k}: ISI {isi}");
            }
        }
    }

    #[test]
    fn singular_points_are_finite() {
        // t = 1/(4 beta) lands on a tap when sps * 1/(4 beta) is an integer: beta 0.25, sps 8 -> t = 1.
        let h = rrc(8, 10, 0.25);
        assert!(h.iter().all(|x| x.is_finite()));
    }
}
