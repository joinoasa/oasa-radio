//! The textbook curves the simulated modem is checked against, and the statistics for the check.
//!
//! All `ebn0` arguments are Eb/N0 as a plain ratio (not dB) unless the name ends in `_db`.
//! Eb is the energy of one information bit, N0 the one-sided noise power density (for complex
//! baseband samples: the variance of one complex noise sample when the sample energy convention in
//! the crate docs is used).

use std::f64::consts::PI;

/// Complementary error function.
///
/// Below 2.5 a power series for erf (all terms positive, so no cancellation) and `1 - erf`; from
/// 2.5 up the continued fraction
/// `erfc(x) = e^(-x^2)/sqrt(pi) / (x + (1/2)/(x + 1/(x + (3/2)/(x + ...))))` with 200 terms.
/// Accuracy: the tests compare 12 values (x from 0 to 6, taken from Python's `math.erfc`) and
/// require a relative error under `1e-12`. Not meant for `x` beyond about 26 (the result underflows).
pub fn erfc(x: f64) -> f64 {
    if x.is_nan() {
        return f64::NAN;
    }
    if x < 0.0 {
        return 2.0 - erfc(-x);
    }
    if x < 2.5 {
        let x2 = x * x;
        let mut term = x;
        let mut sum = x;
        let mut n = 1.0;
        while term > 1e-18 * sum && n < 500.0 {
            term *= 2.0 * x2 / (2.0 * n + 1.0);
            sum += term;
            n += 1.0;
        }
        1.0 - 2.0 / PI.sqrt() * (-x2).exp() * sum
    } else {
        let mut t = x;
        for k in (1..=200).rev() {
            t = x + (k as f64 / 2.0) / t;
        }
        (-x * x).exp() / (PI.sqrt() * t)
    }
}

/// The Gaussian tail probability `Q(x) = P(N(0,1) > x) = erfc(x / sqrt 2) / 2`.
pub fn q(x: f64) -> f64 {
    0.5 * erfc(x / std::f64::consts::SQRT_2)
}

pub fn db_to_ratio(db: f64) -> f64 {
    10f64.powf(db / 10.0)
}

/// BPSK over AWGN, coherent: `Q(sqrt(2 Eb/N0))`.
pub fn ber_bpsk(ebn0: f64) -> f64 {
    q((2.0 * ebn0).sqrt())
}

/// Same curve as BPSK: Gray QPSK is two independent BPSK streams at the same Eb/N0.
pub fn ber_qpsk(ebn0: f64) -> f64 {
    ber_bpsk(ebn0)
}

/// Coherent orthogonal binary FSK: `Q(sqrt(Eb/N0))`.
pub fn ber_bfsk_coherent(ebn0: f64) -> f64 {
    q(ebn0.sqrt())
}

/// Noncoherent orthogonal binary FSK: `0.5 exp(-Eb/(2 N0))`.
pub fn ber_bfsk_noncoherent(ebn0: f64) -> f64 {
    0.5 * (-ebn0 / 2.0).exp()
}

/// Wilson score interval for `k` successes in `n` trials at `z` standard deviations (3 is about 99.7 %).
pub fn wilson(k: u64, n: u64, z: f64) -> (f64, f64) {
    if n == 0 {
        return (0.0, 1.0);
    }
    let n = n as f64;
    let p = k as f64 / n;
    let z2 = z * z;
    let denom = 1.0 + z2 / n;
    let centre = (p + z2 / (2.0 * n)) / denom;
    let half = z * (p * (1.0 - p) / n + z2 / (4.0 * n * n)).sqrt() / denom;
    ((centre - half).max(0.0), (centre + half).min(1.0))
}

/// How many binomial standard deviations `k` errors in `n` bits are from the expected `n p`.
/// Positive means more errors than the theory predicts.
pub fn binomial_z(k: u64, n: u64, p: f64) -> f64 {
    let n = n as f64;
    let sd = (n * p * (1.0 - p)).sqrt();
    if sd == 0.0 {
        return if k as f64 == n * p {
            0.0
        } else {
            f64::INFINITY
        };
    }
    (k as f64 - n * p) / sd
}

#[cfg(test)]
mod tests {
    use super::*;

    // From Python's math.erfc.
    const ERFC: &[(f64, f64)] = &[
        (0.0, 1.0),
        (0.1, 0.8875370839817152),
        (0.5, 0.4795001221869535),
        (1.0, 0.15729920705028513),
        (1.5, 0.033894853524689274),
        (2.0, 0.004677734981047265),
        (2.4, 0.0006885138966450789),
        (2.5, 0.0004069520174449589),
        (3.0, 2.2090496998585438e-05),
        (4.0, 1.541725790028002e-08),
        (5.0, 1.5374597944280351e-12),
        (6.0, 2.1519736712498916e-17),
    ];

    #[test]
    fn erfc_matches_known_values() {
        for &(x, want) in ERFC {
            let got = erfc(x);
            let rel = ((got - want) / want).abs();
            assert!(
                rel < 1e-12,
                "erfc({x}) = {got:e}, want {want:e}, rel {rel:e}"
            );
        }
        assert!((erfc(-1.0) - (2.0 - 0.15729920705028513)).abs() < 1e-15);
    }

    #[test]
    fn erfc_is_continuous_across_the_method_switch() {
        let below = erfc(2.5 - 1e-9);
        let above = erfc(2.5 + 1e-9);
        // the true relative step over 2e-9 is about 1.1e-8; a method mismatch would show as more
        let step = (below - above) / below;
        assert!(step > 0.0 && step < 1.3e-8, "step {step:e}");
    }

    #[test]
    fn ber_curves_match_known_points() {
        let cases: &[(f64, f64, f64, f64)] = &[
            // Eb/N0 dB, BPSK, noncoherent BFSK, coherent BFSK (Python)
            (
                0.0,
                0.07864960352514257,
                0.3032653298563167,
                0.15865525393145702,
            ),
            (
                4.0,
                0.012500818040737556,
                0.1424035434814758,
                0.05649530174936167,
            ),
            (
                8.0,
                0.00019090777407599314,
                0.021323747889132938,
                0.006004386400163561,
            ),
            (
                10.0,
                3.872108215522037e-06,
                0.0033689734995427335,
                0.0007827011290012744,
            ),
        ];
        for &(db, bpsk, nc, co) in cases {
            let e = db_to_ratio(db);
            assert!(((ber_bpsk(e) - bpsk) / bpsk).abs() < 1e-11, "{db}");
            assert!(((ber_qpsk(e) - bpsk) / bpsk).abs() < 1e-11, "{db}");
            assert!(((ber_bfsk_noncoherent(e) - nc) / nc).abs() < 1e-12, "{db}");
            assert!(((ber_bfsk_coherent(e) - co) / co).abs() < 1e-11, "{db}");
        }
    }

    #[test]
    fn wilson_contains_the_estimate_and_narrows() {
        let (lo, hi) = wilson(50, 1000, 3.0);
        assert!(lo < 0.05 && 0.05 < hi);
        let (lo2, hi2) = wilson(500, 10_000, 3.0);
        assert!(hi2 - lo2 < hi - lo);
        assert_eq!(wilson(0, 0, 3.0), (0.0, 1.0));
        let (lo0, _) = wilson(0, 100, 3.0);
        assert_eq!(lo0, 0.0);
    }

    #[test]
    fn binomial_z_sign_and_size() {
        assert!((binomial_z(100, 1000, 0.1)).abs() < 1e-12);
        let z = binomial_z(130, 1000, 0.1); // sd = sqrt(90)
        assert!((z - 30.0 / 90f64.sqrt()).abs() < 1e-9);
        assert!(binomial_z(70, 1000, 0.1) < 0.0);
    }
}
