//! Bit error rate against theory, through the whole receiver (detection, timing, phase, demodulation).
//!
//! Every point sends random 64-byte frames with random start delay and random carrier phase through
//! AWGN at the stated Eb/N0 and counts payload bit errors (`sim::run_ber`). The pass rule is a
//! 3-sigma binomial interval around the theory value; at 0 dB the receiver's own small
//! implementation loss (see docs/waveform-w0.md) is allowed for with 5 %. Seeds are fixed, so a
//! run repeats exactly.
//!
//! Debug builds are fast enough because Cargo.toml optimizes this crate even in the dev profile.
//! `long_sweep` is `#[ignore]`d: run it with `cargo test --release -p oasa-waveform -- --ignored`.

use oasa_waveform::sim::{run_ber, BerOptions, BerPoint};
use oasa_waveform::Modulation;

/// Is the measured error count consistent with the theory, to `z` sigma, allowing the theory value to
/// be up to `slack` (a fraction) higher? (A receiver only loses to the ideal, never beats it.)
fn consistent(p: &BerPoint, z: f64, slack: f64) -> bool {
    let n = p.bits as f64;
    let sd = (n * p.theory * (1.0 - p.theory)).sqrt();
    let lo = n * p.theory - z * sd;
    let hi = n * p.theory * (1.0 + slack) + z * sd;
    (p.errors as f64) >= lo && (p.errors as f64) <= hi
}

fn show(p: &BerPoint) -> String {
    format!(
        "{:?} {} dB: {} errors in {} bits = {:.4e}, theory {:.4e}, z {:+.2}, detected {}/{}",
        p.modulation, p.ebn0_db, p.errors, p.bits, p.ber, p.theory, p.z, p.detected, p.frames
    )
}

fn check_points(m: Modulation, points: &[(f64, u64)], seed: u64) {
    let opts = BerOptions::default();
    for &(db, bits) in points {
        let p = run_ber(m, db, bits, seed, &opts).unwrap();
        println!("{}", show(&p));
        assert_eq!(p.detected, p.frames, "missed frames: {}", show(&p));
        let slack = if db <= 0.5 { 0.05 } else { 0.0 };
        assert!(consistent(&p, 3.0, slack), "off theory: {}", show(&p));
    }
}

#[test]
fn bpsk_matches_theory() {
    check_points(
        Modulation::Bpsk,
        &[
            (0.0, 40_000),
            (2.0, 40_000),
            (4.0, 80_000),
            (6.0, 120_000),
            (8.0, 400_000),
        ],
        101,
    );
}

#[test]
fn qpsk_matches_theory() {
    check_points(
        Modulation::Qpsk,
        &[
            (0.0, 40_000),
            (2.0, 40_000),
            (4.0, 80_000),
            (6.0, 120_000),
            (8.0, 400_000),
        ],
        202,
    );
}

#[test]
fn fsk2_noncoherent_matches_theory() {
    check_points(
        Modulation::Fsk2,
        &[
            (0.0, 20_000),
            (2.0, 20_000),
            (4.0, 20_000),
            (6.0, 20_000),
            (8.0, 40_000),
        ],
        303,
    );
}

#[test]
fn ber_does_not_depend_on_oversampling() {
    // The classic bug is to set the noise from the per-sample SNR. With the energy convention of the
    // crate, Eb/N0 means the same at 4, 8 and 16 samples per symbol.
    for m in [Modulation::Bpsk, Modulation::Fsk2] {
        let db = if m == Modulation::Bpsk { 4.0 } else { 6.0 };
        for sps in [4usize, 16] {
            let opts = BerOptions {
                sps,
                ..BerOptions::default()
            };
            let p = run_ber(m, db, 40_000, 7 + sps as u64, &opts).unwrap();
            println!("sps {sps}: {}", show(&p));
            assert!(consistent(&p, 3.0, 0.0), "sps {sps}: {}", show(&p));
        }
    }
}

#[test]
fn the_check_can_tell_a_wrong_snr() {
    // The pass rule has to be able to fail. Measure BPSK at 4 dB and compare it with the theory
    // for 3 dB and for 5 dB (a 1 dB error): both must be far outside.
    let opts = BerOptions::default();
    let p = run_ber(Modulation::Bpsk, 4.0, 80_000, 11, &opts).unwrap();
    assert!(consistent(&p, 3.0, 0.0), "{}", show(&p));
    for wrong_db in [3.0, 5.0] {
        let th = oasa_waveform::sim::theory_ber(Modulation::Bpsk, wrong_db);
        let z = oasa_waveform::theory::binomial_z(p.errors, p.bits, th);
        assert!(z.abs() > 10.0, "1 dB error only {z} sigma off");
    }
}

#[test]
fn same_seed_same_result() {
    let opts = BerOptions::default();
    let a = run_ber(Modulation::Qpsk, 3.0, 10_000, 5, &opts).unwrap();
    let b = run_ber(Modulation::Qpsk, 3.0, 10_000, 5, &opts).unwrap();
    let c = run_ber(Modulation::Qpsk, 3.0, 10_000, 6, &opts).unwrap();
    assert_eq!(a, b);
    assert_ne!(a.errors, c.errors);
}

#[test]
fn bpsk_and_qpsk_share_one_curve_per_eb_n0() {
    // Same Eb/N0 -> same BER, though QPSK has twice the Es/N0.
    let opts = BerOptions::default();
    let b = run_ber(Modulation::Bpsk, 4.0, 100_000, 21, &opts).unwrap();
    let q = run_ber(Modulation::Qpsk, 4.0, 100_000, 22, &opts).unwrap();
    // two independent counts; their difference has sd sqrt(e1 + e2)
    let d = b.errors as f64 - q.errors as f64;
    let sd = ((b.errors + q.errors) as f64).sqrt();
    assert!(
        d.abs() < 3.5 * sd,
        "BPSK {} vs QPSK {} errors",
        b.errors,
        q.errors
    );
}

#[test]
fn small_frequency_offset_does_not_change_the_ber() {
    // 0.4 % of the symbol rate: the preamble estimate and the whole-frame phase fit take it out.
    for m in [Modulation::Bpsk, Modulation::Qpsk, Modulation::Fsk2] {
        let db = if m == Modulation::Fsk2 { 8.0 } else { 5.0 };
        let opts = BerOptions {
            cfo_cycles_per_symbol: 0.004,
            ..BerOptions::default()
        };
        let p = run_ber(m, db, 60_000, 31, &opts).unwrap();
        println!("cfo 0.004: {}", show(&p));
        assert_eq!(p.detected, p.frames);
        assert!(consistent(&p, 3.0, 0.05), "{}", show(&p));
    }
}

/// 0 to 10 dB, a million bits a point. Slow: run in release with `--ignored`.
#[test]
#[ignore]
fn long_sweep() {
    let opts = BerOptions::default();
    for m in [Modulation::Bpsk, Modulation::Qpsk, Modulation::Fsk2] {
        for db in [0.0, 2.0, 4.0, 6.0, 8.0, 10.0] {
            let bits = if m == Modulation::Fsk2 && db > 8.0 {
                4_000_000
            } else {
                2_000_000
            };
            let p = run_ber(m, db, bits, 1, &opts).unwrap();
            println!("{}", show(&p));
            assert!(consistent(&p, 3.0, 0.05), "{}", show(&p));
        }
    }
}

#[test]
fn random_sub_sample_delay_costs_little() {
    // With the delay anywhere in a sample, the PSK receiver is still within a few percent of theory.
    for (m, db) in [
        (Modulation::Bpsk, 4.0),
        (Modulation::Qpsk, 4.0),
        (Modulation::Fsk2, 6.0),
    ] {
        let opts = BerOptions {
            random_frac_delay: true,
            ..BerOptions::default()
        };
        let p = run_ber(m, db, 80_000, 41, &opts).unwrap();
        println!("fractional delay: {}", show(&p));
        assert_eq!(p.detected, p.frames);
        assert!(consistent(&p, 3.0, 0.08), "{}", show(&p));
    }
}
