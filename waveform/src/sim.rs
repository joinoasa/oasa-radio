//! Monte Carlo bit error rate measurement through the whole chain: frame, modulator, channel,
//! receiver (detection, timing, phase, demodulation). Used by the tests and by
//! `oasa-radio waveform ber`.

use crate::channel::{n0_for_ebn0, Channel, Impairments};
use crate::modem::{ConfigError, Modem, ModemConfig, Modulation, RxConfig};
use crate::theory;
use std::f64::consts::PI;

#[derive(Debug, Clone, PartialEq)]
pub struct BerOptions {
    pub sps: usize,
    /// Payload bytes per frame.
    pub payload_len: usize,
    /// Largest random sample delay before the frame (the delay is 0 up to this many samples, plus 2).
    pub max_delay: usize,
    /// Give every frame a random sub-sample delay (otherwise 0).
    pub random_frac_delay: bool,
    /// Give every frame a random carrier phase (otherwise 0).
    pub random_phase: bool,
    /// Carrier frequency offset in cycles per symbol, same for every frame.
    pub cfo_cycles_per_symbol: f64,
    /// Receiver settings other than the BER-measurement ones (`expected_len`, no sync rejection).
    pub rx: RxConfig,
}

impl Default for BerOptions {
    fn default() -> Self {
        Self {
            sps: 8,
            payload_len: 64,
            max_delay: 61,
            random_frac_delay: false,
            random_phase: true,
            cfo_cycles_per_symbol: 0.0,
            rx: RxConfig::default(),
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct BerPoint {
    pub modulation: Modulation,
    pub ebn0_db: f64,
    pub frames: u64,
    /// Frames the receiver found (at the right place).
    pub detected: u64,
    /// Frames found whose CRC matched.
    pub crc_ok: u64,
    /// Payload bits compared (detected frames only).
    pub bits: u64,
    pub errors: u64,
    pub ber: f64,
    /// Theory: Q(sqrt(2 Eb/N0)) for BPSK and QPSK; noncoherent 0.5 exp(-Eb/2N0) for 2-FSK.
    pub theory: f64,
    /// Wilson interval at 3 sigma around the measured BER.
    pub ci_lo: f64,
    pub ci_hi: f64,
    /// Binomial z-score of the error count against the theory.
    pub z: f64,
    pub seed: u64,
}

/// The theoretical BER this crate compares each modulation with.
pub fn theory_ber(m: Modulation, ebn0_db: f64) -> f64 {
    let e = theory::db_to_ratio(ebn0_db);
    match m {
        Modulation::Bpsk => theory::ber_bpsk(e),
        Modulation::Qpsk => theory::ber_qpsk(e),
        Modulation::Fsk2 => theory::ber_bfsk_noncoherent(e),
    }
}

/// Send about `target_bits` payload bits through the chain at `ebn0_db` and count errors.
/// Everything random (payloads, delays, phases, noise) comes from `seed`, mixed with the Eb/N0.
pub fn run_ber(
    modulation: Modulation,
    ebn0_db: f64,
    target_bits: u64,
    seed: u64,
    opts: &BerOptions,
) -> Result<BerPoint, ConfigError> {
    let modem = Modem::new(ModemConfig {
        sps: opts.sps,
        ..ModemConfig::new(modulation)
    })?;
    if opts.payload_len == 0 || opts.payload_len > 255 {
        return Err(ConfigError("payload_len must be 1 to 255".into()));
    }
    let frame_bits = (opts.payload_len * 8) as u64;
    let frames = target_bits.div_ceil(frame_bits).max(1);
    let mixed = seed ^ ebn0_db.to_bits().rotate_left(17) ^ (modulation as u64) << 60;
    let mut ch = Channel::new(mixed);
    let n0 = n0_for_ebn0(ebn0_db, modem.energy_per_bit());
    // BER measurement: do not drop frames on a sync or header error, take the length as known.
    let rx = RxConfig {
        sync_max_errors: 16,
        expected_len: Some(opts.payload_len),
        ..opts.rx.clone()
    };
    let (mut detected, mut crc_ok, mut bits, mut errors) = (0u64, 0u64, 0u64, 0u64);
    for _ in 0..frames {
        let payload = ch.rng().bytes(opts.payload_len);
        let tx = modem.modulate(&payload)?;
        let lead = 2 + ch.rng().below(opts.max_delay as u64 + 1) as usize;
        let imp = Impairments {
            n0,
            phase_rad: if opts.random_phase {
                ch.rng().uniform() * 2.0 * PI
            } else {
                0.0
            },
            frac_delay: if opts.random_frac_delay {
                ch.rng().uniform()
            } else {
                0.0
            },
            cfo_cycles_per_sample: opts.cfo_cycles_per_symbol / opts.sps as f64,
            lead,
            tail: 8,
        };
        let y = ch.apply(&tx, &imp);
        let found = modem
            .demodulate_with(&y, &rx)
            .into_iter()
            .find(|f| f.start.abs_diff(lead) <= opts.sps);
        if let Some(f) = found {
            detected += 1;
            crc_ok += f.crc_ok as u64;
            bits += frame_bits;
            errors += payload
                .iter()
                .zip(&f.payload)
                .map(|(a, b)| (a ^ b).count_ones() as u64)
                .sum::<u64>();
        }
    }
    let ber = if bits > 0 {
        errors as f64 / bits as f64
    } else {
        f64::NAN
    };
    let th = theory_ber(modulation, ebn0_db);
    let (ci_lo, ci_hi) = theory::wilson(errors, bits, 3.0);
    Ok(BerPoint {
        modulation,
        ebn0_db,
        frames,
        detected,
        crc_ok,
        bits,
        errors,
        ber,
        theory: th,
        ci_lo,
        ci_hi,
        z: theory::binomial_z(errors, bits, th),
        seed,
    })
}
