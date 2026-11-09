//! Monte Carlo bit error rate measurement through the whole chain: frame, modulator, channel,
//! receiver (detection, timing, phase, demodulation). Used by the tests and by
//! `oasa-radio waveform ber`.

use crate::channel::{n0_for_ebn0, Channel, Impairments};
use crate::diversity::Combine;
use crate::fading::{Fader, Fading};
use crate::frame::{FrameFormat, HeaderFields, PayloadCode};
use crate::modem::{ConfigError, Modem, ModemConfig, Modulation, RxConfig, RxReport};
use crate::theory;
use std::f64::consts::PI;

#[derive(Debug, Clone, PartialEq)]
pub struct BerOptions {
    pub sps: usize,
    /// Frame format.
    pub frame: FrameFormat,
    /// Error correction on the body (v1 only). Eb/N0 is per information bit either way.
    pub code: PayloadCode,
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
    /// Add a random offset per frame, uniform in `+-cfo_random` cycles per symbol (0: none).
    pub cfo_random: f64,
    /// Receiver settings other than the BER-measurement ones (`expected_len`, no sync rejection).
    pub rx: RxConfig,
    /// Fading applied to each frame before the rest of the channel (see [`crate::fading`]). None by default.
    pub fading: Fading,
    /// Samples from one frame's start to the next, for the fading's clock (frames sent at this rate).
    pub frame_interval: f64,
    /// [`run_found`] only: receive on two antennas (B14), each with its own independent fading (same
    /// model) and noise at the same mean Eb/N0, combined as given. None (default): one antenna, and the
    /// run is exactly as without this field.
    pub diversity: Option<Combine>,
}

impl Default for BerOptions {
    fn default() -> Self {
        Self {
            sps: 8,
            frame: FrameFormat::W0,
            code: PayloadCode::None,
            payload_len: 64,
            max_delay: 61,
            random_frac_delay: false,
            random_phase: true,
            cfo_cycles_per_symbol: 0.0,
            cfo_random: 0.0,
            rx: RxConfig::default(),
            fading: Fading::None,
            frame_interval: 10_000.0,
            diversity: None,
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
    /// v1 only: found frames whose header decoded to exactly what was sent (0 for W0). The header
    /// is decoded by the real path even though the BER measurement skips its checks.
    pub header_ok: u64,
    /// Payload bits compared (detected frames only).
    pub bits: u64,
    pub errors: u64,
    pub ber: f64,
    /// Theory: Q(sqrt(2 Eb/N0)) for BPSK and QPSK; noncoherent 0.5 exp(-Eb/2N0) for 2-FSK. NaN
    /// with a code (there is no closed form; compare packet loss with `docs/fec-light.md`).
    pub theory: f64,
    /// Wilson interval at 3 sigma around the measured BER.
    pub ci_lo: f64,
    pub ci_hi: f64,
    /// Binomial z-score of the error count against the theory (NaN with a code).
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
        frame: opts.frame,
        code: opts.code,
        ..ModemConfig::new(modulation)
    })?;
    if opts.payload_len == 0 || opts.payload_len > 255 {
        return Err(ConfigError("payload_len must be 1 to 255".into()));
    }
    let frame_bits = (opts.payload_len * 8) as u64;
    let frames = target_bits.div_ceil(frame_bits).max(1);
    let mixed = seed ^ ebn0_db.to_bits().rotate_left(17) ^ (modulation as u64) << 60;
    let mut ch = Channel::new(mixed);
    // its own generator, so the AWGN-only runs stay bit-identical
    let mut fader = Fader::new(
        opts.fading,
        opts.frame_interval,
        mixed ^ 0x0FAD_E5EE_D000_0001,
    );
    let n0 = n0_for_ebn0(ebn0_db, modem.energy_per_bit());
    // BER measurement: do not drop frames on a sync or header error, take the length as known.
    let rx = RxConfig {
        sync_max_errors: 16,
        expected_len: Some(opts.payload_len),
        ..opts.rx.clone()
    };
    let (mut detected, mut crc_ok, mut header_ok, mut bits, mut errors) =
        (0u64, 0u64, 0u64, 0u64, 0u64);
    let sent_header = HeaderFields {
        payload_code: opts.code.code(),
        ..HeaderFields::new(modulation.code(), opts.payload_len as u8)
    };
    for _ in 0..frames {
        let payload = ch.rng().bytes(opts.payload_len);
        let tx = modem.modulate(&payload)?;
        let tx = if fader.is_none() {
            tx
        } else {
            fader.apply(&tx)
        };
        let lead = 2 + ch.rng().below(opts.max_delay as u64 + 1) as usize;
        let cfo = if opts.cfo_random > 0.0 {
            opts.cfo_cycles_per_symbol + (2.0 * ch.rng().uniform() - 1.0) * opts.cfo_random
        } else {
            opts.cfo_cycles_per_symbol
        };
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
            cfo_cycles_per_sample: cfo / opts.sps as f64,
            lead,
            tail: 8,
        };
        let y = ch.apply(&tx, &imp);
        let found = modem
            .demodulate_with(&y, &rx)
            .into_iter()
            .find(|f| f.start.abs_diff(lead as u64) <= opts.sps as u64);
        if let Some(f) = found {
            detected += 1;
            crc_ok += f.crc_ok as u64;
            header_ok += (f.header == Some(sent_header)) as u64;
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
    let th = match opts.code {
        PayloadCode::None => theory_ber(modulation, ebn0_db),
        _ => f64::NAN,
    };
    let (ci_lo, ci_hi) = theory::wilson(errors, bits, 3.0);
    Ok(BerPoint {
        modulation,
        ebn0_db,
        frames,
        detected,
        crc_ok,
        header_ok,
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

/// Frames found when the receiver is left to find the length itself (no `expected_len`).
#[derive(Debug, Clone, PartialEq)]
pub struct FoundPoint {
    pub modulation: Modulation,
    pub frame: FrameFormat,
    pub code: PayloadCode,
    pub ebn0_db: f64,
    pub frames: u64,
    /// Frames returned at the right place: the header was accepted. The payload may still be damaged.
    pub found: u64,
    /// Of those, with a good CRC-16: frames delivered. Packet loss is `1 - crc_ok / frames`.
    pub crc_ok: u64,
    /// What the receiver did with the candidates, summed over all frames.
    pub report: RxReport,
    /// Wall time spent in the receiver (all channels), seconds.
    pub rx_seconds: f64,
}

/// Send `frames` frames at `ebn0_db` through the channel and count the ones the receiver finds with
/// its normal settings (`opts.rx`; `expected_len` is ignored). Random payload, delay and phase as in
/// [`run_ber`]. Everything random comes from `seed`, mixed with the Eb/N0.
pub fn run_found(
    modulation: Modulation,
    ebn0_db: f64,
    frames: u64,
    seed: u64,
    opts: &BerOptions,
) -> Result<FoundPoint, ConfigError> {
    let modem = Modem::new(ModemConfig {
        sps: opts.sps,
        frame: opts.frame,
        code: opts.code,
        ..ModemConfig::new(modulation)
    })?;
    if opts.payload_len > 255 {
        return Err(ConfigError("payload_len must be 0 to 255".into()));
    }
    let mixed = seed ^ ebn0_db.to_bits().rotate_left(17) ^ (modulation as u64) << 60;
    let mut ch = Channel::new(mixed);
    // its own generator, so the AWGN-only runs stay bit-identical
    let mut fader = Fader::new(
        opts.fading,
        opts.frame_interval,
        mixed ^ 0x0FAD_E5EE_D000_0001,
    );
    let n0 = n0_for_ebn0(ebn0_db, modem.energy_per_bit());
    let rx = RxConfig {
        expected_len: None,
        ..opts.rx.clone()
    };
    let (mut found, mut crc_ok) = (0u64, 0u64);
    let mut report = RxReport::default();
    let mut rx_seconds = 0.0f64;
    // The second antenna: its own fading and noise, from generators of its own (the first
    // antenna's stay what they are without diversity).
    let mut ch2 = Channel::new(mixed ^ 0xD1FE_25E7_0000_0002);
    let mut fader2 = Fader::new(
        opts.fading,
        opts.frame_interval,
        mixed ^ 0x0FAD_E5EE_D000_0002,
    );
    for _ in 0..frames {
        let payload = ch.rng().bytes(opts.payload_len);
        let clean = modem.modulate(&payload)?;
        let tx = if fader.is_none() {
            clean.clone()
        } else {
            fader.apply(&clean)
        };
        let lead = 2 + ch.rng().below(opts.max_delay as u64 + 1) as usize;
        let cfo = if opts.cfo_random > 0.0 {
            opts.cfo_cycles_per_symbol + (2.0 * ch.rng().uniform() - 1.0) * opts.cfo_random
        } else {
            opts.cfo_cycles_per_symbol
        };
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
            cfo_cycles_per_sample: cfo / opts.sps as f64,
            lead,
            tail: 8,
        };
        let y = ch.apply(&tx, &imp);
        let t0 = std::time::Instant::now();
        let (got, rep) = match opts.diversity {
            None => modem.demodulate_report(&y, &rx),
            Some(how) => {
                let tx2 = if fader2.is_none() {
                    clean
                } else {
                    fader2.apply(&clean)
                };
                let imp2 = Impairments {
                    // the antennas see the carrier at phases of their own (same offset: one LO)
                    phase_rad: if opts.random_phase {
                        ch2.rng().uniform() * 2.0 * PI
                    } else {
                        0.0
                    },
                    ..imp
                };
                let y2 = ch2.apply(&tx2, &imp2);
                let t1 = std::time::Instant::now();
                let r = modem.demodulate_diversity(&[&y, &y2], &rx, how);
                rx_seconds += t1.elapsed().as_secs_f64();
                r
            }
        };
        if opts.diversity.is_none() {
            rx_seconds += t0.elapsed().as_secs_f64();
        }
        report.add(&rep);
        if let Some(f) = got
            .iter()
            .find(|f| f.start.abs_diff(lead as u64) <= opts.sps as u64)
        {
            found += 1;
            crc_ok += f.crc_ok as u64;
        }
    }
    Ok(FoundPoint {
        modulation,
        frame: opts.frame,
        code: opts.code,
        ebn0_db,
        frames,
        found,
        crc_ok,
        report,
        rx_seconds,
    })
}

/// Settings for [`run_sir`]: the wanted signal and one interferer through a radio's front end (B8).
/// Frequencies in Hz; the defaults are the live link's (1 MS/s, 250 kbaud, IF 175 kHz, frame v1,
/// 54-byte payload, coarse search +-0.1 cycles per symbol, coarse step 4).
#[derive(Debug, Clone, PartialEq)]
pub struct SirOptions {
    pub modulation: Modulation,
    pub sps: usize,
    pub symbol_rate: f64,
    /// The signal sits at `+if_hz` in the radio's band, as on the bench.
    pub if_hz: f64,
    pub code: PayloadCode,
    pub payload_len: usize,
    /// Receiver settings (`expected_len` is ignored: the receiver reads the header).
    pub rx: RxConfig,
    /// The interferer, in Hz: offsets from the signal's centre, widths, sweep spans; a sweep's
    /// period is in seconds (`Interferer::Sweep { period }` taken as seconds here).
    pub interferer: crate::interference::Interferer,
    /// ADC rate over the modem's rate (0: the smallest power of two that holds the interferer).
    pub ratio: usize,
    /// Signal power below ADC full scale, dB (full scale: I and Q at +-1, so a full-scale carrier is
    /// 0 dBFS). `None`: no clipping.
    pub headroom_db: Option<f64>,
    /// Opt-in receiver improvement: cancel the strongest carrier in each buffer before the
    /// receiver, when it stands this many dB above the mean FFT bin (see
    /// [`crate::interference::cancel_tone`]). `None`: off.
    pub notch_db: Option<f64>,
    /// Opt-in receiver improvement: a channel filter before the receiver
    /// ([`crate::interference::ChannelFilter::for_modem`]).
    pub channel_filter: bool,
}

impl Default for SirOptions {
    fn default() -> Self {
        Self {
            modulation: Modulation::Qpsk,
            sps: 4,
            symbol_rate: 250e3,
            if_hz: 175e3,
            code: PayloadCode::None,
            payload_len: 54,
            rx: RxConfig {
                cfo_search: 0.1,
                coarse_step: 4,
                ..RxConfig::default()
            },
            interferer: crate::interference::Interferer::Tone { offset: 0.0 },
            ratio: 0,
            headroom_db: None,
            notch_db: None,
            channel_filter: false,
        }
    }
}

impl SirOptions {
    /// The ADC rate over the modem's rate: `ratio`, or the smallest power of two whose band (0.8 of
    /// it) holds the interferer at its place in the radio's band.
    pub fn ratio(&self) -> usize {
        if self.ratio > 0 {
            return self.ratio;
        }
        let fs = self.symbol_rate * self.sps as f64;
        let reach = self.radio_interferer(1.0).reach() * fs;
        let mut r = 1usize;
        while reach > 0.4 * fs * r as f64 && r < 1024 {
            r *= 2;
        }
        r
    }

    /// The interferer in cycles per sample at `k` times the modem's rate, at its place in the
    /// radio's band (signal at `+if_hz`).
    fn radio_interferer(&self, k: f64) -> crate::interference::Interferer {
        use crate::interference::Interferer;
        let fs = self.symbol_rate * self.sps as f64 * k;
        match self.interferer {
            Interferer::Tone { offset } => Interferer::Tone {
                offset: (offset + self.if_hz) / fs,
            },
            Interferer::Noise { offset, width } => Interferer::Noise {
                offset: (offset + self.if_hz) / fs,
                width: width / fs,
            },
            Interferer::Sweep {
                offset,
                span,
                period,
            } => Interferer::Sweep {
                offset: (offset + self.if_hz) / fs,
                span: span / fs,
                period: period * fs,
            },
        }
    }
}

/// One point of [`run_sir`].
#[derive(Debug, Clone, PartialEq)]
pub struct SirPoint {
    pub ebn0_db: f64,
    /// Signal power over interferer power, both at the antenna (before the radio's filters).
    pub sir_db: f64,
    pub frames: u64,
    /// Frames delivered: CRC good and the payload sent.
    pub delivered: u64,
    /// Frames with any clipped sample.
    pub clipped_frames: u64,
    /// Buffers where the notch removed a carrier.
    pub notched: u64,
    pub ratio: usize,
}

impl SirPoint {
    pub fn loss(&self) -> f64 {
        1.0 - self.delivered as f64 / self.frames as f64
    }
}

/// Send `frames` frames through a radio front end with an interferer `sir_db` below the signal and
/// thermal noise at `ebn0_db`, and count the frames the receiver delivers. Per frame: modulate,
/// put the frame at `+if_hz` with a random phase and delay, raise it to the ADC rate, add the
/// interferer (a continuous stream: its phase and noise run on between frames), clip at full scale
/// (with `headroom_db`), filter and decimate to the modem's rate, add the thermal noise, shift back
/// by the IF, cancel a carrier (with `notch_db`) and run the receiver as on the air.
pub fn run_sir(
    ebn0_db: f64,
    sir_db: f64,
    frames: u64,
    seed: u64,
    opts: &SirOptions,
) -> Result<SirPoint, ConfigError> {
    use crate::interference::{cancel_tone, clip, FrontEnd, InterfererGen};
    let modem = Modem::new(ModemConfig {
        sps: opts.sps,
        frame: FrameFormat::V1,
        code: opts.code,
        ..ModemConfig::new(opts.modulation)
    })?;
    if opts.payload_len == 0 || opts.payload_len > 255 {
        return Err(ConfigError("payload_len must be 1 to 255".into()));
    }
    let r = opts.ratio();
    let fe = FrontEnd::new(r);
    let mixed = seed ^ ebn0_db.to_bits().rotate_left(17) ^ sir_db.to_bits().rotate_left(41);
    let mut ch = Channel::new(mixed);
    let n0 = n0_for_ebn0(ebn0_db, modem.energy_per_bit());
    let rx = RxConfig {
        expected_len: None,
        ..opts.rx.clone()
    };
    let if_cps = opts.if_hz / (opts.symbol_rate * opts.sps as f64);
    // the signal's mean power per sample over a frame: Es / sps
    let ps = modem.energy_per_symbol() / opts.sps as f64;
    let pi = ps / 10f64.powf(sir_db / 10.0);
    let mut gen = InterfererGen::new(opts.radio_interferer(r as f64), pi, mixed ^ 0x1A7E_5EED);
    // scale to the ADC: the signal at -headroom dBFS
    let adc = opts
        .headroom_db
        .map(|h| ((10f64.powf(-h / 10.0) / ps).sqrt() as f32, 1.0f32));
    const GUARD: usize = 32;
    let (mut delivered, mut clipped_frames, mut notched) = (0u64, 0u64, 0u64);
    for _ in 0..frames {
        let payload = ch.rng().bytes(opts.payload_len);
        let tx = modem.modulate(&payload)?;
        let lead = GUARD + 2 + ch.rng().below(62) as usize;
        let len = lead + tx.len() + 64 + GUARD;
        let ph0 = ch.rng().uniform() * 2.0 * PI;
        let mut x = vec![crate::iq::Complex32::ZERO; len];
        for (k, s) in tx.iter().enumerate() {
            let n = lead + k;
            let ph = ph0 + 2.0 * PI * if_cps * n as f64;
            x[n] = *s * crate::iq::Complex32::new(ph.cos() as f32, ph.sin() as f32);
        }
        let mut hi = fe.upsample(&x);
        gen.add(&mut hi);
        if let Some((g, fs)) = adc {
            for v in hi.iter_mut() {
                *v = *v * g;
            }
            if clip(&mut hi, fs) > 0 {
                clipped_frames += 1;
            }
            for v in hi.iter_mut() {
                *v = *v * (1.0 / g);
            }
        }
        let lo = fe.downsample(&hi);
        let mut y = lo[GUARD..len - GUARD].to_vec();
        ch.add_awgn(&mut y, n0);
        for (k, v) in y.iter_mut().enumerate() {
            let ph = -2.0 * PI * if_cps * (k + GUARD) as f64;
            *v = *v * crate::iq::Complex32::new(ph.cos() as f32, ph.sin() as f32);
        }
        // as the radio does it: channel filter, then the notch
        if opts.channel_filter {
            // a fresh filter per frame: its delay only moves the frame, which is found by its payload
            crate::interference::ChannelFilter::for_modem(opts.sps, 0.35).run(&mut y);
        }
        if let Some(db) = opts.notch_db {
            notched += cancel_tone(&mut y, db).is_some() as u64;
        }
        let (got, _) = modem.demodulate_report(&y, &rx);
        if got.iter().any(|f| f.crc_ok && f.payload == payload) {
            delivered += 1;
        }
    }
    Ok(SirPoint {
        ebn0_db,
        sir_db,
        frames,
        delivered,
        clipped_frames,
        notched,
        ratio: r,
    })
}

#[cfg(test)]
mod sir_tests {
    use super::*;
    use crate::interference::Interferer;

    fn loss(sir: f64, opts: &SirOptions) -> f64 {
        run_sir(20.0, sir, 60, 1, opts).unwrap().loss()
    }

    #[test]
    fn interference_costs_frames_and_the_options_win_them_back() {
        let tone = |offset: f64| SirOptions {
            interferer: Interferer::Tone { offset },
            ..SirOptions::default()
        };
        // a weak carrier costs nothing; one as strong as the signal on its centre costs everything
        assert_eq!(loss(30.0, &tone(0.0)), 0.0);
        assert_eq!(loss(0.0, &tone(0.0)), 1.0);
        // ... unless the notch takes it out
        let notch = SirOptions {
            notch_db: Some(25.0),
            ..tone(0.0)
        };
        assert_eq!(loss(-10.0, &notch), 0.0);
        // one channel below, 20 dB over the signal: the detector loses it; the channel filter saves it
        assert!(loss(-20.0, &tone(-420e3)) > 0.5);
        let filtered = SirOptions {
            channel_filter: true,
            ..tone(-420e3)
        };
        assert_eq!(loss(-20.0, &filtered), 0.0);
        // 5 MHz away, 40 dB over: filtered out, until it clips the ADC
        let far = SirOptions {
            interferer: Interferer::Noise {
                offset: 5e6,
                width: 420e3,
            },
            ..SirOptions::default()
        };
        assert_eq!(far.ratio(), 16);
        assert_eq!(loss(-40.0, &far), 0.0);
        let clipped = SirOptions {
            headroom_db: Some(20.0),
            ..far
        };
        let p = run_sir(20.0, -40.0, 30, 1, &clipped).unwrap();
        assert_eq!((p.loss(), p.clipped_frames), (1.0, 30));
    }
}
