//! B15: the receiver's equalizer (`RxConfig::equalizer`) against strong echoes, through the B9
//! two-ray model. See `docs/equalizer.md`.
//!
//! The quick tests run in `cargo test`. The doc's tables come from the ignored ones:
//!
//! ```sh
//! cargo test --release -p oasa-waveform --test equalizer echo_phases -- --ignored --nocapture
//! cargo test --release -p oasa-waveform --test equalizer echo_table -- --ignored --nocapture
//! taskset -c 0 cargo test --release -p oasa-waveform --test equalizer cpu_cost -- --ignored --nocapture
//! ```

use oasa_waveform::channel::{n0_for_ebn0, Channel, Impairments};
use oasa_waveform::fading::{Fader, Fading};
use oasa_waveform::frame::PayloadCode;
use oasa_waveform::sim::{run_found, BerOptions};
use oasa_waveform::stream::LiveReceiver;
use oasa_waveform::{Complex32, FrameFormat, Modem, ModemConfig, Modulation, RxConfig};
use std::time::Instant;

/// The live link's settings (B9): 250 kbaud at 4 samples per symbol (1 MS/s, so 1 us is one
/// sample), frames every 10 ms.
const BAUD: f64 = 250_000.0;
const SPS: usize = 4;
/// A 32-byte oasa-link message on the air: 54 payload bytes.
const MESSAGE_PAYLOAD: usize = 54;

fn opts(code: PayloadCode, delay_us: f64, gain_db: f64, eq: bool) -> BerOptions {
    let fs = BAUD * SPS as f64;
    BerOptions {
        sps: SPS,
        frame: FrameFormat::V1,
        code,
        payload_len: MESSAGE_PAYLOAD,
        rx: RxConfig {
            cfo_search: 0.1,
            equalizer: eq,
            ..RxConfig::default()
        },
        fading: if delay_us > 0.0 {
            Fading::TwoRay {
                delay: delay_us * 1e-6 * fs,
                gain_db,
                doppler: 0.0,
            }
        } else {
            Fading::None
        },
        frame_interval: 0.01 * fs,
        ..BerOptions::default()
    }
}

/// Frames lost of `frames`.
fn lost(code: PayloadCode, delay_us: f64, gain_db: f64, eq: bool, ebn0: f64, frames: u64) -> u64 {
    let p = run_found(
        Modulation::Qpsk,
        ebn0,
        frames,
        1,
        &opts(code, delay_us, gain_db, eq),
    )
    .unwrap();
    p.frames - p.crc_ok
}

#[test]
fn default_is_off() {
    assert!(!RxConfig::default().equalizer);
}

/// The B15 case: a -3 dB echo half a symbol and one symbol late at 20 dB. Without the equalizer
/// a third and nine tenths of the frames are lost; with it, none of these.
#[test]
fn equalizer_opens_strong_echoes() {
    let frames = 300;
    for delay in [2.0, 4.0] {
        let off = lost(PayloadCode::None, delay, -3.0, false, 20.0, frames);
        let on = lost(PayloadCode::None, delay, -3.0, true, 20.0, frames);
        assert!(off > frames / 10, "{delay} us, off: {off} of {frames} lost");
        assert!(
            on <= frames / 100,
            "{delay} us, equalizer: {on} of {frames} lost"
        );
    }
}

/// In white noise near the 1 % loss point the equalizer loses little more than the plain receiver
/// (same frames, same noise).
#[test]
fn equalizer_costs_little_in_white_noise() {
    let frames = 2000;
    let off = lost(PayloadCode::None, 0.0, 0.0, false, 9.25, frames);
    let on = lost(PayloadCode::None, 0.0, 0.0, true, 9.25, frames);
    assert!(off > 5, "the test point is meant to lose frames: {off}");
    assert!(on <= off * 2 + 4, "equalizer {on}, off {off} of {frames}");
}

/// Every length, both PSK modulations, both body codes, with the equalizer on: random phase, delay
/// and a small carrier offset at 20 dB, no echo. Nothing is lost.
#[test]
fn equalized_frames_round_trip() {
    for m in [Modulation::Bpsk, Modulation::Qpsk] {
        for code in [PayloadCode::None, PayloadCode::Light] {
            let md = Modem::new(ModemConfig {
                sps: SPS,
                frame: FrameFormat::V1,
                code,
                ..ModemConfig::new(m)
            })
            .unwrap();
            let rx = RxConfig {
                cfo_search: 0.1,
                equalizer: true,
                ..RxConfig::default()
            };
            let mut ch = Channel::new(31 + code.code() as u64 + 10 * m as u64);
            for len in (0..=255).step_by(17).chain([1, 2, 254, 255]) {
                let payload = ch.rng().bytes(len);
                let tx = md.modulate(&payload).unwrap();
                let imp = Impairments {
                    n0: n0_for_ebn0(20.0, md.energy_per_bit()),
                    phase_rad: ch.rng().uniform() * std::f64::consts::TAU,
                    frac_delay: ch.rng().uniform(),
                    cfo_cycles_per_sample: (ch.rng().uniform() - 0.5) * 0.004 / SPS as f64,
                    lead: 30 + ch.rng().below(50) as usize,
                    tail: 40,
                };
                let got = md.demodulate_with(&ch.apply(&tx, &imp), &rx);
                assert!(
                    got.iter().any(|f| f.crc_ok && f.payload == payload),
                    "{m:?} {code:?} len {len}"
                );
            }
        }
    }
}

/// The light code's soft values come from the equalizer's own error power: with an echo one
/// symbol late at 12 dB, light-coded frames get through.
#[test]
fn equalizer_feeds_the_light_code() {
    let frames = 300;
    let on = lost(PayloadCode::Light, 4.0, -3.0, true, 12.0, frames);
    assert!(
        on <= frames / 100,
        "light, equalizer: {on} of {frames} lost"
    );
}

/// Why B9 has floors: noise-free frames (60 dB) through a -3 dB echo half a symbol and one symbol
/// late, at 72 echo phases, without the equalizer. Prints, per phase, whether the frame decoded,
/// and where the receiver put its timing (whole samples from the main ray, fraction) and carrier
/// phase (degrees; the main ray's is 0).
#[test]
#[ignore]
fn echo_phases() {
    let md = Modem::new(ModemConfig {
        sps: SPS,
        frame: FrameFormat::V1,
        ..ModemConfig::new(Modulation::Qpsk)
    })
    .unwrap();
    let g = 10f64.powf(-3.0 / 20.0);
    let a = (1.0 / (1.0 + g * g)).sqrt() as f32;
    let b = (g / (1.0 + g * g).sqrt()) as f32;
    for d in [2usize, 4] {
        for eq in [false, true] {
            let rx = RxConfig {
                cfo_search: 0.1,
                equalizer: eq,
                ..RxConfig::default()
            };
            let mut ch = Channel::new(3);
            let (mut bad, n) = (0, 72);
            let mut line = String::new();
            for i in 0..n {
                let th = std::f64::consts::TAU * i as f64 / n as f64;
                let p = ch.rng().bytes(MESSAGE_PAYLOAD);
                let x = md.modulate(&p).unwrap();
                let r = Complex32::new(th.cos() as f32 * b, th.sin() as f32 * b);
                let mut y = vec![Complex32::ZERO; x.len() + d];
                for (k, s) in x.iter().enumerate() {
                    y[k] += *s * a;
                    y[k + d] += *s * r;
                }
                let imp = Impairments {
                    n0: n0_for_ebn0(60.0, md.energy_per_bit()),
                    lead: 40,
                    tail: 40,
                    ..Default::default()
                };
                let f = md.demodulate_with(&ch.apply(&y, &imp), &rx);
                let ok = f.iter().any(|f| f.crc_ok && f.payload == p);
                bad += !ok as u32;
                if let Some(f) = f.iter().min_by_key(|f| f.start.abs_diff(40)) {
                    line += &format!(
                        "{:3.0}{} {:+} {:+.2} {:+3.0} | ",
                        th.to_degrees(),
                        if ok { "  ok" } else { " BAD" },
                        f.start as i64 - 40,
                        f.timing_frac,
                        f.phase_rad.to_degrees()
                    );
                }
            }
            println!(
                "echo {d} samples, equalizer {}: {bad} of {n} lost\n{line}\n",
                if eq { "on" } else { "off" }
            );
        }
    }
}

/// Packet loss vs mean Eb/N0, -3 and -6 dB echoes 1, 2 and 4 us late, and white noise, with and
/// without the equalizer. 10,000 frames a point, seed 1 (the B9 settings).
#[test]
#[ignore]
fn echo_table() {
    let rows: [(f64, f64, &[f64]); 7] = [
        (0.0, 0.0, &[8.0, 8.5, 9.0, 9.5, 10.0, 10.5, 11.0]),
        (1.0, -3.0, &[10.0, 12.0, 14.0, 16.0, 18.0, 20.0, 24.0]),
        (2.0, -3.0, &[10.0, 12.0, 14.0, 16.0, 18.0, 20.0, 24.0]),
        (4.0, -3.0, &[10.0, 12.0, 14.0, 16.0, 18.0, 20.0, 24.0]),
        (1.0, -6.0, &[8.0, 10.0, 12.0, 14.0, 16.0, 18.0, 20.0]),
        (2.0, -6.0, &[8.0, 10.0, 12.0, 14.0, 16.0, 18.0, 20.0]),
        (4.0, -6.0, &[8.0, 10.0, 12.0, 14.0, 16.0, 18.0, 20.0]),
    ];
    let frames = 10_000;
    for (delay, gain, points) in rows {
        for eq in [false, true] {
            let cells = parallel(points.len(), |i| {
                lost(PayloadCode::None, delay, gain, eq, points[i], frames)
            });
            let name = if delay > 0.0 {
                format!("{delay} us {gain} dB")
            } else {
                "AWGN".into()
            };
            let row: Vec<String> = points
                .iter()
                .zip(&cells)
                .map(|(p, &l)| format!("{p} dB {}", pct(l, frames)))
                .collect();
            println!(
                "| {name} | {} | {} |",
                if eq { "equalizer" } else { "off" },
                row.join(" | ")
            );
        }
    }
}

/// CPU: the live receiver as the bridge runs it (1 MS/s, 250 kbaud, coarse step 4, frequency
/// search 0.03) on one second of 54-byte frames every 10 ms at 20 dB, white noise or a -3 dB echo
/// 4 us late, in passes of 4096 samples; and the decode of one frame alone. Run it pinned to one
/// CPU (`taskset -c 0`) so the detection's threads count in full.
#[test]
#[ignore]
fn cpu_cost() {
    let md = Modem::new(ModemConfig {
        sps: SPS,
        frame: FrameFormat::V1,
        ..ModemConfig::new(Modulation::Qpsk)
    })
    .unwrap();
    let fs = BAUD * SPS as f64;
    for delay in [0.0, 4.0] {
        let model = if delay > 0.0 {
            Fading::TwoRay {
                delay,
                gain_db: -3.0,
                doppler: 0.0,
            }
        } else {
            Fading::None
        };
        let mut fader = Fader::new(model, 0.01 * fs, 3);
        let mut ch = Channel::new(5);
        let per = (0.01 * fs) as usize;
        let mut tx = Vec::new();
        let mut one = Vec::new();
        for i in 0..100 {
            let p = ch.rng().bytes(MESSAGE_PAYLOAD);
            let mut f = fader.apply(&md.modulate(&p).unwrap());
            if i == 0 {
                one = f.clone();
            }
            f.resize(per, Complex32::ZERO);
            tx.extend(f);
        }
        let imp = Impairments {
            n0: n0_for_ebn0(20.0, md.energy_per_bit()),
            ..Default::default()
        };
        let y = ch.apply(&tx, &imp);
        let y1 = ch.apply(
            &one,
            &Impairments {
                lead: 64,
                tail: 64,
                ..imp
            },
        );
        for eq in [false, true] {
            let rx = RxConfig {
                cfo_search: 0.03,
                coarse_step: 4,
                max_len: Some(200),
                only_code: Some(PayloadCode::None),
                equalizer: eq,
                ..RxConfig::default()
            };
            let mut r = LiveReceiver::new(md.clone(), rx.clone()).unwrap();
            let mut good = 0;
            let t = Instant::now();
            let mut passes = 0;
            for b in y.chunks(4096) {
                good += r.push(b).iter().filter(|f| f.crc_ok).count();
                passes += 1;
            }
            let total = t.elapsed().as_secs_f64();
            let reps = 300;
            let t = Instant::now();
            for _ in 0..reps {
                std::hint::black_box(md.demodulate_with(&y1, &rx));
            }
            let one_frame = t.elapsed().as_secs_f64() / reps as f64;
            println!(
                "{}, equalizer {}: {:.0} ms per second of signal, {:.0} us per 4096-sample pass, {good}/100 frames; one frame's buffer alone {:.0} us",
                if delay > 0.0 { "echo 4 us -3 dB" } else { "AWGN" },
                if eq { "on" } else { "off" },
                total * 1e3,
                total / passes as f64 * 1e6,
                one_frame * 1e6
            );
        }
    }
}

fn pct(lost: u64, of: u64) -> String {
    let p = 100.0 * lost as f64 / of as f64;
    if lost == 0 {
        "0".into()
    } else if p < 1.0 {
        format!("{p:.2} %")
    } else {
        format!("{p:.1} %")
    }
}

/// `f(0..n)` on all CPUs, in order.
fn parallel<T: Send + Default + Clone>(n: usize, f: impl Fn(usize) -> T + Sync) -> Vec<T> {
    let cpus = std::thread::available_parallelism().map_or(1, |c| c.get());
    let next = std::sync::atomic::AtomicUsize::new(0);
    let out = std::sync::Mutex::new(vec![T::default(); n]);
    std::thread::scope(|s| {
        for _ in 0..cpus {
            s.spawn(|| loop {
                let i = next.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                if i >= n {
                    break;
                }
                let v = f(i);
                out.lock().unwrap()[i] = v;
            });
        }
    });
    out.into_inner().unwrap()
}
