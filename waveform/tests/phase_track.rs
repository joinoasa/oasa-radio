//! B12 and B13: the receiver's channel tracker (`RxConfig::phase_track`) against the single phase
//! line, through the B9 fading models. See `docs/phase-tracking.md`.
//!
//! The quick tests run in `cargo test`. The tables in the doc come from the ignored ones:
//!
//! ```sh
//! cargo test --release -p oasa-waveform --test phase_track fading_table -- --ignored --nocapture
//! cargo test --release -p oasa-waveform --test phase_track frame_length_table -- --ignored --nocapture
//! cargo test --release -p oasa-waveform --test phase_track awgn_table -- --ignored --nocapture
//! taskset -c 0 cargo test --release -p oasa-waveform --test phase_track cpu_cost -- --ignored --nocapture
//! ```

use oasa_waveform::channel::{n0_for_ebn0, Channel, Impairments};
use oasa_waveform::fading::{Fader, Fading, RayleighProcess};
use oasa_waveform::frame::PayloadCode;
use oasa_waveform::rng::Rng;
use oasa_waveform::sim::{run_found, BerOptions};
use oasa_waveform::stream::LiveReceiver;
use oasa_waveform::{Complex32, FrameFormat, Modem, ModemConfig, Modulation, RxConfig};
use std::time::Instant;

/// The live link's settings (B9): 250 kbaud at 4 samples per symbol, frames every 10 ms.
const BAUD: f64 = 250_000.0;
const SPS: usize = 4;
/// A 32-byte oasa-link message on the air: 54 payload bytes.
const MESSAGE_PAYLOAD: usize = 54;

/// Tracker setting for 200 Hz at 250 kbaud, cycles per symbol.
fn track_200() -> f32 {
    (200.0 / BAUD) as f32
}

fn opts(code: PayloadCode, payload_len: usize, fading_hz: f64, track: f32) -> BerOptions {
    let fs = BAUD * SPS as f64;
    BerOptions {
        sps: SPS,
        frame: FrameFormat::V1,
        code,
        payload_len,
        rx: RxConfig {
            cfo_search: 0.1,
            phase_track: track,
            ..RxConfig::default()
        },
        fading: if fading_hz > 0.0 {
            Fading::Rayleigh {
                doppler: fading_hz / fs,
            }
        } else {
            Fading::None
        },
        frame_interval: 0.01 * fs,
        ..BerOptions::default()
    }
}

/// Frames lost of `frames`: (lost, of them not found at all).
fn lost(code: PayloadCode, len: usize, fd: f64, track: f32, ebn0: f64, frames: u64) -> (u64, u64) {
    let p = run_found(
        Modulation::Qpsk,
        ebn0,
        frames,
        1,
        &opts(code, len, fd, track),
    )
    .unwrap();
    (p.frames - p.crc_ok, p.frames - p.found)
}

#[test]
fn default_is_off() {
    assert_eq!(RxConfig::default().phase_track, 0.0);
}

/// The B12 case: Rayleigh at 200 Hz, 30 dB mean Eb/N0, the light code. The line loses about half
/// the frames; the tracker under 1 %.
#[test]
fn tracker_keeps_200_hz_fading_frames() {
    let frames = 600;
    let (line, _) = lost(
        PayloadCode::Light,
        MESSAGE_PAYLOAD,
        200.0,
        0.0,
        30.0,
        frames,
    );
    let (tracked, _) = lost(
        PayloadCode::Light,
        MESSAGE_PAYLOAD,
        200.0,
        track_200(),
        30.0,
        frames,
    );
    assert!(line > frames / 5, "line: {line} of {frames} lost");
    assert!(
        tracked <= frames / 100,
        "tracker: {tracked} of {frames} lost"
    );
}

/// In white noise at the 1 % loss point the tracker loses no more than the line (same frames,
/// same noise).
#[test]
fn tracker_costs_nothing_in_white_noise() {
    let frames = 2000;
    let (line, _) = lost(PayloadCode::None, MESSAGE_PAYLOAD, 0.0, 0.0, 9.25, frames);
    let (tracked, _) = lost(
        PayloadCode::None,
        MESSAGE_PAYLOAD,
        0.0,
        track_200(),
        9.25,
        frames,
    );
    assert!(line > 5, "the test point is meant to lose frames: {line}");
    assert!(
        tracked <= line + line / 4 + 2,
        "tracker {tracked}, line {line} of {frames}"
    );
}

/// Every length, both PSK modulations, both body codes, with the tracker on: random phase, delay
/// and a small carrier offset at 20 dB. Nothing is lost (the header path with tracking included).
#[test]
fn tracked_frames_round_trip() {
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
                phase_track: track_200(),
                ..RxConfig::default()
            };
            let mut ch = Channel::new(21 + code.code() as u64 + 10 * m as u64);
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

/// Packet loss vs mean Eb/N0, Rayleigh 5, 50 and 200 Hz, uncoded and light, line and tracker.
/// 10,000 frames a point, seed 1 (the B9 settings). About 6 minutes on 8 CPUs.
#[test]
#[ignore]
fn fading_table() {
    let points = [10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0];
    let mut rows = Vec::new();
    for fd in [5.0, 50.0, 200.0] {
        for code in [PayloadCode::None, PayloadCode::Light] {
            for track in [0.0, track_200()] {
                rows.push((fd, code, track));
            }
        }
    }
    let results = parallel(rows.len() * points.len(), |i| {
        let (fd, code, track) = rows[i / points.len()];
        lost(
            code,
            MESSAGE_PAYLOAD,
            fd,
            track,
            points[i % points.len()],
            10_000,
        )
        .0
    });
    print!("| Channel | Code | Receiver |");
    for p in points {
        print!(" {p} dB |");
    }
    println!();
    for (r, &(fd, code, track)) in rows.iter().enumerate() {
        print!(
            "| Rayleigh {fd} Hz | {} | {} |",
            code.name(),
            if track > 0.0 { "tracker" } else { "line" }
        );
        for j in 0..points.len() {
            print!(" {} |", pct(results[r * points.len() + j], 10_000));
        }
        println!();
    }
}

/// White noise: the 1 % loss point with the line and with the tracker, uncoded and light.
/// 10,000 frames a point.
#[test]
#[ignore]
fn awgn_table() {
    let rows = [
        (
            PayloadCode::None,
            0.0,
            [8.5, 8.75, 9.0, 9.25, 9.5, 9.75, 10.0],
        ),
        (
            PayloadCode::None,
            track_200(),
            [8.5, 8.75, 9.0, 9.25, 9.5, 9.75, 10.0],
        ),
        (
            PayloadCode::Light,
            0.0,
            [3.0, 3.25, 3.5, 3.75, 4.0, 4.25, 4.5],
        ),
        (
            PayloadCode::Light,
            track_200(),
            [3.0, 3.25, 3.5, 3.75, 4.0, 4.25, 4.5],
        ),
    ];
    let n = rows[0].2.len();
    let results = parallel(rows.len() * n, |i| {
        let (code, track, points) = rows[i / n];
        lost(code, MESSAGE_PAYLOAD, 0.0, track, points[i % n], 10_000).0
    });
    for (r, (code, track, points)) in rows.iter().enumerate() {
        print!(
            "| AWGN | {} | {} |",
            code.name(),
            if *track > 0.0 { "tracker" } else { "line" }
        );
        for j in 0..n {
            print!(" {}: {} |", points[j], pct(results[r * n + j], 10_000));
        }
        let at = |j: usize| results[r * n + j] as f64 / 10_000.0;
        // log-linear interpolation of the 1 % crossing
        let one = (0..n - 1)
            .find(|&j| at(j) >= 0.01 && at(j + 1) < 0.01)
            .map(|j| {
                let (a, b) = (at(j).log10(), at(j + 1).max(1e-5).log10());
                points[j] + (points[j + 1] - points[j]) * (a + 2.0) / (a - b)
            });
        println!(" 1 % at {:.2} dB |", one.unwrap_or(f64::NAN));
    }
}

/// B13: how the frame length changes the fading loss. Uncoded v1 QPSK frames of 0 to 54 payload
/// bytes (54: a 32-byte message), Rayleigh 50 and 200 Hz, 30 and 45 dB, line and tracker,
/// 10,000 frames a point; and how often the channel's phase leaves the best line through the frame
/// by more than 45 degrees (QPSK's margin) where the signal is within 10 dB of its mean.
#[test]
#[ignore]
fn frame_length_table() {
    let lens = [0usize, 16, 32, 54];
    let mut rows = Vec::new();
    for fd in [50.0, 200.0] {
        for ebn0 in [30.0, 45.0] {
            for track in [0.0, track_200()] {
                rows.push((fd, ebn0, track));
            }
        }
    }
    let results = parallel(rows.len() * lens.len(), |i| {
        let (fd, ebn0, track) = rows[i / lens.len()];
        lost(
            PayloadCode::None,
            lens[i % lens.len()],
            fd,
            track,
            ebn0,
            10_000,
        )
        .0
    });
    let md = Modem::new(ModemConfig {
        sps: SPS,
        frame: FrameFormat::V1,
        ..ModemConfig::new(Modulation::Qpsk)
    })
    .unwrap();
    print!("| | |");
    for l in lens {
        let s = md.frame_symbols(l);
        print!(" {l} B ({s} sym, {:.2} ms) |", s as f64 / BAUD * 1e3);
    }
    println!();
    for fd in [50.0, 200.0] {
        print!("| {fd} Hz | phase off the line > 45 deg |");
        for l in lens {
            print!(
                " {} |",
                pct(off_the_line(fd, md.frame_symbols(l), 10_000), 10_000)
            );
        }
        println!();
    }
    for (r, &(fd, ebn0, track)) in rows.iter().enumerate() {
        print!(
            "| {fd} Hz, {ebn0} dB | {} |",
            if track > 0.0 { "tracker" } else { "line" }
        );
        for j in 0..lens.len() {
            print!(" {} |", pct(results[r * lens.len() + j], 10_000));
        }
        println!();
    }
}

/// Of `frames` frames of `nsym` symbols, 10 ms apart, through Rayleigh at `fd` Hz: how many have
/// the channel's phase more than 45 degrees off its best (power-weighted) line somewhere the
/// power is above -10 dB of the mean.
fn off_the_line(fd: f64, nsym: usize, frames: usize) -> u64 {
    let mut rng = Rng::new(7);
    let p = RayleighProcess::new(fd / BAUD, &mut rng);
    let mut count = 0;
    for f in 0..frames {
        let t0 = f as f64 * 0.01 * BAUD;
        let h: Vec<Complex32> = (0..nsym).map(|k| p.at(t0 + k as f64)).collect();
        let mut ph = Vec::with_capacity(nsym);
        let mut prev = 0.0f64;
        for (k, g) in h.iter().enumerate() {
            let a = g.arg() as f64;
            let u = if k == 0 {
                a
            } else {
                prev + (a - prev + std::f64::consts::PI).rem_euclid(2.0 * std::f64::consts::PI)
                    - std::f64::consts::PI
            };
            ph.push(u);
            prev = u;
        }
        let (mut sw, mut sx, mut sy, mut sxx, mut sxy) = (0.0, 0.0, 0.0, 0.0, 0.0);
        for (k, &u) in ph.iter().enumerate() {
            let (w, x) = (h[k].norm_sqr() as f64, k as f64);
            sw += w;
            sx += w * x;
            sy += w * u;
            sxx += w * x * x;
            sxy += w * x * u;
        }
        let b = (sw * sxy - sx * sy) / (sw * sxx - sx * sx);
        let a = (sy - b * sx) / sw;
        count += ph
            .iter()
            .enumerate()
            .any(|(k, &u)| h[k].norm_sqr() > 0.1 && (u - a - b * k as f64).abs() > 0.785)
            as u64;
    }
    count
}

/// Step 1 of B12: where the fading loss comes from with the line. The same frames and noise through
/// Rayleigh 50 and 200 Hz at 30 dB, with the channel's gain `h` changed to separate its parts:
/// as is; amplitude only (`|h|`: the phase taken off); phase only (`h / |h|`: no fades); and the
/// amplitude with the phase replaced by its best line over the frame (what the line could follow).
/// 10,000 frames each, uncoded and light, with the line. (Not with the tracker: `|h|` alone and
/// `h / |h|` are not smooth where `h` passes zero, which is what the tracker relies on, and no real
/// channel does that. `loss_with` takes a tracker setting for checks of the real case.)
#[test]
#[ignore]
fn loss_sources_table() {
    let modes = ["as is", "amplitude only", "phase only", "phase as a line"];
    let mut rows = Vec::new();
    for fd in [50.0, 200.0] {
        for code in [PayloadCode::None, PayloadCode::Light] {
            rows.push((fd, code, 0.0));
        }
    }
    let results = parallel(rows.len() * modes.len(), |i| {
        let (fd, code, track) = rows[i / modes.len()];
        loss_with(fd, code, track, i % modes.len(), 30.0, 10_000)
    });
    println!(
        "| Rayleigh, 30 dB | Code | Receiver | {} |",
        modes.join(" | ")
    );
    for (r, &(fd, code, track)) in rows.iter().enumerate() {
        print!(
            "| {fd} Hz | {} | {} |",
            code.name(),
            if track > 0.0 { "tracker" } else { "line" }
        );
        for j in 0..modes.len() {
            let (lost, not_found) = results[r * modes.len() + j];
            print!(" {} ({not_found}) |", pct(lost, 10_000));
        }
        println!();
    }
}

/// Frames lost and not found for [`loss_sources_table`]; `mode` indexes its columns.
fn loss_with(
    fd: f64,
    code: PayloadCode,
    track: f32,
    mode: usize,
    ebn0: f64,
    frames: usize,
) -> (u64, u64) {
    let md = Modem::new(ModemConfig {
        sps: SPS,
        frame: FrameFormat::V1,
        code,
        ..ModemConfig::new(Modulation::Qpsk)
    })
    .unwrap();
    let rx = RxConfig {
        cfo_search: 0.1,
        phase_track: track,
        ..RxConfig::default()
    };
    let fs = BAUD * SPS as f64;
    let p = RayleighProcess::new(fd / fs, &mut Rng::new(7));
    let mut ch = Channel::new(99);
    let n0 = n0_for_ebn0(ebn0, md.energy_per_bit());
    let (mut lost, mut not_found) = (0, 0);
    for f in 0..frames {
        let payload = ch.rng().bytes(MESSAGE_PAYLOAD);
        let tx = md.modulate(&payload).unwrap();
        let h = p.fill(f as f64 * 0.01 * fs, tx.len());
        // the best power-weighted line through the unwrapped phase of h
        let (mut prev, mut ph) = (0.0f64, Vec::with_capacity(h.len()));
        for (k, g) in h.iter().enumerate() {
            let a = g.arg() as f64;
            let u = if k == 0 {
                a
            } else {
                prev + (a - prev + std::f64::consts::PI).rem_euclid(2.0 * std::f64::consts::PI)
                    - std::f64::consts::PI
            };
            ph.push(u);
            prev = u;
        }
        let (mut sw, mut sx, mut sy, mut sxx, mut sxy) = (0.0, 0.0, 0.0, 0.0, 0.0);
        for (k, &u) in ph.iter().enumerate() {
            let (w, x) = (h[k].norm_sqr() as f64, k as f64);
            sw += w;
            sx += w * x;
            sy += w * u;
            sxx += w * x * x;
            sxy += w * x * u;
        }
        let b = (sw * sxy - sx * sy) / (sw * sxx - sx * sx);
        let a = (sy - b * sx) / sw;
        let faded: Vec<Complex32> = tx
            .iter()
            .zip(&h)
            .enumerate()
            .map(|(k, (&s, &g))| {
                let amp = g.norm_sqr().sqrt();
                s * match mode {
                    0 => g,
                    1 => Complex32::new(amp, 0.0),
                    2 => g * (1.0 / amp.max(1e-9)),
                    _ => Complex32::from_polar(amp, (a + b * k as f64) as f32),
                }
            })
            .collect();
        let lead = 2 + ch.rng().below(62) as usize;
        let imp = Impairments {
            n0,
            phase_rad: ch.rng().uniform() * 2.0 * std::f64::consts::PI,
            lead,
            tail: 8,
            ..Default::default()
        };
        let got = md.demodulate_with(&ch.apply(&faded, &imp), &rx);
        match got
            .iter()
            .find(|f| f.start.abs_diff(lead as u64) <= SPS as u64)
        {
            Some(f) if f.crc_ok && f.payload == payload => {}
            Some(_) => lost += 1,
            None => {
                lost += 1;
                not_found += 1;
            }
        }
    }
    (lost, not_found)
}

/// CPU: the live receiver as the bridge runs it (1 MS/s, 250 kbaud, coarse step 4, frequency
/// search 0.03, the locked default) on one second of 54-byte frames every 10 ms, white noise or
/// Rayleigh 200 Hz at 30 dB, in passes of 4096 samples; and the decode of one frame alone. Run it pinned to one CPU
/// (`taskset -c 0`) so the detection's threads count in full.
#[test]
#[ignore]
fn cpu_cost() {
    for (code, fd) in [
        (PayloadCode::None, 0.0),
        (PayloadCode::None, 200.0),
        (PayloadCode::Light, 0.0),
        (PayloadCode::Light, 200.0),
    ] {
        let md = Modem::new(ModemConfig {
            sps: SPS,
            frame: FrameFormat::V1,
            code,
            ..ModemConfig::new(Modulation::Qpsk)
        })
        .unwrap();
        let fs = BAUD * SPS as f64;
        let model = if fd > 0.0 {
            Fading::Rayleigh { doppler: fd / fs }
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
            f.resize(per, Complex32::new(0.0, 0.0));
            tx.extend(f);
        }
        let imp = Impairments {
            n0: n0_for_ebn0(30.0, md.energy_per_bit()),
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
        for track in [0.0, track_200()] {
            let rx = RxConfig {
                cfo_search: 0.03,
                coarse_step: 4,
                max_len: Some(200),
                only_code: Some(code),
                phase_track: track,
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
                "{} {}, {}: {:.0} ms per second of signal, {:.0} us per 4096-sample pass, {good}/100 frames; one frame's buffer alone {:.0} us",
                code.name(),
                if fd > 0.0 { "Rayleigh 200 Hz" } else { "AWGN" },
                if track > 0.0 { "tracker" } else { "line" },
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
