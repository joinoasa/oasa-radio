//! The receiver checks a candidate's sync word before it derotates the rest of the candidate (see
//! `check_sync_first` in `modem.rs`). These tests decode the same samples in that order and in the
//! previous one (derotate everything, then check) and require the same output down to the bit, on
//! every receive path; and they measure the work saved with a sample counter, not a clock.

use crate::channel::Channel;
use crate::diversity::Combine;
use crate::frame::{FrameFormat, PayloadCode, DEFAULT_SYNC_WORD};
use crate::modem::work;
use crate::stream::LiveReceiver;
use crate::{Complex32, Modem, ModemConfig, Modulation, RxConfig, RxFrame, RxReport};

const SPS: usize = 4;

/// Everything a frame carries, with floats as bit patterns so "equal" means bit for bit.
fn key(f: &RxFrame) -> String {
    format!(
        "{} {} {:?} {} {} {:08x} {:08x} {:08x} {:08x} {:?}",
        f.start,
        f.end,
        f.payload,
        f.crc_ok,
        f.sync_errors,
        f.metric.to_bits(),
        f.timing_frac.to_bits(),
        f.cfo_cycles_per_symbol.to_bits(),
        f.phase_rad.to_bits(),
        f.header,
    )
}

fn keys(frames: &[RxFrame]) -> Vec<String> {
    frames.iter().map(key).collect()
}

/// Run `f` with the sync word checked first (`true`) or in the previous order (`false`); returns
/// its result and the samples it derotated.
fn with_order<T>(sync_first: bool, f: impl FnOnce() -> T) -> (T, u64) {
    work::set_sync_first(sync_first);
    work::take();
    let out = f();
    let n = work::take();
    work::set_sync_first(true);
    (out, n)
}

fn coarse(rx: &RxConfig) -> RxConfig {
    RxConfig {
        cfo_search: 0.1,
        ..rx.clone()
    }
}

fn live(md: &Modem, rx: &RxConfig, x: &[Complex32]) -> (Vec<RxFrame>, RxReport) {
    let mut s = LiveReceiver::new(md.clone(), coarse(rx)).unwrap();
    let mut out = Vec::new();
    for b in x.chunks(1500) {
        out.extend(s.push(b));
    }
    out.extend(s.finish());
    (out, *s.report())
}

/// The modem settings the cases rotate through.
fn modem_for(i: u64) -> Modem {
    let (modulation, frame, code) = match i % 4 {
        0 => (Modulation::Bpsk, FrameFormat::W0, PayloadCode::None),
        1 => (Modulation::Qpsk, FrameFormat::V1, PayloadCode::None),
        2 => (Modulation::Qpsk, FrameFormat::V1, PayloadCode::Light),
        _ => (Modulation::Fsk2, FrameFormat::V1, PayloadCode::None),
    };
    Modem::new(ModemConfig {
        sps: SPS,
        frame,
        code,
        ..ModemConfig::new(modulation)
    })
    .unwrap()
}

/// Add `s` at sample `at` with amplitude `amp` and a random phase.
fn add(x: &mut [Complex32], at: usize, s: &[Complex32], amp: f64, ch: &mut Channel) {
    let ph = Complex32::from_polar(
        amp as f32,
        (ch.rng().uniform() * std::f64::consts::TAU) as f32,
    );
    for (d, v) in x[at..at + s.len()].iter_mut().zip(s) {
        *d += *v * ph;
    }
}

/// A preamble followed by a sync word other than the receiver's and `extra` random symbols: energy
/// that the detector takes for a frame start until it reads the sync word.
fn other_start(md: &Modem, ch: &mut Channel, extra: usize) -> Vec<Complex32> {
    let mut word = ch.rng().below(1 << 16) as u16;
    // at least 6 of 16 bits away from the receiver's word, so the check rejects it
    while (word ^ DEFAULT_SYNC_WORD).count_ones() < 6 {
        word = ch.rng().below(1 << 16) as u16;
    }
    let other = Modem::new(ModemConfig {
        sync_word: word,
        ..md.config().clone()
    })
    .unwrap();
    let n_ref = md.reference_len() / SPS;
    let mut x = other.modulate(&ch.rng().bytes(255)).unwrap();
    x.truncate((n_ref + extra) * SPS);
    x
}

/// Case `i`: noise only, a clean frame, or a frame with preceding energy at -10 to +10 dB ending
/// 0 to 150 symbols before it. Es/N0 of the frame 3 to 15 dB. Returns the samples.
fn scene(md: &Modem, i: u64) -> Vec<Complex32> {
    let mut ch = Channel::new(0xe5_0000 + i);
    let kind = (i / 4) % 3;
    let esn0_db = 3.0 + ch.rng().uniform() * 12.0;
    let n0 = 10f64.powf(-esn0_db / 10.0);
    let len = [1usize, 20, 54, 120][ch.rng().below(4) as usize];
    let frame = md.modulate(&ch.rng().bytes(len)).unwrap();
    let lead = 300 + ch.rng().below(400) as usize;
    let before = if kind == 2 {
        let extra = ch.rng().below(200) as usize;
        Some(other_start(md, &mut ch, extra))
    } else {
        None
    };
    let gap = [0usize, 16, 50, 150][ch.rng().below(4) as usize] * SPS;
    let rel_db = [-10.0, 0.0, 6.0, 10.0][ch.rng().below(4) as usize];
    let start = lead + before.as_ref().map_or(0, |b| b.len() + gap);
    let total = start + frame.len() + 300 + ch.rng().below(600) as usize;
    let mut x = vec![Complex32::new(0.0, 0.0); total];
    if let Some(b) = &before {
        add(&mut x, lead, b, 10f64.powf(rel_db / 20.0), &mut ch);
    }
    if kind != 0 {
        add(&mut x, start, &frame, 1.0, &mut ch);
    }
    ch.add_awgn(&mut x, n0);
    x
}

/// Receiver settings for case `i`: default, a given length (BER measurement), or tracking.
fn rx_for(i: u64) -> RxConfig {
    match (i / 12) % 3 {
        0 => RxConfig::default(),
        1 => RxConfig {
            expected_len: Some(54),
            ..RxConfig::default()
        },
        _ => RxConfig {
            phase_track: 0.001,
            ..RxConfig::default()
        },
    }
}

/// Every path, both orders, 240 seeded cases: the same frames (bit for bit) and the same counts.
#[test]
fn sync_first_gives_the_same_frames_and_counts() {
    const CASES: u64 = 360;
    let (mut frames, mut candidates, mut compared) = (0usize, 0usize, 0usize);
    for i in 0..CASES {
        let md = modem_for(i);
        let rx = rx_for(i);
        let x = scene(&md, i);
        let x2 = {
            let mut y = x.clone();
            Channel::new(0xd1_0000 + i).add_awgn(&mut y, 0.05);
            y
        };
        type Path<'a> = Box<dyn Fn() -> (Vec<RxFrame>, RxReport) + 'a>;
        let mut paths: Vec<(&str, Path)> = vec![
            ("fine", Box::new(|| md.demodulate_report(&x, &rx))),
            (
                "coarse",
                Box::new(|| md.demodulate_report(&x, &coarse(&rx))),
            ),
        ];
        if i % 3 == 0 {
            paths.push(("live", Box::new(|| live(&md, &rx, &x))));
        }
        if i % 5 == 0 {
            paths.push((
                "select",
                Box::new(|| md.demodulate_diversity(&[&x, &x2], &coarse(&rx), Combine::Select)),
            ));
            paths.push((
                "mrc",
                Box::new(|| md.demodulate_diversity(&[&x, &x2], &coarse(&rx), Combine::Mrc)),
            ));
        }
        for (name, run) in &paths {
            let ((old_f, old_r), _) = with_order(false, run);
            let ((new_f, new_r), _) = with_order(true, run);
            assert_eq!(keys(&old_f), keys(&new_f), "case {i} {name}: frames");
            assert_eq!(old_r, new_r, "case {i} {name}: counts");
            frames += new_f.len();
            candidates += new_r.preambles;
            compared += 1;
        }
    }
    // the cases exercise what they should: frames found and candidates rejected at the sync word
    assert!(frames > 100, "{frames} frames");
    assert!(candidates > frames, "{candidates} candidates");
    eprintln!("{compared} decodes compared over {CASES} cases: {frames} frames, {candidates} candidates, all identical");
}

/// Work per candidate rejected at the sync word, counted in derotated samples: a run of frame
/// starts with other sync words, coarse search, both orders.
#[test]
fn sync_first_cuts_work_per_rejected_candidate() {
    for i in 0..4u64 {
        let md = modem_for(i);
        let rx = coarse(&RxConfig::default());
        let mut ch = Channel::new(0x77_0000 + i);
        let mut x = vec![Complex32::new(0.0, 0.0); 200];
        for _ in 0..40 {
            let b = other_start(&md, &mut ch, 8);
            let at = x.len();
            x.resize(at + b.len() + 200 * SPS, Complex32::new(0.0, 0.0));
            add(&mut x, at, &b, 1.0, &mut ch);
        }
        // the longest frame after the last start, so every candidate has its samples
        x.resize(
            x.len() + md.longest_frame_samples(&rx),
            Complex32::new(0.0, 0.0),
        );
        ch.add_awgn(&mut x, 10f64.powf(-1.5));

        let ((old_f, old_r), old_n) = with_order(false, || md.demodulate_report(&x, &rx));
        let ((new_f, new_r), new_n) = with_order(true, || md.demodulate_report(&x, &rx));
        assert_eq!(keys(&old_f), keys(&new_f));
        assert_eq!(old_r, new_r);
        assert!(new_f.is_empty(), "case {i}: {} frames", new_f.len());
        assert_eq!(new_r.sync_rejected, 40, "case {i}: {new_r:?}");
        assert_eq!(new_r.preambles, 40, "case {i}: {new_r:?}");
        let per_old = old_n / 40;
        let per_new = new_n / 40;
        eprintln!(
            "{:?} {:?} {:?}: derotated samples per rejected candidate {per_old} -> {per_new} ({:.1}x less)",
            md.config().modulation,
            md.config().frame,
            md.config().code,
            per_old as f64 / per_new as f64
        );
        // the new order derotates the search window and the reference twice at most
        assert!(
            per_new <= 3 * md.reference_len() as u64,
            "case {i}: {per_new} per candidate"
        );
        assert!(per_old >= 4 * per_new, "case {i}: {per_old} -> {per_new}");
    }
}
