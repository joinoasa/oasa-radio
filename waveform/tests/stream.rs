//! The stream receiver against the one-buffer receiver: fed the same samples in blocks of any size,
//! it must return the same frames (start, payload, CRC), each once, while holding a bounded window.

use oasa_waveform::channel::{n0_for_ebn0, Channel, Impairments};
use oasa_waveform::frame::PayloadCode;
use oasa_waveform::stream::StreamReceiver;
use oasa_waveform::{Complex32, FrameFormat, Modem, ModemConfig, Modulation, RxConfig, RxFrame};

fn modem(m: Modulation, frame: FrameFormat) -> Modem {
    Modem::new(ModemConfig {
        frame,
        ..ModemConfig::new(m)
    })
    .unwrap()
}

/// `n` frames with random payload lengths (or `len` if given), random gaps, phase and noise.
fn recording(
    md: &Modem,
    n: usize,
    len: Option<usize>,
    ebn0: f64,
    cfo: f64,
    seed: u64,
) -> Vec<Complex32> {
    let mut ch = Channel::new(seed);
    let mut tx = Vec::new();
    for _ in 0..n {
        let l = len.unwrap_or_else(|| 1 + ch.rng().below(120) as usize);
        let p = ch.rng().bytes(l);
        tx.extend(md.modulate(&p).unwrap());
        let gap = 50 + ch.rng().below(3000) as usize;
        tx.extend(std::iter::repeat_n(Complex32::new(0.0, 0.0), gap));
    }
    let imp = Impairments {
        n0: n0_for_ebn0(ebn0, md.energy_per_bit()),
        phase_rad: 0.7,
        cfo_cycles_per_sample: cfo / md.config().sps as f64,
        lead: 777,
        tail: 500,
        ..Default::default()
    };
    ch.apply(&tx, &imp)
}

fn key(f: &RxFrame) -> (u64, Vec<u8>, bool) {
    (f.start, f.payload.clone(), f.crc_ok)
}

/// Stream `x` in blocks drawn from `sizes` (cycled), return the frames and the largest window held.
fn stream(md: Modem, rx: &RxConfig, x: &[Complex32], sizes: &[usize]) -> (Vec<RxFrame>, usize) {
    let mut s = StreamReceiver::new(md, rx.clone());
    let (mut out, mut i, mut k, mut most) = (Vec::new(), 0usize, 0usize, 0usize);
    while i < x.len() {
        let n = sizes[k % sizes.len()].min(x.len() - i);
        out.extend(s.push(&x[i..i + n]));
        most = most.max(s.window_len());
        i += n;
        k += 1;
    }
    out.extend(s.finish());
    assert_eq!(s.samples_seen(), x.len() as u64);
    (out, most)
}

fn check(
    m: Modulation,
    frame: FrameFormat,
    rx: RxConfig,
    len: Option<usize>,
    ebn0: f64,
    cfo: f64,
    seed: u64,
) {
    check_cfg(
        ModemConfig {
            frame,
            ..ModemConfig::new(m)
        },
        rx,
        len,
        ebn0,
        cfo,
        seed,
    );
}

fn check_cfg(cfg: ModemConfig, rx: RxConfig, len: Option<usize>, ebn0: f64, cfo: f64, seed: u64) {
    let (m, frame) = (cfg.modulation, cfg.frame);
    let md = Modem::new(cfg.clone()).unwrap();
    let x = recording(&md, 30, len, ebn0, cfo, seed);
    let whole: Vec<_> = md.demodulate_with(&x, &rx).iter().map(key).collect();
    assert!(
        whole.len() >= 15,
        "{m:?} {frame:?}: only {} frames in the whole buffer",
        whole.len()
    );
    let longest = md.longest_frame_samples(&rx);
    for sizes in [
        vec![x.len()],
        vec![65_536],
        vec![4096],
        vec![1000, 7, 333, 20_000, 1],
        vec![511],
    ] {
        let (got, most) = stream(Modem::new(cfg.clone()).unwrap(), &rx, &x, &sizes);
        let got: Vec<_> = got.iter().map(key).collect();
        assert_eq!(
            got.len(),
            whole.len(),
            "{m:?} {frame:?} blocks {sizes:?}: frame count"
        );
        assert_eq!(got, whole, "{m:?} {frame:?} blocks {sizes:?}");
        let block = *sizes.iter().max().unwrap();
        if block < x.len() {
            assert!(
                most <= 2 * longest + 2 * md.reference_len() + 200 + block,
                "{m:?} {frame:?} blocks {sizes:?}: window grew to {most} (longest frame {longest})"
            );
        }
    }
}

#[test]
fn stream_matches_the_whole_buffer_plain_receiver() {
    for m in [Modulation::Bpsk, Modulation::Qpsk, Modulation::Fsk2] {
        check(m, FrameFormat::W0, RxConfig::default(), None, 12.0, 0.0, 1);
    }
}

#[test]
fn stream_matches_the_whole_buffer_with_the_coarse_search() {
    let rx = RxConfig {
        cfo_search: 0.1,
        ..RxConfig::default()
    };
    for m in [Modulation::Bpsk, Modulation::Qpsk] {
        check(m, FrameFormat::W0, rx.clone(), None, 10.0, 0.018, 2);
    }
}

#[test]
fn stream_matches_the_whole_buffer_v1_and_known_length() {
    check(
        Modulation::Qpsk,
        FrameFormat::V1,
        RxConfig::default(),
        None,
        6.0,
        0.0,
        3,
    );
    let rx = RxConfig {
        expected_len: Some(64),
        cfo_search: 0.1,
        ..RxConfig::default()
    };
    check(
        Modulation::Bpsk,
        FrameFormat::W0,
        rx,
        Some(64),
        3.0,
        0.01,
        4,
    );
}

#[test]
fn stream_matches_the_whole_buffer_light_code() {
    let light = |m| ModemConfig {
        frame: FrameFormat::V1,
        code: PayloadCode::Light,
        ..ModemConfig::new(m)
    };
    check_cfg(
        light(Modulation::Bpsk),
        RxConfig::default(),
        None,
        4.0,
        0.0,
        6,
    );
    let rx = RxConfig {
        cfo_search: 0.1,
        ..RxConfig::default()
    };
    check_cfg(light(Modulation::Qpsk), rx, None, 4.0, 0.01, 7);
}

#[test]
fn a_frame_cut_off_by_the_end_is_not_returned_and_nothing_twice() {
    let md = modem(Modulation::Bpsk, FrameFormat::W0);
    let x = recording(&md, 5, Some(40), 15.0, 0.0, 5);
    let whole = md.demodulate(&x);
    assert_eq!(whole.len(), 5);
    // stop in the middle of the last frame
    let cut = whole[4].start + (whole[4].end - whole[4].start) / 2;
    let (got, _) = stream(
        modem(Modulation::Bpsk, FrameFormat::W0),
        &RxConfig::default(),
        &x[..cut as usize],
        &[1500],
    );
    assert_eq!(
        got.iter().map(key).collect::<Vec<_>>(),
        whole[..4].iter().map(key).collect::<Vec<_>>()
    );
}

#[test]
fn noise_only_stream_gives_nothing_and_keeps_a_small_window() {
    let md = modem(Modulation::Qpsk, FrameFormat::W0);
    let mut ch = Channel::new(9);
    let x = ch.noise(400_000, 1.0);
    let (got, most) = stream(
        modem(Modulation::Qpsk, FrameFormat::W0),
        &RxConfig::default(),
        &x,
        &[8192],
    );
    assert!(got.is_empty());
    let longest = md.frame_samples(255);
    assert!(
        most <= longest + 2 * md.reference_len() + 200 + 8192,
        "window {most}"
    );
}

/// Many seeds, SNRs and search settings. Slow: `cargo test --release -p oasa-waveform --test stream -- --ignored`.
#[test]
#[ignore]
fn stream_matches_the_whole_buffer_many_seeds() {
    for seed in 10..22u64 {
        for (search, cfo) in [(0.0f32, 0.0), (0.1, 0.02), (2.05, 1.3)] {
            let rx = RxConfig {
                cfo_search: search,
                ..RxConfig::default()
            };
            let m = if seed % 2 == 0 {
                Modulation::Bpsk
            } else {
                Modulation::Qpsk
            };
            let ebn0 = [4.0, 7.0, 12.0][(seed % 3) as usize];
            check(m, FrameFormat::W0, rx, None, ebn0, cfo, seed);
        }
    }
}
