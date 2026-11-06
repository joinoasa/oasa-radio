//! The live receiver against the one-buffer receiver with the coarse search: fed the same samples
//! in blocks of any size it returns the same frames (start, payload, CRC), each once; and each
//! frame comes out soon after its own last sample, not after room for the longest frame.

use oasa_waveform::channel::{n0_for_ebn0, Channel, Impairments};
use oasa_waveform::frame::PayloadCode;
use oasa_waveform::stream::LiveReceiver;
use oasa_waveform::{Complex32, FrameFormat, Modem, ModemConfig, Modulation, RxConfig, RxFrame};

fn recording(
    md: &Modem,
    n: usize,
    max_len: usize,
    ebn0: f64,
    cfo: f64,
    seed: u64,
) -> Vec<Complex32> {
    let mut ch = Channel::new(seed);
    let mut tx = Vec::new();
    for _ in 0..n {
        let l = 1 + ch.rng().below(max_len as u64) as usize;
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

fn key(f: &RxFrame) -> (usize, Vec<u8>, bool) {
    (f.start, f.payload.clone(), f.crc_ok)
}

/// Stream in blocks drawn from `sizes`; returns the frames, the most samples any frame waited
/// after its end, and the largest window held.
fn live(md: Modem, rx: &RxConfig, x: &[Complex32], sizes: &[usize]) -> (Vec<RxFrame>, u64, usize) {
    let mut s = LiveReceiver::new(md, rx.clone()).unwrap();
    let (mut out, mut i, mut k, mut wait, mut most) = (Vec::new(), 0usize, 0usize, 0u64, 0usize);
    while i < x.len() {
        let n = sizes[k % sizes.len()].min(x.len() - i);
        for f in s.push(&x[i..i + n]) {
            wait = wait.max(s.samples_seen() - f.end as u64);
            out.push(f);
        }
        most = most.max(s.window_len());
        i += n;
        k += 1;
    }
    out.extend(s.finish());
    (out, wait, most)
}

fn check(cfg: ModemConfig, rx: RxConfig, max_len: usize, ebn0: f64, cfo: f64, seed: u64) {
    let md = Modem::new(cfg.clone()).unwrap();
    let x = recording(&md, 30, max_len, ebn0, cfo, seed);
    let whole: Vec<_> = md.demodulate_with(&x, &rx).iter().map(key).collect();
    assert!(
        whole.len() >= 15,
        "{cfg:?}: only {} frames in the whole buffer",
        whole.len()
    );
    let longest = md.longest_frame_samples(&rx);
    let refl = md.reference_len() as u64;
    for sizes in [
        vec![x.len()],
        vec![4096],
        vec![256],
        vec![1000, 7, 333, 20_000, 1],
        vec![64],
    ] {
        let (got, wait, most) = live(Modem::new(cfg.clone()).unwrap(), &rx, &x, &sizes);
        let got: Vec<_> = got.iter().map(key).collect();
        assert_eq!(got, whole, "{cfg:?} blocks {sizes:?}");
        let block = *sizes.iter().max().unwrap() as u64;
        if (block as usize) < x.len() {
            // out once the frame and the fine search's reach after it are in
            let sps = md.config().sps as u64;
            assert!(
                wait <= block + refl + 4 * sps,
                "{cfg:?} blocks {sizes:?}: a frame waited {wait} samples after its end"
            );
            assert!(
                most <= longest + 2 * md.reference_len() + 200 + block as usize,
                "{cfg:?} blocks {sizes:?}: window grew to {most} (longest frame {longest})"
            );
        }
    }
}

fn v1(m: Modulation, code: PayloadCode) -> ModemConfig {
    ModemConfig {
        frame: FrameFormat::V1,
        code,
        ..ModemConfig::new(m)
    }
}

/// As the `sdr` radio runs it: v1, at most the MTU, one code.
fn radio_rx(mtu: usize, code: PayloadCode) -> RxConfig {
    RxConfig {
        cfo_search: 0.03,
        max_len: Some(mtu),
        only_code: Some(code),
        ..RxConfig::default()
    }
}

#[test]
fn live_matches_the_whole_buffer_as_the_radio_runs_it() {
    for (m, seed) in [(Modulation::Qpsk, 1), (Modulation::Bpsk, 2)] {
        check(
            v1(m, PayloadCode::None),
            radio_rx(200, PayloadCode::None),
            200,
            10.0,
            0.004,
            seed,
        );
    }
}

#[test]
fn live_matches_the_whole_buffer_light_code() {
    let c = PayloadCode::Light;
    check(
        v1(Modulation::Qpsk, c),
        radio_rx(200, c),
        200,
        6.0,
        -0.003,
        3,
    );
}

#[test]
fn live_matches_the_whole_buffer_wide_search_any_v1() {
    let rx = RxConfig {
        cfo_search: 0.1,
        ..RxConfig::default()
    };
    check(
        v1(Modulation::Qpsk, PayloadCode::None),
        rx,
        120,
        9.0,
        0.018,
        4,
    );
}

#[test]
fn live_matches_the_whole_buffer_w0() {
    let rx = RxConfig {
        cfo_search: 0.1,
        ..RxConfig::default()
    };
    check(ModemConfig::new(Modulation::Bpsk), rx, 120, 10.0, 0.01, 5);
}

#[test]
fn live_needs_the_coarse_search() {
    let md = Modem::new(ModemConfig::new(Modulation::Qpsk)).unwrap();
    assert!(LiveReceiver::new(md, RxConfig::default()).is_err());
}
