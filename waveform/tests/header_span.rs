//! A v1 frame whose CRC-16 fails must not hide what follows its header. Its header gives a length,
//! but the header is not authenticated and a bad CRC-16 means the claimed body is not known to be
//! there: a burst of preamble, sync word and a header claiming 255 bytes, with no body, must not
//! make the receiver skip a real frame that starts inside the claimed span. Checked on every
//! receive path: the whole buffer (with and without the coarse search), the stream receiver, the
//! live receiver and diversity. Controls: the real frame alone, after the span, and a damaged
//! real frame (still not delivered, and its body gives no delivered frame).

use oasa_waveform::channel::Channel;
use oasa_waveform::diversity::Combine;
use oasa_waveform::frame::PayloadCode;
use oasa_waveform::stream::{LiveReceiver, StreamReceiver};
use oasa_waveform::{Complex32, FrameFormat, Modem, ModemConfig, Modulation, RxConfig, RxFrame};

const SPS: usize = 4;
/// Preamble, sync word and header symbols.
const HDR_SYMS: usize = 63 + 16 + 152;

fn modem() -> Modem {
    Modem::new(ModemConfig {
        sps: SPS,
        frame: FrameFormat::V1,
        ..ModemConfig::new(Modulation::Qpsk)
    })
    .unwrap()
}

fn coarse() -> RxConfig {
    RxConfig {
        cfo_search: 0.1,
        ..RxConfig::default()
    }
}

/// A header that claims 255 bytes, cut 8 symbols after the header (the pulse tail): no body.
fn header_only(md: &Modem, ch: &mut Channel) -> Vec<Complex32> {
    let mut x = md.modulate(&ch.rng().bytes(255)).unwrap();
    x.truncate((HDR_SYMS + 8) * SPS);
    x
}

#[derive(Clone, Copy)]
enum Case {
    /// The real frame alone.
    Alone,
    /// Header-only burst, the real frame 50 symbols after it (inside the claimed span).
    Inside,
    /// Header-only burst, the real frame 50 symbols after the claimed span.
    After,
}

/// Samples with the real frame at Es/N0 15 dB and the header-only burst `rel_db` below it, random
/// phases; the buffer runs past the claimed span as a live stream does. Returns the samples, the
/// real frame's start and its payload.
fn scene(md: &Modem, case: Case, rel_db: f64, seed: u64) -> (Vec<Complex32>, u64, Vec<u8>) {
    let mut ch = Channel::new(seed);
    let payload = ch.rng().bytes(54);
    let real = md.modulate(&payload).unwrap();
    let burst = header_only(md, &mut ch);
    let lead = 400;
    let span = md.frame_samples(255);
    let start = match case {
        Case::Alone | Case::Inside => lead + burst.len() + 50 * SPS,
        Case::After => lead + span + 50 * SPS,
    };
    let total = (start + real.len()).max(lead + span) + 2000;
    let mut x = vec![Complex32::new(0.0, 0.0); total];
    let mut add = |at: usize, s: &[Complex32], amp: f64, ch: &mut Channel| {
        let ph = Complex32::from_polar(
            amp as f32,
            (ch.rng().uniform() * std::f64::consts::TAU) as f32,
        );
        for (d, v) in x[at..at + s.len()].iter_mut().zip(s) {
            *d += *v * ph;
        }
    };
    if !matches!(case, Case::Alone) {
        add(lead, &burst, 10f64.powf(rel_db / 20.0), &mut ch);
    }
    add(start, &real, 1.0, &mut ch);
    // Es = 1 per symbol: N0 for Es/N0 15 dB
    ch.add_awgn(&mut x, 10f64.powf(-1.5));
    (x, start as u64, payload)
}

fn delivered(frames: &[RxFrame], start: u64, payload: &[u8]) -> bool {
    frames
        .iter()
        .any(|f| f.crc_ok && f.start.abs_diff(start) <= 8 && f.payload == payload)
}

fn stream(x: &[Complex32]) -> Vec<RxFrame> {
    let mut s = StreamReceiver::new(modem(), coarse());
    let mut out = Vec::new();
    for b in x.chunks(4096) {
        out.extend(s.push(b));
    }
    out.extend(s.finish());
    out
}

fn live(x: &[Complex32]) -> Vec<RxFrame> {
    live_with(modem(), x)
}

fn live_with(md: Modem, x: &[Complex32]) -> Vec<RxFrame> {
    let mut s = LiveReceiver::new(md, coarse()).unwrap();
    let mut out = Vec::new();
    for b in x.chunks(1000) {
        out.extend(s.push(b));
    }
    out.extend(s.finish());
    out
}

/// Delivered counts over `trials` scenes on every path: whole buffer (fine), whole buffer (coarse),
/// stream, live, diversity (select, two copies of the samples with their own noise).
fn run(case: Case, rel_db: f64, trials: u64) -> [u64; 5] {
    let md = modem();
    let mut got = [0u64; 5];
    for t in 0..trials {
        let (x, start, p) = scene(&md, case, rel_db, 1000 + t);
        let (x2, _, _) = scene(&md, case, rel_db, 1000 + t);
        let mut x2 = x2;
        Channel::new(5000 + t).add_awgn(&mut x2, 10f64.powf(-1.5));
        let paths = [
            md.demodulate_report(&x, &RxConfig::default()).0,
            md.demodulate_report(&x, &coarse()).0,
            stream(&x),
            live(&x),
            md.demodulate_diversity(&[&x, &x2], &coarse(), Combine::Select)
                .0,
        ];
        for (g, fr) in got.iter_mut().zip(&paths) {
            *g += delivered(fr, start, &p) as u64;
        }
    }
    got
}

#[test]
fn a_bad_crc_16_does_not_hide_a_frame_inside_its_claimed_span() {
    for rel_db in [0.0, -10.0] {
        let got = run(Case::Inside, rel_db, 10);
        assert_eq!(
            got, [10; 5],
            "burst at {rel_db} dB: delivered per path {got:?}"
        );
    }
}

#[test]
fn controls_alone_and_after_the_span() {
    assert_eq!(run(Case::Alone, 0.0, 10), [10; 5]);
    assert_eq!(run(Case::After, -10.0, 10), [10; 5]);
}

/// A real frame whose body is damaged is returned with a bad CRC-16, and the search through its
/// body (now tried, not skipped) delivers nothing false.
#[test]
fn a_damaged_frame_still_fails_and_its_body_gives_no_false_frame() {
    let md = modem();
    let mut ch = Channel::new(77);
    let body_from = HDR_SYMS * SPS;
    let mut bad_returned = 0;
    for t in 0..40 {
        let p = ch.rng().bytes(255);
        let mut x = vec![Complex32::new(0.0, 0.0); 600];
        let at = x.len();
        x.extend(md.modulate(&p).unwrap());
        let end = x.len();
        x.extend(vec![Complex32::new(0.0, 0.0); md.frame_samples(255) + 3000]);
        ch.add_awgn(&mut x, 10f64.powf(-1.5));
        // a strong burst over the middle of the body
        let mid = at + body_from + 200 * SPS;
        let mut hit = x[mid..mid + 100 * SPS].to_vec();
        ch.add_awgn(&mut hit, 4.0);
        x[mid..mid + 100 * SPS].copy_from_slice(&hit);
        for fr in [
            md.demodulate_report(&x, &RxConfig::default()).0,
            md.demodulate_report(&x, &coarse()).0,
            stream(&x),
            live(&x),
        ] {
            assert!(
                fr.iter().all(|f| !f.crc_ok),
                "trial {t}: a frame was delivered"
            );
            bad_returned += fr
                .iter()
                .filter(|f| f.start.abs_diff(at as u64) <= 8 && f.end <= end as u64 + 8)
                .count();
        }
    }
    // the damaged frame itself is still found (with its bad CRC-16) on every path
    assert_eq!(bad_returned, 160);
}

/// Headers sent one after another, each claiming 255 light-coded bytes (about ten times its own
/// length), cost a bounded amount of decoding: the frames decoded cover at most about twice the
/// stream (each header's own span is searched, within the budget), not ten times.
#[test]
fn a_run_of_headers_costs_a_bounded_number_of_decodes() {
    let md = Modem::new(ModemConfig {
        sps: SPS,
        frame: FrameFormat::V1,
        code: PayloadCode::Light,
        ..ModemConfig::new(Modulation::Qpsk)
    })
    .unwrap();
    let mut ch = Channel::new(31);
    let mut x = Vec::new();
    while x.len() < 120_000 {
        let mut h = md.modulate(&ch.rng().bytes(255)).unwrap();
        h.truncate((HDR_SYMS + 8) * SPS);
        x.extend(h);
    }
    ch.add_awgn(&mut x, 10f64.powf(-2.0));
    let longest = md.longest_frame_samples(&coarse()) as u64;
    let n = x.len() as u64;
    for (path, fr) in [
        ("whole buffer", md.demodulate_report(&x, &coarse()).0),
        ("live", live_with(md.clone(), &x)),
    ] {
        assert!(fr.iter().all(|f| !f.crc_ok));
        let decoded: u64 = fr.iter().map(|f| f.end - f.start).sum();
        // without the budget every header (about 125) would be decoded: ~10 times the stream
        assert!(fr.len() > 2, "{path}: {} frames", fr.len());
        assert!(
            decoded <= 2 * n + 6 * longest,
            "{path}: {} frames covering {decoded} samples of a {n}-sample stream",
            fr.len()
        );
    }
}
