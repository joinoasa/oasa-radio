//! B14: two-antenna receive diversity (`oasa_waveform::diversity`) through the B9 fading models, at
//! the live link's settings. See `docs/diversity.md`; its tables come from `oasa-radio waveform per
//! --diversity ...` (10,000 frames a point), these tests are the quick checks.

use oasa_waveform::diversity::Combine;
use oasa_waveform::fading::Fading;
use oasa_waveform::sim::{run_found, BerOptions};
use oasa_waveform::{FrameFormat, Modulation, RxConfig};

const BAUD: f64 = 250_000.0;
const SPS: usize = 4;

fn opts(fading_hz: f64, interval_s: f64, div: Option<Combine>) -> BerOptions {
    let fs = BAUD * SPS as f64;
    BerOptions {
        sps: SPS,
        frame: FrameFormat::V1,
        payload_len: 54,
        rx: RxConfig {
            cfo_search: 0.1,
            ..RxConfig::default()
        },
        fading: if fading_hz > 0.0 {
            Fading::Rayleigh {
                doppler: fading_hz / fs,
            }
        } else {
            Fading::None
        },
        frame_interval: interval_s * fs,
        diversity: div,
        ..BerOptions::default()
    }
}

fn lost(fading_hz: f64, interval_s: f64, div: Option<Combine>, ebn0: f64, frames: u64) -> u64 {
    let p = run_found(
        Modulation::Qpsk,
        ebn0,
        frames,
        7,
        &opts(fading_hz, interval_s, div),
    )
    .unwrap();
    p.frames - p.crc_ok
}

#[test]
fn every_method_beats_one_antenna_in_slow_fading() {
    // Rayleigh 5 Hz at 15 dB mean, frames 50 ms apart (so 300 frames see many fades): one antenna
    // loses about 12 %, two about 2 %.
    let one = lost(5.0, 0.05, None, 15.0, 300);
    assert!(one >= 20, "one antenna lost {one}");
    for c in [Combine::Select, Combine::PreambleSnr, Combine::Mrc] {
        let two = lost(5.0, 0.05, Some(c), 15.0, 300);
        assert!(two * 3 <= one, "{c:?}: {two} lost, one antenna {one}");
    }
}

#[test]
fn mrc_gains_in_awgn_where_selection_cannot() {
    // No fading: both antennas see the same signal power. Combining adds the signals in phase (3 dB
    // more Eb/N0); selection only picks the luckier noise.
    let (one, sel, mrc) = (
        lost(0.0, 0.01, None, 7.0, 300),
        lost(0.0, 0.01, Some(Combine::Select), 7.0, 300),
        lost(0.0, 0.01, Some(Combine::Mrc), 7.0, 300),
    );
    assert!(one >= 10, "one antenna lost {one}");
    assert!(sel <= one, "select {sel}, one {one}");
    assert!(
        mrc * 5 <= one && mrc <= sel,
        "mrc {mrc}, select {sel}, one {one}"
    );
}
