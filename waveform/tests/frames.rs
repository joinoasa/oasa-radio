//! Frame level behaviour of the modem through the channel: round trips with unknown offsets,
//! false alarms, damaged frames, wrong sync words, gain and frequency offset.

use oasa_waveform::channel::{n0_for_ebn0, Channel, Impairments};
use oasa_waveform::frame::{
    body_bits_v1, crc8, header_symbols, header_symbols_raw, FrameFormat, HeaderFields,
    HEADER_SYMBOLS, MAX_PAYLOAD,
};
use oasa_waveform::sim::{run_found, BerOptions};
use oasa_waveform::{Complex32, Modem, ModemConfig, Modulation, RxConfig, RxFrame, RxReport};
use std::f64::consts::PI;

const ALL: [Modulation; 3] = [Modulation::Fsk2, Modulation::Bpsk, Modulation::Qpsk];

fn modem(m: Modulation) -> Modem {
    Modem::new(ModemConfig::new(m)).unwrap()
}

#[test]
fn symbol_energy_is_one_in_every_modulation() {
    // The convention everything else rests on: the energy of the whole frame waveform is its
    // symbol count, so Es = 1 and Eb = 1/bits_per_symbol, whatever sps is.
    for m in ALL {
        for sps in [4usize, 8, 16] {
            let md = Modem::new(ModemConfig {
                sps,
                ..ModemConfig::new(m)
            })
            .unwrap();
            let payload: Vec<u8> = (0..100u32).map(|i| (i * 31 + 7) as u8).collect();
            let x = md.modulate(&payload).unwrap();
            assert_eq!(x.len(), md.frame_samples(100));
            let e: f64 = x.iter().map(|s| s.norm_sqr() as f64).sum();
            let nsym = (63 + 16) as f64 + (2 + 100 + 2) as f64 * 8.0 / m.bits_per_symbol() as f64;
            assert!(
                (e / nsym - 1.0).abs() < 0.02,
                "{m:?} sps {sps}: energy {e} for {nsym} symbols"
            );
        }
        assert_eq!(modem(m).energy_per_bit(), 1.0 / m.bits_per_symbol() as f64);
    }
}

#[test]
fn noiseless_round_trip_with_random_delay_and_phase() {
    for m in ALL {
        let md = modem(m);
        let mut ch = Channel::new(11);
        for i in 0..40 {
            let len = (ch.rng().below(60) + (i % 3) as u64) as usize; // includes 0
            let payload = ch.rng().bytes(len);
            let tx = md.modulate(&payload).unwrap();
            let lead = ch.rng().below(300) as usize;
            let imp = Impairments {
                phase_rad: ch.rng().uniform() * 2.0 * PI,
                lead,
                tail: 20,
                ..Default::default()
            };
            let y = ch.apply(&tx, &imp);
            let got = md.demodulate(&y);
            assert_eq!(got.len(), 1, "{m:?} frame {i}");
            assert!(
                got[0].crc_ok && got[0].payload == payload,
                "{m:?} frame {i}"
            );
            assert_eq!(got[0].start, lead, "{m:?} frame {i}: timing");
            assert_eq!(got[0].sync_errors, 0);
        }
    }
}

#[test]
fn round_trip_at_high_snr_with_random_offsets() {
    // 14 dB Eb/N0: a frame of 32 bytes should come through nearly always.
    for m in ALL {
        let md = modem(m);
        let mut ch = Channel::new(5);
        let n0 = n0_for_ebn0(14.0, md.energy_per_bit());
        let (mut ok, total) = (0, 60);
        for _ in 0..total {
            let payload = ch.rng().bytes(32);
            let tx = md.modulate(&payload).unwrap();
            let imp = Impairments {
                n0,
                phase_rad: ch.rng().uniform() * 2.0 * PI,
                lead: ch.rng().below(400) as usize,
                tail: 50,
                ..Default::default()
            };
            let y = ch.apply(&tx, &imp);
            ok += md
                .demodulate(&y)
                .iter()
                .any(|f| f.crc_ok && f.payload == payload) as usize;
        }
        assert!(ok >= total - 1, "{m:?}: {ok} of {total}");
    }
}

#[test]
fn frequency_offset_is_estimated_and_tracked() {
    // 0.002 cycles per symbol is 0.2 % of the symbol rate: over a 600-symbol frame the phase would
    // turn by 7.5 rad if nothing corrected it.
    for m in [Modulation::Bpsk, Modulation::Qpsk, Modulation::Fsk2] {
        let md = modem(m);
        let mut ch = Channel::new(9);
        for cfo in [0.002f64, -0.003] {
            // noncoherent FSK needs more Eb/N0 for a whole 64-byte frame to get through
            let ebn0 = if m == Modulation::Fsk2 { 18.0 } else { 12.0 };
            let n0 = n0_for_ebn0(ebn0, md.energy_per_bit());
            let mut ok = 0;
            for _ in 0..20 {
                let payload = ch.rng().bytes(64);
                let tx = md.modulate(&payload).unwrap();
                let imp = Impairments {
                    n0,
                    phase_rad: ch.rng().uniform() * 2.0 * PI,
                    cfo_cycles_per_sample: cfo / 8.0,
                    lead: 100,
                    tail: 50,
                    ..Default::default()
                };
                let y = ch.apply(&tx, &imp);
                if let Some(f) = md.demodulate(&y).first() {
                    assert!(
                        (f.cfo_cycles_per_symbol as f64 - cfo).abs() < 0.0015,
                        "{m:?}: estimated {} for {cfo}",
                        f.cfo_cycles_per_symbol
                    );
                    ok += (f.crc_ok && f.payload == payload) as usize;
                }
            }
            assert!(ok >= 19, "{m:?} cfo {cfo}: {ok} of 20");
        }
    }
}

#[test]
fn frequency_search_bins_widen_the_detection_range() {
    // 2.53 % of the symbol rate turns the signal exactly twice around during the 79 reference symbols,
    // which is a null of one coherent correlation. The frequency bins find it.
    let md = modem(Modulation::Bpsk);
    let mut ch = Channel::new(2);
    let payload = ch.rng().bytes(16);
    let tx = md.modulate(&payload).unwrap();
    let imp = Impairments {
        cfo_cycles_per_sample: 2.0 / 79.0 / 8.0,
        lead: 64,
        tail: 64,
        ..Default::default()
    };
    let y = ch.apply(&tx, &imp);
    let plain = md.demodulate(&y);
    assert!(
        plain.iter().all(|f| !(f.crc_ok && f.payload == payload)),
        "default receiver unexpectedly decoded at the correlation null"
    );
    let rx = RxConfig {
        cfo_bins: 6,
        cfo_step: 0.005,
        ..RxConfig::default()
    };
    let got = md.demodulate_with(&y, &rx);
    assert!(got.iter().any(|f| f.crc_ok && f.payload == payload));
}

#[test]
fn gain_does_not_matter() {
    for m in ALL {
        let md = modem(m);
        let payload = vec![0xA5u8; 20];
        let tx = md.modulate(&payload).unwrap();
        for g in [0.001f32, 1.0, 300.0] {
            let mut y = vec![Complex32::ZERO; 50];
            y.extend(tx.iter().map(|s| *s * g));
            y.extend(vec![Complex32::ZERO; 50]);
            let got = md.demodulate(&y);
            assert!(
                got.len() == 1 && got[0].crc_ok && got[0].payload == payload,
                "{m:?} gain {g}"
            );
        }
    }
}

#[test]
fn several_frames_in_one_buffer() {
    for m in ALL {
        let md = modem(m);
        let mut ch = Channel::new(4);
        let n0 = n0_for_ebn0(15.0, md.energy_per_bit());
        let payloads: Vec<Vec<u8>> = (0..4).map(|i| ch.rng().bytes(10 + 7 * i)).collect();
        let mut stream = Vec::new();
        for p in &payloads {
            stream.extend(vec![Complex32::ZERO; 120]);
            stream.extend(md.modulate(p).unwrap());
        }
        stream.extend(vec![Complex32::ZERO; 120]);
        ch.add_awgn(&mut stream, n0);
        let got: Vec<Vec<u8>> = md
            .demodulate(&stream)
            .into_iter()
            .filter(|f| f.crc_ok)
            .map(|f| f.payload)
            .collect();
        assert_eq!(got, payloads, "{m:?}");
    }
}

#[test]
fn noise_only_input_gives_no_frames() {
    // Threshold 0.04, M = 632 samples: false alarm probability per position (1 - 0.04)^631 = 6e-12.
    // 600 000 noise positions (3 modulations): the chance of even one is about 4e-6.
    for m in ALL {
        let md = modem(m);
        let mut ch = Channel::new(77);
        let y = ch.noise(200_000, 1.0);
        let got = md.demodulate(&y);
        assert!(got.is_empty(), "{m:?}: {} false frames", got.len());
    }
}

#[test]
fn detection_metric_follows_the_noise_theory() {
    // In white noise the normalized metric is Beta(1, M-1): P(m > t) = (1-t)^(M-1).
    // Count how often positions exceed a low threshold and compare with the formula.
    let md = modem(Modulation::Bpsk);
    let m_len = 79 * 8usize;
    let t = 0.008f64;
    let predicted = (1.0 - t).powi(m_len as i32 - 1);
    let mut ch = Channel::new(31);
    let y = ch.noise(2_000_000, 1.0);
    let metric = md.metric(&y, &RxConfig::default());
    let hits = metric.iter().filter(|&&v| v as f64 > t).count() as f64;
    let measured = hits / metric.len() as f64;
    // neighbouring positions are correlated, so hits come in clusters (about 3000 here): +-15 %
    assert!(
        measured > predicted * 0.85 && measured < predicted * 1.15,
        "measured {measured:e}, predicted {predicted:e}"
    );
}

#[test]
fn corrupted_frames_fail_the_crc() {
    for m in ALL {
        let md = modem(m);
        let mut ch = Channel::new(8);
        let payload = ch.rng().bytes(40);
        let tx = md.modulate(&payload).unwrap();
        let sps = 8;
        let n_ref = 79;
        // flip 3 neighbouring symbols deep in the payload: negate the samples (BPSK, QPSK: a
        // sign flip is a 180 degree turn; FSK: conjugate swaps the tones)
        let mut bad = tx.clone();
        let from = (n_ref + 60) * sps;
        for s in &mut bad[from..from + 3 * sps] {
            *s = if m == Modulation::Fsk2 {
                s.conj()
            } else {
                *s * -1.0
            };
        }
        let mut y = vec![Complex32::ZERO; 40];
        y.extend(bad);
        y.extend(vec![Complex32::ZERO; 40]);
        let got = md.demodulate(&y);
        assert_eq!(got.len(), 1, "{m:?}");
        assert!(!got[0].crc_ok, "{m:?}: damaged frame passed the CRC");
        assert_ne!(got[0].payload, payload);
    }
}

#[test]
fn damage_anywhere_in_the_body_is_caught() {
    // Negate three symbols' worth of samples around each body position in turn (a 180 degree
    // turn of the middle symbol at least): the CRC must fail every time. Skips the 16 length bits and their neighbours,
    // where damage loses the frame instead.
    let md = modem(Modulation::Bpsk);
    let payload = vec![0x5Au8; 12];
    let tx = md.modulate(&payload).unwrap();
    let body_syms = (2 + 12 + 2) * 8;
    for k in 18..body_syms {
        let mut bad = tx.clone();
        let centre = (79 + k) * 8 + 64; // symbol k's pulse peak (span 16 symbols: half of it, 8 symbols, is 64 samples)
        for s in &mut bad[centre - 12..centre + 12] {
            *s = *s * -1.0;
        }
        let mut y = vec![Complex32::ZERO; 30];
        y.extend(bad);
        y.extend(vec![Complex32::ZERO; 30]);
        let f = md.demodulate(&y);
        assert!(
            !f.is_empty() && f.iter().all(|f| !f.crc_ok),
            "damage at symbol {k} was not detected"
        );
    }
}

#[test]
fn a_different_sync_word_is_not_accepted() {
    let a = Modem::new(ModemConfig::new(Modulation::Bpsk)).unwrap();
    let b = Modem::new(ModemConfig {
        sync_word: !0x2DD4,
        ..ModemConfig::new(Modulation::Bpsk)
    })
    .unwrap();
    let payload = vec![1, 2, 3, 4];
    let mut y = vec![Complex32::ZERO; 64];
    y.extend(b.modulate(&payload).unwrap());
    y.extend(vec![Complex32::ZERO; 64]);
    assert!(
        a.demodulate(&y).is_empty(),
        "a foreign sync word got through"
    );
    assert!(b.demodulate(&y).iter().any(|f| f.crc_ok));
}

#[test]
fn no_preamble_no_frame() {
    // Cut the preamble and sync word off the front: the body alone must not be found.
    for m in ALL {
        let md = modem(m);
        let tx = md.modulate(&[9u8; 30]).unwrap();
        let body = &tx[(79 + 8) * 8..]; // drop preamble, sync and a few body symbols
        let mut y = vec![Complex32::ZERO; 100];
        y.extend_from_slice(body);
        y.extend(vec![Complex32::ZERO; 100]);
        assert!(
            md.demodulate(&y).iter().all(|f| !f.crc_ok),
            "{m:?}: a frame without a preamble was accepted"
        );
    }
}

#[test]
fn payload_limits() {
    let md = modem(Modulation::Qpsk);
    assert!(md.modulate(&vec![0u8; MAX_PAYLOAD]).is_ok());
    assert!(md.modulate(&vec![0u8; MAX_PAYLOAD + 1]).is_err());
    let y = {
        let mut y = vec![Complex32::ZERO; 10];
        y.extend(md.modulate(&vec![0xFFu8; MAX_PAYLOAD]).unwrap());
        y.extend(vec![Complex32::ZERO; 10]);
        y
    };
    let f = md.demodulate(&y);
    assert!(f.len() == 1 && f[0].crc_ok && f[0].payload.len() == MAX_PAYLOAD);
}

#[test]
fn truncated_frame_gives_nothing() {
    for m in ALL {
        let md = modem(m);
        let tx = md.modulate(&[3u8; 50]).unwrap();
        let y = &tx[..tx.len() - 200];
        assert!(md.demodulate(y).is_empty(), "{m:?}");
    }
}

#[test]
fn raw_bits_round_trip_at_known_timing() {
    let mut ch = Channel::new(1);
    for m in ALL {
        let md = modem(m);
        let bits: Vec<u8> = (0..400)
            .map(|_| (ch.rng().next_u64() >> 63) as u8)
            .collect();
        let x = md.modulate_bits(&bits);
        assert_eq!(md.demod_bits_known_timing(&x, bits.len()), bits, "{m:?}");
    }
}

#[test]
fn bad_configs_are_refused() {
    let bad = |f: &dyn Fn(&mut ModemConfig)| {
        let mut c = ModemConfig::default();
        f(&mut c);
        Modem::new(c).is_err()
    };
    assert!(bad(&|c| c.sps = 2));
    assert!(bad(&|c| c.rolloff = 0.0));
    assert!(bad(&|c| c.rrc_span = 7));
}

#[test]
fn sub_sample_delay_is_handled() {
    // The channel delays by a fraction of a sample on top of a whole number; the receiver finds the
    // frame to the nearest sample and takes the rest out with its fractional matched filter.
    for m in ALL {
        let md = modem(m);
        let mut ch = Channel::new(13);
        let n0 = n0_for_ebn0(
            if m == Modulation::Fsk2 { 16.0 } else { 12.0 },
            md.energy_per_bit(),
        );
        for &mu in &[0.0, 0.2, 0.5, 0.8] {
            let mut ok = 0;
            for _ in 0..20 {
                let payload = ch.rng().bytes(32);
                let tx = md.modulate(&payload).unwrap();
                let imp = Impairments {
                    n0,
                    frac_delay: mu,
                    phase_rad: ch.rng().uniform() * 2.0 * PI,
                    lead: 100,
                    tail: 100,
                    ..Default::default()
                };
                let y = ch.apply(&tx, &imp);
                ok += md
                    .demodulate(&y)
                    .iter()
                    .any(|f| f.crc_ok && f.payload == payload) as usize;
            }
            assert!(ok >= 19, "{m:?} delay {mu}: {ok} of 20");
        }
    }
}

/// Receiver settings with the coarse frequency search on, to two symbol rates and a little more.
fn wide() -> RxConfig {
    RxConfig {
        cfo_search: 2.05,
        ..RxConfig::default()
    }
}

#[test]
fn coarse_search_decodes_across_two_symbol_rates() {
    // Two Plutos with plain crystals can be about 46 kHz apart at 915 MHz: 1.8 symbol rates at
    // 25 kbaud. Every frame here must decode, start at the right sample, and give its offset to
    // within 0.2 % of the symbol rate. 0.0253 is the null of the plain correlation (two turns over
    // the 79 reference symbols).
    let offsets = [
        -2.0, -1.62, -1.0, -0.37, -0.0253, 0.0, 0.0253, 0.5, 0.999, 1.31, 1.77, 2.0,
    ];
    for m in ALL {
        let md = modem(m);
        let mut ch = Channel::new(12);
        let ebn0 = if m == Modulation::Fsk2 { 16.0 } else { 12.0 };
        let n0 = n0_for_ebn0(ebn0, md.energy_per_bit());
        for cfo in offsets {
            for _ in 0..3 {
                let payload = ch.rng().bytes(32);
                let tx = md.modulate(&payload).unwrap();
                let lead = 10 + ch.rng().below(190) as usize;
                let imp = Impairments {
                    n0,
                    phase_rad: ch.rng().uniform() * 2.0 * PI,
                    cfo_cycles_per_sample: cfo / 8.0,
                    frac_delay: ch.rng().uniform(),
                    lead,
                    tail: 40,
                };
                let y = ch.apply(&tx, &imp);
                let got = md.demodulate_with(&y, &wide());
                assert_eq!(got.len(), 1, "{m:?} cfo {cfo}: {} frames", got.len());
                let f = &got[0];
                assert!(
                    f.crc_ok && f.payload == payload,
                    "{m:?} cfo {cfo}: bad frame"
                );
                assert!(
                    f.start.abs_diff(lead) <= 1,
                    "{m:?} cfo {cfo}: start {} not {lead}",
                    f.start
                );
                let err = (f.cfo_cycles_per_symbol as f64 - cfo).abs();
                assert!(
                    err < 0.002,
                    "{m:?} cfo {cfo}: estimated {}",
                    f.cfo_cycles_per_symbol
                );
            }
        }
    }
}

#[test]
fn coarse_search_reports_the_true_carrier_phase() {
    // The phase is given at the frame start in the buffer as received, not after the receiver's
    // own frequency correction.
    let md = modem(Modulation::Bpsk);
    let payload = vec![0x3Cu8; 24];
    let tx = md.modulate(&payload).unwrap();
    for (cfo, phase) in [(1.37f64, 0.4f64), (-0.81, -2.5), (0.0, 1.0)] {
        let imp = Impairments {
            phase_rad: phase,
            cfo_cycles_per_sample: cfo / 8.0,
            lead: 133,
            tail: 20,
            ..Default::default()
        };
        let y = Channel::new(1).apply(&tx, &imp);
        let f = &md.demodulate_with(&y, &wide())[0];
        let d = (f.phase_rad as f64 - phase)
            .sin()
            .atan2((f.phase_rad as f64 - phase).cos());
        assert!(
            d.abs() < 0.05,
            "cfo {cfo}: phase {} not {phase}",
            f.phase_rad
        );
    }
}

#[test]
fn the_plain_receiver_misses_what_the_coarse_search_finds() {
    let md = modem(Modulation::Qpsk);
    let payload = vec![0x5Au8; 16];
    let tx = md.modulate(&payload).unwrap();
    let imp = Impairments {
        cfo_cycles_per_sample: 1.0 / 8.0,
        lead: 50,
        tail: 50,
        ..Default::default()
    };
    let y = Channel::new(3).apply(&tx, &imp);
    assert!(md.demodulate(&y).iter().all(|f| !f.crc_ok));
    let got = md.demodulate_with(&y, &wide());
    assert!(got.len() == 1 && got[0].crc_ok && got[0].payload == payload);
}

#[test]
fn coarse_search_separates_frames_with_their_own_offsets() {
    for m in ALL {
        let md = modem(m);
        let mut ch = Channel::new(8);
        let n0 = n0_for_ebn0(16.0, md.energy_per_bit());
        let offsets = [-1.7f64, 0.02, 0.9, 1.95];
        let payloads: Vec<Vec<u8>> = (0..4).map(|i| ch.rng().bytes(12 + 9 * i)).collect();
        let mut stream = Vec::new();
        for (p, cfo) in payloads.iter().zip(offsets) {
            stream.extend(vec![Complex32::ZERO; 150]);
            let tx = md.modulate(p).unwrap();
            let imp = Impairments {
                cfo_cycles_per_sample: cfo / 8.0,
                phase_rad: ch.rng().uniform() * 2.0 * PI,
                ..Default::default()
            };
            stream.extend(ch.apply(&tx, &imp));
        }
        stream.extend(vec![Complex32::ZERO; 150]);
        ch.add_awgn(&mut stream, n0);
        let got: Vec<Vec<u8>> = md
            .demodulate_with(&stream, &wide())
            .into_iter()
            .filter(|f| f.crc_ok)
            .map(|f| f.payload)
            .collect();
        assert_eq!(got, payloads, "{m:?}");
    }
}

#[test]
fn coarse_search_on_noise_only_gives_no_frames() {
    // The threshold is raised so that a position passes no more often than without the search:
    // about 6e-12. 600 000 noise samples (3 modulations): the chance of even one frame is about 4e-6.
    for m in ALL {
        let md = modem(m);
        let y = Channel::new(77).noise(200_000, 1.0);
        let got = md.demodulate_with(&y, &wide());
        assert!(got.is_empty(), "{m:?}: {} false frames", got.len());
    }
}

#[test]
fn coarse_metric_follows_its_noise_bound() {
    // Each bin is Beta(1, M-1) in noise, so a position passes t with probability at most about
    // cells * (1-t)^(M-1). That bound is what the threshold is built on. Check it at a low t, where
    // hits are countable, for a wide search (1-sample chunks) and a narrow one (chunked): it must
    // hold, and not be loose by more than a factor of 2 (or the threshold costs sensitivity).
    let md = modem(Modulation::Bpsk);
    let y = Channel::new(31).noise(400_000, 1.0);
    let t = 0.018f64;
    for search in [2.05f32, 0.5] {
        let rx = RxConfig {
            cfo_search: search,
            ..RxConfig::default()
        };
        let cs = md.coarse_search(&rx).unwrap();
        let cm = md.coarse_metric(&y, &rx);
        let bound = cs.cells * (1.0 - t).powi(79 * 8 - 1);
        let measured = cm.iter().filter(|v| v.0 as f64 > t).count() as f64 / cm.len() as f64;
        println!(
            "search {search}: chunk {} fft {} cells {} measured {measured:e} bound {bound:e}",
            cs.chunk, cs.fft_len, cs.cells
        );
        assert!(
            measured < bound * 1.15 && measured > bound * 0.5,
            "search {search}: measured {measured:e}, bound {bound:e}"
        );
        // and the threshold it sets keeps the per-position rate at the plain receiver's
        let plain = (1.0 - 0.04f64).powi(79 * 8 - 1);
        let ours = cs.cells * (1.0 - cs.threshold as f64).powi(79 * 8 - 1);
        assert!((ours / plain - 1.0).abs() < 0.01, "{ours:e} vs {plain:e}");
    }
}

#[test]
fn coarse_search_settings() {
    let md = modem(Modulation::Bpsk);
    assert!(md.coarse_search(&RxConfig::default()).is_none());
    assert!(md
        .coarse_metric(&[Complex32::ZERO; 2000], &RxConfig::default())
        .is_empty());
    // wide: no chunking, FFT about 1.5 times the 632-sample window
    let w = md.coarse_search(&wide()).unwrap();
    assert_eq!((w.chunk, w.fft_len, w.step), (1, 1024, 2));
    // narrow: chunks shrink the FFT
    let n = md
        .coarse_search(&RxConfig {
            cfo_search: 0.1,
            ..RxConfig::default()
        })
        .unwrap();
    assert!(n.chunk > 1 && n.fft_len < w.fft_len);
    // a search past half the sample rate is capped there
    let c = md
        .coarse_search(&RxConfig {
            cfo_search: 100.0,
            ..RxConfig::default()
        })
        .unwrap();
    assert!(c.bins < c.fft_len / 2);
}

/// A fingerprint of a waveform: every sample rounded to 1e-4 (so a last-bit difference in a
/// sine does not matter) and folded into an FNV-1a hash.
fn fingerprint(x: &[Complex32]) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for s in x {
        for v in [s.re, s.im] {
            for b in ((v * 1e4).round() as i32).to_le_bytes() {
                h = (h ^ b as u64).wrapping_mul(0x0100_0000_01b3);
            }
        }
    }
    h
}

#[test]
fn w0_waveform_does_not_drift() {
    // The default frame format is W0 and its samples must never change: pinned here (length and a
    // fingerprint of the whole frame, taken from `main` before v1) for payload 0x00..0x2F, 8 samples per symbol.
    let payload: Vec<u8> = (0..48u8).collect();
    for (m, len, fp) in GOLDEN {
        let md = modem(m);
        assert_eq!(md.config().frame, FrameFormat::W0);
        let x = md.modulate(&payload).unwrap();
        assert_eq!(x.len(), len, "{m:?} length");
        assert_eq!(
            fingerprint(&x),
            fp,
            "{m:?} fingerprint {:#x}",
            fingerprint(&x)
        );
    }
}

const GOLDEN: [(Modulation, usize, u64); 3] = [
    (Modulation::Fsk2, 3960, 0x766b_7f2c_708b_112c),
    (Modulation::Bpsk, 4081, 0xf66f_4d10_d066_050e),
    (Modulation::Qpsk, 2417, 0x8e88_0300_2278_daa9),
];

// ---- frame format v1: protected header ----

fn modem_v1(m: Modulation) -> Modem {
    Modem::new(ModemConfig {
        frame: FrameFormat::V1,
        ..ModemConfig::new(m)
    })
    .unwrap()
}

/// A noiseless buffer with the frame `tx` after `lead` samples.
fn clean(tx: &[Complex32], lead: usize) -> Vec<Complex32> {
    let mut ch = Channel::new(1);
    ch.apply(
        tx,
        &Impairments {
            lead,
            tail: 40,
            ..Default::default()
        },
    )
}

#[test]
fn v1_frame_length_and_energy() {
    for m in ALL {
        let md = modem_v1(m);
        for n in [0usize, 1, 64, 255] {
            let x = md.modulate(&vec![7; n]).unwrap();
            assert_eq!(x.len(), md.frame_samples(n), "{m:?} {n}");
            let nsym = (63 + 16 + HEADER_SYMBOLS) as f64
                + (n + 2) as f64 * 8.0 / m.bits_per_symbol() as f64;
            let e: f64 = x.iter().map(|s| s.norm_sqr() as f64).sum();
            assert!((e / nsym - 1.0).abs() < 0.02, "{m:?} {n}: {e} for {nsym}");
        }
    }
    // v1 replaces the 4 bytes of W0 length, complement and (shared) overhead by 152 header symbols:
    // for 64 bytes of BPSK, 152 + 66 * 8 symbols against 68 * 8
    let (w0, v1) = (modem(Modulation::Bpsk), modem_v1(Modulation::Bpsk));
    assert_eq!(
        v1.frame_samples(64) - w0.frame_samples(64),
        (152 + 66 * 8 - 68 * 8) * 8
    );
}

#[test]
fn v1_round_trip_random_payloads_and_offsets() {
    // Payloads of 0 to 255 bytes, random delay, sub-sample delay, phase and a small frequency
    // offset, at an Eb/N0 where W0 also gets through.
    for m in ALL {
        let md = modem_v1(m);
        let mut ch = Channel::new(21);
        let ebn0 = if m == Modulation::Fsk2 { 18.0 } else { 12.0 };
        let n0 = n0_for_ebn0(ebn0, md.energy_per_bit());
        let mut ok = 0;
        let total = 40;
        for i in 0..total {
            let len = if i < 3 {
                [0, 1, 255][i]
            } else {
                ch.rng().below(256) as usize
            };
            let payload = ch.rng().bytes(len);
            let tx = md.modulate(&payload).unwrap();
            let lead = ch.rng().below(300) as usize;
            let imp = Impairments {
                n0,
                phase_rad: ch.rng().uniform() * 2.0 * PI,
                frac_delay: ch.rng().uniform(),
                cfo_cycles_per_sample: (ch.rng().uniform() - 0.5) * 0.004 / 8.0,
                lead,
                tail: 30,
            };
            let y = ch.apply(&tx, &imp);
            let got = md.demodulate(&y);
            let hit = got.iter().find(|f| f.crc_ok && f.payload == payload);
            if let Some(f) = hit {
                ok += 1;
                assert!(f.start.abs_diff(lead) <= 1, "{m:?} frame {i}");
                assert_eq!(f.header, Some(HeaderFields::new(m.code(), len as u8)));
            }
        }
        assert!(ok >= total - 2, "{m:?}: {ok} of {total}");
    }
}

#[test]
fn v1_noiseless_round_trip_is_exact() {
    for m in ALL {
        let md = modem_v1(m);
        let mut ch = Channel::new(22);
        for i in 0..30 {
            let len = ch.rng().below(256) as usize;
            let payload = ch.rng().bytes(len);
            let tx = md.modulate(&payload).unwrap();
            let imp = Impairments {
                phase_rad: ch.rng().uniform() * 2.0 * PI,
                frac_delay: ch.rng().uniform(),
                lead: 50 + i,
                tail: 20,
                ..Default::default()
            };
            let (got, rep) = md.demodulate_report(&ch.apply(&tx, &imp), &RxConfig::default());
            assert_eq!(got.len(), 1, "{m:?} {i}");
            assert!(got[0].crc_ok && got[0].payload == payload, "{m:?} {i}");
            assert_eq!(rep.header_rejected, 0, "{m:?} {i}");
        }
    }
}

#[test]
fn v1_qpsk_long_body_needs_the_estimate_over_the_header() {
    // QPSK, 255-byte payloads, offsets within +-0.002 cycles per symbol, 12 dB, fixed seed. The
    // preamble alone cannot see so small an offset, and by the end of the header and a long body
    // the phase has turned too far. The second estimate over the known header fixes it. With that
    // step removed 4 of these 40 frames fail the CRC-16 (checked by disabling it).
    let md = modem_v1(Modulation::Qpsk);
    let mut ch = Channel::new(41);
    let n0 = n0_for_ebn0(12.0, md.energy_per_bit());
    let mut ok = 0;
    for _ in 0..40 {
        let payload = ch.rng().bytes(255);
        let tx = md.modulate(&payload).unwrap();
        let cfo = (ch.rng().uniform() - 0.5) * 0.004;
        let imp = Impairments {
            n0,
            phase_rad: ch.rng().uniform() * 2.0 * PI,
            frac_delay: ch.rng().uniform(),
            cfo_cycles_per_sample: cfo / 8.0,
            lead: 80,
            tail: 30,
        };
        let got = md.demodulate(&ch.apply(&tx, &imp));
        ok += got.iter().any(|f| f.crc_ok && f.payload == payload) as usize;
    }
    assert_eq!(ok, 40);
}

#[test]
fn v1_coarse_search_decodes_across_the_range() {
    for m in ALL {
        let md = modem_v1(m);
        let mut ch = Channel::new(23);
        let ebn0 = if m == Modulation::Fsk2 { 16.0 } else { 12.0 };
        let n0 = n0_for_ebn0(ebn0, md.energy_per_bit());
        for cfo in [-1.5, -0.6, -0.0253, 0.0, 0.31, 1.0, 1.5] {
            for _ in 0..3 {
                let len = ch.rng().below(256) as usize;
                let payload = ch.rng().bytes(len);
                let tx = md.modulate(&payload).unwrap();
                let lead = 10 + ch.rng().below(190) as usize;
                let imp = Impairments {
                    n0,
                    phase_rad: ch.rng().uniform() * 2.0 * PI,
                    cfo_cycles_per_sample: cfo / 8.0,
                    frac_delay: ch.rng().uniform(),
                    lead,
                    tail: 40,
                };
                let got = md.demodulate_with(&ch.apply(&tx, &imp), &wide());
                assert_eq!(got.len(), 1, "{m:?} cfo {cfo}: {} frames", got.len());
                assert!(
                    got[0].crc_ok && got[0].payload == payload,
                    "{m:?} cfo {cfo}"
                );
                assert!(got[0].start.abs_diff(lead) <= 1, "{m:?} cfo {cfo}");
            }
        }
    }
}

/// Send a frame made from `tx` through a clean channel and return what the receiver says.
fn receive(md: &Modem, tx: &[Complex32], rx: &RxConfig) -> (Vec<RxFrame>, RxReport) {
    md.demodulate_report(&clean(tx, 77), rx)
}

#[test]
fn v1_header_rules_reject_and_count() {
    let payload = [1u8, 2, 3, 4, 5];
    for m in ALL {
        let md = modem_v1(m);
        let good = HeaderFields::new(m.code(), 5);
        let other_mod = Modulation::from_code((m.code() + 1) % 3).unwrap().code();
        let bad = [
            HeaderFields { version: 1, ..good },
            HeaderFields {
                extension: 1,
                ..good
            },
            HeaderFields { pilots: 1, ..good },
            HeaderFields {
                payload_code: 1,
                ..good
            },
            HeaderFields {
                payload_code: 7,
                ..good
            },
            HeaderFields {
                modulation: other_mod,
                ..good
            },
            HeaderFields {
                modulation: 3,
                ..good
            },
        ];
        for h in bad {
            let tx = md.modulate_v1(&payload, &h).unwrap();
            let (got, rep) = receive(&md, &tx, &RxConfig::default());
            assert!(got.is_empty(), "{m:?} {h:?} was accepted");
            assert!(rep.header_rejected >= 1, "{m:?} {h:?}");
        }
        // a wrong CRC-8
        let f = good.to_bytes();
        let block = [f[0], f[1], f[2], crc8(&f) ^ 0x10];
        let tx = md
            .modulate_v1_parts(&header_symbols_raw(block), &body_bits_v1(&good, &payload))
            .unwrap();
        let (got, rep) = receive(&md, &tx, &RxConfig::default());
        assert!(got.is_empty(), "{m:?}");
        assert!(rep.header_rejected >= 1, "{m:?}");
        // and the good one passes
        let tx = md.modulate_v1(&payload, &good).unwrap();
        let (got, rep) = receive(&md, &tx, &RxConfig::default());
        assert_eq!(got.len(), 1);
        assert_eq!(rep.header_rejected, 0);
    }
}

#[test]
fn v1_header_rules_hold_on_the_coarse_path_too() {
    let md = modem_v1(Modulation::Bpsk);
    let payload = [9u8; 6];
    let h = HeaderFields {
        extension: 1,
        ..HeaderFields::new(0, 6)
    };
    let tx = md.modulate_v1(&payload, &h).unwrap();
    let (got, rep) = receive(&md, &tx, &wide());
    assert!(got.is_empty());
    assert!(rep.header_rejected >= 1);
}

#[test]
fn v1_reserved_fields_are_accepted_and_reported() {
    for m in ALL {
        let md = modem_v1(m);
        let h = HeaderFields {
            channel_plan: 0xB,
            coexistence: 0x5,
            ..HeaderFields::new(m.code(), 9)
        };
        let payload = [0x5Au8; 9];
        let tx = md.modulate_v1(&payload, &h).unwrap();
        for rx in [RxConfig::default(), wide()] {
            let (got, rep) = receive(&md, &tx, &rx);
            assert_eq!(got.len(), 1, "{m:?}");
            assert!(got[0].crc_ok && got[0].payload == payload);
            assert_eq!(got[0].header, Some(h));
            assert_eq!(rep.header_rejected, 0);
        }
    }
}

#[test]
fn v1_damaged_body_fails_the_crc16() {
    // Blank a stretch of samples inside the body: the header decodes, the CRC-16 fails.
    for m in ALL {
        let md = modem_v1(m);
        let payload: Vec<u8> = (0..40u8).collect();
        let mut tx = md.modulate(&payload).unwrap();
        let body0 = (63 + 16 + HEADER_SYMBOLS) * 8 + 8 * 8;
        for s in &mut tx[body0..body0 + 5 * 8] {
            *s = Complex32::ZERO;
        }
        let (got, _) = receive(&md, &tx, &RxConfig::default());
        assert_eq!(got.len(), 1, "{m:?}");
        assert!(!got[0].crc_ok, "{m:?}");
        assert!(got[0].header.is_some());
    }
}

#[test]
fn v1_a_miscorrected_length_is_caught_by_the_crc16() {
    // A header that decodes (CRC-8 fine) to another length than the one the CRC-16 was made
    // for: here the sender's CRC-16 covered length 20, the header says 21 and 19.
    for m in ALL {
        let md = modem_v1(m);
        let payload: Vec<u8> = (0..20u8).collect();
        let real = HeaderFields::new(m.code(), 20);
        for wrong in [19u8, 21] {
            let h = HeaderFields::new(m.code(), wrong);
            let tx = md
                .modulate_v1_parts(&header_symbols(&h), &body_bits_v1(&real, &payload))
                .unwrap();
            let (got, _) = receive(&md, &tx, &RxConfig::default());
            // 21 may run off the buffer; whatever comes back must not be trusted
            assert!(got.iter().all(|f| !f.crc_ok), "{m:?} length {wrong}");
        }
    }
}

#[test]
fn v1_noise_only_gives_no_frames() {
    for m in ALL {
        let md = modem_v1(m);
        let mut ch = Channel::new(78);
        let y = ch.noise(200_000, 1.0);
        let (got, rep) = md.demodulate_report(&y, &RxConfig::default());
        assert!(got.is_empty(), "{m:?}: {} false frames", got.len());
        assert_eq!(rep.preambles, 0);
        let got = md.demodulate_with(&y, &wide());
        assert!(got.is_empty(), "{m:?} coarse");
    }
}

#[test]
fn a_w0_modem_does_not_send_v1_and_formats_do_not_mix() {
    let w0 = modem(Modulation::Bpsk);
    let v1 = modem_v1(Modulation::Bpsk);
    assert!(w0.modulate_v1(&[1], &HeaderFields::default()).is_err());
    let tx = v1.modulate(&[1, 2, 3]).unwrap();
    let (got, rep) = receive(&w0, &tx, &RxConfig::default());
    assert!(got.iter().all(|f| !f.crc_ok));
    assert!(rep.preambles >= 1);
    let tx = w0.modulate(&[1, 2, 3]).unwrap();
    let (got, _) = receive(&v1, &tx, &RxConfig::default());
    assert!(got.is_empty());
}

#[test]
fn report_counts_add_up() {
    // Every candidate ends in a frame or exactly one count, in both formats and on both paths.
    for frame in [FrameFormat::W0, FrameFormat::V1] {
        let md = Modem::new(ModemConfig {
            frame,
            ..ModemConfig::new(Modulation::Bpsk)
        })
        .unwrap();
        let mut ch = Channel::new(31);
        let n0 = n0_for_ebn0(1.0, md.energy_per_bit());
        for rx in [RxConfig::default(), wide()] {
            for _ in 0..30 {
                let tx = md.modulate(&ch.rng().bytes(30)).unwrap();
                let imp = Impairments {
                    n0,
                    lead: 40,
                    tail: if ch.rng().below(2) == 0 { 8 } else { 3000 },
                    ..Default::default()
                };
                let (got, r) = md.demodulate_report(&ch.apply(&tx, &imp), &rx);
                assert_eq!(
                    r.preambles,
                    got.len() + r.sync_rejected + r.header_rejected + r.too_short + r.inside_frame,
                    "{frame:?} {r:?}"
                );
            }
        }
    }
}

#[test]
fn candidates_inside_a_found_frame_are_not_lost_frames() {
    // A long v1 frame at 6 dB: its header is good, so its span is known, and the candidates its
    // body makes are counted as inside the frame.
    let md = modem_v1(Modulation::Bpsk);
    let mut ch = Channel::new(32);
    let n0 = n0_for_ebn0(6.0, md.energy_per_bit());
    let mut inside = 0;
    for _ in 0..40 {
        let tx = md.modulate(&ch.rng().bytes(255)).unwrap();
        let y = ch.apply(
            &tx,
            &Impairments {
                n0,
                lead: 50,
                tail: 50,
                ..Default::default()
            },
        );
        let (got, r) = md.demodulate_report(&y, &RxConfig::default());
        assert_eq!(got.len(), 1);
        assert_eq!(
            (r.sync_rejected, r.header_rejected, r.too_short),
            (0, 0, 0),
            "{r:?}"
        );
        inside += r.inside_frame;
    }
    assert!(
        inside > 0,
        "no body candidates seen; the test checks nothing"
    );
}

/// The report says where W0 loses frames. BPSK, 200 frames each, seed 5, no `expected_len`:
/// `header_rejected` is close to frames sent minus frames found, and far above the other counts.
/// Measured: lost 161 / header_rejected 147 at 0 dB, 100 / 98 at 2 dB, 34 / 34 at 4 dB.
#[test]
fn w0_report_blames_the_header() {
    let w0 = BerOptions::default();
    for db in [0.0, 2.0, 4.0] {
        let a = run_found(Modulation::Bpsk, db, 200, 5, &w0).unwrap();
        let (r, lost) = (&a.report, 200 - a.found as usize);
        assert!(lost >= 20, "{db} dB: too few losses to say anything");
        assert!(
            r.header_rejected as f64 >= 0.8 * lost as f64
                && r.header_rejected as f64 <= 1.1 * lost as f64,
            "{db} dB: lost {lost}, {r:?}"
        );
        assert!(
            r.header_rejected >= 5 * (r.sync_rejected + r.too_short),
            "{db} dB: {r:?}"
        );
    }
}

/// The point of v1: at low Eb/N0 the W0 header is the weak link. BPSK, no frequency offset, no
/// `expected_len`, 200 frames each, fixed seeds. Theory for W0: found = 0.9214^16 = 27 % at 0 dB,
/// 54 % at 2 dB (the preamble detector itself finds about every frame).
#[test]
fn v1_finds_frames_where_w0_loses_them_to_the_header() {
    let w0 = BerOptions::default();
    let v1 = BerOptions {
        frame: FrameFormat::V1,
        ..BerOptions::default()
    };
    for (db, w0_max) in [(0.0, 0.40), (2.0, 0.65)] {
        let a = run_found(Modulation::Bpsk, db, 200, 5, &w0).unwrap();
        let b = run_found(Modulation::Bpsk, db, 200, 5, &v1).unwrap();
        eprintln!(
            "{db} dB: W0 {} / 200, {:?}; V1 {} / 200, {:?}",
            a.found, a.report, b.found, b.report
        );
        assert!(
            a.found as f64 <= w0_max * 200.0,
            "W0 at {db} dB: {} found",
            a.found
        );
        assert!(
            b.found as f64 >= 0.97 * 200.0,
            "V1 at {db} dB: {} found",
            b.found
        );
        assert!(b.report.header_rejected <= 2, "{:?}", b.report);
    }
    // 6 dB: v1 finds every frame; the report shows no lost frames, only candidates inside frames
    let b = run_found(Modulation::Bpsk, 6.0, 200, 5, &v1).unwrap();
    assert_eq!(b.found, 200);
    let r = &b.report;
    assert!(
        r.header_rejected + r.sync_rejected + r.too_short <= 3,
        "{r:?}"
    );
    assert!(r.inside_frame > 100, "{r:?}");
}

/// Sync tolerance for the docs (v1, BPSK, 400 frames, seed 11): `cargo test --release -p
/// oasa-waveform --test frames sync_tolerance_table -- --ignored --nocapture`.
#[test]
#[ignore]
fn sync_tolerance_table() {
    for sync_max_errors in [3, 5, 8] {
        let opts = BerOptions {
            frame: FrameFormat::V1,
            rx: RxConfig {
                sync_max_errors,
                ..RxConfig::default()
            },
            ..BerOptions::default()
        };
        let mut row = format!("sync_max_errors {sync_max_errors}:");
        for db in [-2.0, -1.0, 0.0, 2.0, 6.0] {
            let p = run_found(Modulation::Bpsk, db, 400, 11, &opts).unwrap();
            row += &format!(" {} ({:?})", p.found, p.report);
        }
        println!("{row}");
    }
}

/// Found-frame rates for the docs: `cargo test --release -p oasa-waveform --test frames
/// found_rate_table -- --ignored --nocapture`.
#[test]
#[ignore]
fn found_rate_table() {
    for (label, cfo_search, cfo_random) in [
        ("no offset", 0.0f32, 0.0),
        ("cfo_search 0.1, offset +-0.018", 0.1, 0.018),
    ] {
        println!("{label}: frames found / 400");
        for m in [Modulation::Bpsk, Modulation::Qpsk] {
            for frame in [FrameFormat::W0, FrameFormat::V1] {
                let opts = BerOptions {
                    frame,
                    cfo_random,
                    rx: RxConfig {
                        cfo_search,
                        ..RxConfig::default()
                    },
                    ..BerOptions::default()
                };
                let mut row = format!("{:5} {:2}:", m.name(), frame.name());
                for db in [-2.0, -1.0, 0.0, 1.0, 2.0, 4.0, 6.0] {
                    let p = run_found(m, db, 400, 11, &opts).unwrap();
                    row += &format!(
                        " {:3} ({:3} hdr rej, {:3} pre)",
                        p.found, p.report.header_rejected, p.report.preambles
                    );
                }
                println!("{row}");
            }
        }
    }
}
