//! Frame level behaviour of the modem through the channel: round trips with unknown offsets,
//! false alarms, damaged frames, wrong sync words, gain and frequency offset.

use oasa_waveform::channel::{n0_for_ebn0, Channel, Impairments};
use oasa_waveform::frame::MAX_PAYLOAD;
use oasa_waveform::{Complex32, Modem, ModemConfig, Modulation, RxConfig};
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
