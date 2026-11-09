//! Light error correction (frame v1, `payload_code` 1): K=7 rate 1/2 code and a block interleaver over
//! the body. See `docs/fec-light.md`.

use oasa_waveform::channel::{n0_for_ebn0, Channel, Impairments};
use oasa_waveform::fec::{conv_encode, viterbi_decode};
use oasa_waveform::frame::{
    body_bits_light, body_bits_v1, crc8, header_symbols, header_symbols_raw, interleave, whiten,
    PayloadCode,
};
use oasa_waveform::sim::{run_ber, run_found, BerOptions};
use oasa_waveform::{
    Complex32, FrameFormat, HeaderFields, Modem, ModemConfig, Modulation, RxConfig, RxFrame,
    RxReport,
};
use std::f64::consts::PI;

const ALL: [Modulation; 3] = [Modulation::Fsk2, Modulation::Bpsk, Modulation::Qpsk];

fn modem(m: Modulation, code: PayloadCode) -> Modem {
    Modem::new(ModemConfig {
        frame: FrameFormat::V1,
        code,
        ..ModemConfig::new(m)
    })
    .unwrap()
}

fn light(m: Modulation) -> Modem {
    modem(m, PayloadCode::Light)
}

fn light_header(m: Modulation, n: usize) -> HeaderFields {
    HeaderFields {
        payload_code: 1,
        ..HeaderFields::new(m.code(), n as u8)
    }
}

/// The frame `tx` after `lead` samples of silence, no noise.
fn clean(tx: &[Complex32], lead: usize) -> Vec<Complex32> {
    Channel::new(1).apply(
        tx,
        &Impairments {
            lead,
            tail: 40,
            ..Default::default()
        },
    )
}

fn receive(md: &Modem, tx: &[Complex32]) -> (Vec<RxFrame>, RxReport) {
    md.demodulate_report(&clean(tx, 77), &RxConfig::default())
}

#[test]
fn config_energy_and_airtime() {
    // W0 has no payload_code: the light code needs v1
    assert!(Modem::new(ModemConfig {
        code: PayloadCode::Light,
        ..ModemConfig::new(Modulation::Bpsk)
    })
    .is_err());
    // Eb is per information bit: a rate 1/2 code doubles it per coded bit
    for (m, eb, syms) in [
        (Modulation::Bpsk, 2.0, 1299),
        (Modulation::Qpsk, 1.0, 765),
        (Modulation::Fsk2, 2.0, 1299),
    ] {
        let md = light(m);
        assert_eq!(md.energy_per_bit(), eb, "{m:?}");
        // 79 preamble and sync + 152 header + 2 (66 * 8 + 6) coded bits
        assert_eq!(md.frame_symbols(64), syms, "{m:?}");
        assert_eq!(
            md.frame_symbols(64) - modem(m, PayloadCode::None).frame_symbols(64),
            (66 * 8 + 12) / m.bits_per_symbol(),
            "{m:?}"
        );
        for n in [0usize, 1, 64, 255] {
            let x = md.modulate(&vec![9; n]).unwrap();
            assert_eq!(x.len(), md.frame_samples(n), "{m:?} {n}");
            // every symbol still has unit energy
            let e: f64 = x.iter().map(|s| s.norm_sqr() as f64).sum();
            let nsym = md.frame_symbols(n) as f64;
            assert!((e / nsym - 1.0).abs() < 0.02, "{m:?} {n}: {e} for {nsym}");
        }
    }
}

#[test]
fn noiseless_round_trip_is_exact() {
    let mut ch = Channel::new(40);
    for m in ALL {
        let md = light(m);
        for n in [0usize, 1, 2, 3, 64, 255] {
            let payload = ch.rng().bytes(n);
            let tx = md.modulate(&payload).unwrap();
            let lead = 3 + ch.rng().below(200) as usize;
            let phase_rad = ch.rng().uniform() * 2.0 * PI;
            let y = ch.apply(
                &tx,
                &Impairments {
                    phase_rad,
                    lead,
                    tail: 30,
                    ..Default::default()
                },
            );
            let got = md.demodulate(&y);
            assert_eq!(got.len(), 1, "{m:?} {n}");
            assert!(got[0].crc_ok && got[0].payload == payload, "{m:?} {n}");
            assert_eq!(got[0].header, Some(light_header(m, n)), "{m:?} {n}");
            assert_eq!(got[0].start, lead as u64, "{m:?} {n}");
            assert_eq!(got[0].end, (lead + tx.len()) as u64, "{m:?} {n}");
        }
    }
}

/// Every frame of `count`, 64 bytes, through noise with random phase, start, sub-sample delay
/// (PSK) and carrier offset in `+-cfo`, received with `rx`. Returns frames found (at the right
/// place, within a symbol) and frames delivered (found with the payload right).
fn delivered(
    m: Modulation,
    ebn0: f64,
    cfo: f64,
    rx: &RxConfig,
    count: usize,
    seed: u64,
) -> (usize, usize) {
    let md = light(m);
    let mut ch = Channel::new(seed);
    let n0 = n0_for_ebn0(ebn0, md.energy_per_bit());
    let (mut found, mut ok) = (0, 0);
    for _ in 0..count {
        let payload = ch.rng().bytes(64);
        let tx = md.modulate(&payload).unwrap();
        let lead = 5 + ch.rng().below(300) as usize;
        let imp = Impairments {
            n0,
            phase_rad: ch.rng().uniform() * 2.0 * PI,
            cfo_cycles_per_sample: (2.0 * ch.rng().uniform() - 1.0) * cfo / 8.0,
            frac_delay: if m == Modulation::Fsk2 {
                0.0
            } else {
                ch.rng().uniform()
            },
            lead,
            tail: 50,
        };
        let got = md.demodulate_with(&ch.apply(&tx, &imp), rx);
        if let Some(f) = got.iter().find(|f| f.start.abs_diff(lead as u64) <= 8) {
            found += 1;
            ok += (f.crc_ok && f.payload == payload) as usize;
        }
    }
    (found, ok)
}

#[test]
fn round_trip_with_phase_delay_and_offset() {
    // 6 dB Eb/N0 for PSK (3 dB Es/N0 per coded BPSK symbol), 12 dB for noncoherent 2-FSK. Random
    // phase, start and sub-sample delay with the plain receiver; with an offset per frame within
    // +-0.018 cycles per symbol (the W1 bench's 2.5 ppm) through `cfo_search 0.1`, as the bench runs.
    // (The plain receiver with an offset loses v1 headers at this Es/N0, coded or not: see
    // docs/fec-light.md.)
    let search = RxConfig {
        cfo_search: 0.1,
        ..RxConfig::default()
    };
    for (m, ebn0) in [
        (Modulation::Bpsk, 6.0),
        (Modulation::Qpsk, 6.0),
        (Modulation::Fsk2, 12.0),
    ] {
        let got = delivered(m, ebn0, 0.0, &RxConfig::default(), 16, 41);
        assert_eq!(got, (16, 16), "{m:?} plain receiver");
        // Finding a frame at 3 dB Es/N0 with an offset loses a header now and then (1 of 60 BPSK
        // frames in a longer run); every frame found must decode.
        let (found, ok) = delivered(m, ebn0, 0.018, &search, 16, 45);
        assert!(
            found >= 15 && ok == found,
            "{m:?} with an offset: {found} {ok}"
        );
    }
}

#[test]
fn round_trip_with_the_coarse_search() {
    let rx = RxConfig {
        cfo_search: 1.5,
        ..RxConfig::default()
    };
    for (m, ebn0) in [
        (Modulation::Bpsk, 8.0),
        (Modulation::Qpsk, 8.0),
        (Modulation::Fsk2, 13.0),
    ] {
        let got = delivered(m, ebn0, 1.4, &rx, 8, 42);
        assert_eq!(got, (8, 8), "{m:?}");
    }
}

#[test]
fn any_v1_receiver_takes_the_code_from_the_header() {
    // A modem set to send uncoded frames still receives light ones and the other way round, in one
    // buffer.
    for m in ALL {
        let (plain, coded) = (modem(m, PayloadCode::None), light(m));
        let p: Vec<Vec<u8>> = (0..4u8).map(|i| vec![i; 30 + i as usize]).collect();
        let mut tx = Vec::new();
        for (i, pl) in p.iter().enumerate() {
            let md = if i % 2 == 0 { &coded } else { &plain };
            tx.extend(md.modulate(pl).unwrap());
            tx.extend(std::iter::repeat_n(Complex32::ZERO, 200));
        }
        for rx in [&plain, &coded] {
            let got = rx.demodulate(&clean(&tx, 50));
            assert_eq!(got.len(), 4, "{m:?}");
            for (i, f) in got.iter().enumerate() {
                assert!(f.crc_ok && f.payload == p[i], "{m:?} frame {i}");
                assert_eq!(f.header.unwrap().payload_code, (i % 2 == 0) as u8);
            }
        }
    }
}

/// Body bits of a 64-byte light frame whose coded bits `from..from + n` are flipped: on air (after
/// the interleaver) if `on_air`, or in code order (as if there were no interleaver: the burst
/// reaches the decoder as it happened).
fn burst_body(h: &HeaderFields, payload: &[u8], from: usize, n: usize, on_air: bool) -> Vec<u8> {
    if on_air {
        let mut b = body_bits_light(h, payload);
        b[from..from + n].iter_mut().for_each(|x| *x ^= 1);
        b
    } else {
        let mut c = conv_encode(&body_bits_v1(h, payload));
        c[from..from + n].iter_mut().for_each(|x| *x ^= 1);
        interleave(&c)
    }
}

#[test]
fn the_interleaver_breaks_up_a_burst() {
    // A fade wipes out N consecutive symbols. Uncoded, one wrong bit loses the frame. Coded but not
    // interleaved, the burst reaches the decoder in one piece and beats the code (free distance 10).
    // Interleaved, it reaches the decoder as single errors 32 coded bits apart, which the code fixes.
    // BPSK: 32 symbols (32 bits); QPSK: 16 symbols (32 bits). No noise, so this is exact.
    let payload: Vec<u8> = (0..64u8).map(|i| i.wrapping_mul(53)).collect();
    for m in [Modulation::Bpsk, Modulation::Qpsk] {
        let md = light(m);
        let h = light_header(m, 64);
        let hs = header_symbols(&h);
        for from in [0usize, 100, 333, 500, 700, 1068 - 32] {
            let tx = md
                .modulate_v1_parts(&hs, &burst_body(&h, &payload, from, 32, true))
                .unwrap();
            let (got, _) = receive(&md, &tx);
            assert!(
                got.len() == 1 && got[0].crc_ok && got[0].payload == payload,
                "{m:?} burst at {from}: interleaved frame lost"
            );
            let tx = md
                .modulate_v1_parts(&hs, &burst_body(&h, &payload, from, 32, false))
                .unwrap();
            let (got, _) = receive(&md, &tx);
            assert!(
                got.len() == 1 && !got[0].crc_ok,
                "{m:?} burst at {from}: survived without the interleaver"
            );
        }
        // uncoded v1 frame: the same burst on air loses it
        let plain = modem(m, PayloadCode::None);
        let h0 = HeaderFields::new(m.code(), 64);
        let mut body = body_bits_v1(&h0, &payload);
        body[100..132].iter_mut().for_each(|x| *x ^= 1);
        let tx = plain
            .modulate_v1_parts(&header_symbols(&h0), &body)
            .unwrap();
        let (got, _) = receive(&plain, &tx);
        assert!(got.len() == 1 && !got[0].crc_ok, "{m:?} uncoded");
    }
}

#[test]
fn the_header_still_protects() {
    let payload: Vec<u8> = (0..64u8).collect();
    for m in ALL {
        let md = light(m);
        let h = light_header(m, 64);
        // reserved codes are refused (2 to 7: the heavy tier, later)
        for code in [2u8, 3, 7] {
            let bad = HeaderFields {
                payload_code: code,
                ..h
            };
            let (got, rep) = receive(&md, &md.modulate_v1(&payload, &bad).unwrap());
            assert!(got.is_empty(), "{m:?} code {code}");
            assert!(rep.header_rejected >= 1, "{m:?} code {code}");
        }
        // a header that says "uncoded" in front of a coded body (or the other way round) fails the
        // CRC-16: the code is in the header bytes the CRC covers, and the body does not parse
        let h0 = HeaderFields::new(m.code(), 64);
        for (hdr, body) in [
            (h0, body_bits_light(&h, &payload)),
            (h, body_bits_v1(&h0, &payload)),
        ] {
            let tx = md.modulate_v1_parts(&header_symbols(&hdr), &body).unwrap();
            let (got, _) = receive(&md, &tx);
            assert!(got.iter().all(|f| !f.crc_ok), "{m:?} {hdr:?}");
        }
        // a header that decoded to another length fails the CRC-16
        let short = HeaderFields { length: 63, ..h };
        let tx = md
            .modulate_v1_parts(&header_symbols(&short), &body_bits_light(&h, &payload))
            .unwrap();
        let (got, _) = receive(&md, &tx);
        assert!(got.iter().all(|f| !f.crc_ok), "{m:?} miscorrected length");
        // a wrong CRC-8 is still refused
        let f = h.to_bytes();
        let block = [f[0], f[1], f[2], crc8(&f) ^ 0x01];
        let tx = md
            .modulate_v1_parts(&header_symbols_raw(block), &body_bits_light(&h, &payload))
            .unwrap();
        let (got, rep) = receive(&md, &tx);
        assert!(got.is_empty() && rep.header_rejected >= 1, "{m:?} CRC-8");
    }
}

#[test]
fn noise_only_gives_no_frames() {
    let mut ch = Channel::new(43);
    let wide = RxConfig {
        cfo_search: 1.5,
        ..RxConfig::default()
    };
    for m in ALL {
        let md = light(m);
        let y = ch.noise(60_000, 1.0);
        for rx in [RxConfig::default(), wide.clone()] {
            let (got, rep) = md.demodulate_report(&y, &rx);
            assert!(got.is_empty(), "{m:?} {rx:?}");
            assert_eq!(rep.preambles, 0, "{m:?}");
        }
    }
}

#[test]
fn ber_mode_uses_the_code_and_the_information_bit_energy() {
    // BER measurement (length given): at 4 dB Eb/N0 per information bit, the coded body is at
    // 1 dB Es/N0 per BPSK symbol, where uncoded BPSK would be at 5.6e-2. The code brings the
    // payload bits far below that.
    let opts = BerOptions {
        frame: FrameFormat::V1,
        code: PayloadCode::Light,
        ..BerOptions::default()
    };
    let p = run_ber(Modulation::Bpsk, 4.0, 64 * 8 * 60, 44, &opts).unwrap();
    assert_eq!(p.detected, 60);
    assert!(p.ber < 1e-3, "{p:?}");
    assert!(p.header_ok >= 58, "{p:?}");
    assert!(p.theory.is_nan() && p.z.is_nan());
}

/// Packet loss of the real receiver (length, code and everything read from the frame), BPSK, 64-byte
/// payloads, light code. Measured (seed 11, see docs/fec-light.md): 0 lost of 300 at 4.5 dB, 6 of
/// 200 at 3.5 dB. Limits leave margin: at least 99 % at 4.5 dB and 90 % at 3.5 dB.
#[test]
fn packet_loss_curve_light_bpsk() {
    let opts = BerOptions {
        frame: FrameFormat::V1,
        code: PayloadCode::Light,
        ..BerOptions::default()
    };
    for (db, frames, min_ok) in [(4.5, 300u64, 297u64), (3.5, 200, 180)] {
        let p = run_found(Modulation::Bpsk, db, frames, 11, &opts).unwrap();
        eprintln!("{db} dB: delivered {} / {frames}, {:?}", p.crc_ok, p.report);
        assert!(p.crc_ok >= min_ok, "{db} dB: {} / {frames}", p.crc_ok);
    }
}

/// The code alone, with ideal soft decisions (exact LLRs, known timing and phase, no sync or header):
/// 64-byte frames, BPSK. What the modem's curve is compared with.
fn ideal_loss(ebn0_db: f64, frames: usize, seed: u64) -> f64 {
    let mut ch = Channel::new(seed);
    // Eb = 2 Es (rate 1/2), Es = 1: N0 = 2 / (Eb/N0); per dimension N0 / 2; LLR = 4 y / N0
    let n0 = 2.0 / 10f64.powf(ebn0_db / 10.0);
    let mut lost = 0;
    for _ in 0..frames {
        let mut info: Vec<u8> = (0..528).map(|_| (ch.rng().next_u64() & 1) as u8).collect();
        whiten(&mut info);
        let soft: Vec<f32> = conv_encode(&info)
            .iter()
            .map(|&b| {
                let y = 1.0 - 2.0 * b as f64 + (n0 / 2.0).sqrt() * ch.rng().gaussian();
                (4.0 * y / n0) as f32
            })
            .collect();
        lost += (viterbi_decode(&soft, info.len()) != info) as usize;
    }
    lost as f64 / frames as f64
}

/// Packet loss against Eb/N0 for the docs. Slow (minutes): `cargo test --release -p oasa-waveform
/// --test fec_light loss_table -- --ignored --nocapture`.
#[test]
#[ignore]
fn loss_table() {
    let frames = 400u64;
    let dbs: Vec<f64> = (0..=12).map(|i| i as f64 * 0.5).collect();
    println!("packet loss, 64-byte payloads, {frames} frames a point, seed 11; Eb/N0 per information bit");
    print!("{:24}", "Eb/N0 dB");
    for db in &dbs {
        print!(" {db:>6}");
    }
    println!();
    print!("{:24}", "ideal code, BPSK");
    for &db in &dbs {
        print!(" {:>6.4}", ideal_loss(db, 4000, 11));
    }
    println!();
    for m in [Modulation::Bpsk, Modulation::Qpsk] {
        for (code, sync) in [
            (PayloadCode::None, 3),
            (PayloadCode::Light, 3),
            (PayloadCode::Light, 5),
        ] {
            let opts = BerOptions {
                frame: FrameFormat::V1,
                code,
                rx: RxConfig {
                    sync_max_errors: sync,
                    ..RxConfig::default()
                },
                ..BerOptions::default()
            };
            print!(
                "{:24}",
                format!("{} {} sync<={sync}", m.name(), code.name())
            );
            let mut found_row = String::new();
            for &db in &dbs {
                let p = run_found(m, db, frames, 11, &opts).unwrap();
                print!(" {:>6.4}", 1.0 - p.crc_ok as f64 / frames as f64);
                found_row += &format!(" {:>6.4}", 1.0 - p.found as f64 / frames as f64);
            }
            println!();
            println!("{:24}{found_row}", "  of which not found");
        }
    }
    // uncoded reaches 1 % near 9 dB; 2-FSK (noncoherent) needs more
    let tail: Vec<f64> = (13..=20).map(|i| i as f64 * 0.5).collect();
    for m in [Modulation::Bpsk, Modulation::Qpsk] {
        let opts = BerOptions {
            frame: FrameFormat::V1,
            ..BerOptions::default()
        };
        print!("{:24}", format!("{} none 6.5..10", m.name()));
        for &db in &tail {
            let p = run_found(m, db, frames, 11, &opts).unwrap();
            print!(" {:>6.4}", 1.0 - p.crc_ok as f64 / frames as f64);
        }
        println!();
    }
    for code in [PayloadCode::None, PayloadCode::Light] {
        let opts = BerOptions {
            frame: FrameFormat::V1,
            code,
            ..BerOptions::default()
        };
        print!("{:24}", format!("fsk2 {} 7..14", code.name()));
        for db in 7..=14 {
            let p = run_found(Modulation::Fsk2, db as f64, frames, 11, &opts).unwrap();
            print!(" {:>6.4}", 1.0 - p.crc_ok as f64 / frames as f64);
        }
        println!();
    }
}

/// Deep fades for the docs: 64-byte light frames at 6 dB Eb/N0 with the body faded 20 dB (amplitude
/// 0.1) over `L` symbols at a random place, 200 frames per length. `cargo test --release -p
/// oasa-waveform --test fec_light fade_table -- --ignored --nocapture`.
#[test]
#[ignore]
fn fade_table() {
    let lens = [0usize, 16, 32, 48, 64, 96, 128, 192, 256];
    println!("frames delivered of 200, 6 dB Eb/N0, body faded 20 dB over L symbols");
    print!("{:12}", "L");
    for l in lens {
        print!(" {l:>4}");
    }
    println!();
    for m in [Modulation::Bpsk, Modulation::Qpsk] {
        let md = light(m);
        let sps = md.sps();
        let body_start = (63 + 16 + 152) * sps;
        let body_syms = md.frame_symbols(64) - 63 - 16 - 152;
        let n0 = n0_for_ebn0(6.0, md.energy_per_bit());
        print!("{:12}", m.name());
        for l in lens {
            let mut ch = Channel::new(50 + l as u64);
            let mut ok = 0;
            for _ in 0..200 {
                let payload = ch.rng().bytes(64);
                let mut tx = md.modulate(&payload).unwrap();
                let at = ch.rng().below((body_syms - l) as u64 + 1) as usize;
                for s in &mut tx[body_start + at * sps..body_start + (at + l) * sps] {
                    *s = *s * 0.1;
                }
                let imp = Impairments {
                    n0,
                    phase_rad: ch.rng().uniform() * 2.0 * PI,
                    lead: 40,
                    tail: 40,
                    ..Default::default()
                };
                ok += md
                    .demodulate(&ch.apply(&tx, &imp))
                    .iter()
                    .any(|f| f.crc_ok && f.payload == payload) as usize;
            }
            print!(" {ok:>4}");
        }
        println!();
    }
}
