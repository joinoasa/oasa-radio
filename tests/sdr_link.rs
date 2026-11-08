//! Two `sdr` radios joined by two simulated cables (FIFOs, in real time): packets cross both ways,
//! nothing is invented, a frame with a bad CRC is not delivered, and the delay is a few frames.

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use nix::sys::stat::Mode;
use nix::unistd::mkfifo;
use oasa_radio::adapters::{SdrConfig, SdrRadio};
use oasa_radio::cable::{self, CableConfig};
use oasa_radio::radio::Radio;

fn fifo(dir: &Path, name: &str) -> PathBuf {
    let p = dir.join(name);
    mkfifo(&p, Mode::S_IRUSR | Mode::S_IWUSR).unwrap();
    p
}

fn cable_thread(from: PathBuf, to: PathBuf, cfo_hz: f64, seed: u64) {
    std::thread::spawn(move || {
        let mut i = std::fs::File::open(&from).unwrap();
        let mut o = std::fs::OpenOptions::new().write(true).open(&to).unwrap();
        let cfg = CableConfig {
            gain_db: -20.0,
            noise_dbfs: -62.0,
            cfo_hz,
            sample_rate: 200e3,
            seed,
            ..CableConfig::default()
        };
        let _ = cable::run(&cfg, &mut i, &mut o);
    });
}

fn radio(tx: PathBuf, rx: PathBuf, cfo_hz: f64, log: PathBuf) -> SdrRadio {
    let mut c = SdrConfig::new(tx, rx);
    // a fifth of the bench's rates, so a debug build keeps up
    c.sample_rate = 200e3;
    c.symbol_rate = 25e3;
    c.if_hz = 40e3;
    c.cfo_hz = cfo_hz;
    c.block = 2048;
    c.log = Some(log);
    SdrRadio::open(c).unwrap()
}

fn tmpdir() -> PathBuf {
    static N: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
    let d = std::env::temp_dir().join(format!(
        "oasa-sdr-test-{}-{}",
        std::process::id(),
        N.fetch_add(1, Ordering::SeqCst)
    ));
    std::fs::create_dir_all(&d).unwrap();
    d
}

#[test]
fn packets_cross_both_ways_in_real_time() {
    let d = tmpdir();
    let (a_tx, a_rx, b_tx, b_rx) = (
        fifo(&d, "a.tx"),
        fifo(&d, "a.rx"),
        fifo(&d, "b.tx"),
        fifo(&d, "b.rx"),
    );
    // the radios' clocks differ: B hears A 300 Hz high, A hears B 300 Hz low (0.012 cycles/symbol)
    cable_thread(a_tx.clone(), b_rx.clone(), 300.0, 1);
    cable_thread(b_tx.clone(), a_rx.clone(), -300.0, 2);
    let mut a = radio(a_tx, a_rx, 0.0, d.join("a.jsonl"));
    let mut b = radio(b_tx, b_rx, 0.0, d.join("b.jsonl"));

    let n = 12;
    let a2b: Vec<Vec<u8>> = (0..n)
        .map(|i| vec![0xa0 | (i as u8 & 0xf); 40 + 10 * i])
        .collect();
    let b2a: Vec<Vec<u8>> = (0..n)
        .map(|i| vec![0xb0 | (i as u8 & 0xf); 150 - 5 * i])
        .collect();
    let (mut got_a, mut got_b) = (Vec::new(), Vec::new());
    let mut worst = Duration::ZERO;
    for i in 0..n {
        let t = Instant::now();
        a.send(&a2b[i]).unwrap();
        b.send(&b2a[i]).unwrap();
        let (mut ok_a, mut ok_b) = (false, false);
        while t.elapsed() < Duration::from_secs(3) && !(ok_a && ok_b) {
            if let Some(p) = b.recv(Duration::from_millis(2)).unwrap() {
                ok_b = true;
                got_b.push(p);
            }
            if let Some(p) = a.recv(Duration::from_millis(2)).unwrap() {
                ok_a = true;
                got_a.push(p);
            }
        }
        worst = worst.max(t.elapsed());
    }
    assert_eq!(got_b, a2b, "A to B");
    assert_eq!(got_a, b2a, "B to A");
    // 150 bytes of QPSK at 25 kbaud is about 26 ms of air; the receiver decides a frame once the
    // longest frame (200 bytes) past its start has arrived, plus a block of 10 ms.
    assert!(
        worst < Duration::from_millis(600),
        "worst delivery {worst:?}"
    );
    assert_eq!(a.counters().drop_total() + b.counters().drop_total(), 0);
    // the offset tracker found the clocks
    let log = std::fs::read_to_string(d.join("b.jsonl")).unwrap();
    let last = log.lines().rfind(|l| l.contains("\"rx_frame\"")).unwrap();
    let cfo: f64 = last
        .split("\"cfo_hz\":")
        .nth(1)
        .unwrap()
        .split(',')
        .next()
        .unwrap()
        .parse()
        .unwrap();
    assert!((cfo - 300.0).abs() < 30.0, "B tracked {cfo} Hz");
    drop(a);
    drop(b);
    let _ = std::fs::remove_dir_all(&d);
}

#[test]
fn a_stopped_reader_is_a_radio_error() {
    let d = tmpdir();
    let (tx, rx) = (fifo(&d, "tx"), fifo(&d, "rx"));
    let stop = Arc::new(AtomicBool::new(false));
    // a reader that takes a little of the TX stream and then goes away
    let tx2 = tx.clone();
    let reader = std::thread::spawn(move || {
        use std::io::Read;
        let mut f = std::fs::File::open(&tx2).unwrap();
        let mut b = vec![0u8; 8 * 1000];
        f.read_exact(&mut b).unwrap();
    });
    // a writer for the RX stream that stays open
    let rx2 = rx.clone();
    let s2 = stop.clone();
    std::thread::spawn(move || {
        let _f = std::fs::OpenOptions::new().write(true).open(&rx2).unwrap();
        while !s2.load(Ordering::SeqCst) {
            std::thread::sleep(Duration::from_millis(10));
        }
    });
    let mut c = SdrConfig::new(tx, rx);
    c.sample_rate = 200e3;
    c.symbol_rate = 25e3;
    c.if_hz = 40e3;
    let mut r = SdrRadio::open(c).unwrap();
    reader.join().unwrap();
    let t = Instant::now();
    let mut failed = false;
    while t.elapsed() < Duration::from_secs(3) {
        if r.recv(Duration::from_millis(10)).is_err() || r.send(&[1, 2, 3]).is_err() {
            failed = true;
            break;
        }
    }
    stop.store(true, Ordering::SeqCst);
    assert!(
        failed,
        "the TX stream's reader left, so the radio must fail"
    );
    let _ = std::fs::remove_dir_all(&d);
}

#[test]
fn burst_mode_writes_one_decodable_record_per_packet() {
    use oasa_radio::adapters::sdr::TxMode;
    use oasa_waveform::{Complex32, FrameFormat, Modem, ModemConfig, Modulation, RxConfig};
    let d = tmpdir();
    let tx = d.join("tx.bin");
    std::fs::File::create(&tx).unwrap();
    // an RX writer that stays open, so the radio does not fail
    let rx = fifo(&d, "rx");
    let stop = Arc::new(AtomicBool::new(false));
    let (rx2, s2) = (rx.clone(), stop.clone());
    std::thread::spawn(move || {
        let _f = std::fs::OpenOptions::new().write(true).open(&rx2).unwrap();
        while !s2.load(Ordering::SeqCst) {
            std::thread::sleep(Duration::from_millis(10));
        }
    });
    let mut c = SdrConfig::new(tx.clone(), rx);
    c.tx_mode = TxMode::Burst;
    let log = d.join("sdr.jsonl");
    c.log = Some(log.clone());
    let fs = c.sample_rate;
    let if_hz = c.if_hz;
    let mut r = SdrRadio::open(c).unwrap();
    let packets: Vec<Vec<u8>> = vec![vec![7; 119], (0..200).map(|i| i as u8).collect(), vec![1]];
    for p in &packets {
        r.send(p).unwrap();
    }
    // all three records written, and nothing else (no zeros streamed between frames)
    let t = Instant::now();
    let want: usize = {
        let md = Modem::new(ModemConfig {
            frame: FrameFormat::V1,
            ..ModemConfig::new(Modulation::Qpsk)
        })
        .unwrap();
        packets
            .iter()
            .map(|p| 4 + 8 * (md.modulate(p).unwrap().len() + 50 * 8))
            .sum()
    };
    while std::fs::metadata(&tx).unwrap().len() < want as u64
        && t.elapsed() < Duration::from_secs(3)
    {
        std::thread::sleep(Duration::from_millis(10));
    }
    std::thread::sleep(Duration::from_millis(50));
    let bytes = std::fs::read(&tx).unwrap();
    assert_eq!(bytes.len(), want, "three records, nothing between them");
    let md = Modem::new(ModemConfig {
        frame: FrameFormat::V1,
        ..ModemConfig::new(Modulation::Qpsk)
    })
    .unwrap();
    let rxc = RxConfig {
        cfo_search: 0.03,
        ..RxConfig::default()
    };
    let (mut at, mut phase) = (0usize, 0.0f64);
    for p in &packets {
        let n = u32::from_le_bytes(bytes[at..at + 4].try_into().unwrap()) as usize;
        at += 4;
        // shift the record back from the IF and decode it alone
        let w = 2.0 * std::f64::consts::PI * if_hz / fs;
        let x: Vec<Complex32> = bytes[at..at + 8 * n]
            .chunks(8)
            .map(|b| {
                let v = Complex32::new(
                    f32::from_le_bytes(b[..4].try_into().unwrap()),
                    f32::from_le_bytes(b[4..].try_into().unwrap()),
                );
                let (s, c) = (-phase).sin_cos();
                phase += w;
                v * Complex32::new(c as f32, s as f32)
            })
            .collect();
        at += 8 * n;
        let frames = md.demodulate_with(&x, &rxc);
        assert_eq!(frames.len(), 1);
        assert!(frames[0].crc_ok);
        assert_eq!(&frames[0].payload, p);
    }
    stop.store(true, Ordering::SeqCst);
    drop(r);
    // the log thread wrote every line, the last one included (dropping the radio waits for it)
    let text = std::fs::read_to_string(&log).unwrap();
    assert_eq!(text.matches("\"event\":\"tx_air\"").count(), 3, "{text}");
    assert!(
        text.lines()
            .last()
            .unwrap()
            .contains("\"event\":\"sdr_stats\""),
        "{text}"
    );
    assert!(text.contains("\"log_dropped\":0"), "{text}");
    let _ = std::fs::remove_dir_all(&d);
}
