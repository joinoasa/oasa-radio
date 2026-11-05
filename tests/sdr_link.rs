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
