//! `ptyrelay`: join two serial devices with a pty pair for tests and scripts.
//! A test tool; it is not part of any radio path.
//!
//!   ptyrelay --link-a /tmp/a --link-b /tmp/b [--baud N] [--drop RATE] [--garble RATE]
//!            [--seed N] [--capture-a FILE] [--capture-b FILE] [--duration SECS]
//!
//! Makes symlinks at the two --link paths; give them to `--device` of two
//! bridges. Prints `ready` when the links exist. Stops on SIGTERM or SIGINT.
// The only unsafe code is the signal binding below.
#![deny(unsafe_code)]

use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use oasa_radio::relay::{Relay, RelayConfig};

static STOP: AtomicBool = AtomicBool::new(false);

mod signals {
    #![allow(unsafe_code)]
    use super::STOP;
    use std::sync::atomic::Ordering;

    extern "C" fn on_signal(_: i32) {
        STOP.store(true, Ordering::SeqCst);
    }

    extern "C" {
        fn signal(signum: i32, handler: extern "C" fn(i32)) -> usize;
    }

    pub fn install() {
        // SAFETY: `signal` is the C library function; the handler is
        // async-signal-safe (one atomic store).
        unsafe {
            signal(2, on_signal);
            signal(15, on_signal);
        }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut cfg = RelayConfig::default();
    let (mut link_a, mut link_b, mut duration) = (None, None, None);
    let mut it = args.iter();
    while let Some(k) = it.next() {
        let v = it.next().ok_or(format!("{k} needs a value"))?;
        match k.as_str() {
            "--link-a" => link_a = Some(PathBuf::from(v)),
            "--link-b" => link_b = Some(PathBuf::from(v)),
            "--baud" => cfg.baud = Some(v.parse()?),
            "--drop" => cfg.drop_rate = v.parse()?,
            "--garble" => cfg.garble_rate = v.parse()?,
            "--seed" => cfg.seed = v.parse()?,
            "--capture-a" => cfg.capture_a = Some(PathBuf::from(v)),
            "--capture-b" => cfg.capture_b = Some(PathBuf::from(v)),
            "--duration" => duration = Some(Duration::from_secs(v.parse()?)),
            _ => return Err(format!("unknown option {k}").into()),
        }
    }
    let (link_a, link_b) = (
        link_a.ok_or("missing --link-a")?,
        link_b.ok_or("missing --link-b")?,
    );
    let relay = Relay::start(cfg)?;
    for (link, target) in [(&link_a, &relay.path_a), (&link_b, &relay.path_b)] {
        let _ = std::fs::remove_file(link);
        std::os::unix::fs::symlink(target, link)?;
    }
    signals::install();
    println!("ready");
    let end = duration.map(|d| Instant::now() + d);
    while !STOP.load(Ordering::SeqCst) && end.is_none_or(|e| Instant::now() < e) {
        std::thread::sleep(Duration::from_millis(50));
    }
    println!(
        "{{\"event\":\"relay_stats\",\"a_to_b_bytes\":{},\"b_to_a_bytes\":{}}}",
        relay.a_to_b.bytes.load(Ordering::SeqCst),
        relay.b_to_a.bytes.load(Ordering::SeqCst)
    );
    let _ = std::fs::remove_file(link_a);
    let _ = std::fs::remove_file(link_b);
    Ok(())
}
