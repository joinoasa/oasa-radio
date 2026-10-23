//! Test tools: pseudo-terminal pairs and a serial line relay that can drop and
//! garble bytes, limit speed and capture what crosses it. Linux only. Used by
//! the tests, `examples/ptyrelay.rs` and `scripts/serial-e2e.sh`. Not part of
//! any radio path.
//!
//! ```text
//!   bridge A --- /dev/pts/x ==[ relay ]== /dev/pts/y --- bridge B
//! ```

use std::fs::File;
use std::io::{Read, Write};
use std::os::fd::{AsFd, AsRawFd, OwnedFd};
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use nix::poll::{poll, PollFd, PollFlags, PollTimeout};
use nix::sys::termios::BaudRate;

use crate::adapters::serial::{baud_rate, set_raw};
pub use crate::rng::Rng;

/// A pseudo-terminal. We keep the slave open so the master never sees a hangup
/// when a bridge opens and closes the same path.
pub struct Pty {
    pub master: OwnedFd,
    _slave: OwnedFd,
    /// The slave's path, for example `/dev/pts/7`. Give this to `--device`.
    pub path: PathBuf,
}

impl Pty {
    pub fn open() -> std::io::Result<Self> {
        let p = nix::pty::openpty(None, None).map_err(std::io::Error::from)?;
        set_raw(&p.slave, BaudRate::B57600).map_err(|e| std::io::Error::other(e.to_string()))?;
        let path = std::fs::read_link(format!("/proc/self/fd/{}", p.slave.as_raw_fd()))?;
        Ok(Self {
            master: p.master,
            _slave: p.slave,
            path,
        })
    }
}

/// How the relay treats the line. Applied to each direction on its own.
#[derive(Debug, Clone)]
pub struct RelayConfig {
    /// Chance that a byte is lost.
    pub drop_rate: f64,
    /// Chance that a byte has one bit flipped.
    pub garble_rate: f64,
    /// Line speed limit in baud (10 bits per byte); `None` = as fast as it goes.
    pub baud: Option<u32>,
    pub seed: u64,
    /// Write every byte A sent (before any damage) to this file.
    pub capture_a: Option<PathBuf>,
    /// Same for B.
    pub capture_b: Option<PathBuf>,
}

impl Default for RelayConfig {
    fn default() -> Self {
        Self {
            drop_rate: 0.0,
            garble_rate: 0.0,
            baud: None,
            seed: 1,
            capture_a: None,
            capture_b: None,
        }
    }
}

/// Byte counts for one direction.
#[derive(Debug, Default)]
pub struct DirStats {
    pub bytes: AtomicU64,
    pub dropped: AtomicU64,
    pub garbled: AtomicU64,
}

/// Two ptys joined by copy threads. Stops when dropped.
pub struct Relay {
    /// Give to bridge A's `--device`.
    pub path_a: PathBuf,
    /// Give to bridge B's `--device`.
    pub path_b: PathBuf,
    pub a_to_b: Arc<DirStats>,
    pub b_to_a: Arc<DirStats>,
    stop: Arc<AtomicBool>,
    threads: Vec<JoinHandle<()>>,
    // Keep the ptys (and their slave ends) alive.
    _a: Arc<Pty>,
    _b: Arc<Pty>,
}

impl Relay {
    pub fn start(cfg: RelayConfig) -> std::io::Result<Self> {
        if let Some(b) = cfg.baud {
            if b == 0 || baud_rate(b).is_none() {
                return Err(std::io::Error::other(format!("baud {b} is not supported")));
            }
        }
        let a = Arc::new(Pty::open()?);
        let b = Arc::new(Pty::open()?);
        let stop = Arc::new(AtomicBool::new(false));
        let (ab, ba) = (Arc::new(DirStats::default()), Arc::new(DirStats::default()));
        let mut threads = Vec::new();
        let dirs = [
            (&a, &b, &cfg.capture_a, &ab, cfg.seed),
            (&b, &a, &cfg.capture_b, &ba, cfg.seed ^ 0x5555),
        ];
        for (from, to, cap, stats, seed) in dirs {
            let rd = File::from(from.master.try_clone()?);
            let wr = File::from(to.master.try_clone()?);
            let cap = cap.as_ref().map(File::create).transpose()?;
            let (stop, stats, c) = (stop.clone(), stats.clone(), cfg.clone());
            threads.push(std::thread::spawn(move || {
                copy_loop(rd, wr, cap, stats, stop, c, seed)
            }));
        }
        Ok(Self {
            path_a: a.path.clone(),
            path_b: b.path.clone(),
            a_to_b: ab,
            b_to_a: ba,
            stop,
            threads,
            _a: a,
            _b: b,
        })
    }

    pub fn stop(&mut self) {
        self.stop.store(true, Ordering::SeqCst);
        for t in self.threads.drain(..) {
            let _ = t.join();
        }
    }
}

impl Drop for Relay {
    fn drop(&mut self) {
        self.stop();
    }
}

fn copy_loop(
    mut rd: File,
    mut wr: File,
    mut cap: Option<File>,
    stats: Arc<DirStats>,
    stop: Arc<AtomicBool>,
    cfg: RelayConfig,
    seed: u64,
) {
    let mut rng = Rng::new(seed);
    let mut buf = [0u8; 256];
    let mut out = Vec::with_capacity(256);
    // The time the line is busy until: 10 bit times per byte at `baud`.
    let mut line_free = Instant::now();
    while !stop.load(Ordering::SeqCst) {
        let mut fds = [PollFd::new(rd.as_fd(), PollFlags::POLLIN)];
        match poll(&mut fds, PollTimeout::from(20u8)) {
            Ok(n) if n > 0 => {}
            _ => continue,
        }
        let n = match rd.read(&mut buf) {
            Ok(n) if n > 0 => n,
            _ => continue,
        };
        stats.bytes.fetch_add(n as u64, Ordering::SeqCst);
        if let Some(c) = cap.as_mut() {
            let _ = c.write_all(&buf[..n]);
        }
        out.clear();
        for &b in &buf[..n] {
            if rng.chance(cfg.drop_rate) {
                stats.dropped.fetch_add(1, Ordering::SeqCst);
                continue;
            }
            if rng.chance(cfg.garble_rate) {
                stats.garbled.fetch_add(1, Ordering::SeqCst);
                out.push(b ^ (1 << rng.below(8)));
            } else {
                out.push(b);
            }
        }
        if let Some(baud) = cfg.baud {
            // Pace in small pieces so the receiver sees a steady trickle.
            for piece in out.chunks(32) {
                let now = Instant::now();
                if line_free > now {
                    std::thread::sleep(line_free - now);
                }
                line_free = Instant::now().max(line_free)
                    + Duration::from_secs_f64(piece.len() as f64 * 10.0 / f64::from(baud));
                if wr.write_all(piece).is_err() {
                    return;
                }
            }
        } else if !out.is_empty() && wr.write_all(&out).is_err() {
            return;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn read_some(f: &mut File, want: usize, within: Duration) -> Vec<u8> {
        let mut got = Vec::new();
        let end = Instant::now() + within;
        let mut b = [0u8; 512];
        while got.len() < want && Instant::now() < end {
            let mut fds = [PollFd::new(f.as_fd(), PollFlags::POLLIN)];
            if poll(&mut fds, PollTimeout::from(20u8)).unwrap_or(0) > 0 {
                if let Ok(n) = f.read(&mut b) {
                    got.extend_from_slice(&b[..n]);
                }
            }
        }
        got
    }

    #[test]
    fn clean_relay_passes_all_bytes_both_ways_unchanged() {
        let relay = Relay::start(RelayConfig::default()).unwrap();
        let mut a = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .open(&relay.path_a)
            .unwrap();
        let mut b = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .open(&relay.path_b)
            .unwrap();
        let data: Vec<u8> = (0..=255u8).chain(0..=255u8).collect();
        a.write_all(&data).unwrap();
        assert_eq!(read_some(&mut b, data.len(), Duration::from_secs(3)), data);
        b.write_all(&data[..100]).unwrap();
        assert_eq!(read_some(&mut a, 100, Duration::from_secs(3)), data[..100]);
    }

    #[test]
    fn faults_and_speed_limit_work() {
        let relay = Relay::start(RelayConfig {
            drop_rate: 0.1,
            garble_rate: 0.1,
            baud: Some(57_600),
            seed: 3,
            ..RelayConfig::default()
        })
        .unwrap();
        let mut a = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .open(&relay.path_a)
            .unwrap();
        let mut b = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .open(&relay.path_b)
            .unwrap();
        let data = vec![0x55u8; 2880];
        let t = Instant::now();
        a.write_all(&data).unwrap();
        let got = read_some(&mut b, data.len(), Duration::from_secs(2));
        // 2880 bytes at 57600 baud is 0.5 s.
        assert!(
            t.elapsed() >= Duration::from_millis(400),
            "{:?}",
            t.elapsed()
        );
        let dropped = relay.a_to_b.dropped.load(Ordering::SeqCst) as usize;
        assert!((150..450).contains(&dropped), "dropped {dropped}");
        assert_eq!(got.len(), data.len() - dropped);
        let changed = got.iter().filter(|b| **b != 0x55).count();
        assert_eq!(changed as u64, relay.a_to_b.garbled.load(Ordering::SeqCst));
    }

    #[test]
    fn capture_records_what_was_sent() {
        let dir = std::env::temp_dir().join(format!("oasa-radio-cap-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let cap = dir.join("a.bin");
        let relay = Relay::start(RelayConfig {
            drop_rate: 1.0,
            capture_a: Some(cap.clone()),
            ..RelayConfig::default()
        })
        .unwrap();
        let mut a = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .open(&relay.path_a)
            .unwrap();
        a.write_all(b"hello line").unwrap();
        let end = Instant::now() + Duration::from_secs(3);
        while std::fs::metadata(&cap).map(|m| m.len()).unwrap_or(0) < 10 && Instant::now() < end {
            std::thread::sleep(Duration::from_millis(20));
        }
        // Captured even though the relay dropped every byte after it.
        assert_eq!(std::fs::read(&cap).unwrap(), b"hello line");
        drop(relay);
        let _ = std::fs::remove_dir_all(dir);
    }
}
