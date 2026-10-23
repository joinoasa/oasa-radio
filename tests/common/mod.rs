//! Test harness: a bridge running in a thread, with a UDP socket standing in
//! for oasa-link on the other side of it.
#![allow(dead_code)]

use std::io::Write;
use std::net::{SocketAddr, UdpSocket};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use oasa_radio::adapters::{SerialConfig, SerialRadio};
use oasa_radio::bridge::{self, BridgeConfig, BridgeStats};
use oasa_radio::radio::Radio;

#[derive(Clone, Default)]
pub struct SharedBuf(pub Arc<Mutex<Vec<u8>>>);

impl Write for SharedBuf {
    fn write(&mut self, b: &[u8]) -> std::io::Result<usize> {
        self.0.lock().unwrap().extend_from_slice(b);
        Ok(b.len())
    }
    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

impl SharedBuf {
    pub fn text(&self) -> String {
        String::from_utf8_lossy(&self.0.lock().unwrap()).into_owned()
    }
}

pub fn free_port() -> u16 {
    UdpSocket::bind("127.0.0.1:0")
        .unwrap()
        .local_addr()
        .unwrap()
        .port()
}

/// A bridge plus the "oasa-link" socket that talks to it.
pub struct Harness {
    /// Stands in for oasa-link: bound at the bridge's `--udp-send`.
    pub link: UdpSocket,
    pub listen: SocketAddr,
    stop: Arc<AtomicBool>,
    log: SharedBuf,
    handle: Option<JoinHandle<Result<BridgeStats, String>>>,
}

impl Harness {
    pub fn start(radio: Box<dyn Radio + Send>) -> Self {
        let link = UdpSocket::bind("127.0.0.1:0").unwrap();
        let listen: SocketAddr = format!("127.0.0.1:{}", free_port()).parse().unwrap();
        Self::start_with(radio, link, listen, None)
    }

    pub fn start_with(
        radio: Box<dyn Radio + Send>,
        link: UdpSocket,
        listen: SocketAddr,
        udp_send: Option<SocketAddr>,
    ) -> Self {
        let cfg = BridgeConfig {
            udp_listen: listen,
            udp_send: udp_send.unwrap_or_else(|| link.local_addr().unwrap()),
            allow_remote_udp: false,
            duration: None,
        };
        let stop = Arc::new(AtomicBool::new(false));
        let log = SharedBuf::default();
        let (s, mut l) = (stop.clone(), log.clone());
        let mut radio = radio;
        let handle = std::thread::spawn(move || {
            bridge::run(radio.as_mut(), &cfg, &s, &mut l).map_err(|e| e.to_string())
        });
        // Wait until the bridge is listening.
        let t = Instant::now();
        while !log.text().contains("\"event\":\"start\"") {
            assert!(t.elapsed() < Duration::from_secs(5), "bridge did not start");
            std::thread::sleep(Duration::from_millis(5));
        }
        Self {
            link,
            listen,
            stop,
            log,
            handle: Some(handle),
        }
    }

    /// oasa-link sends a packet into the bridge.
    pub fn send(&self, p: &[u8]) {
        self.link.send_to(p, self.listen).unwrap();
    }

    /// The next packet the bridge delivers to oasa-link.
    pub fn recv(&self, timeout: Duration) -> Option<Vec<u8>> {
        self.link.set_read_timeout(Some(timeout)).unwrap();
        let mut b = vec![0u8; 65_536];
        self.link
            .recv_from(&mut b)
            .ok()
            .map(|(n, _)| b[..n].to_vec())
    }

    /// Everything that arrives until `quiet` passes with nothing, or `max` packets.
    pub fn recv_all(&self, max: usize, total: Duration, quiet: Duration) -> Vec<Vec<u8>> {
        let end = Instant::now() + total;
        let mut got = Vec::new();
        while got.len() < max && Instant::now() < end {
            match self.recv(
                quiet
                    .min(end - Instant::now())
                    .max(Duration::from_millis(1)),
            ) {
                Some(p) => got.push(p),
                None => break,
            }
        }
        got
    }

    pub fn log_text(&self) -> String {
        self.log.text()
    }

    /// Stop the bridge. Returns its result and its log.
    pub fn finish(mut self) -> (Result<BridgeStats, String>, String) {
        self.stop.store(true, Ordering::SeqCst);
        let r = self.handle.take().unwrap().join().unwrap();
        (r, self.log.text())
    }

    /// Wait for the bridge to stop by itself (a radio failure).
    pub fn join(mut self) -> (Result<BridgeStats, String>, String) {
        let r = self.handle.take().unwrap().join().unwrap();
        (r, self.log.text())
    }
}

pub fn serial(path: &std::path::Path, baud: u32, mtu: usize) -> Box<dyn Radio + Send> {
    Box::new(
        SerialRadio::open(SerialConfig {
            device: path.display().to_string(),
            baud,
            mtu,
        })
        .unwrap(),
    )
}

/// Every log line must be a JSON object with `ts` and `event`, and no packet
/// bytes: neither raw nor as hex or decimal lists of the marker.
pub fn check_log(log: &str, forbidden: &[&[u8]]) -> Vec<String> {
    let mut events = Vec::new();
    for line in log.lines() {
        assert!(
            line.starts_with("{\"ts\":\"") && line.ends_with('}'),
            "{line}"
        );
        let after = line.split("\"event\":\"").nth(1).expect(line);
        events.push(after.split('"').next().unwrap().to_string());
        for f in forbidden {
            let hex: String = f.iter().map(|b| format!("{b:02x}")).collect();
            assert!(!line.contains(&hex) && !line.as_bytes().windows(f.len()).any(|w| w == *f));
        }
    }
    events
}

/// Number after `"key":` in a stats line.
pub fn stat(log: &str, key: &str) -> u64 {
    let line = log
        .lines()
        .rev()
        .find(|l| l.contains("\"event\":\"stats\""))
        .expect("no stats line");
    let needle = format!("\"{key}\":");
    let rest = line
        .split(&needle)
        .nth(1)
        .unwrap_or_else(|| panic!("{key}: {line}"));
    rest.chars()
        .take_while(char::is_ascii_digit)
        .collect::<String>()
        .parse()
        .unwrap()
}
