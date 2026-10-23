//! The bridge: oasa-link's UDP on one side, a radio on the other.
//!
//! ```text
//!   oasa-link --UDP--> [udp-listen  bridge  radio] ---> air ---> (peer)
//!   oasa-link <--UDP-- [udp-send            radio] <--- air <--- (peer)
//! ```
//!
//! The bridge copies packets and never looks inside them. It logs sizes and
//! counts only. Packets to oasa-link are sent from the `udp-listen` socket, so
//! oasa-link sees one stable address for its peer (it learns the peer address
//! from packets that authenticate).

use std::collections::BTreeMap;
use std::io::Write;
use std::net::{SocketAddr, UdpSocket};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant, SystemTime};

use crate::log::{json_str, rfc3339, DropLog};
use crate::radio::{Radio, RadioError};

/// How long one pass waits for the radio. This is also the most a packet
/// from oasa-link waits before it is sent.
const TICK: Duration = Duration::from_millis(2);
/// Packets taken from the UDP socket per pass.
const UDP_BATCH: usize = 64;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BridgeConfig {
    /// Where oasa-link sends. Loopback unless `allow_remote_udp`.
    pub udp_listen: SocketAddr,
    /// Where radio packets go (oasa-link's `--bind`).
    pub udp_send: SocketAddr,
    /// Allow a non-loopback `udp_listen` and packets from any host. Anyone
    /// who can reach the port could then send on the radio.
    pub allow_remote_udp: bool,
    /// Stop after this long.
    pub duration: Option<Duration>,
}

/// Totals when the bridge stops. Counts and sizes only.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct BridgeStats {
    pub to_radio: u64,
    pub to_radio_bytes: u64,
    pub from_radio: u64,
    pub from_radio_bytes: u64,
    pub drops: BTreeMap<String, u64>,
}

/// Check settings that don't need a socket.
pub fn check(cfg: &BridgeConfig) -> Result<(), String> {
    if !cfg.udp_listen.ip().is_loopback() && !cfg.allow_remote_udp {
        return Err(
            "--udp-listen must be a loopback address (127.0.0.1 or ::1); anyone who can reach \
             it could send on the radio. Use --allow-remote-udp to override"
                .into(),
        );
    }
    Ok(())
}

struct Out<'a, W: Write> {
    w: &'a mut W,
    drop_log: DropLog,
    drops: BTreeMap<String, u64>,
}

impl<W: Write> Out<'_, W> {
    fn line(&mut self, event: &str, fields: &str) {
        let _ = writeln!(
            self.w,
            "{{\"ts\":\"{}\",\"event\":\"{event}\"{fields}}}",
            rfc3339(SystemTime::now())
        );
        let _ = self.w.flush();
    }

    fn drop_one(&mut self, reason: &str, n: u64) {
        *self.drops.entry(reason.to_string()).or_insert(0) += n;
        for _ in 0..n {
            if self.drop_log.allow() {
                self.line("drop", &format!(",\"reason\":{}", json_str(reason)));
            }
        }
    }

    fn summary(&mut self, force: bool) {
        let mut n = self.drop_log.roll(Instant::now());
        if force {
            n += self.drop_log.take_suppressed();
        }
        if n > 0 {
            self.line("drops_suppressed", &format!(",\"count\":{n}"));
        }
    }
}

/// Run until `stop` is set, `duration` ends, or the radio fails. Logs JSON lines
/// to `log`; the last is `stats`. Returns the totals, or the radio's error
/// (the `stats` line is written first either way).
pub fn run<W: Write>(
    radio: &mut dyn Radio,
    cfg: &BridgeConfig,
    stop: &AtomicBool,
    log: &mut W,
) -> Result<BridgeStats, RadioError> {
    check(cfg).map_err(RadioError::Config)?;
    let sock = UdpSocket::bind(cfg.udp_listen)
        .map_err(|e| RadioError::Config(format!("--udp-listen {}: {e}", cfg.udp_listen)))?;
    sock.set_nonblocking(true)?;
    let start = Instant::now();
    let deadline = match cfg.duration {
        Some(d) => Some(
            start
                .checked_add(d)
                .ok_or_else(|| RadioError::Config("--duration is too large".into()))?,
        ),
        None => None,
    };
    let mut o = Out {
        w: log,
        drop_log: DropLog::new(start),
        drops: BTreeMap::new(),
    };
    let info = radio.info();
    let bitrate = info
        .bitrate
        .map(|b| format!(",\"bitrate\":{b}"))
        .unwrap_or_default();
    o.line(
        "start",
        &format!(
            ",\"radio\":\"{}\",\"target\":{},\"mtu\":{}{bitrate},\"udp_listen\":\"{}\",\"udp_send\":\"{}\"",
            info.kind,
            json_str(&info.target),
            info.mtu,
            sock.local_addr().unwrap_or(cfg.udp_listen),
            cfg.udp_send
        ),
    );

    let mut buf = vec![0u8; 65_536];
    let mut seen_drops = 0u64;
    let mut seen: BTreeMap<&'static str, u64> = BTreeMap::new();
    let mut failure = None;
    'main: while !stop.load(Ordering::SeqCst) && deadline.is_none_or(|d| Instant::now() < d) {
        // oasa-link to the radio.
        for _ in 0..UDP_BATCH {
            let (n, from) = match sock.recv_from(&mut buf) {
                Ok(x) => x,
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                // ICMP from an earlier send shows up here on some systems.
                Err(e) if e.kind() == std::io::ErrorKind::ConnectionReset => continue,
                Err(e) if e.kind() == std::io::ErrorKind::Interrupted => break,
                Err(e) => {
                    failure = Some(e.into());
                    break 'main;
                }
            };
            if !cfg.allow_remote_udp && from.ip() != cfg.udp_send.ip() {
                o.drop_one("udp_source", 1);
            } else if n == 0 {
                o.drop_one("empty", 1);
            } else if n > radio.mtu() {
                o.drop_one("too_large", 1);
            } else if let Err(e) = radio.send(&buf[..n]) {
                failure = Some(e);
                break 'main;
            }
        }
        // The radio to oasa-link.
        match radio.recv(TICK) {
            Ok(Some(p)) => match sock.send_to(&p, cfg.udp_send) {
                Ok(_) => {}
                // Nothing is listening yet (ICMP came back): a lost packet.
                Err(_) => o.drop_one("udp_send_failed", 1),
            },
            Ok(None) => {}
            Err(e) => {
                failure = Some(e);
                break 'main;
            }
        }
        // Drops inside the radio (bad frames and the like) get log lines too.
        let total = radio.counters().drop_total();
        if total != seen_drops {
            seen_drops = total;
            for (k, v) in radio.counters().drops.clone() {
                let before = seen.insert(k, v).unwrap_or(0);
                if v > before {
                    o.drop_one_logged_only(k, v - before);
                }
            }
        }
        o.summary(false);
    }
    // Radio drops that arrived in the last pass.
    for (k, v) in radio.counters().drops.clone() {
        let before = seen.insert(k, v).unwrap_or(0);
        if v > before {
            o.drop_one_logged_only(k, v - before);
        }
    }
    o.summary(true);

    let c = radio.counters();
    let mut drops: BTreeMap<String, u64> = c
        .drops
        .iter()
        .map(|(k, v)| ((*k).to_string(), *v))
        .collect();
    for (k, v) in &o.drops {
        *drops.entry(k.clone()).or_insert(0) += v;
    }
    // Radio drops were logged but must not be counted twice: o.drops only
    // holds the bridge's own.
    let stats = BridgeStats {
        to_radio: c.sent,
        to_radio_bytes: c.sent_bytes,
        from_radio: c.received,
        from_radio_bytes: c.received_bytes,
        drops,
    };
    let drops_json = stats
        .drops
        .iter()
        .map(|(k, v)| format!("{}:{v}", json_str(k)))
        .collect::<Vec<_>>()
        .join(",");
    if let Some(e) = &failure {
        o.line("error", &format!(",\"what\":{}", json_str(&e.to_string())));
    }
    o.line(
        "stats",
        &format!(
            ",\"to_radio\":{},\"to_radio_bytes\":{},\"from_radio\":{},\"from_radio_bytes\":{},\"drops\":{{{drops_json}}}",
            stats.to_radio, stats.to_radio_bytes, stats.from_radio, stats.from_radio_bytes
        ),
    );
    match failure {
        Some(e) => Err(e),
        None => Ok(stats),
    }
}

impl<W: Write> Out<'_, W> {
    /// Log `n` drops that the radio already counted (so they are not added to `drops` again).
    fn drop_one_logged_only(&mut self, reason: &str, n: u64) {
        for _ in 0..n {
            if self.drop_log.allow() {
                self.line("drop", &format!(",\"reason\":{}", json_str(reason)));
            }
        }
    }
}
