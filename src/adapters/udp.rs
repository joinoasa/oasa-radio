//! `udp`: plain UDP passthrough. For simulation (put the lab's simulated
//! channel between two bridges) and for Wi-Fi or Ethernet links.
//!
//! Packets go out as they are, one datagram each. Anyone can send to the bind
//! address; that is fine because oasa-link authenticates every packet.

use std::net::{SocketAddr, UdpSocket};
use std::time::Duration;

use crate::radio::{Counters, Radio, RadioError, RadioInfo};

/// Default and largest MTU (a UDP datagram over IPv4 is at most 65,507 bytes).
pub const DEFAULT_MTU: usize = 1400;
pub const MAX_MTU: usize = 65_507;

pub struct UdpRadio {
    sock: UdpSocket,
    remote: SocketAddr,
    bind: SocketAddr,
    mtu: usize,
    buf: Vec<u8>,
    counters: Counters,
}

impl UdpRadio {
    /// Listen on `bind`, send to `remote`.
    pub fn new(bind: SocketAddr, remote: SocketAddr, mtu: usize) -> Result<Self, RadioError> {
        if !(1..=MAX_MTU).contains(&mtu) {
            return Err(RadioError::Config(format!("--mtu must be 1 to {MAX_MTU}")));
        }
        let sock = UdpSocket::bind(bind)?;
        let bind = sock.local_addr()?;
        Ok(Self {
            sock,
            remote,
            bind,
            mtu,
            buf: vec![0; 65_536],
            counters: Counters::default(),
        })
    }

    /// The address actually bound (useful with port 0).
    pub fn local_addr(&self) -> SocketAddr {
        self.bind
    }
}

impl Radio for UdpRadio {
    fn info(&self) -> RadioInfo {
        RadioInfo {
            kind: "udp",
            target: format!("{} -> {}", self.bind, self.remote),
            mtu: self.mtu,
            bitrate: None,
        }
    }

    fn send(&mut self, packet: &[u8]) -> Result<(), RadioError> {
        if packet.is_empty() {
            self.counters.add_drop("empty", 1);
            return Ok(());
        }
        if packet.len() > self.mtu {
            self.counters.add_drop("too_large", 1);
            return Ok(());
        }
        match self.sock.send_to(packet, self.remote) {
            Ok(_) => {
                self.counters.sent += 1;
                self.counters.sent_bytes += packet.len() as u64;
                Ok(())
            }
            // Nobody listening yet (ICMP came back) or no buffer space: UDP is
            // best effort, so this is a lost packet, not a failed radio.
            Err(e)
                if matches!(
                    e.kind(),
                    std::io::ErrorKind::ConnectionRefused
                        | std::io::ErrorKind::ConnectionReset
                        | std::io::ErrorKind::WouldBlock
                ) =>
            {
                self.counters.add_drop("send_failed", 1);
                Ok(())
            }
            Err(e) => Err(e.into()),
        }
    }

    fn recv(&mut self, timeout: Duration) -> Result<Option<Vec<u8>>, RadioError> {
        // A zero timeout would mean "block forever" to the socket.
        self.sock
            .set_read_timeout(Some(timeout.max(Duration::from_millis(1))))?;
        loop {
            match self.sock.recv_from(&mut self.buf) {
                Ok((n, _)) => {
                    if n == 0 {
                        self.counters.add_drop("empty", 1);
                        return Ok(None);
                    }
                    if n > self.mtu {
                        self.counters.add_drop("oversize", 1);
                        return Ok(None);
                    }
                    self.counters.received += 1;
                    self.counters.received_bytes += n as u64;
                    return Ok(Some(self.buf[..n].to_vec()));
                }
                Err(e) => match e.kind() {
                    std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut => {
                        return Ok(None)
                    }
                    std::io::ErrorKind::Interrupted => return Ok(None),
                    // ICMP "port unreachable" from an earlier send; not about this recv.
                    std::io::ErrorKind::ConnectionReset | std::io::ErrorKind::ConnectionRefused => {
                        continue
                    }
                    _ => return Err(e.into()),
                },
            }
        }
    }

    fn counters(&self) -> &Counters {
        &self.counters
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pair(mtu: usize) -> (UdpRadio, UdpRadio) {
        let a_addr: SocketAddr = "127.0.0.1:0".parse().unwrap();
        let a = UdpSocket::bind(a_addr).unwrap();
        let b = UdpSocket::bind(a_addr).unwrap();
        let (pa, pb) = (a.local_addr().unwrap(), b.local_addr().unwrap());
        drop((a, b));
        (
            UdpRadio::new(pa, pb, mtu).unwrap(),
            UdpRadio::new(pb, pa, mtu).unwrap(),
        )
    }

    #[test]
    fn packets_pass_unchanged_in_both_directions() {
        let (mut a, mut b) = pair(200);
        let p: Vec<u8> = (0..=199).collect();
        a.send(&p).unwrap();
        assert_eq!(b.recv(Duration::from_secs(2)).unwrap(), Some(p.clone()));
        b.send(&p[..7]).unwrap();
        assert_eq!(
            a.recv(Duration::from_secs(2)).unwrap(),
            Some(p[..7].to_vec())
        );
        assert_eq!((a.counters().sent, a.counters().received), (1, 1));
        assert_eq!(a.counters().sent_bytes, 200);
        assert_eq!(a.counters().received_bytes, 7);
        assert_eq!(a.recv(Duration::from_millis(5)).unwrap(), None);
    }

    #[test]
    fn empty_and_oversize_are_dropped_and_counted() {
        let (mut a, mut b) = pair(50);
        a.send(&[]).unwrap();
        a.send(&[1; 51]).unwrap();
        assert_eq!(a.counters().sent, 0);
        assert_eq!(a.counters().drops["empty"], 1);
        assert_eq!(a.counters().drops["too_large"], 1);
        // A big datagram from outside is dropped on receive.
        let s = UdpSocket::bind("127.0.0.1:0").unwrap();
        s.send_to(&[2; 80], b.local_addr()).unwrap();
        assert_eq!(b.recv(Duration::from_millis(300)).unwrap(), None);
        assert_eq!(b.counters().drops["oversize"], 1);
    }

    #[test]
    fn mtu_is_checked() {
        let a: SocketAddr = "127.0.0.1:0".parse().unwrap();
        assert!(UdpRadio::new(a, a, 0).is_err());
        assert!(UdpRadio::new(a, a, MAX_MTU + 1).is_err());
    }
}
