//! Two serial bridges joined by a pseudo-terminal relay, with UDP sockets
//! standing in for the two oasa-link processes. No hardware, no socat.

mod common;

use std::net::UdpSocket;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use common::{check_log, serial, stat, Harness};
use oasa_radio::relay::{Relay, RelayConfig};
use oasa_radio::rng::Rng;

const MARK: &[u8] = b"OASA-MARKER-7f3a";

/// A packet with the marker, a sequence number and random filler (many zeros).
fn marked(rng: &mut Rng, dir: u8, i: u16, min: u64, span: u64) -> Vec<u8> {
    let len = (min + rng.below(span)) as usize;
    let mut p = MARK.to_vec();
    p.push(dir);
    p.extend_from_slice(&i.to_be_bytes());
    while p.len() < len {
        p.push(if rng.below(4) == 0 {
            0
        } else {
            rng.next_u64() as u8
        });
    }
    p.truncate(len.max(MARK.len() + 3));
    p
}

fn pair(cfg: RelayConfig, baud: u32, mtu: usize) -> (Relay, Harness, Harness) {
    let relay = Relay::start(cfg).unwrap();
    let a = Harness::start(serial(&relay.path_a, baud, mtu));
    let b = Harness::start(serial(&relay.path_b, baud, mtu));
    (relay, a, b)
}

#[test]
fn packets_cross_a_clean_line_both_ways_unchanged_and_in_order() {
    let (_relay, a, b) = pair(RelayConfig::default(), 115_200, 252);
    let mut rng = Rng::new(11);
    let to_b: Vec<_> = (0..150).map(|i| marked(&mut rng, 1, i, 20, 233)).collect();
    let to_a: Vec<_> = (0..150).map(|i| marked(&mut rng, 2, i, 20, 233)).collect();
    assert!(to_b.iter().all(|p| p.len() <= 252));
    for i in 0..150 {
        a.send(&to_b[i]);
        b.send(&to_a[i]);
        std::thread::sleep(Duration::from_millis(2));
    }
    let got_b = b.recv_all(150, Duration::from_secs(15), Duration::from_secs(2));
    let got_a = a.recv_all(150, Duration::from_secs(15), Duration::from_secs(2));
    assert_eq!(got_b, to_b);
    assert_eq!(got_a, to_a);

    let (sa, la) = a.finish();
    let (sb, lb) = b.finish();
    let (sa, sb) = (sa.unwrap(), sb.unwrap());
    assert_eq!((sa.to_radio, sa.from_radio), (150, 150));
    assert_eq!((sb.to_radio, sb.from_radio), (150, 150));
    assert!(sa.drops.is_empty() && sb.drops.is_empty(), "{:?}", sa.drops);
    // The log has sizes and counts, never packet bytes.
    for log in [&la, &lb] {
        let events = check_log(log, &[MARK, &to_b[0][..24]]);
        assert_eq!(events.first().map(String::as_str), Some("start"));
        assert_eq!(events.last().map(String::as_str), Some("stats"));
    }
    assert_eq!(stat(&la, "to_radio"), 150);
    assert_eq!(
        stat(&la, "to_radio_bytes"),
        to_b.iter().map(|p| p.len() as u64).sum::<u64>()
    );
}

#[test]
fn a_slow_noisy_line_loses_packets_but_never_changes_one() {
    // 57600 baud, bytes lost and bits flipped at random.
    let cfg = RelayConfig {
        drop_rate: 0.002,
        garble_rate: 0.002,
        baud: Some(57_600),
        seed: 21,
        ..RelayConfig::default()
    };
    let (relay, a, b) = pair(cfg, 57_600, 252);
    let mut rng = Rng::new(22);
    let sent: Vec<_> = (0..120).map(|i| marked(&mut rng, 1, i, 30, 100)).collect();
    let t = Instant::now();
    for p in &sent {
        a.send(p);
        std::thread::sleep(Duration::from_millis(5));
    }
    let got = b.recv_all(sent.len(), Duration::from_secs(30), Duration::from_secs(2));
    let took = t.elapsed();
    let faults =
        relay.a_to_b.dropped.load(Ordering::SeqCst) + relay.a_to_b.garbled.load(Ordering::SeqCst);
    let bytes = relay.a_to_b.bytes.load(Ordering::SeqCst);
    let (_, _) = a.finish();
    let (sb, lb) = b.finish();
    let sb = sb.unwrap();

    // Speed limit: the line moved `bytes` at 5760 bytes a second at most.
    assert!(
        took.as_secs_f64() >= bytes as f64 / 5760.0 * 0.9,
        "{took:?} for {bytes} bytes"
    );
    // Never delivered changed, never twice, order kept.
    let mut last = None;
    for g in &got {
        let at = sent
            .iter()
            .position(|p| p == g)
            .expect("a changed or invented packet arrived");
        assert!(last.is_none_or(|l| at > l), "out of order or duplicate");
        last = Some(at);
    }
    assert!(faults > 5, "the relay did too little damage ({faults})");
    assert!(got.len() < sent.len(), "faults should cost some packets");
    assert!(got.len() > 40, "got only {}", got.len());
    // The receiver counted what it threw away.
    assert!(stat(&lb, "from_radio") == got.len() as u64);
    assert!(
        sb.drops.get("bad_frame").copied().unwrap_or(0) >= 1,
        "{:?}",
        sb.drops
    );
    check_log(&lb, &[MARK]);
}

#[test]
fn the_line_recovers_after_a_burst_of_garbage() {
    let relay = Relay::start(RelayConfig::default()).unwrap();
    let a = Harness::start(serial(&relay.path_a, 115_200, 252));
    let b = Harness::start(serial(&relay.path_b, 115_200, 252));
    // Garbage straight onto B's line, from "the radio side", then real traffic.
    let mut line = std::fs::OpenOptions::new()
        .write(true)
        .open(&relay.path_a)
        .unwrap();
    use std::io::Write;
    let mut rng = Rng::new(31);
    let junk: Vec<u8> = (0..3000).map(|_| rng.next_u64() as u8).collect();
    line.write_all(&junk).unwrap();
    std::thread::sleep(Duration::from_millis(300));
    for i in 0..10u8 {
        a.send(&[i; 40]);
        std::thread::sleep(Duration::from_millis(20));
    }
    let got = b.recv_all(10, Duration::from_secs(5), Duration::from_secs(1));
    assert_eq!(got, (0..10u8).map(|i| vec![i; 40]).collect::<Vec<_>>());
    let (_, _) = a.finish();
    let (sb, lb) = b.finish();
    assert!(sb.unwrap().drops["bad_frame"] >= 1);
    assert!(lb.contains("\"event\":\"drop\",\"reason\":\"bad_frame\""));
}

#[test]
fn oversize_and_empty_packets_from_the_link_are_dropped_not_sent() {
    let (_relay, a, b) = pair(RelayConfig::default(), 115_200, 100);
    a.send(&[1u8; 101]);
    a.send(&[]);
    a.send(&[2u8; 100]);
    assert_eq!(b.recv(Duration::from_secs(3)), Some(vec![2u8; 100]));
    assert_eq!(b.recv(Duration::from_millis(300)), None);
    let (sa, la) = a.finish();
    let sa = sa.unwrap();
    assert_eq!(sa.drops["too_large"], 1);
    assert_eq!(sa.drops["empty"], 1);
    assert_eq!(sa.to_radio, 1);
    assert!(la.contains("\"reason\":\"too_large\""));
    b.finish().0.unwrap();
}

#[test]
fn packets_from_another_host_are_not_sent_on_the_radio() {
    let relay = Relay::start(RelayConfig::default()).unwrap();
    // The bridge expects oasa-link on 127.0.0.2; our sender is 127.0.0.1.
    let link = UdpSocket::bind("127.0.0.1:0").unwrap();
    let listen = format!("127.0.0.1:{}", common::free_port())
        .parse()
        .unwrap();
    let a = Harness::start_with(
        serial(&relay.path_a, 115_200, 252),
        link,
        listen,
        Some("127.0.0.2:9".parse().unwrap()),
    );
    let b = Harness::start(serial(&relay.path_b, 115_200, 252));
    a.send(&[7u8; 30]);
    assert_eq!(b.recv(Duration::from_millis(500)), None);
    let (sa, _) = a.finish();
    let sa = sa.unwrap();
    assert_eq!(sa.drops["udp_source"], 1);
    assert_eq!(sa.to_radio, 0);
    b.finish().0.unwrap();
}

#[test]
fn a_dead_line_ends_the_bridge_with_an_error_and_stats() {
    let relay = Relay::start(RelayConfig::default()).unwrap();
    let a = Harness::start(serial(&relay.path_a, 115_200, 252));
    drop(relay);
    let (r, log) = a.join();
    let e = r.unwrap_err();
    assert!(!e.is_empty());
    let events = check_log(&log, &[]);
    assert_eq!(&events[events.len() - 2..], ["error", "stats"]);
}
