//! Two UDP-radio bridges back to back: oasa-link plus these is an encrypted
//! link over plain UDP (the simulation and Wi-Fi case).

mod common;

use std::net::UdpSocket;
use std::time::Duration;

use common::Harness;
use oasa_radio::adapters::UdpRadio;
use oasa_radio::rng::Rng;

#[test]
fn two_udp_bridges_carry_packets_both_ways() {
    let addr = "127.0.0.1:0";
    let (pa, pb) = (
        UdpSocket::bind(addr).unwrap().local_addr().unwrap(),
        UdpSocket::bind(addr).unwrap().local_addr().unwrap(),
    );
    let a = Harness::start(Box::new(UdpRadio::new(pa, pb, 1400).unwrap()));
    let b = Harness::start(Box::new(UdpRadio::new(pb, pa, 1400).unwrap()));
    let mut rng = Rng::new(41);
    let pk: Vec<Vec<u8>> = (0..100)
        .map(|_| {
            (0..1 + rng.below(1400))
                .map(|_| rng.next_u64() as u8)
                .collect()
        })
        .collect();
    for p in &pk {
        a.send(p);
        b.send(p);
        std::thread::sleep(Duration::from_millis(2));
    }
    let mut got_b = b.recv_all(100, Duration::from_secs(10), Duration::from_secs(1));
    let mut got_a = a.recv_all(100, Duration::from_secs(10), Duration::from_secs(1));
    // UDP may reorder in theory; compare as sets.
    let mut want = pk.clone();
    for v in [&mut got_a, &mut got_b, &mut want] {
        v.sort();
    }
    assert_eq!(got_a, want);
    assert_eq!(got_b, want);
    let (sa, _) = a.finish();
    assert_eq!(sa.unwrap().to_radio, 100);
    b.finish().0.unwrap();
}
