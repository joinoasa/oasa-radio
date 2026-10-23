//! The host side against the real modem engine (a simulated radio behind a
//! pseudo-terminal). No hardware needed.

mod common;

use std::process::Command;
use std::time::Duration;

use common::sim_modem::SimModem;
use common::{serial, Harness};
use oasa_modem_core::control::SetArgs;
use oasa_modem_core::settings::{Bandwidth, Settings};
use oasa_modem_core::sim::{Air, Fault};
use oasa_radio::adapters::{SerialConfig, SerialRadio};
use oasa_radio::lora::{LoraError, LoraModem};
use oasa_radio::radio::Radio;
use oasa_radio::rng::Rng;

const BIN: &str = env!("CARGO_BIN_EXE_oasa-radio");
const WAIT: Duration = Duration::from_secs(5);

fn open(m: &SimModem) -> LoraModem {
    LoraModem::open(&m.path.display().to_string(), 115_200).unwrap()
}

#[test]
fn control_commands_work_over_a_serial_port() {
    let air = Air::new();
    let m = SimModem::start(&air);
    let mut c = open(&m);
    let (proto, fw, board) = c.version(WAIT).unwrap();
    assert_eq!((proto, fw, board.as_str()), (1, [0, 1, 0], "sim"));
    assert_eq!(c.get().unwrap(), Settings::LAB_SF7_BW125);
    assert_eq!(c.ping(&[1, 0, 2]).unwrap(), [1, 0, 2]);

    let s = c
        .set(&SetArgs {
            sf: Some(9),
            bw_code: Some(Bandwidth::Khz250.code()),
            tx_power_dbm: Some(-3),
            ..Default::default()
        })
        .unwrap();
    assert_eq!((s.sf, s.bw, s.tx_power_dbm), (9, Bandwidth::Khz250, -3));
    assert_eq!(c.get().unwrap(), s);

    // Refused: the error names the setting and nothing changes.
    let e = c
        .set(&SetArgs {
            freq_hz: Some(433_000_000),
            sf: Some(7),
            ..Default::default()
        })
        .unwrap_err();
    assert!(
        matches!(
            e,
            LoraError::Refused {
                status: 3,
                key: Some(1)
            }
        ),
        "{e}"
    );
    assert_eq!(c.get().unwrap(), s);

    let st = c.stats().unwrap();
    assert_eq!(st.radio_state, 1);
    c.clear_stats().unwrap();
    std::thread::sleep(Duration::from_millis(20));
    assert!(c.stats().unwrap().uptime_ms > 0);
}

#[test]
fn no_modem_means_a_clear_timeout() {
    // A pty with nobody behind it.
    let pty = oasa_radio::relay::Pty::open().unwrap();
    let mut c = LoraModem::open(&pty.path.display().to_string(), 115_200).unwrap();
    let e = c.version(Duration::from_millis(600)).unwrap_err();
    assert!(matches!(e, LoraError::Timeout), "{e}");
    assert!(e.to_string().contains("did not answer"));
}

#[test]
fn bridges_carry_packets_across_two_modems_unchanged() {
    // The same `bridge --radio serial` as for any serial radio.
    let air = Air::new();
    let (ma, mb) = (SimModem::start(&air), SimModem::start(&air));
    let a = Harness::start(serial(&ma.path, 115_200, 255));
    let b = Harness::start(serial(&mb.path, 115_200, 255));
    let mut rng = Rng::new(5);
    let pk: Vec<Vec<u8>> = (0..6)
        .map(|_| {
            let n = 1 + rng.below(255) as usize;
            (0..n)
                .map(|_| {
                    if rng.below(4) == 0 {
                        0
                    } else {
                        rng.next_u64() as u8
                    }
                })
                .collect()
        })
        .collect();
    for p in &pk {
        a.send(p);
        std::thread::sleep(Duration::from_millis(30));
    }
    let got = b.recv_all(6, Duration::from_secs(10), Duration::from_secs(1));
    assert_eq!(got, pk);
    b.send(&[9, 9, 9]);
    assert_eq!(a.recv(Duration::from_secs(5)), Some(vec![9, 9, 9]));
    assert_eq!(ma.radio.sent().len(), 6);
    let (ra, _) = a.finish();
    let (rb, _) = b.finish();
    assert_eq!(ra.unwrap().to_radio, 6);
    assert_eq!(rb.unwrap().from_radio, 6);
}

#[test]
fn a_packet_over_the_lora_limit_is_counted_not_sent() {
    let air = Air::new();
    let m = SimModem::start(&air);
    // An old-style host with a big MTU can send 300 bytes. The modem refuses it.
    let mut r = SerialRadio::open(SerialConfig {
        device: m.path.display().to_string(),
        baud: 115_200,
        mtu: 400,
    })
    .unwrap();
    r.send(&[7; 300]).unwrap();
    std::thread::sleep(Duration::from_millis(300));
    let mut c = open(&m);
    c.version(WAIT).unwrap();
    assert_eq!(c.stats().unwrap().tx_oversize, 1);
    assert!(m.radio.sent().is_empty());
}

#[test]
fn air_mode_shows_bad_crc_frames_and_old_hosts_ignore_it() {
    let air = Air::new();
    let (tx, mon) = (SimModem::start(&air), SimModem::start(&air));
    let mut sender = serial(&tx.path, 115_200, 255);
    // An old-style serial reader on the monitor, next to the control client.
    let mut c = open(&mon);
    c.version(WAIT).unwrap();
    c.set(&SetArgs {
        air_mode: Some(true),
        ..Default::default()
    })
    .unwrap();
    sender.send(&[1, 2, 3]).unwrap();
    let a = c.next_air(Duration::from_secs(3)).unwrap().unwrap();
    assert!(a.crc_ok);
    assert_eq!(a.payload, [1, 2, 3]);
    assert_eq!((a.rssi_qdb, a.snr_qdb), (-400, 36));
    air.set_fault(Fault {
        corrupt: 1,
        drop: 0,
    });
    sender.send(&[4, 5, 6]).unwrap();
    let b = c.next_air(Duration::from_secs(3)).unwrap().unwrap();
    assert!(!b.crc_ok, "the corrupted frame is shown, flagged");
    assert_eq!(b.payload, [4 ^ 0x10, 5, 6]);
    let st = c.stats().unwrap();
    assert_eq!((st.air_frames, st.rx_packets, st.rx_crc_err), (2, 1, 1));
}

#[test]
fn old_serial_adapter_counts_extended_frames_without_errors() {
    let air = Air::new();
    let m = SimModem::start(&air);
    let mut c = open(&m);
    c.version(WAIT).unwrap();
    drop(c);
    let mut r = SerialRadio::open(SerialConfig::new(&m.path.display().to_string())).unwrap();
    // The modem answers a request an old adapter never sent: nothing. So make
    // some extended traffic: air mode with an injected frame.
    let mut c = open(&m);
    c.set(&SetArgs {
        air_mode: Some(true),
        ..Default::default()
    })
    .unwrap();
    m.radio.inject(&[1, 2, 3], true, -80, 4);
    std::thread::sleep(Duration::from_millis(300));
    assert_eq!(r.recv(Duration::from_millis(500)).unwrap(), None);
    let fs = r.frame_stats();
    assert_eq!(fs.bad_frame, 0, "{fs:?}");
    assert!(fs.ext_frames >= 1);
}

#[test]
fn command_line_get_set_stats_ping_version() {
    let air = Air::new();
    let m = SimModem::start(&air);
    let dev = m.path.display().to_string();
    let run = |args: &[&str]| {
        let mut a = vec!["lora"];
        a.extend_from_slice(args);
        a.extend_from_slice(&["--device", &dev, "--wait", "5"]);
        Command::new(BIN).args(&a).output().unwrap()
    };
    let o = run(&["version"]);
    let out = String::from_utf8_lossy(&o.stdout);
    assert!(o.status.success(), "{}", String::from_utf8_lossy(&o.stderr));
    assert!(
        out.contains("\"firmware\":\"0.1.0\"") && out.contains("\"board\":\"sim\""),
        "{out}"
    );

    let out = String::from_utf8_lossy(&run(&["get"]).stdout).into_owned();
    assert!(
        out.contains("\"freq_hz\":915000000") && out.contains("\"sf\":7"),
        "{out}"
    );
    assert!(
        out.contains("\"cr\":\"4/5\"") && out.contains("\"tx_power_dbm\":2"),
        "{out}"
    );

    let o = run(&[
        "set", "--sf", "10", "--bw", "62.5", "--freq", "914.5", "--sync", "0x34", "--power", "0",
    ]);
    let out = String::from_utf8_lossy(&o.stdout).into_owned();
    assert!(o.status.success(), "{}", String::from_utf8_lossy(&o.stderr));
    assert!(
        out.contains("\"freq_hz\":914500000") && out.contains("\"sf\":10"),
        "{out}"
    );
    assert!(
        out.contains("\"bw_khz\":62.5") && out.contains("\"sync_word\":\"0x3444\""),
        "{out}"
    );

    // Out of range: exit 1, and the message names the setting.
    let o = run(&["set", "--freq", "433.0"]);
    assert!(!o.status.success());
    let err = String::from_utf8_lossy(&o.stderr);
    assert!(
        err.contains("out of range") && err.contains("frequency"),
        "{err}"
    );

    let out = String::from_utf8_lossy(&run(&["stats"]).stdout).into_owned();
    assert!(
        out.contains("\"event\":\"stats\"") && out.contains("\"radio_state\":\"rx\""),
        "{out}"
    );
    let out = String::from_utf8_lossy(&run(&["ping"]).stdout).into_owned();
    assert!(out.contains("\"event\":\"pong\""), "{out}");
    let o = run(&["set", "--profile", "lora-sf7-bw125"]);
    assert!(String::from_utf8_lossy(&o.stdout).contains("\"sf\":7"));
}

#[test]
fn command_line_air_prints_frames_and_restores_air_mode() {
    let air = Air::new();
    let (tx, mon) = (SimModem::start(&air), SimModem::start(&air));
    let dev = mon.path.display().to_string();
    let mut sender = serial(&tx.path, 115_200, 255);
    let child = Command::new(BIN)
        .args(["lora", "air", "--device", &dev, "--duration", "3", "--hex"])
        .stdout(std::process::Stdio::piped())
        .spawn()
        .unwrap();
    std::thread::sleep(Duration::from_millis(1200));
    sender.send(&[0xAB, 0xCD]).unwrap();
    std::thread::sleep(Duration::from_millis(300));
    air.set_fault(Fault {
        corrupt: 1,
        drop: 0,
    });
    sender.send(&[0x01, 0x02]).unwrap();
    let o = child.wait_with_output().unwrap();
    let out = String::from_utf8_lossy(&o.stdout);
    assert!(o.status.success(), "{out}");
    assert!(out.contains("\"event\":\"air_start\""), "{out}");
    assert!(
        out.contains("\"crc_ok\":true") && out.contains("\"hex\":\"abcd\""),
        "{out}"
    );
    assert!(
        out.contains("\"crc_ok\":false") && out.contains("\"hex\":\"1102\""),
        "{out}"
    );
    assert!(out.contains("\"frames\":2,\"crc_bad\":1"), "{out}");
    // Air mode is off again.
    let mut c = open(&mon);
    c.version(WAIT).unwrap();
    assert!(!c.get().unwrap().air_mode);
}
