//! `oasa-radio sik` and the bridge's settings check (`--sik-expect`, H1)
//! against two simulated SiK radios on pseudo-terminals.

mod common;

use std::io::{BufRead, BufReader, Read, Write};
use std::net::UdpSocket;
use std::process::{Child, Command, Stdio};
use std::sync::mpsc;
use std::time::{Duration, Instant};

use common::free_port;
use common::sim_sik::{reg, SimSikPair, DEFAULTS};
use oasa_radio::sik::AtPort;

const BIN: &str = env!("CARGO_BIN_EXE_oasa-radio");
const MARK: &[u8] = b"OASA-MARKER-5ik1";

fn run(args: &[&str]) -> std::process::Output {
    Command::new(BIN).args(args).output().unwrap()
}

fn text(b: &[u8]) -> String {
    String::from_utf8_lossy(b).into_owned()
}

/// A bridge process; its stdout lines come through a channel.
struct Bridge {
    child: Child,
    lines: mpsc::Receiver<String>,
    listen: u16,
    link: UdpSocket,
}

impl Bridge {
    fn start(dev: &std::path::Path, expect: &str) -> Self {
        let link = UdpSocket::bind("127.0.0.1:0").unwrap();
        let listen = free_port();
        let mut child = Command::new(BIN)
            .args([
                "bridge",
                "--radio",
                "serial",
                "--device",
                &dev.display().to_string(),
                "--sik-expect",
                expect,
                "--udp-listen",
                &format!("127.0.0.1:{listen}"),
                "--udp-send",
                &link.local_addr().unwrap().to_string(),
                "--duration",
                "60",
            ])
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .unwrap();
        let out = child.stdout.take().unwrap();
        let (tx, lines) = mpsc::channel();
        std::thread::spawn(move || {
            for l in BufReader::new(out).lines().map_while(Result::ok) {
                let _ = tx.send(l);
            }
        });
        Self {
            child,
            lines,
            listen,
            link,
        }
    }

    /// Lines until one contains `want` (returned with all before it).
    fn wait_for(&self, want: &str, t: Duration) -> Vec<String> {
        let end = Instant::now() + t;
        let mut got = Vec::new();
        while Instant::now() < end {
            if let Ok(l) = self.lines.recv_timeout(Duration::from_millis(50)) {
                let hit = l.contains(want);
                got.push(l);
                if hit {
                    return got;
                }
            }
        }
        panic!("no {want:?} within {t:?}; got {got:?}");
    }

    fn send(&self, p: &[u8]) {
        self.link.send_to(p, ("127.0.0.1", self.listen)).unwrap();
    }

    fn recv(&self, t: Duration) -> Option<Vec<u8>> {
        self.link.set_read_timeout(Some(t)).unwrap();
        let mut b = [0u8; 2048];
        self.link
            .recv_from(&mut b)
            .ok()
            .map(|(n, _)| b[..n].to_vec())
    }

    /// Stop it; all remaining stdout and stderr.
    fn stop(mut self) -> (String, String) {
        let _ = Command::new("kill")
            .args(["-TERM", &self.child.id().to_string()])
            .status();
        let _ = self.child.wait();
        let mut err = String::new();
        let _ = self.child.stderr.take().unwrap().read_to_string(&mut err);
        let rest: Vec<String> = self.lines.try_iter().collect();
        (rest.join("\n"), err)
    }
}

const EXPECT: &str = "netid=4242,air_speed=64,ecc=0,mavlink=0,txpower=20,min_freq=915000,max_freq=928000,num_channels=50";

#[test]
fn show_reads_the_settings_and_leaves_data_mode() {
    let sik = SimSikPair::matched(4242);
    let o = run(&["sik", "show", "--device", &sik.path_a.display().to_string()]);
    assert!(o.status.success(), "{}", text(&o.stderr));
    let out = text(&o.stdout);
    assert!(out.contains("\"event\":\"sik\""), "{out}");
    assert!(
        out.contains("\"version\":\"SiK 2.2 on SIM-HM-TRP\""),
        "{out}"
    );
    assert!(
        out.contains("\"NETID\":4242") && out.contains("\"MAVLINK\":0"),
        "{out}"
    );
    assert!(out.contains("\"MAX_WINDOW\":131"), "{out}");
    assert!(!sik.with(|r| r[0].at_mode), "left in command mode");
    // Data flows again: bytes into A come out of B.
    let mut a = std::fs::OpenOptions::new()
        .write(true)
        .open(&sik.path_a)
        .unwrap();
    let mut b = AtPortReader::open(&sik.path_b);
    a.write_all(b"\x00hello\x00").unwrap();
    assert!(b
        .read_for(Duration::from_millis(500))
        .ends_with(b"\x00hello\x00"));
}

/// Reads raw bytes from a pty (for data-mode checks).
struct AtPortReader(std::fs::File);
impl AtPortReader {
    fn open(p: &std::path::Path) -> Self {
        let f = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .open(p)
            .unwrap();
        oasa_radio::adapters::serial::set_raw(&f, nix::sys::termios::BaudRate::B57600).unwrap();
        Self(f)
    }
    fn read_for(&mut self, d: Duration) -> Vec<u8> {
        use std::os::fd::AsFd;
        let end = Instant::now() + d;
        let mut got = Vec::new();
        let mut buf = [0u8; 512];
        while Instant::now() < end {
            let mut fds = [nix::poll::PollFd::new(
                self.0.as_fd(),
                nix::poll::PollFlags::POLLIN,
            )];
            if nix::poll::poll(&mut fds, nix::poll::PollTimeout::from(20u16)).unwrap_or(0) > 0 {
                if let Ok(n) = self.0.read(&mut buf) {
                    got.extend_from_slice(&buf[..n]);
                }
            }
        }
        got
    }
}

#[test]
fn show_works_when_the_radio_is_already_in_command_mode() {
    let sik = SimSikPair::matched(7);
    sik.with(|r| r[0].at_mode = true);
    let o = run(&["sik", "show", "--device", &sik.path_a.display().to_string()]);
    assert!(o.status.success(), "{}", text(&o.stderr));
    assert!(text(&o.stdout).contains("\"NETID\":7"));
    assert!(!sik.with(|r| r[0].at_mode));
}

#[test]
fn show_on_something_that_is_not_a_sik_radio_fails_clearly() {
    let relay = oasa_radio::relay::Relay::start(Default::default()).unwrap();
    let o = run(&[
        "sik",
        "show",
        "--device",
        &relay.path_a.display().to_string(),
    ]);
    assert!(!o.status.success());
    assert!(
        text(&o.stderr).contains("no reply to +++"),
        "{}",
        text(&o.stderr)
    );
}

#[test]
fn set_changes_saves_reboots_and_reads_back() {
    let sik = SimSikPair::start(DEFAULTS, DEFAULTS);
    let o = run(&[
        "sik",
        "set",
        "--device",
        &sik.path_a.display().to_string(),
        "--netid",
        "4242",
        "--mavlink",
        "0",
        "--txpower",
        "11",
    ]);
    assert!(o.status.success(), "{}", text(&o.stderr));
    let out = text(&o.stdout);
    assert!(
        out.contains("\"event\":\"sik_set\",") && out.contains("\"ok\":true"),
        "{out}"
    );
    assert!(
        out.contains("\"before\":{") && out.contains("\"NETID\":25"),
        "{out}"
    );
    sik.with(|r| {
        assert_eq!(r[0].saved[reg("NETID")], 4242);
        assert_eq!(r[0].params[reg("MAVLINK")], 0);
        assert_eq!(r[0].params[reg("TXPOWER")], 11);
        assert_eq!(r[0].reboots, 1);
        assert!(!r[0].at_mode);
        // the other radio is untouched
        assert_eq!(r[1].params, DEFAULTS);
    });
}

#[test]
fn bridge_refuses_a_radio_whose_settings_do_not_match() {
    let sik = SimSikPair::matched(25);
    let b = Bridge::start(&sik.path_a, EXPECT);
    let lines = b.wait_for("sik_check", Duration::from_secs(10));
    let l = lines.last().unwrap();
    assert!(l.contains("\"ok\":false"), "{l}");
    assert!(
        l.contains("{\"name\":\"NETID\",\"want\":4242,\"got\":25}"),
        "{l}"
    );
    let status = {
        let mut c = b;
        let s = c.child.wait().unwrap();
        let mut err = String::new();
        c.child
            .stderr
            .take()
            .unwrap()
            .read_to_string(&mut err)
            .unwrap();
        assert!(
            err.contains("NETID is 25, expected 4242") && err.contains("Not bridging"),
            "{err}"
        );
        let rest: Vec<String> = c.lines.try_iter().collect();
        assert!(
            !rest.iter().any(|l| l.contains("\"event\":\"start\"")),
            "{rest:?}"
        );
        s
    };
    assert!(!status.success());
    // nothing went on the air but the +++ escape
    assert_eq!(sik.with(|r| r[0].aired.clone()), b"+++");
}

#[test]
fn bridge_checks_then_carries_packets_and_never_prints_what_arrived_meanwhile() {
    let sik = SimSikPair::matched(4242);
    let b = Bridge::start(&sik.path_b, EXPECT);
    b.wait_for("\"event\":\"start\"", Duration::from_secs(10));
    // B's side sends marked packets the whole time A's bridge checks its radio:
    // they reach A's serial port while the AT code is reading it.
    let a = Bridge::start(&sik.path_a, EXPECT);
    let t = Instant::now();
    while t.elapsed() < Duration::from_millis(3500) {
        let mut p = MARK.to_vec();
        p.extend_from_slice(b"|OK|ATI5|S3:NETID=1|");
        b.send(&p);
        std::thread::sleep(Duration::from_millis(40));
    }
    let lines = a.wait_for("\"event\":\"start\"", Duration::from_secs(10));
    let check = lines.iter().find(|l| l.contains("sik_check")).unwrap();
    assert!(
        check.contains("\"ok\":true") && check.contains("\"mismatch\":[]"),
        "{check}"
    );
    while a.recv(Duration::from_millis(300)).is_some() {}
    // Now both ways.
    for i in 0..20u8 {
        a.send(&[i; 100]);
        b.send(&[i ^ 0xff; 120]);
        std::thread::sleep(Duration::from_millis(10));
    }
    let mut to_b = Vec::new();
    while let Some(p) = b.recv(Duration::from_millis(500)) {
        to_b.push(p);
    }
    let mut to_a = Vec::new();
    while let Some(p) = a.recv(Duration::from_millis(500)) {
        to_a.push(p);
    }
    assert_eq!(to_b, (0..20u8).map(|i| vec![i; 100]).collect::<Vec<_>>());
    assert_eq!(
        to_a,
        (0..20u8).map(|i| vec![i ^ 0xff; 120]).collect::<Vec<_>>()
    );
    let (out_a, err_a) = a.stop();
    let all = format!("{}\n{out_a}\n{err_a}", lines.join("\n"));
    assert!(!all.contains("MARKER"), "packet bytes were printed: {all}");
    b.stop();
}

#[test]
fn a_remote_change_by_another_radio_is_caught_before_bridging() {
    // H1: any radio with the same air settings can change ours with RT commands.
    let sik = SimSikPair::matched(4242);
    let mut attacker = AtPort::open(&sik.path_b.display().to_string(), 57_600).unwrap();
    attacker.enter(true).unwrap();
    attacker.command_ok("RTS4=1").unwrap();
    attacker.leave().unwrap();
    drop(attacker);
    assert_eq!(
        sik.param(0, "TXPOWER"),
        1,
        "the simulated radio should take RT commands"
    );
    let b = Bridge::start(&sik.path_a, EXPECT);
    let lines = b.wait_for("sik_check", Duration::from_secs(10));
    assert!(
        lines
            .last()
            .unwrap()
            .contains("{\"name\":\"TXPOWER\",\"want\":20,\"got\":1}"),
        "{lines:?}"
    );
    let (_, err) = b.stop();
    assert!(err.contains("TXPOWER is 1, expected 20"), "{err}");
}

#[test]
fn a_lost_ato_is_sent_again() {
    let sik = SimSikPair::matched(4242);
    sik.with(|r| r[0].lose_ato = 1);
    let o = run(&["sik", "show", "--device", &sik.path_a.display().to_string()]);
    assert!(o.status.success(), "{}", text(&o.stderr));
    sik.with(|r| {
        assert_eq!(r[0].lose_ato, 0, "the first ATO should have been lost");
        assert!(!r[0].at_mode, "left in command mode");
    });
}

#[test]
fn the_bridge_refuses_when_ato_is_never_confirmed() {
    let sik = SimSikPair::matched(4242);
    sik.with(|r| r[0].lose_ato = 2);
    let mut b = Bridge::start(&sik.path_a, EXPECT);
    let s = b.child.wait().unwrap();
    assert!(!s.success());
    let mut err = String::new();
    b.child
        .stderr
        .take()
        .unwrap()
        .read_to_string(&mut err)
        .unwrap();
    assert!(err.contains("did not confirm ATO"), "{err}");
    let out: Vec<String> = b.lines.try_iter().collect();
    assert!(
        !out.iter().any(|l| l.contains("\"event\":\"start\"")),
        "{out:?}"
    );
}

#[test]
fn the_bridge_does_not_poke_a_radio_left_in_command_mode() {
    let sik = SimSikPair::matched(4242);
    sik.with(|r| r[0].at_mode = true);
    let mut b = Bridge::start(&sik.path_a, EXPECT);
    assert!(!b.child.wait().unwrap().success());
    let mut err = String::new();
    b.child
        .stderr
        .take()
        .unwrap()
        .read_to_string(&mut err)
        .unwrap();
    assert!(err.contains("sik show recovers"), "{err}");
    // no recovery bytes were sent: nothing but +++ reached the AT parser, nothing went on the air
    assert!(sik.with(|r| r[0].aired.is_empty()));
}

#[test]
fn a_failed_save_reboots_without_the_changes() {
    let sik = SimSikPair::matched(4242);
    sik.with(|r| r[0].fail_save = true);
    let o = run(&[
        "sik",
        "set",
        "--device",
        &sik.path_a.display().to_string(),
        "--txpower",
        "11",
    ]);
    assert!(!o.status.success());
    assert!(
        text(&o.stderr).contains("nothing was saved"),
        "{}",
        text(&o.stderr)
    );
    std::thread::sleep(Duration::from_millis(100));
    sik.with(|r| {
        assert_eq!(r[0].reboots, 1);
        assert_eq!(
            r[0].params[reg("TXPOWER")],
            20,
            "the unsaved change must be gone"
        );
        assert!(!r[0].at_mode);
    });
}
