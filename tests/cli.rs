//! The `oasa-radio` program itself.

use std::io::{BufRead, BufReader, Read};
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

use oasa_radio::relay::{Relay, RelayConfig};

const BIN: &str = env!("CARGO_BIN_EXE_oasa-radio");

fn run(args: &[&str]) -> std::process::Output {
    Command::new(BIN).args(args).output().unwrap()
}

#[test]
fn help_and_info() {
    let o = run(&["--help"]);
    assert!(o.status.success());
    assert!(String::from_utf8_lossy(&o.stdout).contains("oasa-radio bridge"));
    let o = run(&["info"]);
    let out = String::from_utf8_lossy(&o.stdout);
    assert!(o.status.success());
    for k in ["udp", "serial", "lora", "sdr"] {
        assert!(out.contains(&format!("\"kind\":\"{k}\"")), "{out}");
    }
    assert!(out.contains("\"status\":\"planned\""));
}

#[test]
fn info_for_a_serial_radio() {
    let relay = Relay::start(RelayConfig::default()).unwrap();
    let dev = relay.path_a.display().to_string();
    let o = run(&[
        "info", "--radio", "serial", "--device", &dev, "--baud", "115200", "--mtu", "200",
    ]);
    assert!(o.status.success(), "{}", String::from_utf8_lossy(&o.stderr));
    let out = String::from_utf8_lossy(&o.stdout);
    assert!(out.contains("\"kind\":\"serial\"") && out.contains("\"mtu\":200"));
    assert!(out.contains("\"bitrate\":115200"), "{out}");
}

#[test]
fn bad_arguments_exit_1_with_a_message() {
    for args in [
        vec!["bridge"],
        vec![
            "bridge",
            "--radio",
            "lora",
            "--udp-listen",
            "127.0.0.1:1",
            "--udp-send",
            "127.0.0.1:2",
        ],
        vec![
            "bridge",
            "--radio",
            "serial",
            "--device",
            "/dev/nope",
            "--udp-listen",
            "0.0.0.0:1",
            "--udp-send",
            "127.0.0.1:2",
        ],
        vec![
            "bridge",
            "--radio",
            "serial",
            "--device",
            "/dev/nope",
            "--udp-listen",
            "127.0.0.1:1",
            "--udp-send",
            "127.0.0.1:2",
        ],
        vec!["nonsense"],
    ] {
        let o = run(&args);
        assert_eq!(o.status.code(), Some(1), "{args:?}");
        assert!(
            String::from_utf8_lossy(&o.stderr).starts_with("error: "),
            "{args:?}"
        );
        assert!(o.stdout.is_empty(), "{args:?}");
    }
}

fn free_port() -> u16 {
    std::net::UdpSocket::bind("127.0.0.1:0")
        .unwrap()
        .local_addr()
        .unwrap()
        .port()
}

fn bridge_cmd(dev: &str, extra: &[&str]) -> Command {
    let mut c = Command::new(BIN);
    c.args([
        "bridge",
        "--radio",
        "serial",
        "--device",
        dev,
        "--udp-listen",
    ])
    .arg(format!("127.0.0.1:{}", free_port()))
    .arg("--udp-send")
    .arg(format!("127.0.0.1:{}", free_port()))
    .args(extra)
    .stdout(Stdio::piped())
    .stderr(Stdio::piped());
    c
}

#[test]
fn duration_ends_the_run_with_stats_and_exit_0() {
    let relay = Relay::start(RelayConfig::default()).unwrap();
    let o = bridge_cmd(&relay.path_a.display().to_string(), &["--duration", "1"])
        .output()
        .unwrap();
    assert!(o.status.success(), "{}", String::from_utf8_lossy(&o.stderr));
    let out = String::from_utf8_lossy(&o.stdout);
    let lines: Vec<&str> = out.lines().collect();
    assert!(lines[0].contains("\"event\":\"start\"") && lines[0].contains("\"radio\":\"serial\""));
    assert!(
        lines.last().unwrap().contains("\"event\":\"stats\""),
        "{out}"
    );
    assert!(lines.last().unwrap().contains("\"to_radio\":0"));
}

#[test]
fn sigterm_and_sigint_print_stats_and_exit_0() {
    for sig in ["TERM", "INT"] {
        let relay = Relay::start(RelayConfig::default()).unwrap();
        let mut child = bridge_cmd(&relay.path_a.display().to_string(), &[])
            .spawn()
            .unwrap();
        let mut out = BufReader::new(child.stdout.take().unwrap());
        let mut first = String::new();
        out.read_line(&mut first).unwrap();
        assert!(first.contains("\"event\":\"start\""), "{first}");
        let k = Command::new("kill")
            .args([&format!("-{sig}"), &child.id().to_string()])
            .status()
            .unwrap();
        assert!(k.success());
        let t = Instant::now();
        let status = loop {
            if let Some(s) = child.try_wait().unwrap() {
                break s;
            }
            assert!(
                t.elapsed() < Duration::from_secs(5),
                "did not stop on SIG{sig}"
            );
            std::thread::sleep(Duration::from_millis(20));
        };
        assert!(status.success(), "SIG{sig}: {status:?}");
        let mut rest = String::new();
        out.read_to_string(&mut rest).unwrap();
        assert!(
            rest.trim_end()
                .lines()
                .last()
                .unwrap()
                .contains("\"event\":\"stats\""),
            "{rest}"
        );
    }
}
