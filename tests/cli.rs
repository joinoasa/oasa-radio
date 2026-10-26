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

fn field<'a>(line: &'a str, key: &str) -> &'a str {
    let pat = format!("\"{key}\":");
    let i = line
        .find(&pat)
        .unwrap_or_else(|| panic!("no {key} in {line}"))
        + pat.len();
    let rest = &line[i..];
    let end = rest.find([',', '}']).unwrap();
    rest[..end].trim_matches('"')
}

#[test]
fn waveform_ber_prints_json_lines_that_repeat_from_the_seed() {
    let args = [
        "waveform",
        "ber",
        "--mod",
        "bpsk",
        "--ebn0-db",
        "2,4",
        "--bits",
        "20000",
        "--seed",
        "9",
        "--json",
    ];
    let a = run(&args);
    let b = run(&args);
    assert!(a.status.success(), "{}", String::from_utf8_lossy(&a.stderr));
    let (ta, tb) = (
        String::from_utf8_lossy(&a.stdout).to_string(),
        String::from_utf8_lossy(&b.stdout).to_string(),
    );
    let lines: Vec<&str> = ta.lines().collect();
    assert_eq!(lines.len(), 2);
    for (l, db) in lines.iter().zip(["2e0", "4e0"]) {
        assert_eq!(field(l, "event"), "ber");
        assert_eq!(field(l, "mod"), "bpsk");
        assert_eq!(field(l, "ebn0_db"), db);
        assert_eq!(field(l, "seed"), "9");
        let (errors, bits): (u64, u64) = (
            field(l, "errors").parse().unwrap(),
            field(l, "bits").parse().unwrap(),
        );
        assert!(bits >= 20_000 && errors > 0 && errors < bits / 2);
        for k in ["ber", "theory", "ci_lo", "ci_hi", "z"] {
            field(l, k).parse::<f64>().unwrap();
        }
        assert_eq!(field(l, "detected"), field(l, "frames"));
    }
    // same seed, same numbers (the time stamp differs)
    let strip = |t: &str| {
        t.lines()
            .map(|l| l.split_once(",\"event\"").unwrap().1.to_string())
            .collect::<Vec<_>>()
    };
    assert_eq!(strip(&ta), strip(&tb));
}

#[test]
fn waveform_iq_writes_a_sigmf_pair_that_decode_reads_back() {
    let dir = std::env::temp_dir().join(format!("oasa-wf-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    for m in ["bpsk", "qpsk", "fsk2"] {
        let base = dir.join(format!("frame-{m}"));
        let base_s = base.display().to_string();
        let o = run(&[
            "waveform",
            "iq",
            "--mod",
            m,
            "--out",
            &base_s,
            "--ebn0-db",
            "20",
            "--payload-hex",
            "00ff10a5c3",
            "--phase-deg",
            "123",
            "--delay",
            "77",
            "--frac-delay",
            "0.4",
            "--seed",
            "3",
        ]);
        assert!(o.status.success(), "{}", String::from_utf8_lossy(&o.stderr));
        let line = String::from_utf8_lossy(&o.stdout).to_string();
        let samples: usize = field(&line, "samples").parse().unwrap();
        let data = std::fs::read(format!("{base_s}.sigmf-data")).unwrap();
        assert_eq!(data.len(), samples * 8, "cf32_le is 8 bytes a sample");
        let meta = std::fs::read_to_string(format!("{base_s}.sigmf-meta")).unwrap();
        for want in [
            "\"core:datatype\": \"cf32_le\"",
            "\"core:sample_rate\": 200000.0",
            "\"core:version\"",
            "\"core:datetime\"",
        ] {
            assert!(meta.contains(want), "{want} missing from {meta}");
        }
        let d = run(&["waveform", "decode", "--mod", m, "--in", &base_s, "--hex"]);
        let out = String::from_utf8_lossy(&d.stdout).to_string();
        assert!(d.status.success());
        let frame = out
            .lines()
            .find(|l| l.contains("\"event\":\"frame\""))
            .unwrap_or_else(|| panic!("{m}: {out}"));
        assert_eq!(field(frame, "crc_ok"), "true", "{m}");
        assert_eq!(field(frame, "payload_hex"), "00ff10a5c3", "{m}");
        // the wrong modulation finds nothing valid
        let w = run(&[
            "waveform",
            "decode",
            "--mod",
            if m == "bpsk" { "fsk2" } else { "bpsk" },
            "--in",
            &base_s,
        ]);
        assert!(!String::from_utf8_lossy(&w.stdout).contains("\"crc_ok\":true"));
    }
    std::fs::remove_dir_all(&dir).ok();
}

#[test]
fn waveform_bad_arguments_fail_cleanly() {
    for args in [
        vec!["waveform", "ber", "--mod", "bpsk"],
        vec!["waveform", "ber", "--mod", "16qam", "--ebn0-db", "3"],
        vec![
            "waveform",
            "decode",
            "--mod",
            "bpsk",
            "--in",
            "/nonexistent/x",
        ],
    ] {
        let o = run(&args);
        assert!(!o.status.success(), "{args:?}");
        assert!(String::from_utf8_lossy(&o.stderr).starts_with("error:"));
    }
    let o = run(&["waveform"]);
    assert!(o.status.success() && String::from_utf8_lossy(&o.stdout).contains("WAVEFORM"));
}
