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
fn waveform_decode_can_take_the_length_as_known() {
    let dir = std::env::temp_dir().join(format!("oasa-wf-len-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let base_s = dir.join("frame").display().to_string();
    let o = run(&[
        "waveform",
        "iq",
        "--mod",
        "bpsk",
        "--out",
        &base_s,
        "--ebn0-db",
        "20",
        "--payload-hex",
        "00ff10a5c3",
        "--seed",
        "4",
    ]);
    assert!(o.status.success(), "{}", String::from_utf8_lossy(&o.stderr));
    let frame_with = |len: &str| {
        let d = run(&[
            "waveform",
            "decode",
            "--mod",
            "bpsk",
            "--in",
            &base_s,
            "--payload-len",
            len,
            "--sync-max-errors",
            "16",
            "--hex",
        ]);
        assert!(d.status.success());
        String::from_utf8_lossy(&d.stdout)
            .lines()
            .find(|l| l.contains("\"event\":\"frame\""))
            .unwrap()
            .to_string()
    };
    let right = frame_with("5");
    assert_eq!(field(&right, "crc_ok"), "true");
    assert_eq!(field(&right, "payload_hex"), "00ff10a5c3");
    // a wrong known length is used as given (the header is not read): 4 bytes, and the CRC fails
    let wrong = frame_with("4");
    assert_eq!(field(&wrong, "len"), "4");
    assert_eq!(field(&wrong, "crc_ok"), "false");
    std::fs::remove_dir_all(&dir).ok();
}

#[test]
fn waveform_v1_frames_round_trip_and_report_the_header() {
    let dir = std::env::temp_dir().join(format!("oasa-wf1-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    for (m, name) in [("bpsk", "BPSK"), ("qpsk", "QPSK"), ("fsk2", "FSK2")] {
        let base_s = dir.join(format!("v1-{m}")).display().to_string();
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
            "--delay",
            "50",
            "--seed",
            "4",
            "--frame",
            "v1",
        ]);
        assert!(o.status.success(), "{}", String::from_utf8_lossy(&o.stderr));
        assert_eq!(field(&String::from_utf8_lossy(&o.stdout), "frame"), "v1");
        let meta = std::fs::read_to_string(format!("{base_s}.sigmf-meta")).unwrap();
        assert!(meta.contains("\"oasa:frame\": \"v1\""), "{name}: {meta}");
        let d = run(&[
            "waveform", "decode", "--mod", m, "--in", &base_s, "--hex", "--frame", "v1",
        ]);
        assert!(d.status.success());
        let out = String::from_utf8_lossy(&d.stdout).to_string();
        let frame = out
            .lines()
            .find(|l| l.contains("\"event\":\"frame\""))
            .unwrap_or_else(|| panic!("{m}: {out}"));
        assert_eq!(field(frame, "crc_ok"), "true", "{m}");
        assert_eq!(field(frame, "payload_hex"), "00ff10a5c3", "{m}");
        assert!(frame.contains(&format!("\"header\":{{\"version\":0,\"extension\":0,\"modulation\":\"{m}\",\"payload_code\":0,\"pilots\":0,\"length\":5,\"channel_plan\":0,\"coexistence\":0}}")), "{frame}");
        let done = out.lines().find(|l| l.contains("decode_done")).unwrap();
        assert_eq!(field(done, "frame"), "v1");
        assert_eq!(field(done, "frames"), "1");
        assert_eq!(field(done, "header_rejected"), "0");
        for k in ["preambles", "sync_rejected", "too_short", "inside_frame"] {
            field(done, k).parse::<u64>().unwrap();
        }
        // read as W0 the same file is not a good frame
        let w = run(&["waveform", "decode", "--mod", m, "--in", &base_s]);
        let w = String::from_utf8_lossy(&w.stdout).to_string();
        assert!(!w.contains("\"crc_ok\":true"), "{m}: {w}");
        assert!(w.contains("\"frame\":\"w0\""));
    }
    std::fs::remove_dir_all(&dir).ok();
}

#[test]
fn waveform_ber_v1_counts_headers_through_the_real_path() {
    let o = run(&[
        "waveform",
        "ber",
        "--mod",
        "bpsk",
        "--ebn0-db",
        "6",
        "--bits",
        "20000",
        "--seed",
        "2",
        "--frame",
        "v1",
    ]);
    assert!(o.status.success(), "{}", String::from_utf8_lossy(&o.stderr));
    let out = String::from_utf8_lossy(&o.stdout).to_string();
    let l = out.lines().next().unwrap();
    assert_eq!(field(l, "frame"), "v1");
    let (frames, detected, header_ok): (u64, u64, u64) = (
        field(l, "frames").parse().unwrap(),
        field(l, "detected").parse().unwrap(),
        field(l, "header_ok").parse().unwrap(),
    );
    assert_eq!(detected, frames);
    assert!(header_ok >= frames - 1, "{l}");
    // W0 lines have no header_ok
    let w = run(&[
        "waveform",
        "ber",
        "--mod",
        "bpsk",
        "--ebn0-db",
        "6",
        "--bits",
        "2000",
    ]);
    let w = String::from_utf8_lossy(&w.stdout).to_string();
    assert!(!w.contains("header_ok") && w.contains("\"frame\":\"w0\""));
}

#[test]
fn waveform_rx_stream_gives_what_decode_gives() {
    // Three frames written by `iq` one after another into one raw file, then decoded as a whole
    // (`decode`) and as a stream in small blocks (`rx-stream`): the same frames, positions counted
    // from the stream's start.
    let dir = std::env::temp_dir().join(format!("oasa-wf-stream-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let mut raw = Vec::new();
    for (i, hex) in ["00ff10a5c3", "deadbeef", "0102030405060708"]
        .iter()
        .enumerate()
    {
        let base = dir.join(format!("f{i}")).display().to_string();
        let o = run(&[
            "waveform",
            "iq",
            "--mod",
            "qpsk",
            "--out",
            &base,
            "--ebn0-db",
            "20",
            "--payload-hex",
            hex,
            "--seed",
            &i.to_string(),
            "--delay",
            "300",
            "--tail",
            "700",
        ]);
        assert!(o.status.success(), "{}", String::from_utf8_lossy(&o.stderr));
        raw.extend(std::fs::read(format!("{base}.sigmf-data")).unwrap());
    }
    let all = dir.join("all");
    std::fs::write(all.with_extension("sigmf-data"), &raw).unwrap();
    std::fs::copy(dir.join("f0.sigmf-meta"), all.with_extension("sigmf-meta")).unwrap();
    let frames = |out: &std::process::Output| -> Vec<(String, String, String)> {
        assert!(
            out.status.success(),
            "{}",
            String::from_utf8_lossy(&out.stderr)
        );
        String::from_utf8_lossy(&out.stdout)
            .lines()
            .filter(|l| l.contains("\"event\":\"frame\""))
            .map(|l| {
                (
                    field(l, "start").to_string(),
                    field(l, "payload_hex").to_string(),
                    field(l, "crc_ok").to_string(),
                )
            })
            .collect()
    };
    let whole = frames(&run(&[
        "waveform",
        "decode",
        "--mod",
        "qpsk",
        "--in",
        &all.display().to_string(),
        "--hex",
    ]));
    let data = all.with_extension("sigmf-data").display().to_string();
    let streamed = run(&[
        "waveform",
        "rx-stream",
        "--mod",
        "qpsk",
        "--in",
        &data,
        "--hex",
        "--block",
        "1024",
    ]);
    let s = frames(&streamed);
    assert_eq!(whole.len(), 3, "{whole:?}");
    assert_eq!(s, whole);
    assert_eq!(
        s.iter().map(|f| f.1.as_str()).collect::<Vec<_>>(),
        ["00ff10a5c3", "deadbeef", "0102030405060708"]
    );
    let done = String::from_utf8_lossy(&streamed.stdout)
        .lines()
        .last()
        .unwrap()
        .to_string();
    assert_eq!(field(&done, "event"), "stream_done");
    assert_eq!(field(&done, "frames"), "3");
    assert_eq!(
        field(&done, "samples"),
        (raw.len() / 8).to_string().as_str()
    );
    std::fs::remove_dir_all(&dir).ok();
}

#[test]
fn waveform_light_code_round_trips_through_iq_decode_and_rx_stream() {
    let dir = std::env::temp_dir().join(format!("oasa-wf-light-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    for m in ["bpsk", "qpsk", "fsk2"] {
        let base = dir.join(format!("light-{m}")).display().to_string();
        let o = run(&[
            "waveform",
            "iq",
            "--mod",
            m,
            "--out",
            &base,
            "--ebn0-db",
            "12",
            "--payload-len",
            "64",
            "--seed",
            "5",
            "--frame",
            "v1",
            "--code",
            "light",
            "--phase-deg",
            "40",
        ]);
        assert!(o.status.success(), "{}", String::from_utf8_lossy(&o.stderr));
        let line = String::from_utf8_lossy(&o.stdout).to_string();
        assert_eq!(field(&line, "code"), "light");
        // airtime: 79 + 152 + 2 (66 * 8 + 6) / bits per symbol
        let syms = if m == "qpsk" { "765" } else { "1299" };
        assert_eq!(field(&line, "frame_symbols"), syms, "{m}");
        let meta = std::fs::read_to_string(format!("{base}.sigmf-meta")).unwrap();
        assert!(meta.contains("\"oasa:payload_code\": \"light\""), "{meta}");
        // the receiver reads the code from the header: no --code needed
        let d = run(&[
            "waveform", "decode", "--mod", m, "--in", &base, "--frame", "v1",
        ]);
        let out = String::from_utf8_lossy(&d.stdout).to_string();
        let frame = out
            .lines()
            .find(|l| l.contains("\"event\":\"frame\""))
            .unwrap_or_else(|| panic!("{m}: {out}"));
        assert_eq!(field(frame, "crc_ok"), "true", "{m}");
        assert_eq!(field(frame, "payload_code"), "1", "{m}");
        assert_eq!(field(frame, "len"), "64", "{m}");
        // with the length given, the code is what --code says
        for (code, ok) in [("light", "true"), ("none", "false")] {
            let d = run(&[
                "waveform",
                "decode",
                "--mod",
                m,
                "--in",
                &base,
                "--frame",
                "v1",
                "--code",
                code,
                "--payload-len",
                "64",
            ]);
            let out = String::from_utf8_lossy(&d.stdout).to_string();
            let frame = out
                .lines()
                .find(|l| l.contains("\"event\":\"frame\""))
                .unwrap();
            assert_eq!(field(frame, "crc_ok"), ok, "{m} {code}");
        }
        let s = run(&[
            "waveform",
            "rx-stream",
            "--mod",
            m,
            "--in",
            &format!("{base}.sigmf-data"),
            "--frame",
            "v1",
            "--block",
            "2048",
        ]);
        let out = String::from_utf8_lossy(&s.stdout).to_string();
        let done = out.lines().last().unwrap();
        assert_eq!(field(done, "crc_ok_frames"), "1", "{m}: {out}");
    }
    std::fs::remove_dir_all(&dir).ok();
}

#[test]
fn waveform_per_and_ber_with_the_light_code() {
    let o = run(&[
        "waveform",
        "per",
        "--mod",
        "bpsk",
        "--ebn0-db",
        "5,9",
        "--frames",
        "20",
        "--frame",
        "v1",
        "--code",
        "light",
        "--seed",
        "3",
    ]);
    assert!(o.status.success(), "{}", String::from_utf8_lossy(&o.stderr));
    let out = String::from_utf8_lossy(&o.stdout).to_string();
    let lines: Vec<&str> = out.lines().collect();
    assert_eq!(lines.len(), 2, "{out}");
    for l in &lines {
        assert_eq!(field(l, "event"), "per");
        assert_eq!(field(l, "code"), "light");
        assert_eq!(field(l, "frames"), "20");
        assert_eq!(field(l, "frame_symbols"), "1299");
        let loss: f64 = field(l, "packet_loss").parse().unwrap();
        assert!(loss <= 0.1, "{l}");
    }
    let b = run(&[
        "waveform",
        "ber",
        "--mod",
        "qpsk",
        "--ebn0-db",
        "4",
        "--bits",
        "5120",
        "--frame",
        "v1",
        "--code",
        "light",
    ]);
    assert!(b.status.success(), "{}", String::from_utf8_lossy(&b.stderr));
    let l = String::from_utf8_lossy(&b.stdout).to_string();
    assert_eq!(field(&l, "code"), "light");
    assert_eq!(field(&l, "theory"), "null");
    assert_eq!(field(&l, "z"), "null");
    // W0 has no payload_code
    let w = run(&[
        "waveform", "iq", "--mod", "bpsk", "--out", "/tmp/x", "--code", "light",
    ]);
    assert!(!w.status.success());
    assert!(String::from_utf8_lossy(&w.stderr).contains("--frame v1"));
}

#[test]
fn waveform_bad_arguments_fail_cleanly() {
    for args in [
        vec![
            "waveform",
            "ber",
            "--mod",
            "bpsk",
            "--ebn0-db",
            "3",
            "--frame",
            "v9",
        ],
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

/// Two ci16 channels as UHD writes them: noise, then three tone bursts with channel 2 leading by
/// `phase` degrees and a shared frequency offset that must cancel.
fn df_files(dir: &std::path::Path, phase: f64) -> (String, String) {
    let mut seed = 12345u64;
    let mut noise = || {
        seed = seed
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((seed >> 33) as f64 / (1u64 << 31) as f64 - 0.5) * 0.004
    };
    let (mut b1, mut b2) = (Vec::new(), Vec::new());
    for i in 0..60_000usize {
        let on = [(5_000..15_000), (25_000..37_000), (45_000..55_000)]
            .iter()
            .any(|r| r.contains(&i));
        let t = std::f64::consts::TAU * 0.0123 * i as f64;
        let a = if on { 0.25 } else { 0.0 };
        let p2 = t + phase.to_radians();
        for (buf, re, im) in [
            (&mut b1, a * t.cos() + noise(), a * t.sin() + noise()),
            (&mut b2, a * p2.cos() + noise(), a * p2.sin() + noise()),
        ] {
            for x in [re, im] {
                buf.extend_from_slice(&((x * 32767.0).round() as i16).to_le_bytes());
            }
        }
    }
    let (f1, f2) = (dir.join("rx.00.dat"), dir.join("rx.01.dat"));
    std::fs::write(&f1, b1).unwrap();
    std::fs::write(&f2, b2).unwrap();
    (f1.display().to_string(), f2.display().to_string())
}

#[test]
fn waveform_df_measures_phase_per_burst_and_direction() {
    let dir = std::env::temp_dir().join(format!("oasa-df-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    // receiver offset 20 degrees plus a source 30 degrees toward channel 2 at half-wavelength spacing:
    // 20 + 180 sin(30) = 110
    let (f1, f2) = df_files(&dir, 110.0);
    let o = run(&[
        "waveform",
        "df",
        "--in1",
        &f1,
        "--in2",
        &f2,
        "--cal-deg",
        "20",
        "--spacing-wl",
        "0.5",
    ]);
    assert!(o.status.success(), "{}", String::from_utf8_lossy(&o.stderr));
    let out = String::from_utf8_lossy(&o.stdout);
    let lines: Vec<&str> = out.lines().collect();
    let segs: Vec<&&str> = lines
        .iter()
        .filter(|l| field(l, "event") == "segment")
        .collect();
    assert_eq!(segs.len(), 3, "{out}");
    for s in &segs {
        let p: f64 = field(s, "phase_deg").parse().unwrap();
        let d: f64 = field(s, "doa_deg").parse().unwrap();
        assert!((p - 110.0).abs() < 0.5, "{s}");
        assert!((d - 30.0).abs() < 0.5, "{s}");
        assert!(field(s, "snr1_db").parse::<f64>().unwrap() > 20.0, "{s}");
    }
    let done = lines.last().unwrap();
    assert_eq!(field(done, "event"), "df_done");
    assert_eq!(field(done, "segments"), "3");
    assert!((field(done, "doa_mean_deg").parse::<f64>().unwrap() - 30.0).abs() < 0.5);

    // fixed blocks over a stretch that is all signal
    let o = run(&[
        "waveform", "df", "--in1", &f1, "--in2", &f2, "--skip", "25000", "--block", "4000",
    ]);
    let out = String::from_utf8_lossy(&o.stdout);
    let segs = out
        .lines()
        .filter(|l| field(l, "event") == "segment")
        .count();
    assert_eq!(segs, 8, "{out}"); // 35000 samples after the skip
    let first = out.lines().next().unwrap();
    assert_eq!(field(first, "start"), "25000");
    assert_eq!(field(first, "snr1_db"), "null");

    // a channel cut short is refused: the two files must be one stream
    std::fs::write(&f2, [0u8; 400]).unwrap();
    let o = run(&["waveform", "df", "--in1", &f1, "--in2", &f2]);
    assert!(!o.status.success());
    assert!(String::from_utf8_lossy(&o.stderr).contains("differ in length"));
    for bad in [
        vec!["waveform", "df", "--in1", &f1],
        vec![
            "waveform", "df", "--in1", &f1, "--in2", &f1, "--format", "sc8",
        ],
        vec![
            "waveform",
            "df",
            "--in1",
            &f1,
            "--in2",
            &f1,
            "--spacing-wl",
            "0",
        ],
        vec![
            "waveform", "df", "--in1", &f1, "--in2", &f1, "--mod", "bpsk",
        ],
    ] {
        let o = run(&bad);
        assert!(!o.status.success(), "{bad:?}");
    }
    std::fs::remove_dir_all(&dir).ok();
}
