//! `oasa-radio`: bridge oasa-link's UDP packets to a radio. See `--help`.

// The only unsafe code is the signal binding below.
#![deny(unsafe_code)]

use std::io::Write;
use std::process::ExitCode;
use std::sync::atomic::AtomicBool;
use std::time::SystemTime;

use oasa_radio::bridge;
use oasa_radio::cli::{self, Command, RadioSpec, SikCommand};
use oasa_radio::log::{json_str, rfc3339};

static STOP: AtomicBool = AtomicBool::new(false);

#[cfg(unix)]
mod signals {
    #![allow(unsafe_code)]
    use super::STOP;
    use std::sync::atomic::Ordering;

    extern "C" fn on_signal(_: i32) {
        STOP.store(true, Ordering::SeqCst);
    }

    extern "C" {
        fn signal(signum: i32, handler: extern "C" fn(i32)) -> usize;
    }

    /// Ask for SIGINT and SIGTERM to set the stop flag. The handler only
    /// touches an atomic, which is safe in a signal handler.
    pub fn install() {
        const SIGINT: i32 = 2;
        const SIGTERM: i32 = 15;
        // SAFETY: `signal` is the C library function; the handler is
        // async-signal-safe (one atomic store).
        unsafe {
            signal(SIGINT, on_signal);
            signal(SIGTERM, on_signal);
        }
    }
}

#[cfg(not(unix))]
mod signals {
    pub fn install() {}
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match run(&args) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    }
}

fn run(args: &[String]) -> Result<(), Box<dyn std::error::Error>> {
    match cli::parse(args)? {
        Command::Help => {
            print!(
                "{}\n{}\nLORA MODEM:\n{}\n{}",
                cli::USAGE,
                cli::SIK_USAGE,
                oasa_radio::lora::USAGE,
                oasa_radio::waveform_cmd::USAGE
            );
            Ok(())
        }
        Command::Info(None) => {
            for (kind, status, note) in [
                (
                    "udp",
                    "ready",
                    "plain UDP passthrough for simulation and Wi-Fi/Ethernet",
                ),
                (
                    "serial",
                    "ready",
                    "transparent serial radios: SiK, mLRS, RFD900x, UART modems",
                ),
                (
                    "lora",
                    "ready",
                    "OASA modem firmware (Heltec V3): --radio serial for packets, oasa-radio lora for settings. RNode planned",
                ),
                (
                    "sdr",
                    "planned",
                    "our own waveforms on software-defined radios",
                ),
            ] {
                println!(
                    "{{\"ts\":\"{}\",\"event\":\"adapter\",\"kind\":\"{kind}\",\"status\":\"{status}\",\"note\":{}}}",
                    rfc3339(SystemTime::now()),
                    json_str(note)
                );
            }
            Ok(())
        }
        Command::Info(Some(spec)) => {
            let r = spec.open()?;
            let i = r.info();
            let bitrate = i
                .bitrate
                .map(|b| format!(",\"bitrate\":{b}"))
                .unwrap_or_default();
            println!(
                "{{\"ts\":\"{}\",\"event\":\"radio\",\"kind\":\"{}\",\"target\":{},\"mtu\":{}{bitrate}}}",
                rfc3339(SystemTime::now()),
                i.kind,
                json_str(&i.target),
                i.mtu
            );
            Ok(())
        }
        Command::Lora(c) => {
            signals::install();
            let mut out = std::io::stdout();
            oasa_radio::lora::run(&c, &STOP, &mut out).map_err(Into::into)
        }
        Command::Waveform(c) => {
            let mut out = std::io::stdout();
            oasa_radio::waveform_cmd::run(&c, &mut out).map_err(Into::into)
        }
        Command::Sik(c) => sik(&c),
        Command::Bridge(spec, cfg) => {
            if let RadioSpec::Serial(s) = &spec {
                if let Some(expect) = &s.sik_expect {
                    sik_check(&s.device, s.baud, expect)?;
                }
            }
            let mut radio = spec.open()?;
            signals::install();
            let mut out = std::io::stdout();
            let r = bridge::run(radio.as_mut(), &cfg, &STOP, &mut out);
            let _ = out.flush();
            r.map(|_| ()).map_err(Into::into)
        }
    }
}

/// The bridge's settings check (H1): read the SiK radio's settings and refuse
/// to bridge unless they match. Runs before the radio is opened for packets.
fn sik_check(
    device: &str,
    baud: u32,
    expect: &oasa_radio::sik::Expect,
) -> Result<(), Box<dyn std::error::Error>> {
    let ts = || rfc3339(SystemTime::now());
    let report = match oasa_radio::sik::read_report(device, baud, false) {
        Ok(r) => r,
        Err(e) => {
            println!(
                "{{\"ts\":\"{}\",\"event\":\"sik_check\",\"device\":{},\"ok\":false,\"what\":{}}}",
                ts(),
                json_str(device),
                json_str(&e)
            );
            return Err(format!("--sik-expect: cannot read the radio's settings: {e}").into());
        }
    };
    let bad = expect.mismatches(&report);
    let list = bad.iter().map(|m| m.json()).collect::<Vec<_>>().join(",");
    println!(
        "{{\"ts\":\"{}\",\"event\":\"sik_check\",\"device\":{},\"ok\":{},{},\"mismatch\":[{list}]}}",
        ts(),
        json_str(device),
        bad.is_empty(),
        report.json_fields()
    );
    let _ = std::io::stdout().flush();
    if bad.is_empty() {
        Ok(())
    } else {
        let text = bad.iter().map(|m| m.text()).collect::<Vec<_>>().join("; ");
        Err(format!(
            "the SiK radio's settings do not match --sik-expect ({text}). Not bridging. \
             Someone in range may have changed them (RT commands), or the radio was set up \
             differently: check with oasa-radio sik show, fix with oasa-radio sik set"
        )
        .into())
    }
}

fn sik(c: &SikCommand) -> Result<(), Box<dyn std::error::Error>> {
    let ts = rfc3339(SystemTime::now());
    match c {
        SikCommand::Show { device, baud } => {
            let r = oasa_radio::sik::read_report(device, *baud, true)?;
            println!(
                "{{\"ts\":\"{ts}\",\"event\":\"sik\",\"device\":{},{}}}",
                json_str(device),
                r.json_fields()
            );
        }
        SikCommand::Set {
            device,
            baud,
            changes,
        } => {
            let after_baud = changes
                .get("SERIAL_SPEED")
                .and_then(|s| oasa_radio::sik::baud_of(*s))
                .unwrap_or(*baud);
            let (before, after) = oasa_radio::sik::set(device, *baud, changes, after_baud)?;
            let expect = oasa_radio::sik::Expect(changes.clone());
            let bad = expect.mismatches(&after);
            println!(
                "{{\"ts\":\"{ts}\",\"event\":\"sik_set\",\"device\":{},\"ok\":{},\"before\":{{{}}},\"after\":{{{}}}}}",
                json_str(device),
                bad.is_empty(),
                before.json_fields(),
                after.json_fields()
            );
            if !bad.is_empty() {
                let text = bad.iter().map(|m| m.text()).collect::<Vec<_>>().join("; ");
                let hint = if bad.iter().any(|m| m.name == "TXPOWER") {
                    " (SiK rounds TXPOWER up to its board's steps: 1, 2, 5, 8, 11, 14, 17, 20 dBm on 100 mW radios)"
                } else {
                    ""
                };
                return Err(
                    format!("saved, but the radio reads back differently: {text}{hint}").into(),
                );
            }
        }
    }
    Ok(())
}
