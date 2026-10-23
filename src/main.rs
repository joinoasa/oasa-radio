//! `oasa-radio`: bridge oasa-link's UDP packets to a radio. See `--help`.

// The only unsafe code is the signal binding below.
#![deny(unsafe_code)]

use std::io::Write;
use std::process::ExitCode;
use std::sync::atomic::AtomicBool;
use std::time::SystemTime;

use oasa_radio::bridge;
use oasa_radio::cli::{self, Command};
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
            print!("{}", cli::USAGE);
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
                ("lora", "planned", "RNode (KISS) or our own firmware"),
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
        Command::Bridge(spec, cfg) => {
            let mut radio = spec.open()?;
            signals::install();
            let mut out = std::io::stdout();
            let r = bridge::run(radio.as_mut(), &cfg, &STOP, &mut out);
            let _ = out.flush();
            r.map(|_| ()).map_err(Into::into)
        }
    }
}
