//! Command line for `oasa-radio`.

use std::collections::BTreeMap;
use std::net::{SocketAddr, ToSocketAddrs};
use std::time::Duration;

use crate::adapters::{serial, udp, SerialConfig, SerialRadio, UdpRadio};
use crate::bridge::BridgeConfig;
use crate::framing;
use crate::radio::{Radio, RadioError};

pub const USAGE: &str = "\
oasa-radio: carry oasa-link's encrypted packets over a radio (R-COM1).

USAGE:
  oasa-radio bridge --radio KIND [radio options]
                    --udp-listen HOST:PORT --udp-send HOST:PORT
                    [--allow-remote-udp] [--duration SECS]
  oasa-radio info [--radio KIND [radio options]]
  oasa-radio lora ACTION --device PATH [...]     (the OASA LoRa modem; see LORA MODEM below)
  oasa-radio waveform ber|iq|decode [...]        (OASA's own modem in simulation; see WAVEFORM below)

  --udp-listen   oasa-link's --remote points here. Packets from oasa-link go out the
                 radio. Must be a loopback address unless --allow-remote-udp is given.
  --udp-send     oasa-link's --bind. Packets from the radio go here. Packets are accepted
                 on --udp-listen only from this host unless --allow-remote-udp is given.
  --duration     Stop after this many seconds. Stops on SIGINT or SIGTERM too.

RADIOS:
  udp     Plain UDP passthrough, for simulation and Wi-Fi/Ethernet.
            --bind HOST:PORT     local address
            --remote HOST:PORT   where the other end is
            --mtu BYTES          default 1400, at most 65507
  serial  Transparent serial radios (SiK, mLRS, RFD900x, UART modems).
            --device PATH        for example /dev/ttyUSB0
            --baud N             default 57600
            --mtu BYTES          default 252, at most 65535. Give oasa-link the same --mtu.
  lora    LoRa through the OASA modem firmware (firmware/heltec-v3): use `--radio serial
          --baud 115200 --mtu 255` for packets and `oasa-radio lora` for settings.
          Other LoRa boards (RNode): planned.
  sdr     Planned. Not built yet.

Prints one JSON line per event: start, drop, drops_suppressed, error. At most 10 drop lines
a second are printed; a drops_suppressed line counts the rest. On exit prints
{\"event\":\"stats\",...} and exits 0. Packet contents are never printed.

Encryption is not done here. Run oasa-link on both ends; the bridge only moves its packets.
";

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RadioSpec {
    Udp {
        bind: SocketAddr,
        remote: SocketAddr,
        mtu: usize,
    },
    Serial(SerialSpec),
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SerialSpec {
    pub device: String,
    pub baud: u32,
    pub mtu: usize,
}

impl RadioSpec {
    pub fn open(&self) -> Result<Box<dyn Radio>, RadioError> {
        match self {
            RadioSpec::Udp { bind, remote, mtu } => {
                Ok(Box::new(UdpRadio::new(*bind, *remote, *mtu)?))
            }
            RadioSpec::Serial(s) => Ok(Box::new(SerialRadio::open(SerialConfig {
                device: s.device.clone(),
                baud: s.baud,
                mtu: s.mtu,
            })?)),
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum Command {
    Help,
    Bridge(RadioSpec, BridgeConfig),
    Info(Option<RadioSpec>),
    Lora(crate::lora::LoraCommand),
    Waveform(crate::waveform_cmd::WaveformCommand),
}

/// About ten years, in seconds.
const MAX_SECS: u64 = 10 * 365 * 86_400;

type R<T> = Result<T, String>;

pub fn parse(args: &[String]) -> R<Command> {
    if args.first().map(String::as_str) == Some("lora") {
        if args
            .get(1)
            .map(String::as_str)
            .is_none_or(|a| matches!(a, "-h" | "--help" | "help"))
        {
            return Ok(Command::Help);
        }
        return crate::lora::parse(&args[1..]).map(Command::Lora);
    }
    if args.first().map(String::as_str) == Some("waveform") {
        return crate::waveform_cmd::parse(&args[1..]).map(Command::Waveform);
    }
    let mut it = args.iter();
    let cmd = match it.next().map(String::as_str) {
        None | Some("-h" | "--help" | "help") => return Ok(Command::Help),
        Some(c @ ("bridge" | "info")) => c,
        Some(other) => return Err(format!("unknown command {other:?}; run oasa-radio --help")),
    };
    let mut named: BTreeMap<String, String> = BTreeMap::new();
    let mut allow_remote = false;
    while let Some(a) = it.next() {
        match a.as_str() {
            "-h" | "--help" => return Ok(Command::Help),
            "--allow-remote-udp" if cmd == "bridge" => allow_remote = true,
            _ => {
                let k = a
                    .strip_prefix("--")
                    .ok_or(format!("unexpected argument {a:?}; run oasa-radio --help"))?;
                let v = it.next().ok_or(format!("--{k} needs a value"))?;
                if named.insert(k.to_string(), v.clone()).is_some() {
                    return Err(format!("--{k} given twice"));
                }
            }
        }
    }
    let radio = match named.remove("radio") {
        None => None,
        Some(kind) => Some(radio_spec(&kind, &mut named)?),
    };
    if cmd == "info" {
        if let Some(k) = named.keys().next() {
            return Err(format!(
                "unknown option --{k} for info; run oasa-radio --help"
            ));
        }
        return Ok(Command::Info(radio));
    }
    let radio = radio.ok_or("missing --radio")?;
    let udp_listen = addr("udp-listen", named.remove("udp-listen"))?;
    let udp_send = addr("udp-send", named.remove("udp-send"))?;
    let duration = match named.remove("duration") {
        None => None,
        Some(v) => {
            let n: u64 = v.parse().map_err(|_| "--duration must be a whole number")?;
            if n > MAX_SECS {
                return Err(format!("--duration is too large (at most {MAX_SECS})"));
            }
            Some(Duration::from_secs(n))
        }
    };
    if let Some(k) = named.keys().next() {
        return Err(format!("unknown option --{k}; run oasa-radio --help"));
    }
    let cfg = BridgeConfig {
        udp_listen,
        udp_send,
        allow_remote_udp: allow_remote,
        duration,
    };
    crate::bridge::check(&cfg)?;
    Ok(Command::Bridge(radio, cfg))
}

fn addr(k: &str, v: Option<String>) -> R<SocketAddr> {
    let v = v.ok_or(format!("missing --{k}"))?;
    v.to_socket_addrs()
        .map_err(|e| format!("--{k} {v}: {e}"))?
        .next()
        .ok_or(format!("--{k} {v}: no address"))
}

fn number(k: &str, v: Option<String>, default: u64, min: u64, max: u64) -> R<u64> {
    match v {
        None => Ok(default),
        Some(v) => v
            .parse::<u64>()
            .ok()
            .filter(|n| (min..=max).contains(n))
            .ok_or(format!("--{k} must be {min} to {max}")),
    }
}

/// Take this radio's options out of `named`.
fn radio_spec(kind: &str, named: &mut BTreeMap<String, String>) -> R<RadioSpec> {
    match kind {
        "udp" => {
            let bind = addr("bind", named.remove("bind"))?;
            let remote = addr("remote", named.remove("remote"))?;
            let mtu = number(
                "mtu",
                named.remove("mtu"),
                udp::DEFAULT_MTU as u64,
                1,
                udp::MAX_MTU as u64,
            )? as usize;
            Ok(RadioSpec::Udp { bind, remote, mtu })
        }
        "serial" => {
            let device = named.remove("device").ok_or("missing --device")?;
            let baud = number(
                "baud",
                named.remove("baud"),
                u64::from(serial::DEFAULT_BAUD),
                1,
                u64::from(u32::MAX),
            )? as u32;
            if serial::baud_rate(baud).is_none() {
                return Err(format!(
                    "--baud {baud} is not supported (try 9600, 57600, 115200)"
                ));
            }
            let mtu = number(
                "mtu",
                named.remove("mtu"),
                framing::DEFAULT_MTU as u64,
                1,
                framing::MAX_MTU as u64,
            )? as usize;
            Ok(RadioSpec::Serial(SerialSpec { device, baud, mtu }))
        }
        "lora" => Err(
            "there is no separate lora adapter: with the OASA modem firmware use --radio serial \
             --baud 115200 --mtu 255 (settings: oasa-radio lora). RNode support is planned"
                .to_string(),
        ),
        "sdr" => Err(format!(
            "radio {kind:?} is planned and not built yet; see the README (Adapters)"
        )),
        other => Err(format!("unknown radio {other:?}; use udp or serial")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn v(s: &str) -> Vec<String> {
        s.split_whitespace().map(String::from).collect()
    }

    const UDPS: &str = "--udp-listen 127.0.0.1:2 --udp-send 127.0.0.1:3";

    #[test]
    fn serial_bridge_parses_with_defaults() {
        let c = parse(&v(&format!(
            "bridge --radio serial --device /dev/ttyUSB0 {UDPS}"
        )))
        .unwrap();
        match c {
            Command::Bridge(RadioSpec::Serial(s), b) => {
                assert_eq!(s.baud, 57_600);
                assert_eq!(s.mtu, 252);
                assert_eq!(b.duration, None);
                assert!(!b.allow_remote_udp);
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn udp_bridge_parses() {
        let c = parse(&v(&format!(
            "bridge --radio udp --bind 127.0.0.1:5 --remote 127.0.0.1:6 --mtu 300 {UDPS} --duration 9"
        )))
        .unwrap();
        match c {
            Command::Bridge(RadioSpec::Udp { mtu, .. }, b) => {
                assert_eq!(mtu, 300);
                assert_eq!(b.duration, Some(Duration::from_secs(9)));
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn bad_arguments_are_refused() {
        let ok = format!("bridge --radio serial --device d {UDPS}");
        assert!(parse(&v(&ok)).is_ok());
        for bad in [
            "bridge".to_string(),
            format!("bridge --radio serial {UDPS}"), // no device
            format!("{ok} --baud 12345"),            // unsupported
            format!("{ok} --baud x"),
            format!("{ok} --mtu 0"),
            format!("{ok} --mtu 65536"),
            format!("{ok} --bind 127.0.0.1:1"), // udp option on serial
            format!("{ok} --nonsense 1"),
            format!("{ok} --duration 99999999999"),
            format!("{ok} --duration x"),
            format!("{ok} --device e"), // twice
            format!("{ok} stray"),
            format!("bridge --radio lora --device d {UDPS}"),
            format!("bridge --radio sdr {UDPS}"),
            format!("bridge --radio wifi {UDPS}"),
            "bridge --radio serial --device d --udp-send 127.0.0.1:3".to_string(),
            "bridge --radio serial --device d --udp-listen 127.0.0.1:3".to_string(),
            "frobnicate".to_string(),
            "info --udp-listen 127.0.0.1:1".to_string(),
        ] {
            assert!(parse(&v(&bad)).is_err(), "{bad}");
        }
    }

    #[test]
    fn udp_listen_must_be_loopback_unless_allowed() {
        let base = "bridge --radio serial --device d --udp-send 127.0.0.1:3";
        let e = parse(&v(&format!("{base} --udp-listen 0.0.0.0:2"))).unwrap_err();
        assert!(e.contains("loopback"), "{e}");
        assert!(parse(&v(&format!("{base} --udp-listen 192.168.1.5:2"))).is_err());
        assert!(parse(&v(&format!(
            "{base} --udp-listen 0.0.0.0:2 --allow-remote-udp"
        )))
        .is_ok());
        assert!(parse(&v(&format!("{base} --udp-listen [::1]:2"))).is_ok());
    }

    #[test]
    fn help_and_info() {
        assert_eq!(parse(&[]).unwrap(), Command::Help);
        assert_eq!(parse(&v("--help")).unwrap(), Command::Help);
        assert_eq!(parse(&v("bridge --help")).unwrap(), Command::Help);
        assert_eq!(parse(&v("info")).unwrap(), Command::Info(None));
        assert!(matches!(
            parse(&v("info --radio serial --device d --baud 9600")).unwrap(),
            Command::Info(Some(RadioSpec::Serial(_)))
        ));
    }
}
