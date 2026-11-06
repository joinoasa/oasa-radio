//! Command line for `oasa-radio`.

use std::collections::BTreeMap;
use std::net::{SocketAddr, ToSocketAddrs};
use std::time::Duration;

use crate::adapters::{sdr, serial, udp, SdrConfig, SdrRadio, SerialConfig, SerialRadio, UdpRadio};
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
  oasa-radio sik show|set --device PATH [...]    (SiK radio settings; see SIK RADIOS below)
  oasa-radio lora ACTION --device PATH [...]     (the OASA LoRa modem; see LORA MODEM below)
  oasa-radio waveform ber|iq|decode|df [...]     (OASA's own modem, and two-channel DF; see WAVEFORM below)

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
            --sik-expect LIST    SiK radios: before bridging, read the radio's settings (AT
                                 commands) and refuse to start unless they match, for example
                                 netid=4242,air_speed=64,ecc=0,mavlink=0. Never while bridging.
  lora    LoRa through the OASA modem firmware (firmware/heltec-v3): use `--radio serial
          --baud 115200 --mtu 255` for packets and `oasa-radio lora` for settings.
          Other LoRa boards (RNode): planned.
  sdr     OASA's own waveform (frame v1) on a software-defined radio, as two continuous IQ
          streams (cf32, little endian) that another program moves to and from the radio,
          for example UHD's tx_samples_from_file and rx_samples_to_file through two FIFOs.
          Between frames the TX stream carries zeros, so it never stops. Both streams sit at
          a digital IF; the receiver shifts back, follows the clock offset, decimates and
          decodes as samples arrive. See docs/sdr-adapter.md.
            --tx PATH            TX stream (the radio tool reads it)
            --rx PATH            RX stream (the radio tool writes it)
            --rx-format cf32|ci16  default cf32
            --mod bpsk|qpsk      default qpsk
            --code none|light    default none (light: K=7 rate 1/2, docs/fec-light.md)
            --sample-rate S/s    default 1000000
            --symbol-rate BAUD   default 125000 (sample rate / symbol rate: whole, 4 or more)
            --if-hz HZ           default 250000
            --cfo-hz HZ          starting estimate of the received offset, default 0
            --track-cfo on|off   follow the offset from good frames, default on
            --cfo-search CYC     receiver frequency search, cycles per symbol, above 0, default 0.03
            --decimate 1|2       default 2
            --block N            samples per receiver pass after decimation, default 512 (1 ms)
            --coarse-step N      samples between the positions the detector tests, 0 (default)
                                 = a quarter symbol; 2 at 4 samples per symbol halves its CPU
            --tx-peak A          frame peak amplitude, full scale 1, default 0.5
            --tx-mode stream|burst  default stream. burst: one record per frame (u32 LE
                                 sample count, then cf32), nothing between frames; the radio
                                 tool sends each as a timed burst (lowest delay)
            --tx-lead-ms N       stream: most the TX stream runs ahead of the clock, default 20
            --mtu BYTES          default 200, at most 255. Give oasa-link the same --mtu.
            --sdr-log PATH       JSON lines: frames, offsets, timings (sizes only)
            --rx-record PATH     the received baseband after the shift (cf32), for the lab
            --rx-record-seconds  how much of it, default 60

Prints one JSON line per event: start, drop, drops_suppressed, error. At most 10 drop lines
a second are printed; a drops_suppressed line counts the rest. On exit prints
{\"event\":\"stats\",...} and exits 0. Packet contents are never printed.

Encryption is not done here. Run oasa-link on both ends; the bridge only moves its packets.
";

#[derive(Debug, Clone, PartialEq)]
pub enum RadioSpec {
    Udp {
        bind: SocketAddr,
        remote: SocketAddr,
        mtu: usize,
    },
    Serial(SerialSpec),
    Sdr(SdrConfig),
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SerialSpec {
    pub device: String,
    pub baud: u32,
    pub mtu: usize,
    /// Check the SiK radio's settings before bridging.
    pub sik_expect: Option<crate::sik::Expect>,
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
            RadioSpec::Sdr(c) => Ok(Box::new(SdrRadio::open(c.clone())?)),
        }
    }
}

/// `oasa-radio sik show|set`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SikCommand {
    Show {
        device: String,
        baud: u32,
    },
    Set {
        device: String,
        baud: u32,
        /// SiK name to value.
        changes: BTreeMap<String, u32>,
    },
}

pub const SIK_USAGE: &str = "\
SIK RADIOS (settings over AT commands; never while a bridge has the port):
  oasa-radio sik show --device PATH [--baud N]
      Enter command mode (+++ with 1 s of silence each side), read ATI and ATI5, leave (ATO).
      Prints one JSON line: {\"event\":\"sik\",\"version\":...,\"settings\":{\"NETID\":...}}.
  oasa-radio sik set --device PATH [--baud N] [--netid N] [--air-speed KBPS] [--ecc 0|1]
                     [--mavlink 0|1|2] [--serial-speed 57|115|...] [--txpower DBM]
                     [--min-freq KHZ] [--max-freq KHZ] [--num-channels N] [--duty-cycle PCT]
                     [--lbt-rssi N] [--max-window MS] [--oppresend 0|1] [--manchester 0|1]
                     [--rtscts 0|1] [--any-band]
      Set (ATSn=v), save (AT&W), reboot (ATZ), read back and print before and after.
      TXPOWER steps on 100 mW radios: 1, 2, 5, 8, 11, 14, 17, 20 dBm (others round up).
      MIN_FREQ/MAX_FREQ (kHz) must be inside 414-454, 863-870 or 902-928 MHz unless
      --any-band; the rules where you are are yours to follow.
      Set both radios of a pair the same. For the bridge: --mavlink 0 (transparent bytes) and
      a NETID of your own (see docs/sik.md). NETID is not security; oasa-link is.
  --baud is the computer's side, the radio's SERIAL_SPEED (default 57600).
";

#[derive(Debug, Clone, PartialEq)]
pub enum Command {
    Help,
    Bridge(RadioSpec, BridgeConfig),
    Info(Option<RadioSpec>),
    Lora(crate::lora::LoraCommand),
    Sik(SikCommand),
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
    if args.first().map(String::as_str) == Some("sik") {
        return parse_sik(&args[1..]);
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

fn parse_sik(args: &[String]) -> R<Command> {
    let action = match args.first().map(String::as_str) {
        None | Some("-h" | "--help" | "help") => return Ok(Command::Help),
        Some(a @ ("show" | "set")) => a,
        Some(other) => return Err(format!("unknown sik action {other:?}; use show or set")),
    };
    let mut named: BTreeMap<String, String> = BTreeMap::new();
    let mut it = args[1..].iter();
    let mut any_band = false;
    while let Some(a) = it.next() {
        if matches!(a.as_str(), "-h" | "--help") {
            return Ok(Command::Help);
        }
        if a == "--any-band" && action == "set" {
            any_band = true;
            continue;
        }
        let k = a
            .strip_prefix("--")
            .ok_or(format!("unexpected argument {a:?}; run oasa-radio --help"))?;
        let v = it.next().ok_or(format!("--{k} needs a value"))?;
        if named.insert(k.to_string(), v.clone()).is_some() {
            return Err(format!("--{k} given twice"));
        }
    }
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
            "--baud {baud} is not supported (try 57600, 115200)"
        ));
    }
    if action == "show" {
        if let Some(k) = named.keys().next() {
            return Err(format!("unknown option --{k} for sik show"));
        }
        return Ok(Command::Sik(SikCommand::Show { device, baud }));
    }
    let mut changes = BTreeMap::new();
    for (opt, name) in crate::sik::SETTABLE {
        if let Some(v) = named.remove(*opt) {
            let n: u32 = v
                .parse()
                .map_err(|_| format!("--{opt} needs a whole number"))?;
            let band_skip = any_band && matches!(*name, "MIN_FREQ" | "MAX_FREQ");
            if !band_skip {
                crate::sik::check_value(name, n).map_err(|e| {
                    if matches!(*name, "MIN_FREQ" | "MAX_FREQ") {
                        format!("--{opt}: {e} (outside 414-454, 863-870 and 902-928 MHz; --any-band to allow)")
                    } else {
                        format!("--{opt}: {e}")
                    }
                })?;
            }
            changes.insert((*name).to_string(), n);
        }
    }
    if let Some(k) = named.keys().next() {
        return Err(format!("unknown option --{k} for sik set"));
    }
    if let (Some(lo), Some(hi)) = (changes.get("MIN_FREQ"), changes.get("MAX_FREQ")) {
        if lo >= hi {
            return Err("--min-freq must be below --max-freq".into());
        }
    }
    if changes.is_empty() {
        return Err("sik set: nothing to set (for example --netid 4242 --mavlink 0)".into());
    }
    Ok(Command::Sik(SikCommand::Set {
        device,
        baud,
        changes,
    }))
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

fn real(k: &str, v: Option<String>, default: f64) -> R<f64> {
    match v {
        None => Ok(default),
        Some(v) => v
            .parse::<f64>()
            .ok()
            .filter(|x| x.is_finite())
            .ok_or(format!("--{k} must be a number")),
    }
}

fn sdr_spec(named: &mut BTreeMap<String, String>) -> R<SdrConfig> {
    let tx = named
        .remove("tx")
        .ok_or("missing --tx (the TX stream: a FIFO the radio tool reads)")?;
    let rx = named
        .remove("rx")
        .ok_or("missing --rx (the RX stream: a FIFO the radio tool writes)")?;
    let mut c = SdrConfig::new(tx.into(), rx.into());
    if let Some(v) = named.remove("rx-format") {
        c.rx_format = sdr::IqFormat::parse(&v).ok_or("--rx-format must be cf32 or ci16")?;
    }
    if let Some(v) = named.remove("mod") {
        c.modulation = oasa_waveform::Modulation::parse(&v)
            .filter(|m| *m != oasa_waveform::Modulation::Fsk2)
            .ok_or("--mod must be bpsk or qpsk")?;
    }
    if let Some(v) = named.remove("code") {
        c.code =
            oasa_waveform::frame::PayloadCode::parse(&v).ok_or("--code must be none or light")?;
    }
    c.sample_rate = real("sample-rate", named.remove("sample-rate"), c.sample_rate)?;
    c.symbol_rate = real("symbol-rate", named.remove("symbol-rate"), c.symbol_rate)?;
    c.if_hz = real("if-hz", named.remove("if-hz"), c.if_hz)?;
    c.cfo_hz = real("cfo-hz", named.remove("cfo-hz"), c.cfo_hz)?;
    c.cfo_search = real(
        "cfo-search",
        named.remove("cfo-search"),
        f64::from(c.cfo_search),
    )? as f32;
    c.tx_peak = real("tx-peak", named.remove("tx-peak"), f64::from(c.tx_peak))? as f32;
    c.rx_record_seconds = real(
        "rx-record-seconds",
        named.remove("rx-record-seconds"),
        c.rx_record_seconds,
    )?;
    if let Some(v) = named.remove("track-cfo") {
        c.track_cfo = match v.as_str() {
            "on" => true,
            "off" => false,
            _ => return Err("--track-cfo must be on or off".into()),
        };
    }
    c.decimate = number(
        "decimate",
        named.remove("decimate"),
        c.decimate as u64,
        1,
        2,
    )? as usize;
    c.block = number("block", named.remove("block"), c.block as u64, 64, 1 << 20)? as usize;
    c.coarse_step = number(
        "coarse-step",
        named.remove("coarse-step"),
        c.coarse_step as u64,
        0,
        64,
    )? as usize;
    c.mtu = number(
        "mtu",
        named.remove("mtu"),
        c.mtu as u64,
        1,
        sdr::MAX_MTU as u64,
    )? as usize;
    c.tx_lead = Duration::from_millis(number(
        "tx-lead-ms",
        named.remove("tx-lead-ms"),
        c.tx_lead.as_millis() as u64,
        1,
        1000,
    )?);
    if let Some(v) = named.remove("tx-mode") {
        c.tx_mode = sdr::TxMode::parse(&v).ok_or("--tx-mode must be stream or burst")?;
    }
    c.log = named.remove("sdr-log").map(Into::into);
    c.rx_record = named.remove("rx-record").map(Into::into);
    c.check()?;
    Ok(c)
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
            let sik_expect = named
                .remove("sik-expect")
                .map(|v| crate::sik::Expect::parse(&v))
                .transpose()?;
            Ok(RadioSpec::Serial(SerialSpec {
                device,
                baud,
                mtu,
                sik_expect,
            }))
        }
        "lora" => Err(
            "there is no separate lora adapter: with the OASA modem firmware use --radio serial \
             --baud 115200 --mtu 255 (settings: oasa-radio lora). RNode support is planned"
                .to_string(),
        ),
        "sdr" => sdr_spec(named).map(RadioSpec::Sdr),
        other => Err(format!("unknown radio {other:?}; use udp, serial or sdr")),
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
            format!("{ok} --sik-expect netid"),
            format!("{ok} --sik-expect mavlink=9"),
            format!("bridge --radio udp --bind 127.0.0.1:5 --remote 127.0.0.1:6 {UDPS} --sik-expect netid=1"),
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
    fn sik_expect_on_the_bridge() {
        let c = parse(&v(&format!(
            "bridge --radio serial --device d --sik-expect netid=7,mavlink=0 {UDPS}"
        )))
        .unwrap();
        match c {
            Command::Bridge(RadioSpec::Serial(s), _) => {
                let e = s.sik_expect.unwrap();
                assert_eq!(e.0["NETID"], 7);
                assert_eq!(e.0["MAVLINK"], 0);
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn sik_commands_parse() {
        assert_eq!(
            parse(&v("sik show --device /dev/x")).unwrap(),
            Command::Sik(SikCommand::Show {
                device: "/dev/x".into(),
                baud: 57_600
            })
        );
        match parse(&v(
            "sik set --device d --baud 115200 --netid 4242 --mavlink 0 --air-speed 64",
        ))
        .unwrap()
        {
            Command::Sik(SikCommand::Set { baud, changes, .. }) => {
                assert_eq!(baud, 115_200);
                assert_eq!(changes.len(), 3);
                assert_eq!(changes["AIR_SPEED"], 64);
            }
            other => panic!("{other:?}"),
        }
        assert_eq!(parse(&v("sik")).unwrap(), Command::Help);
        assert!(parse(&v("sik set --device d --max-freq 950000 --any-band")).is_ok());
        assert!(parse(&v("sik set --device d --min-freq 902000 --max-freq 928000")).is_ok());
        for bad in [
            "sik frob --device d",
            "sik show",
            "sik show --device d --netid 1",
            "sik set --device d",
            "sik set --device d --netid 70000",
            "sik set --device d --mavlink 3",
            "sik set --device d --air-speed 65",
            "sik set --device d --serial-speed 56",
            "sik set --device d --netid 1 --netid 2",
            "sik set --device d --format 1",
            "sik set --device d --max-freq 950000",
            "sik set --device d --min-freq 920000 --max-freq 910000",
            "sik show --device d --any-band",
            "sik set --device d --baud 7 --netid 1",
            "sik set --device d stray",
        ] {
            assert!(parse(&v(bad)).is_err(), "{bad}");
        }
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
