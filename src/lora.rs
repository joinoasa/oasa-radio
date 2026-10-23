//! `oasa-radio lora ...`: talk to the OASA LoRa modem's control channel.
//!
//! The modem is the firmware in `firmware/heltec-v3`. Its data path needs no
//! special code: `oasa-radio bridge --radio serial` carries packets. This
//! module reads and sets the radio settings, reads the counters and watches
//! the air. Frame formats: `docs/serial-framing.md`.

use std::collections::VecDeque;
use std::fs::File;
use std::io::{Read, Write};
use std::os::fd::AsFd;
use std::os::unix::fs::OpenOptionsExt;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant, SystemTime};

use nix::poll::{poll, PollFd, PollFlags, PollTimeout};
use oasa_modem_core::control::{
    self, cmd, parse_air, status, AirFrame, Response, SetArgs, Version,
};
use oasa_modem_core::settings::{sync_from_byte, Bandwidth, Settings, SETTINGS_LEN};
use oasa_modem_core::stats::Stats;

use crate::adapters::serial::{baud_rate, set_raw};
use crate::framing::{Frame, Framer, KIND_REQUEST, KIND_RESPONSE};
use crate::log::{json_str, rfc3339};

/// The modem's UART speed (the CP2102 on the Heltec V3 handles it).
pub const DEFAULT_BAUD: u32 = 115_200;
/// Largest packet the modem carries; also the framer's MTU here.
pub const MODEM_MTU: usize = 255;
/// How long to keep trying when the board is still booting after the port opens.
pub const DEFAULT_WAIT_SECS: u64 = 5;
/// How long one request waits for its reply.
const REPLY_TIMEOUT: Duration = Duration::from_millis(400);

#[derive(Debug)]
pub enum LoraError {
    Io(std::io::Error),
    /// No reply in time.
    Timeout,
    /// The modem refused: status code and, if it named one, the setting key.
    Refused {
        status: u8,
        key: Option<u8>,
    },
    /// Something the modem sent did not make sense.
    Protocol(&'static str),
    Config(String),
}

impl std::fmt::Display for LoraError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            LoraError::Io(e) => write!(f, "{e}"),
            LoraError::Timeout => write!(
                f,
                "the modem did not answer (is it the OASA firmware? is the port right?)"
            ),
            LoraError::Refused { status, key } => {
                let what = match *status {
                    status::UNKNOWN_CMD => "unknown command",
                    status::BAD_ARGS => "bad arguments",
                    status::OUT_OF_RANGE => "value out of range",
                    status::RADIO_ERROR => "the radio did not accept it",
                    _ => "refused",
                };
                match key.and_then(key_name) {
                    Some(k) => write!(f, "the modem refused: {what} ({k})"),
                    None => write!(f, "the modem refused: {what}"),
                }
            }
            LoraError::Protocol(s) => write!(f, "unexpected reply from the modem: {s}"),
            LoraError::Config(s) => write!(f, "{s}"),
        }
    }
}

impl std::error::Error for LoraError {}

impl From<std::io::Error> for LoraError {
    fn from(e: std::io::Error) -> Self {
        LoraError::Io(e)
    }
}

fn key_name(k: u8) -> Option<&'static str> {
    use oasa_modem_core::settings::key;
    Some(match k {
        key::FREQ => "frequency",
        key::SF => "spreading factor",
        key::BW => "bandwidth",
        key::CR => "coding rate",
        key::POWER => "power",
        key::SYNC => "sync word",
        key::PREAMBLE => "preamble",
        key::AIR => "air mode",
        _ => return None,
    })
}

/// A frame heard on the air, as the modem reported it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AirReport {
    pub crc_ok: bool,
    pub t_ms: u32,
    pub rssi_qdb: i16,
    pub snr_qdb: i16,
    pub signal_rssi_qdb: i16,
    pub payload: Vec<u8>,
}

/// A serial connection to the modem.
pub struct LoraModem {
    file: File,
    framer: Framer,
    frames: VecDeque<Frame>,
    next_id: u8,
    buf: Vec<u8>,
}

impl LoraModem {
    pub fn open(device: &str, baud: u32) -> Result<Self, LoraError> {
        let rate = baud_rate(baud)
            .ok_or_else(|| LoraError::Config(format!("--baud {baud} is not supported")))?;
        let file = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .custom_flags(nix::libc::O_NOCTTY)
            .open(device)
            .map_err(|e| LoraError::Config(format!("cannot open {device}: {e}")))?;
        set_raw(&file, rate).map_err(|e| LoraError::Config(e.to_string()))?;
        Ok(Self {
            file,
            framer: Framer::new(MODEM_MTU).map_err(LoraError::Config)?,
            frames: VecDeque::new(),
            next_id: 0,
            buf: vec![0; 4096],
        })
    }

    /// Wait up to `timeout` for the next frame (of any kind).
    fn next_frame(&mut self, timeout: Duration) -> Result<Option<Frame>, LoraError> {
        if let Some(f) = self.frames.pop_front() {
            return Ok(Some(f));
        }
        let end = Instant::now() + timeout;
        loop {
            let left = end.saturating_duration_since(Instant::now());
            let ms = u16::try_from(left.as_millis()).unwrap_or(u16::MAX);
            let mut fds = [PollFd::new(self.file.as_fd(), PollFlags::POLLIN)];
            match poll(&mut fds, PollTimeout::from(ms)) {
                Ok(0) => return Ok(None),
                Ok(_) => {}
                Err(nix::errno::Errno::EINTR) => return Ok(None),
                Err(e) => return Err(std::io::Error::from(e).into()),
            }
            let n = match self.file.read(&mut self.buf) {
                Ok(0) => {
                    return Err(std::io::Error::new(
                        std::io::ErrorKind::UnexpectedEof,
                        "the serial device closed",
                    )
                    .into())
                }
                Ok(n) => n,
                Err(e) if e.kind() == std::io::ErrorKind::Interrupted => continue,
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => continue,
                Err(e) => return Err(e.into()),
            };
            self.frames.extend(self.framer.push_frames(&self.buf[..n]));
            if let Some(f) = self.frames.pop_front() {
                return Ok(Some(f));
            }
            if Instant::now() >= end {
                return Ok(None);
            }
        }
    }

    fn send_request(&mut self, c: u8, args: &[u8]) -> Result<u8, LoraError> {
        self.next_id = self.next_id.wrapping_add(1);
        let id = self.next_id;
        let mut body = vec![0u8; 2 + args.len()];
        control::build_request(id, c, args, &mut body);
        let mut wire = Vec::new();
        self.framer
            .encode_ext(KIND_REQUEST, &body, &mut wire)
            .map_err(|e| LoraError::Config(e.to_string()))?;
        self.file.write_all(&wire)?;
        Ok(id)
    }

    /// Send a request once and wait for its reply. Returns the reply data.
    pub fn request_once(&mut self, c: u8, args: &[u8]) -> Result<Vec<u8>, LoraError> {
        let id = self.send_request(c, args)?;
        let end = Instant::now() + REPLY_TIMEOUT;
        loop {
            let left = end.saturating_duration_since(Instant::now());
            match self.next_frame(left)? {
                None => return Err(LoraError::Timeout),
                Some(Frame::Ext { kind, body }) if kind == KIND_RESPONSE => {
                    let Some(r) = Response::parse(&body) else {
                        continue;
                    };
                    if r.req_id != id || r.cmd != c {
                        continue; // an old reply
                    }
                    return reply(r);
                }
                // Data packets, air frames: not ours. Keep waiting.
                Some(_) => {}
            }
            if Instant::now() >= end {
                return Err(LoraError::Timeout);
            }
        }
    }

    /// Like [`Self::request_once`], but keep asking for up to `wait`. Right
    /// after the port opens the board may still be rebooting (the USB-UART
    /// chip pulses its reset line), so the first tries can go unanswered.
    pub fn request(&mut self, c: u8, args: &[u8], wait: Duration) -> Result<Vec<u8>, LoraError> {
        let end = Instant::now() + wait;
        loop {
            match self.request_once(c, args) {
                Err(LoraError::Timeout) if Instant::now() < end => {}
                other => return other,
            }
        }
    }

    pub fn version(&mut self, wait: Duration) -> Result<(u8, [u8; 3], String), LoraError> {
        let d = self.request(cmd::VERSION, &[], wait)?;
        let v = Version::parse(&d).ok_or(LoraError::Protocol("short version"))?;
        Ok((
            v.protocol,
            [v.major, v.minor, v.patch],
            String::from_utf8_lossy(v.board).into_owned(),
        ))
    }

    pub fn get(&mut self) -> Result<Settings, LoraError> {
        let d = self.request(cmd::GET, &[], Duration::ZERO)?;
        Settings::from_bytes(&d).ok_or(LoraError::Protocol("short settings"))
    }

    pub fn set(&mut self, a: &SetArgs) -> Result<Settings, LoraError> {
        let mut buf = [0u8; control::SET_ARGS_MAX];
        let n = a.encode(&mut buf);
        let d = self.request(cmd::SET, &buf[..n], Duration::ZERO)?;
        Settings::from_bytes(&d).ok_or(LoraError::Protocol("short settings"))
    }

    pub fn stats(&mut self) -> Result<Stats, LoraError> {
        let d = self.request(cmd::STATS, &[], Duration::ZERO)?;
        Stats::from_bytes(&d).ok_or(LoraError::Protocol("short stats"))
    }

    pub fn clear_stats(&mut self) -> Result<(), LoraError> {
        self.request(cmd::CLEAR_STATS, &[], Duration::ZERO)
            .map(|_| ())
    }

    pub fn ping(&mut self, payload: &[u8]) -> Result<Vec<u8>, LoraError> {
        self.request(cmd::PING, payload, Duration::ZERO)
    }

    /// Wait up to `timeout` for the next air frame. Other frames are skipped.
    pub fn next_air(&mut self, timeout: Duration) -> Result<Option<AirReport>, LoraError> {
        let end = Instant::now() + timeout;
        loop {
            let left = end.saturating_duration_since(Instant::now());
            match self.next_frame(left)? {
                None => return Ok(None),
                Some(Frame::Ext { kind, body }) if kind == oasa_modem_core::frame::KIND_AIR => {
                    if let Some(AirFrame {
                        crc_ok,
                        t_ms,
                        rssi_qdb,
                        snr_qdb,
                        signal_rssi_qdb,
                        payload,
                    }) = parse_air(&body)
                    {
                        return Ok(Some(AirReport {
                            crc_ok,
                            t_ms,
                            rssi_qdb,
                            snr_qdb,
                            signal_rssi_qdb,
                            payload: payload.to_vec(),
                        }));
                    }
                }
                Some(_) => {}
            }
            if Instant::now() >= end {
                return Ok(None);
            }
        }
    }
}

fn reply(r: Response<'_>) -> Result<Vec<u8>, LoraError> {
    if r.status == status::OK {
        Ok(r.data.to_vec())
    } else {
        Err(LoraError::Refused {
            status: r.status,
            key: r.data.first().copied(),
        })
    }
}

// ---- command line ----

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LoraAction {
    Version,
    Get,
    Set(SetArgs),
    Stats,
    ClearStats,
    Ping,
    Air {
        duration: Option<Duration>,
        hex: bool,
    },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LoraCommand {
    pub device: String,
    pub baud: u32,
    pub wait: Duration,
    pub action: LoraAction,
}

pub const USAGE: &str = "\
  oasa-radio lora ACTION --device PATH [--baud N] [--wait SECS]

  Talk to the OASA LoRa modem (firmware/heltec-v3). Packets themselves go through
  `oasa-radio bridge --radio serial --baud 115200 --mtu 255`. Prints JSON lines.

  ACTION:
    version       firmware and protocol version, board
    get           show the radio settings
    set           change settings; only the options given change, all or none:
                    --profile lora-sf7-bw125   the lab profile (915 MHz, SF7, 125 kHz, 4/5, 2 dBm)
                    --freq MHZ                 for example 915.0 (board range 863 to 928)
                    --sf 5..12   --bw KHZ      7.8 10.4 15.6 20.8 31.25 41.7 62.5 125 250 500
                    --cr 5..8                  or 4/5 .. 4/8
                    --power DBM                -9 to 22. Over coax with attenuators, keep it low.
                    --sync 0x12|0x1424         one byte (SX127x style) or the 16-bit register
                    --preamble SYMBOLS         6 to 65535 (12 or more for SF5 and SF6)
                    --air on|off               report every received frame, CRC failures included
    stats         counters and the last packet's RSSI and SNR
    clear-stats   zero the counters
    ping          check the modem answers
    air           turn air mode on and print every frame heard until --duration SECS or Ctrl-C,
                  then turn it off. Add --hex to print payload bytes: for the lab's captures of
                  encrypted traffic only.

  --wait         Keep trying this long for the first answer (default 5). The board may reset
                 when the port opens.
";

type R<T> = Result<T, String>;

/// Parse `oasa-radio lora ...` (without the word `lora`).
pub fn parse(args: &[String]) -> R<LoraCommand> {
    let mut it = args.iter();
    let action = it
        .next()
        .ok_or("lora needs an action; run oasa-radio --help")?
        .clone();
    let mut named: std::collections::BTreeMap<String, String> = Default::default();
    let mut hex = false;
    while let Some(a) = it.next() {
        if a == "--hex" {
            hex = true;
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
    let baud = match named.remove("baud") {
        None => DEFAULT_BAUD,
        Some(v) => v
            .parse::<u32>()
            .ok()
            .filter(|b| baud_rate(*b).is_some())
            .ok_or(format!("--baud {v} is not supported (try 115200)"))?,
    };
    let wait = match named.remove("wait") {
        None => Duration::from_secs(DEFAULT_WAIT_SECS),
        Some(v) => Duration::from_secs(
            v.parse::<u64>()
                .ok()
                .filter(|s| *s <= 600)
                .ok_or("--wait must be 0 to 600")?,
        ),
    };
    let action = match action.as_str() {
        "version" => LoraAction::Version,
        "get" => LoraAction::Get,
        "stats" => LoraAction::Stats,
        "clear-stats" => LoraAction::ClearStats,
        "ping" => LoraAction::Ping,
        "set" => LoraAction::Set(set_args(&mut named)?),
        "air" => {
            let duration = match named.remove("duration") {
                None => None,
                Some(v) => Some(Duration::from_secs(
                    v.parse::<u64>()
                        .ok()
                        .filter(|s| *s <= 10 * 365 * 86_400)
                        .ok_or("--duration must be a whole number of seconds")?,
                )),
            };
            LoraAction::Air { duration, hex }
        }
        other => {
            return Err(format!(
                "unknown lora action {other:?}; run oasa-radio --help"
            ))
        }
    };
    if hex && !matches!(action, LoraAction::Air { .. }) {
        return Err("--hex only works with the air action".into());
    }
    if let Some(k) = named.keys().next() {
        return Err(format!("unknown option --{k} for this action"));
    }
    Ok(LoraCommand {
        device,
        baud,
        wait,
        action,
    })
}

fn set_args(named: &mut std::collections::BTreeMap<String, String>) -> R<SetArgs> {
    let mut a = SetArgs::default();
    if let Some(p) = named.remove("profile") {
        if p != "lora-sf7-bw125" {
            return Err(format!(
                "unknown profile {p:?}; the only one is lora-sf7-bw125"
            ));
        }
        let s = Settings::LAB_SF7_BW125;
        a = SetArgs {
            freq_hz: Some(s.freq_hz),
            sf: Some(s.sf),
            bw_code: Some(s.bw.code()),
            cr: Some(s.cr),
            tx_power_dbm: Some(s.tx_power_dbm),
            sync_word: Some(s.sync_word),
            preamble: Some(s.preamble),
            air_mode: None,
        };
    }
    if let Some(v) = named.remove("freq") {
        a.freq_hz = Some(parse_mhz(&v).ok_or("--freq must be MHz, like 915.0")?);
    }
    if let Some(v) = named.remove("sf") {
        a.sf = Some(v.parse().map_err(|_| "--sf must be 5 to 12")?);
    }
    if let Some(v) = named.remove("bw") {
        a.bw_code = Some(
            parse_bw(&v)
                .ok_or("--bw must be one of 7.8 10.4 15.6 20.8 31.25 41.7 62.5 125 250 500 (kHz)")?
                .code(),
        );
    }
    if let Some(v) = named.remove("cr") {
        a.cr = Some(parse_cr(&v).ok_or("--cr must be 5 to 8, or 4/5 to 4/8")?);
    }
    if let Some(v) = named.remove("power") {
        a.tx_power_dbm = Some(
            v.parse()
                .map_err(|_| "--power must be a whole number of dBm")?,
        );
    }
    if let Some(v) = named.remove("sync") {
        a.sync_word = Some(parse_sync(&v).ok_or("--sync must be like 0x12 or 0x1424")?);
    }
    if let Some(v) = named.remove("preamble") {
        a.preamble = Some(v.parse().map_err(|_| "--preamble must be 6 to 65535")?);
    }
    if let Some(v) = named.remove("air") {
        a.air_mode = Some(match v.as_str() {
            "on" => true,
            "off" => false,
            _ => return Err("--air must be on or off".into()),
        });
    }
    if a == SetArgs::default() {
        return Err("set needs at least one option; run oasa-radio --help".into());
    }
    Ok(a)
}

/// `915.0` to hertz, exactly (no floating point).
pub fn parse_mhz(s: &str) -> Option<u32> {
    let (int, frac) = s.split_once('.').unwrap_or((s, ""));
    if int.is_empty() || frac.len() > 6 || !int.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    if !frac.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    let mut hz = int.parse::<u64>().ok()?.checked_mul(1_000_000)?;
    let padded = format!("{frac:0<6}");
    hz += padded.parse::<u64>().ok()?;
    u32::try_from(hz).ok()
}

const BW_NAMES: [(&str, Bandwidth); 13] = [
    ("7.8", Bandwidth::Khz7_8),
    ("7.81", Bandwidth::Khz7_8),
    ("10.4", Bandwidth::Khz10_4),
    ("10.42", Bandwidth::Khz10_4),
    ("15.6", Bandwidth::Khz15_6),
    ("15.63", Bandwidth::Khz15_6),
    ("20.8", Bandwidth::Khz20_8),
    ("31.25", Bandwidth::Khz31_25),
    ("41.7", Bandwidth::Khz41_7),
    ("62.5", Bandwidth::Khz62_5),
    ("125", Bandwidth::Khz125),
    ("250", Bandwidth::Khz250),
    ("500", Bandwidth::Khz500),
];

pub fn parse_bw(s: &str) -> Option<Bandwidth> {
    BW_NAMES.iter().find(|(n, _)| *n == s).map(|(_, b)| *b)
}

/// Bandwidth in kHz as people write it.
pub fn bw_label(b: Bandwidth) -> &'static str {
    [
        "7.8", "10.4", "15.6", "20.8", "31.25", "41.7", "62.5", "125", "250", "500",
    ][usize::from(b.code())]
}

pub fn parse_cr(s: &str) -> Option<u8> {
    let n = s.strip_prefix("4/").unwrap_or(s).parse::<u8>().ok()?;
    (5..=8).contains(&n).then_some(n)
}

/// `0x12` (one byte, SX127x style) or `0x1424` (the SX126x register).
pub fn parse_sync(s: &str) -> Option<u16> {
    let v = match s.strip_prefix("0x") {
        Some(h) => u32::from_str_radix(h, 16).ok()?,
        None => s.parse::<u32>().ok()?,
    };
    match v {
        0..=0xFF => Some(sync_from_byte(v as u8)),
        0x100..=0xFFFF => Some(v as u16),
        _ => None,
    }
}

// ---- running ----

fn now() -> String {
    rfc3339(SystemTime::now())
}

fn settings_line(s: &Settings) -> String {
    format!(
        "{{\"ts\":\"{}\",\"event\":\"settings\",\"freq_hz\":{},\"sf\":{},\"bw_khz\":{},\"cr\":\"4/{}\",\"tx_power_dbm\":{},\"sync_word\":\"0x{:04x}\",\"preamble\":{},\"air_mode\":{}}}",
        now(),
        s.freq_hz,
        s.sf,
        bw_label(s.bw),
        s.cr,
        s.tx_power_dbm,
        s.sync_word,
        s.preamble,
        s.air_mode
    )
}

fn q(v: i16) -> String {
    format!("{:.2}", f32::from(v) / 4.0)
}

fn stats_line(s: &Stats) -> String {
    let state = match s.radio_state {
        0 => "fault",
        1 => "rx",
        2 => "tx",
        _ => "unknown",
    };
    let last = if s.last_valid != 0 {
        format!(
            ",\"last_rssi_dbm\":{},\"last_snr_db\":{}",
            q(s.last_rssi_qdb),
            q(s.last_snr_qdb)
        )
    } else {
        String::new()
    };
    format!(
        "{{\"ts\":\"{}\",\"event\":\"stats\",\"uptime_ms\":{},\"radio_state\":\"{state}\",\"tx\":{},\"tx_bytes\":{},\"rx\":{},\"rx_bytes\":{},\"rx_crc_err\":{},\"rx_header_err\":{},\"rx_empty\":{},\"air_frames\":{},\"tx_queue_drop\":{},\"tx_oversize\":{},\"tx_timeouts\":{},\"tx_aborted\":{},\"radio_errors\":{},\"serial_out_drop\":{},\"serial_bad_frame\":{},\"serial_overrun\":{},\"queue_len\":{},\"queue_cap\":{}{last}}}",
        now(),
        s.uptime_ms,
        s.tx_packets,
        s.tx_bytes,
        s.rx_packets,
        s.rx_bytes,
        s.rx_crc_err,
        s.rx_header_err,
        s.rx_empty,
        s.air_frames,
        s.tx_queue_drop,
        s.tx_oversize,
        s.tx_timeouts,
        s.tx_aborted,
        s.radio_errors,
        s.serial_out_drop,
        s.serial_bad_frame,
        s.serial_overrun,
        s.queue_len,
        s.queue_cap
    )
}

/// Run one command, printing JSON lines to `out`. `stop` ends `air`.
pub fn run<W: Write>(c: &LoraCommand, stop: &AtomicBool, out: &mut W) -> Result<(), LoraError> {
    let mut m = LoraModem::open(&c.device, c.baud)?;
    // The first request carries the wait for a board that is still booting.
    let (proto, fw, board) = m.version(c.wait)?;
    match &c.action {
        LoraAction::Version => {
            writeln!(
                out,
                "{{\"ts\":\"{}\",\"event\":\"version\",\"protocol\":{proto},\"firmware\":\"{}.{}.{}\",\"board\":{}}}",
                now(),
                fw[0],
                fw[1],
                fw[2],
                json_str(&board)
            )?;
        }
        LoraAction::Get => writeln!(out, "{}", settings_line(&m.get()?))?,
        LoraAction::Set(a) => writeln!(out, "{}", settings_line(&m.set(a)?))?,
        LoraAction::Stats => writeln!(out, "{}", stats_line(&m.stats()?))?,
        LoraAction::ClearStats => {
            m.clear_stats()?;
            writeln!(out, "{{\"ts\":\"{}\",\"event\":\"cleared\"}}", now())?;
        }
        LoraAction::Ping => {
            let t = Instant::now();
            let data = [0x5A, 0xA5, 0x00, 0xFF];
            if m.ping(&data)? != data {
                return Err(LoraError::Protocol("ping echo differs"));
            }
            writeln!(
                out,
                "{{\"ts\":\"{}\",\"event\":\"pong\",\"rtt_ms\":{:.1}}}",
                now(),
                t.elapsed().as_secs_f64() * 1000.0
            )?;
        }
        LoraAction::Air { duration, hex } => air(&mut m, *duration, *hex, stop, out)?,
    }
    out.flush()?;
    Ok(())
}

fn air<W: Write>(
    m: &mut LoraModem,
    duration: Option<Duration>,
    hex: bool,
    stop: &AtomicBool,
    out: &mut W,
) -> Result<(), LoraError> {
    let before = m.get()?;
    m.set(&SetArgs {
        air_mode: Some(true),
        ..Default::default()
    })?;
    writeln!(
        out,
        "{{\"ts\":\"{}\",\"event\":\"air_start\",\"freq_hz\":{},\"sf\":{},\"bw_khz\":{}}}",
        now(),
        before.freq_hz,
        before.sf,
        bw_label(before.bw)
    )?;
    let end = duration.map(|d| Instant::now() + d);
    let (mut frames, mut bad) = (0u64, 0u64);
    let result = loop {
        if stop.load(Ordering::SeqCst) || end.is_some_and(|e| Instant::now() >= e) {
            break Ok(());
        }
        match m.next_air(Duration::from_millis(200)) {
            Ok(None) => {}
            Ok(Some(f)) => {
                frames += 1;
                bad += u64::from(!f.crc_ok);
                let bytes = if hex {
                    let h: String = f.payload.iter().map(|b| format!("{b:02x}")).collect();
                    format!(",\"hex\":\"{h}\"")
                } else {
                    String::new()
                };
                let _ = writeln!(
                    out,
                    "{{\"ts\":\"{}\",\"event\":\"air\",\"t_ms\":{},\"len\":{},\"crc_ok\":{},\"rssi_dbm\":{},\"snr_db\":{},\"signal_rssi_dbm\":{}{bytes}}}",
                    now(),
                    f.t_ms,
                    f.payload.len(),
                    f.crc_ok,
                    q(f.rssi_qdb),
                    q(f.snr_qdb),
                    q(f.signal_rssi_qdb)
                );
                let _ = out.flush();
            }
            Err(e) => break Err(e),
        }
    };
    // Put air mode back whatever happened.
    let off = m.set(&SetArgs {
        air_mode: Some(before.air_mode),
        ..Default::default()
    });
    writeln!(
        out,
        "{{\"ts\":\"{}\",\"event\":\"air_stop\",\"frames\":{frames},\"crc_bad\":{bad}}}",
        now()
    )?;
    result?;
    off.map(|_| ())
}

const _: () = assert!(SETTINGS_LEN == 13);

#[cfg(test)]
mod tests {
    use super::*;

    fn v(s: &str) -> Vec<String> {
        s.split_whitespace().map(String::from).collect()
    }

    #[test]
    fn mhz_is_exact() {
        assert_eq!(parse_mhz("915.0"), Some(915_000_000));
        assert_eq!(parse_mhz("915"), Some(915_000_000));
        assert_eq!(parse_mhz("868.1"), Some(868_100_000));
        assert_eq!(parse_mhz("902.123456"), Some(902_123_456));
        for bad in [
            "",
            ".5",
            "9.1234567",
            "-1",
            "1e3",
            "abc",
            "5000",
            "4294.967296",
        ] {
            assert_eq!(parse_mhz(bad), None, "{bad}");
        }
        assert_eq!(parse_mhz("4294.967295"), Some(4_294_967_295));
    }

    #[test]
    fn small_parsers() {
        assert_eq!(parse_bw("125"), Some(Bandwidth::Khz125));
        assert_eq!(parse_bw("7.8"), Some(Bandwidth::Khz7_8));
        assert_eq!(parse_bw("126"), None);
        assert_eq!(parse_cr("5"), Some(5));
        assert_eq!(parse_cr("4/8"), Some(8));
        assert_eq!(parse_cr("4/4"), None);
        assert_eq!(parse_cr("9"), None);
        assert_eq!(parse_sync("0x12"), Some(0x1424));
        assert_eq!(parse_sync("0x34"), Some(0x3444));
        assert_eq!(parse_sync("0x1424"), Some(0x1424));
        assert_eq!(parse_sync("0x24B4"), Some(0x24B4));
        assert_eq!(parse_sync("0x10000"), None);
        assert_eq!(parse_sync("zz"), None);
        for b in Bandwidth::ALL {
            assert_eq!(parse_bw(bw_label(b)), Some(b));
        }
    }

    #[test]
    fn parses_actions_and_defaults() {
        let c = parse(&v("get --device /dev/ttyUSB0")).unwrap();
        assert_eq!(c.action, LoraAction::Get);
        assert_eq!((c.baud, c.wait), (115_200, Duration::from_secs(5)));
        let c = parse(&v("air --device d --duration 30 --hex --wait 0")).unwrap();
        assert_eq!(
            c.action,
            LoraAction::Air {
                duration: Some(Duration::from_secs(30)),
                hex: true
            }
        );
        assert_eq!(c.wait, Duration::ZERO);
    }

    #[test]
    fn set_collects_options_and_profile_is_overridable() {
        let c = parse(&v(
            "set --device d --sf 9 --bw 250 --cr 4/6 --power -3 --freq 913.5 --sync 0x34 --preamble 10 --air on",
        ))
        .unwrap();
        let LoraAction::Set(a) = c.action else {
            panic!()
        };
        assert_eq!(
            a,
            SetArgs {
                freq_hz: Some(913_500_000),
                sf: Some(9),
                bw_code: Some(Bandwidth::Khz250.code()),
                cr: Some(6),
                tx_power_dbm: Some(-3),
                sync_word: Some(0x3444),
                preamble: Some(10),
                air_mode: Some(true),
            }
        );
        let c = parse(&v("set --device d --profile lora-sf7-bw125 --power 10")).unwrap();
        let LoraAction::Set(a) = c.action else {
            panic!()
        };
        assert_eq!(
            (a.sf, a.freq_hz, a.tx_power_dbm),
            (Some(7), Some(915_000_000), Some(10))
        );
        assert_eq!(a.air_mode, None);
    }

    #[test]
    fn bad_commands_are_refused() {
        for bad in [
            "",
            "get",
            "frob --device d",
            "get --device d --sf 7",
            "get --device d --hex",
            "set --device d",
            "set --device d --sf x",
            "set --device d --freq 9.1.2",
            "set --device d --bw 126",
            "set --device d --air maybe",
            "set --device d --profile other",
            "get --device d --baud 12345",
            "get --device d --wait 9999",
            "get --device d --device e",
            "air --device d --duration x",
            "get stray --device d",
        ] {
            assert!(parse(&v(bad)).is_err(), "{bad:?}");
        }
    }

    #[test]
    fn lines_are_valid_looking_json() {
        let s = settings_line(&Settings::LAB_SF7_BW125);
        assert!(s.contains("\"freq_hz\":915000000") && s.contains("\"cr\":\"4/5\""));
        assert!(s.contains("\"sync_word\":\"0x1424\"") && s.contains("\"bw_khz\":125"));
        let st = stats_line(&Stats {
            last_valid: 1,
            last_rssi_qdb: -162,
            last_snr_qdb: -10,
            ..Default::default()
        });
        assert!(st.contains("\"last_rssi_dbm\":-40.50") && st.contains("\"last_snr_db\":-2.50"));
        assert!(!stats_line(&Stats::default()).contains("last_rssi"));
    }

    #[test]
    fn error_text_names_the_setting() {
        let e = LoraError::Refused {
            status: status::OUT_OF_RANGE,
            key: Some(oasa_modem_core::settings::key::FREQ),
        };
        assert_eq!(
            e.to_string(),
            "the modem refused: value out of range (frequency)"
        );
    }
}
