//! SiK telemetry radios: read and set their settings with AT commands, and
//! check them before the bridge starts (H1 in `docs/security-review-r-com1.md`).
//!
//! SiK behaviour used here (ArduPilot SiK firmware, `Firmware/radio/at.c`,
//! `serial.c`, `parameters.c`; see `docs/sik.md`):
//!
//! - `+++` with one second of silence on the serial line before and after
//!   enters command mode; the radio then prints `OK`.
//! - In command mode the radio echoes what it gets, runs a line on CR and ends
//!   its lines with CR LF. `ATI` prints the version, `ATI5` one `Sn:NAME=value`
//!   line per setting, `ATSn=v` sets one (`OK` or `ERROR`), `AT&W` saves,
//!   `ATZ` reboots, `ATO` leaves command mode.
//!
//! This runs only before bridging. The bridge never sends AT commands, and no
//! AT code sees the bytes it carries. Bytes that come from the radio while we
//! talk to it may be packets from the air: they are thrown away, never printed,
//! and only lines that look exactly like a setting are read.

use std::collections::BTreeMap;
use std::fs::File;
use std::io::{Read, Write};
use std::os::fd::AsFd;
use std::os::unix::fs::OpenOptionsExt;
use std::time::{Duration, Instant};

use nix::poll::{poll, PollFd, PollFlags, PollTimeout};
use nix::sys::termios::{self, FlushArg};

use crate::adapters::serial::{baud_rate, set_raw};

/// Silence before and after `+++`. SiK wants 1 s (`ATP_COUNT_1S`, 100 ticks of
/// its 100 Hz timer); the extra 0.2 s covers USB latency.
pub const GUARD: Duration = Duration::from_millis(1200);
/// How long to wait for `OK` after the second guard.
const ENTER_WAIT: Duration = Duration::from_millis(1500);
/// A reply is complete when the radio has been quiet this long.
const REPLY_IDLE: Duration = Duration::from_millis(300);
/// The longest wait for one reply.
const REPLY_MAX: Duration = Duration::from_secs(3);
/// The longest wait for the answer to an `RT` command (est.).
const REMOTE_MAX: Duration = Duration::from_secs(5);
/// What we keep of a reply. ATI5 is about 400 bytes; anything longer is not a reply.
const REPLY_CAP: usize = 4096;
/// How long a SiK radio takes to boot after `ATZ` (est.; it is about 1 s).
pub const BOOT: Duration = Duration::from_secs(3);

/// The settings `oasa-radio sik set` can change, by option name.
/// (option, SiK name, allowed values or `None` for a range check below)
pub const SETTABLE: &[(&str, &str)] = &[
    ("serial-speed", "SERIAL_SPEED"),
    ("air-speed", "AIR_SPEED"),
    ("netid", "NETID"),
    ("txpower", "TXPOWER"),
    ("ecc", "ECC"),
    ("mavlink", "MAVLINK"),
    ("oppresend", "OPPRESEND"),
    ("min-freq", "MIN_FREQ"),
    ("max-freq", "MAX_FREQ"),
    ("num-channels", "NUM_CHANNELS"),
    ("duty-cycle", "DUTY_CYCLE"),
    ("lbt-rssi", "LBT_RSSI"),
    ("manchester", "MANCHESTER"),
    ("rtscts", "RTSCTS"),
    ("max-window", "MAX_WINDOW"),
];

/// SiK's one-byte serial speeds (`SERIAL_SPEED`) and the baud they mean.
pub const SERIAL_SPEEDS: &[(u32, u32)] = &[
    (1, 1200),
    (2, 2400),
    (4, 4800),
    (9, 9600),
    (19, 19_200),
    (38, 38_400),
    (57, 57_600),
    (115, 115_200),
    (230, 230_400),
];

/// Air speeds in kbps that SiK radios support (the ArduPilot SiK page).
pub const AIR_SPEEDS: &[u32] = &[2, 4, 8, 16, 19, 24, 32, 48, 64, 96, 128, 192, 250];

/// Check one value for `sik set` and `--sik-expect`. `name` is the SiK name.
pub fn check_value(name: &str, v: u32) -> Result<(), String> {
    let ok = match name {
        "SERIAL_SPEED" => SERIAL_SPEEDS.iter().any(|(s, _)| *s == v),
        "AIR_SPEED" => AIR_SPEEDS.contains(&v),
        // radio_set_network_id(uint16_t)
        "NETID" => v <= 65_535,
        "TXPOWER" => v <= 30,
        "ECC" | "OPPRESEND" | "MANCHESTER" | "RTSCTS" => v <= 1,
        "MAVLINK" => v <= 2,
        "DUTY_CYCLE" => v <= 100,
        "LBT_RSSI" => v == 0 || (25..=220).contains(&v),
        "MAX_WINDOW" => (20..=131).contains(&v),
        "NUM_CHANNELS" => (1..=50).contains(&v),
        _ => true,
    };
    if ok {
        Ok(())
    } else {
        Err(format!("{name}={v} is not a value SiK accepts"))
    }
}

/// The baud a `SERIAL_SPEED` value means.
pub fn baud_of(serial_speed: u32) -> Option<u32> {
    SERIAL_SPEEDS
        .iter()
        .find(|(s, _)| *s == serial_speed)
        .map(|(_, b)| *b)
}

/// One setting as `ATI5` prints it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Param {
    pub reg: u8,
    pub name: String,
    pub value: u32,
}

/// What a radio reports.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Report {
    /// The `ATI` banner, for example `SiK 2.2 on HM-TRP`.
    pub version: String,
    pub params: Vec<Param>,
}

impl Report {
    pub fn get(&self, name: &str) -> Option<u32> {
        self.params.iter().find(|p| p.name == name).map(|p| p.value)
    }

    /// `{"version":...,"settings":{"NAME":value,...}}` (in register order).
    pub fn json_fields(&self) -> String {
        let settings = self
            .params
            .iter()
            .map(|p| format!("\"{}\":{}", p.name, p.value))
            .collect::<Vec<_>>()
            .join(",");
        format!(
            "\"version\":{},\"settings\":{{{settings}}}",
            crate::log::json_str(&self.version)
        )
    }
}

/// Settings the radio must have: `NAME=value` pairs.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Expect(pub BTreeMap<String, u32>);

impl Expect {
    /// `netid=4242,air_speed=64,ecc=0,mavlink=0`. Names are SiK's (any case;
    /// `-` for `_` is fine). Values are checked like `sik set`'s.
    pub fn parse(spec: &str) -> Result<Self, String> {
        let mut m = BTreeMap::new();
        for item in spec.split(',').map(str::trim).filter(|s| !s.is_empty()) {
            let (k, v) = item
                .split_once('=')
                .ok_or(format!("--sik-expect: {item:?} is not NAME=VALUE"))?;
            let name = k.trim().to_ascii_uppercase().replace('-', "_");
            if name.is_empty()
                || name.len() > 32
                || !name
                    .bytes()
                    .all(|b| b.is_ascii_uppercase() || b.is_ascii_digit() || b == b'_')
            {
                return Err(format!("--sik-expect: {k:?} is not a SiK setting name"));
            }
            if name == "FORMAT" {
                return Err("--sik-expect: FORMAT is the firmware's, not a setting".into());
            }
            let v: u32 = v
                .trim()
                .parse()
                .map_err(|_| format!("--sik-expect: {name} needs a whole number"))?;
            check_value(&name, v).map_err(|e| format!("--sik-expect: {e}"))?;
            if m.insert(name.clone(), v).is_some() {
                return Err(format!("--sik-expect: {name} given twice"));
            }
        }
        if m.is_empty() {
            return Err("--sik-expect needs at least one NAME=VALUE".into());
        }
        Ok(Self(m))
    }

    /// Differences between this and what the radio reports.
    pub fn mismatches(&self, r: &Report) -> Vec<Mismatch> {
        self.0
            .iter()
            .filter_map(|(name, want)| {
                let got = r.get(name);
                (got != Some(*want)).then(|| Mismatch {
                    name: name.clone(),
                    want: *want,
                    got,
                })
            })
            .collect()
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Mismatch {
    pub name: String,
    pub want: u32,
    /// `None`: the radio does not report this setting.
    pub got: Option<u32>,
}

impl Mismatch {
    pub fn json(&self) -> String {
        let got = self.got.map_or("null".to_string(), |g| g.to_string());
        format!(
            "{{\"name\":{},\"want\":{},\"got\":{got}}}",
            crate::log::json_str(&self.name),
            self.want
        )
    }

    pub fn text(&self) -> String {
        match self.got {
            Some(g) => format!("{} is {g}, expected {}", self.name, self.want),
            None => format!("{} is not reported by the radio", self.name),
        }
    }
}

/// Lines of a reply: split on LF, CR dropped. Bytes that are not printable
/// ASCII make a line unreadable (it is skipped, never shown).
fn lines(reply: &[u8]) -> Vec<Option<String>> {
    reply
        .split(|b| *b == b'\n')
        .map(|l| {
            let l: Vec<u8> = l.iter().copied().filter(|b| *b != b'\r').collect();
            if l.iter().all(|b| (0x20..0x7f).contains(b)) {
                Some(String::from_utf8_lossy(&l).trim().to_string())
            } else {
                None
            }
        })
        .collect()
}

/// The reply that follows the echo of `cmd`, if the echo is there.
fn after_echo(reply: &[u8], cmd: &str) -> Vec<Option<String>> {
    let ls = lines(reply);
    match ls.iter().rposition(|l| l.as_deref() == Some(cmd)) {
        Some(i) => ls[i + 1..].to_vec(),
        None => ls,
    }
}

/// Parse an `ATI5` reply: `S<reg>:<NAME>=<value>` lines. Anything else after
/// the echo except `OK` and empty lines is an error (counted, never shown).
pub fn parse_ati5(reply: &[u8]) -> Result<Vec<Param>, String> {
    let mut out: Vec<Param> = Vec::new();
    let mut other = 0usize;
    for l in after_echo(reply, "ATI5") {
        let Some(l) = l else {
            other += 1;
            continue;
        };
        if l.is_empty() || l == "OK" {
            continue;
        }
        match parse_param(&l) {
            Some(p) => {
                if out.iter().any(|q| q.reg == p.reg || q.name == p.name) {
                    return Err(format!("ATI5 lists S{} or {} twice", p.reg, p.name));
                }
                out.push(p);
            }
            None => other += 1,
        }
    }
    if other > 0 {
        return Err(format!(
            "ATI5 reply has {other} line(s) that are not settings (wrong --baud, or not a SiK radio?)"
        ));
    }
    if out.is_empty() {
        return Err("ATI5 gave no settings (wrong --baud, or not a SiK radio?)".into());
    }
    Ok(out)
}

fn parse_param(l: &str) -> Option<Param> {
    let rest = l.strip_prefix('S')?;
    let (reg, rest) = rest.split_once(':')?;
    let (name, value) = rest.split_once('=')?;
    if reg.is_empty() || reg.len() > 3 || !reg.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    if name.is_empty()
        || name.len() > 32
        || !name
            .bytes()
            .all(|b| b.is_ascii_uppercase() || b.is_ascii_digit() || b == b'_')
    {
        return None;
    }
    if value.is_empty() || value.len() > 10 || !value.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    Some(Param {
        reg: reg.parse().ok()?,
        name: name.to_string(),
        value: value.parse().ok()?,
    })
}

/// The `ATI` banner: the first printable line after the echo, at most 64 characters.
pub fn parse_ati(reply: &[u8]) -> Result<String, String> {
    after_echo(reply, "ATI")
        .into_iter()
        .flatten()
        .find(|l| !l.is_empty() && l != "OK")
        .filter(|l| l.len() <= 64)
        .ok_or_else(|| "ATI gave no version line".to_string())
}

/// Does the reply hold a line that is exactly `word`?
fn has_line(reply: &[u8], word: &str) -> bool {
    lines(reply).iter().any(|l| l.as_deref() == Some(word))
}

/// A serial port talking to a SiK radio in command mode.
pub struct AtPort {
    file: File,
    device: String,
}

impl AtPort {
    pub fn open(device: &str, baud: u32) -> Result<Self, String> {
        let rate = baud_rate(baud).ok_or(format!("--baud {baud} is not supported"))?;
        let file = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .custom_flags(nix::libc::O_NOCTTY)
            .open(device)
            .map_err(|e| format!("cannot open {device}: {e}"))?;
        set_raw(&file, rate).map_err(|e| e.to_string())?;
        Ok(Self {
            file,
            device: device.to_string(),
        })
    }

    fn write(&mut self, b: &[u8]) -> Result<(), String> {
        self.file
            .write_all(b)
            .and_then(|_| self.file.flush())
            .map_err(|e| format!("{}: write failed: {e}", self.device))
    }

    /// Read until `done(bytes so far)` or `max` passes, or the line has been
    /// quiet for `idle` after some bytes (if `idle` is given). Keeps at most
    /// `REPLY_CAP` bytes (the last ones).
    fn read_until(
        &mut self,
        max: Duration,
        idle: Option<Duration>,
        done: &dyn Fn(&[u8]) -> bool,
    ) -> Result<Vec<u8>, String> {
        let end = Instant::now() + max;
        let mut got = Vec::new();
        let mut last = Instant::now();
        let mut buf = [0u8; 512];
        loop {
            let now = Instant::now();
            if now >= end || done(&got) {
                return Ok(got);
            }
            if let Some(i) = idle {
                if !got.is_empty() && now.duration_since(last) >= i {
                    return Ok(got);
                }
            }
            let wait = end.duration_since(now).min(Duration::from_millis(20));
            let mut fds = [PollFd::new(self.file.as_fd(), PollFlags::POLLIN)];
            let ms = u16::try_from(wait.as_millis()).unwrap_or(20);
            match poll(&mut fds, PollTimeout::from(ms)) {
                Ok(0) => continue,
                Ok(_) => {}
                Err(nix::errno::Errno::EINTR) => continue,
                Err(e) => return Err(format!("{}: {e}", self.device)),
            }
            match self.file.read(&mut buf) {
                Ok(0) => return Err(format!("{}: the device closed", self.device)),
                Ok(n) => {
                    got.extend_from_slice(&buf[..n]);
                    if got.len() > REPLY_CAP {
                        got.drain(..got.len() - REPLY_CAP);
                    }
                    last = Instant::now();
                }
                Err(e) if e.kind() == std::io::ErrorKind::Interrupted => {}
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {}
                Err(e) => return Err(format!("{}: read failed: {e}", self.device)),
            }
        }
    }

    /// Enter command mode. Waits out the guard time on both sides of `+++`.
    /// Anything the radio sends meanwhile (packets from the air) is thrown away.
    pub fn enter(&mut self) -> Result<(), String> {
        let _ = termios::tcflush(&self.file, FlushArg::TCIFLUSH);
        self.read_until(GUARD, None, &|_| false)?;
        self.write(b"+++")?;
        let r = self.read_until(GUARD + ENTER_WAIT, None, &|b| has_line(b, "OK"))?;
        if has_line(&r, "OK") {
            return Ok(());
        }
        // Maybe it was in command mode already: then "+++" sits in its line buffer.
        self.write(b"\r")?;
        self.read_until(Duration::from_millis(300), None, &|_| false)?;
        self.write(b"AT\r")?;
        let r = self.read_until(Duration::from_secs(1), None, &|b| has_line(b, "OK"))?;
        if has_line(&r, "OK") {
            return Ok(());
        }
        Err(format!(
            "{}: no reply to +++ (is it a SiK radio? check --baud against its SERIAL_SPEED, and that nothing else has the port open)",
            self.device
        ))
    }

    /// Send one command; return the reply (the echo included).
    pub fn command(&mut self, cmd: &str) -> Result<Vec<u8>, String> {
        let _ = termios::tcflush(&self.file, FlushArg::TCIFLUSH);
        self.write(format!("{cmd}\r").as_bytes())?;
        self.read_until(REPLY_MAX, Some(REPLY_IDLE), &|b| {
            has_line(b, "OK") || has_line(b, "ERROR")
        })
    }

    /// A command that must answer `OK`. Waits for `OK` or `ERROR` (no idle
    /// cut-off: an `RT` answer crosses the air twice, in the radios' TDM windows).
    pub fn command_ok(&mut self, cmd: &str) -> Result<(), String> {
        let _ = termios::tcflush(&self.file, FlushArg::TCIFLUSH);
        self.write(format!("{cmd}\r").as_bytes())?;
        let wait = if cmd.starts_with("RT") {
            REMOTE_MAX
        } else {
            REPLY_MAX
        };
        let r = self.read_until(wait, None, &|b| has_line(b, "OK") || has_line(b, "ERROR"))?;
        if has_line(&r, "OK") {
            Ok(())
        } else if has_line(&r, "ERROR") {
            Err(format!("{}: the radio refused {cmd}", self.device))
        } else {
            Err(format!("{}: no answer to {cmd}", self.device))
        }
    }

    /// `ATI` and `ATI5`.
    pub fn report(&mut self) -> Result<Report, String> {
        let version =
            parse_ati(&self.command("ATI")?).map_err(|e| format!("{}: {e}", self.device))?;
        let params =
            parse_ati5(&self.command("ATI5")?).map_err(|e| format!("{}: {e}", self.device))?;
        Ok(Report { version, params })
    }

    /// Back to data mode (`ATO` prints nothing but its echo).
    pub fn leave(&mut self) -> Result<(), String> {
        self.write(b"ATO\r")?;
        self.read_until(Duration::from_millis(200), None, &|_| false)?;
        Ok(())
    }

    /// Save and reboot. The radio comes back in data mode after about a second.
    pub fn save_and_reboot(&mut self) -> Result<(), String> {
        self.command_ok("AT&W")?;
        self.write(b"ATZ\r")?;
        Ok(())
    }
}

/// Enter command mode, read everything, leave. Leaves the radio in data mode
/// even when reading fails.
pub fn read_report(device: &str, baud: u32) -> Result<Report, String> {
    let mut p = AtPort::open(device, baud)?;
    p.enter()?;
    let r = p.report();
    let left = p.leave();
    let r = r?;
    left?;
    Ok(r)
}

/// Change settings (by SiK name), save, reboot, and read them back.
/// Returns (before, after). `baud_after` is the baud to read back at.
pub fn set(
    device: &str,
    baud: u32,
    changes: &BTreeMap<String, u32>,
    baud_after: u32,
) -> Result<(Report, Report), String> {
    let mut p = AtPort::open(device, baud)?;
    p.enter()?;
    let before = match p.report() {
        Ok(r) => r,
        Err(e) => {
            let _ = p.leave();
            return Err(e);
        }
    };
    let mut cmds = Vec::new();
    for (name, v) in changes {
        let Some(reg) = before
            .params
            .iter()
            .find(|q| &q.name == name)
            .map(|q| q.reg)
        else {
            let _ = p.leave();
            return Err(format!("the radio has no setting {name}"));
        };
        cmds.push(format!("ATS{reg}={v}"));
    }
    for c in &cmds {
        if let Err(e) = p.command_ok(c) {
            // Nothing is saved yet; a reboot puts the old settings back.
            let _ = p.write(b"ATZ\r");
            return Err(format!(
                "{e}; nothing was saved, the radio is rebooting with its old settings"
            ));
        }
    }
    p.save_and_reboot()?;
    drop(p);
    std::thread::sleep(BOOT);
    let after = read_report(device, baud_after)
        .map_err(|e| format!("saved, but reading back failed: {e}"))?;
    Ok((before, after))
}

#[cfg(test)]
mod tests {
    use super::*;

    const ATI5: &[u8] =
        b"ATI5\r\nS0:FORMAT=25\r\nS1:SERIAL_SPEED=57\r\nS2:AIR_SPEED=64\r\nS3:NETID=25\r\n\
S4:TXPOWER=20\r\nS5:ECC=0\r\nS6:MAVLINK=1\r\nS7:OPPRESEND=0\r\nS8:MIN_FREQ=915000\r\n\
S9:MAX_FREQ=928000\r\nS10:NUM_CHANNELS=50\r\nS11:DUTY_CYCLE=100\r\nS12:LBT_RSSI=0\r\n\
S13:MANCHESTER=0\r\nS14:RTSCTS=0\r\nS15:MAX_WINDOW=131\r\n";

    #[test]
    fn ati5_parses() {
        let p = parse_ati5(ATI5).unwrap();
        assert_eq!(p.len(), 16);
        assert_eq!(
            p[3],
            Param {
                reg: 3,
                name: "NETID".into(),
                value: 25
            }
        );
        // LF only, a trailing OK, junk before the echo (bytes from the air) are fine.
        let mut r = b"\x00\x9f+junk\nATI5\n".to_vec();
        r.extend_from_slice(b"S0:FORMAT=25\nS3:NETID=7\nOK\n");
        let p = parse_ati5(&r).unwrap();
        assert_eq!(p.len(), 2);
    }

    #[test]
    fn ati5_rejects_what_is_not_a_setting() {
        for bad in [
            &b""[..],
            b"ATI5\r\n",
            b"ATI5\r\nS3:NETID=25\r\nhello\r\n",
            b"ATI5\r\nS3:NETID=25\r\nS3:NETID=26\r\n",
            b"ATI5\r\nS3:NETID=25\r\nS4:NETID=26\r\n",
            b"ATI5\r\nS3:NETID=-1\r\n",
            b"ATI5\r\nS3:NETID=99999999999\r\n",
            b"ATI5\r\nS3:netid=1\r\n",
            b"ATI5\r\nS1000:X=1\r\n",
            b"ATI5\r\nS3:NETID=2\xff\r\n",
        ] {
            assert!(
                parse_ati5(bad).is_err(),
                "{:?}",
                String::from_utf8_lossy(bad)
            );
        }
    }

    #[test]
    fn errors_never_show_the_bytes() {
        let marker = b"OASA-MARKER-1234";
        let mut r = b"ATI5\r\n".to_vec();
        r.extend_from_slice(marker);
        r.extend_from_slice(b"\r\nS3:NETID=1\r\n");
        let e = parse_ati5(&r).unwrap_err();
        assert!(!e.contains("MARKER"), "{e}");
        let mut v = b"ATI\r\n".to_vec();
        v.extend_from_slice(&[b'x'; 80]);
        assert!(parse_ati(&v).is_err());
    }

    #[test]
    fn hostile_replies_never_panic() {
        let mut rng = crate::rng::Rng::new(5);
        for _ in 0..20_000 {
            let n = rng.below(300) as usize;
            let alphabet = b"S0123456789:=ABNETID_\r\n OKATI5";
            let r: Vec<u8> = (0..n)
                .map(|_| {
                    if rng.below(8) == 0 {
                        rng.next_u64() as u8
                    } else {
                        alphabet[rng.below(alphabet.len() as u64) as usize]
                    }
                })
                .collect();
            let _ = parse_ati5(&r);
            let _ = parse_ati(&r);
            let _ = has_line(&r, "OK");
        }
    }

    #[test]
    fn ati_banner() {
        assert_eq!(
            parse_ati(b"ATI\r\nSiK 2.2 on HM-TRP\r\n").unwrap(),
            "SiK 2.2 on HM-TRP"
        );
        assert!(parse_ati(b"ATI\r\n").is_err());
    }

    #[test]
    fn expect_parses_and_checks() {
        let e = Expect::parse("netid=4242, air_speed=64,ECC=0,mavlink=0,max-window=131").unwrap();
        assert_eq!(e.0.len(), 5);
        assert_eq!(e.0["MAX_WINDOW"], 131);
        let r = Report {
            version: "SiK".into(),
            params: parse_ati5(ATI5).unwrap(),
        };
        let m = e.mismatches(&r);
        let names: Vec<_> = m.iter().map(|x| x.name.as_str()).collect();
        assert_eq!(names, ["MAVLINK", "NETID"]);
        assert_eq!(m[1].text(), "NETID is 25, expected 4242");
        assert_eq!(m[1].json(), "{\"name\":\"NETID\",\"want\":4242,\"got\":25}");
        let m = Expect::parse("ENCRYPTION_LEVEL=0").unwrap().mismatches(&r);
        assert_eq!(m[0].got, None);
        for bad in [
            "",
            ",",
            "netid",
            "netid=x",
            "netid=70000",
            "netid=1,NETID=2",
            "mavlink=3",
            "air_speed=65",
            "format=25",
            "net id=1",
            "ecc=-1",
        ] {
            assert!(Expect::parse(bad).is_err(), "{bad}");
        }
    }

    #[test]
    fn value_tables() {
        assert_eq!(baud_of(57), Some(57_600));
        assert_eq!(baud_of(58), None);
        assert!(check_value("AIR_SPEED", 64).is_ok());
        assert!(check_value("AIR_SPEED", 63).is_err());
        assert!(check_value("LBT_RSSI", 10).is_err());
        assert!(check_value("MAX_WINDOW", 131).is_ok());
        assert!(check_value("MAX_WINDOW", 132).is_err());
        for (_, b) in SERIAL_SPEEDS {
            assert!(baud_rate(*b).is_some());
        }
    }
}
