//! Two simulated SiK radios on pseudo-terminals, linked when their air settings
//! match. They speak the SiK AT protocol as the firmware does (ArduPilot SiK
//! `at.c`, `serial.c`, `tdm.c`; see docs/sik.md):
//!
//! - `+++` enters command mode only with 1 s of silence before and after; the
//!   `+` bytes also go out on the air, as on a real radio.
//! - Command mode echoes (upper case), ends lines with CR LF, runs a line on CR,
//!   gives up on a line over 16 characters.
//! - `AT`, `ATI`, `ATI5`, `ATSn=v`, `ATSn?`, `AT&W`, `ATZ`, `ATO`, and `RT...`,
//!   which the linked radio runs and answers (no authentication: H1).
//!
//! No airtime, loss or hopping: the lab's simulated bench models those.
#![allow(dead_code)]

use std::fs::File;
use std::io::{Read, Write};
use std::os::fd::AsFd;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use nix::poll::{poll, PollFd, PollFlags, PollTimeout};
use oasa_radio::relay::Pty;

pub const NAMES: [&str; 16] = [
    "FORMAT",
    "SERIAL_SPEED",
    "AIR_SPEED",
    "NETID",
    "TXPOWER",
    "ECC",
    "MAVLINK",
    "OPPRESEND",
    "MIN_FREQ",
    "MAX_FREQ",
    "NUM_CHANNELS",
    "DUTY_CYCLE",
    "LBT_RSSI",
    "MANCHESTER",
    "RTSCTS",
    "MAX_WINDOW",
];
/// SiK defaults (parameters.c), with the 915 MHz band of a Holybro V3.
pub const DEFAULTS: [u32; 16] = [
    25, 57, 64, 25, 20, 0, 1, 0, 915_000, 928_000, 50, 100, 0, 0, 0, 131,
];
/// Settings that must match for two radios to link (ArduPilot SiK page).
const LINK: [usize; 8] = [2, 3, 5, 8, 9, 10, 12, 15];
const GUARD: Duration = Duration::from_secs(1);
const LINE_MAX: usize = 16;
pub const VERSION: &str = "SiK 2.2 on SIM-HM-TRP";

pub fn reg(name: &str) -> usize {
    NAMES.iter().position(|n| *n == name).unwrap()
}

pub struct Radio {
    pub params: [u32; 16],
    pub saved: [u32; 16],
    pub at_mode: bool,
    line: Vec<u8>,
    last_in: Option<Instant>,
    plus: u8,
    enable_at: Option<Instant>,
    booting_until: Option<Instant>,
    /// Bytes waiting to go to the host.
    out: Vec<u8>,
    /// Bytes this radio put on the air (data only).
    pub aired: Vec<u8>,
    pub reboots: u32,
}

impl Radio {
    fn new(params: [u32; 16]) -> Self {
        Self {
            params,
            saved: params,
            at_mode: false,
            line: Vec::new(),
            last_in: None,
            plus: 0,
            enable_at: None,
            booting_until: None,
            out: Vec::new(),
            aired: Vec::new(),
            reboots: 0,
        }
    }

    fn print(&mut self, s: &str) {
        self.out
            .extend_from_slice(s.replace('\n', "\r\n").as_bytes());
    }

    /// Run one AT line (already upper case, `AT` prefix). Returns the output.
    fn run(&mut self, cmd: &str, now: Instant) -> String {
        let rest = &cmd[2..];
        match rest {
            "" => "OK\n".into(),
            "I" | "I0" => format!("{VERSION}\n"),
            "I5" => (0..16)
                .map(|i| format!("S{i}:{}={}\n", NAMES[i], self.params[i]))
                .collect(),
            "O" => {
                self.at_mode = false;
                String::new()
            }
            "Z" => {
                self.params = self.saved;
                self.at_mode = false;
                self.booting_until = Some(now + Duration::from_millis(500));
                self.reboots += 1;
                String::new()
            }
            "&W" => {
                self.saved = self.params;
                "OK\n".into()
            }
            _ => {
                if let Some(s) = rest.strip_prefix('S') {
                    if let Some(n) = s.strip_suffix('?') {
                        if let Ok(n) = n.parse::<usize>() {
                            if n < 16 {
                                return format!("{}\n", self.params[n]);
                            }
                        }
                    } else if let Some((n, v)) = s.split_once('=') {
                        if let (Ok(n), Ok(v)) = (n.parse::<usize>(), v.parse::<u32>()) {
                            let ok = match n {
                                1..=4 | 7..=15 => true,
                                5 | 6 => v <= if n == 6 { 2 } else { 1 },
                                _ => false,
                            };
                            if ok && n > 0 && n < 16 {
                                self.params[n] = v;
                                return "OK\n".into();
                            }
                        }
                    }
                }
                "ERROR\n".into()
            }
        }
    }

    fn booting(&self, now: Instant) -> bool {
        self.booting_until.is_some_and(|t| now < t)
    }
}

struct Air {
    r: [Radio; 2],
}

impl Air {
    fn linked(&self) -> bool {
        LINK.iter()
            .all(|i| self.r[0].params[*i] == self.r[1].params[*i])
    }

    fn byte(&mut self, i: usize, c: u8, now: Instant) {
        if self.r[i].booting(now) {
            return;
        }
        if self.r[i].at_mode {
            self.at_input(i, c, now);
            return;
        }
        // the +++ detector (at_plus_detector)
        let me = &mut self.r[i];
        if me.enable_at.is_some() {
            me.enable_at = None;
            me.plus = 0;
        } else if c == b'+' {
            let idle = me.last_in.is_none_or(|t| now.duration_since(t) >= GUARD);
            if me.plus > 0 || idle {
                me.plus += 1;
            }
            if me.plus == 3 {
                me.plus = 0;
                me.enable_at = Some(now);
            }
        } else {
            me.plus = 0;
        }
        me.last_in = Some(now);
        // the byte also goes on the air
        me.aired.push(c);
        let o = 1 - i;
        if self.linked() && !self.r[o].at_mode && !self.r[o].booting(now) {
            self.r[o].out.push(c);
        }
    }

    fn at_input(&mut self, i: usize, c: u8, now: Instant) {
        let me = &mut self.r[i];
        if c == b'\r' {
            me.out.extend_from_slice(b"\r\n");
            let line = String::from_utf8_lossy(&me.line).into_owned();
            me.line.clear();
            if line.starts_with("RT") && line.len() >= 2 {
                if self.linked() {
                    let o = 1 - i;
                    let reply = self.r[o].run(&format!("AT{}", &line[2..]), now);
                    self.r[i].print(&reply);
                }
            } else if line.starts_with("AT") {
                let reply = self.r[i].run(&line, now);
                self.r[i].print(&reply);
            }
            return;
        }
        if (0x20..0x7f).contains(&c) {
            if me.line.len() >= LINE_MAX {
                me.at_mode = false;
                me.line.clear();
                return;
            }
            let u = c.to_ascii_uppercase();
            me.line.push(u);
            me.out.push(u);
        }
    }

    fn tick(&mut self, now: Instant) {
        for r in &mut self.r {
            if let Some(t) = r.enable_at {
                if now.duration_since(t) >= GUARD {
                    r.enable_at = None;
                    r.at_mode = true;
                    r.line.clear();
                    r.print("OK\n");
                }
            }
        }
    }
}

pub struct SimSikPair {
    pub path_a: PathBuf,
    pub path_b: PathBuf,
    air: Arc<Mutex<Air>>,
    stop: Arc<AtomicBool>,
    handle: Option<JoinHandle<()>>,
}

impl SimSikPair {
    pub fn start(a: [u32; 16], b: [u32; 16]) -> Self {
        let pa = Pty::open().unwrap();
        let pb = Pty::open().unwrap();
        let (path_a, path_b) = (pa.path.clone(), pb.path.clone());
        let air = Arc::new(Mutex::new(Air {
            r: [Radio::new(a), Radio::new(b)],
        }));
        let stop = Arc::new(AtomicBool::new(false));
        let (ai, st) = (air.clone(), stop.clone());
        let handle = std::thread::spawn(move || run([pa, pb], ai, st));
        Self {
            path_a,
            path_b,
            air,
            stop,
            handle: Some(handle),
        }
    }

    /// A pair with the same settings, `netid` and `MAVLINK=0`.
    pub fn matched(netid: u32) -> Self {
        let mut p = DEFAULTS;
        p[reg("NETID")] = netid;
        p[reg("MAVLINK")] = 0;
        Self::start(p, p)
    }

    pub fn with<T>(&self, f: impl FnOnce(&mut [Radio; 2]) -> T) -> T {
        f(&mut self.air.lock().unwrap().r)
    }

    pub fn param(&self, i: usize, name: &str) -> u32 {
        self.with(|r| r[i].params[reg(name)])
    }
}

impl Drop for SimSikPair {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::SeqCst);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

fn run(ptys: [Pty; 2], air: Arc<Mutex<Air>>, stop: Arc<AtomicBool>) {
    let mut files: Vec<File> = ptys
        .iter()
        .map(|p| File::from(p.master.try_clone().unwrap()))
        .collect();
    let mut buf = [0u8; 4096];
    while !stop.load(Ordering::SeqCst) {
        {
            let mut fds: Vec<PollFd> = files
                .iter()
                .map(|f| PollFd::new(f.as_fd(), PollFlags::POLLIN))
                .collect();
            let _ = poll(&mut fds, PollTimeout::from(5u16));
            let ready: Vec<bool> = fds
                .iter()
                .map(|f| f.revents().is_some_and(|r| r.contains(PollFlags::POLLIN)))
                .collect();
            drop(fds);
            for (i, f) in files.iter_mut().enumerate() {
                if !ready[i] {
                    continue;
                }
                if let Ok(n) = f.read(&mut buf) {
                    let now = Instant::now();
                    let mut a = air.lock().unwrap();
                    for &c in &buf[..n] {
                        a.byte(i, c, now);
                    }
                }
            }
        }
        let outs: Vec<Vec<u8>> = {
            let mut a = air.lock().unwrap();
            a.tick(Instant::now());
            a.r.iter_mut().map(|r| std::mem::take(&mut r.out)).collect()
        };
        for (i, o) in outs.iter().enumerate() {
            if !o.is_empty() {
                let _ = files[i].write_all(o);
            }
        }
    }
    drop(ptys);
}
