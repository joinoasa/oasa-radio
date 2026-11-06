//! `sdr`: OASA's own waveform (frame v1) on a software-defined radio, as two continuous streams of
//! IQ samples. The radio itself is driven by another program (for UHD radios, the UHD example tools
//! `tx_samples_from_file` and `rx_samples_to_file` / `rx_samples_c`) through two files, normally
//! FIFOs:
//!
//! ```text
//!   bridge --send--> modulate --> [TX stream, zeros between frames] --> tx FIFO --> radio --> air
//!   bridge <--recv-- frames <-- LiveReceiver   <-- decimate <-- mix <-- rx FIFO <-- radio <-- air
//! ```
//!
//! The TX stream never stops: between frames it carries zeros, so the radio's transmit chain stays
//! up and a packet goes out as soon as the stream reaches it (no start-up per packet). Writing to a
//! FIFO paces the stream at the radio's rate. So a sink that never blocks (a plain file, a test)
//! cannot fill the disk, the writer also never runs more than `tx_lead` (default 20 ms) ahead of the wall clock.
//! When the program reading the TX file stops, the write fails and the radio reports an error: the
//! transmitter cannot outlive the bridge, nor the bridge the transmitter.
//!
//! Both streams sit at a digital IF (`if_hz`, default 250 kHz at 1 MS/s): the DC offset and LO
//! leakage of a direct-conversion radio then fall outside the signal (oasa-radio docs/waveform-w0.md,
//! W1). The receive side shifts back by `if_hz` plus the tracked clock offset, low-pass filters,
//! keeps every `decimate`-th sample and hands blocks to [`LiveReceiver`], which releases each frame as soon as its own samples are in. Every frame whose CRC
//! passes is one packet; a frame with a bad CRC is dropped and counted (`bad_frame`).
//!
//! The radio never looks inside a packet and never logs one. Its optional log (`log`) has sizes,
//! counts, offsets and timings only. A recording of what it received (`rx_record`) is raw samples,
//! as anyone with a receiver could make; it is for the lab's checks that the air carries only
//! ciphertext.

use std::collections::VecDeque;
use std::fs::{File, OpenOptions};
use std::io::{BufWriter, Read, Write};
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, SyncSender};
use std::sync::{Arc, Condvar, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant, SystemTime};

use oasa_waveform::frame::{FrameFormat, PayloadCode};
use oasa_waveform::iq::{from_cf32_le, from_ci16_le};
use oasa_waveform::stream::LiveReceiver;
use oasa_waveform::{Complex32, Modem, ModemConfig, Modulation, RxConfig};

use crate::log::{json_str, rfc3339};
use crate::radio::{Counters, Radio, RadioError, RadioInfo};

/// Largest packet: a frame v1 carries 1 to 255 bytes.
pub const MAX_MTU: usize = 255;
/// Default MTU: oasa-link's handshake (119 bytes) and data packets with room to spare.
pub const DEFAULT_MTU: usize = 200;
/// Packets waiting for the air before new ones are dropped (`tx_queue_full`). A radio is not a buffer.
pub const MAX_TX_QUEUE: usize = 64;
/// Zeros after each frame, in symbols: room for the receiver's matched filter and the next preamble.
const GAP_SYMBOLS: usize = 50;
/// Samples per TX write: 1 ms at 1 MS/s.
const TX_CHUNK_SECONDS: f64 = 0.001;
/// Samples per RX read.
const RX_READ: usize = 4096;
/// Low-pass filter for decimation, taps (odd).
const FIR_TAPS: usize = 31;
/// How much of each frame's residual offset the tracker takes.
const CFO_TRACK_GAIN: f64 = 0.3;

/// How frames go to the radio tool.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TxMode {
    /// One continuous cf32 stream, zeros between frames, paced at most `tx_lead` ahead of the
    /// clock. Works with any tool that plays a stream (UHD's `tx_samples_from_file`).
    Stream,
    /// One record per frame: a little-endian u32 sample count, then the samples (cf32). Nothing
    /// between frames. The radio tool sends each record as one timed burst a little after its
    /// radio's "now", so a frame never waits behind buffered zeros and an underflow cannot add
    /// delay for good.
    Burst,
}

impl TxMode {
    pub fn parse(s: &str) -> Option<Self> {
        match s {
            "stream" => Some(TxMode::Stream),
            "burst" => Some(TxMode::Burst),
            _ => None,
        }
    }
}

/// Most samples in one burst record (the radio tool refuses more).
pub const MAX_BURST_SAMPLES: usize = 1 << 20;

/// Sample format of the receive stream.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IqFormat {
    /// Little-endian float32 I and Q (UHD `--type float`, `rx_samples_c`).
    Cf32,
    /// Little-endian int16 I and Q, full scale 32768 (UHD `--type short`).
    Ci16,
}

impl IqFormat {
    pub fn parse(s: &str) -> Option<Self> {
        match s {
            "cf32" => Some(IqFormat::Cf32),
            "ci16" => Some(IqFormat::Ci16),
            _ => None,
        }
    }

    fn bytes(self) -> usize {
        match self {
            IqFormat::Cf32 => 8,
            IqFormat::Ci16 => 4,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct SdrConfig {
    /// Where the TX stream goes (cf32, little endian). Normally a FIFO the radio tool reads.
    pub tx: PathBuf,
    /// Where the RX stream comes from. Normally a FIFO the radio tool writes.
    pub rx: PathBuf,
    pub rx_format: IqFormat,
    pub modulation: Modulation,
    pub code: PayloadCode,
    /// Samples per second of both streams.
    pub sample_rate: f64,
    /// Symbols per second; `sample_rate / symbol_rate` must be a whole number of 4 or more.
    pub symbol_rate: f64,
    /// Digital IF of both streams, Hz (the signal sits at +if_hz).
    pub if_hz: f64,
    /// Starting estimate of the received signal's offset from `if_hz` (the two radios' clocks), Hz.
    pub cfo_hz: f64,
    /// Follow the offset from each good frame's estimate.
    pub track_cfo: bool,
    /// The receiver's coarse frequency search, cycles per symbol (0 = off).
    pub cfo_search: f32,
    /// Keep every n-th sample after the filter (1 or 2; the receiver needs 4 or more per symbol).
    pub decimate: usize,
    /// Samples (after decimation) per receiver pass. Smaller: less delay, a little more work.
    pub block: usize,
    /// Samples between the detector's test positions (0: a quarter symbol). See `RxConfig::coarse_step`.
    pub coarse_step: usize,
    /// Peak amplitude of a frame in the TX stream (full scale = 1).
    pub tx_peak: f32,
    pub mtu: usize,
    /// JSON-lines log of frames and timings (sizes only).
    pub log: Option<PathBuf>,
    /// Record the received baseband (after the shift and decimation), cf32, at most `rx_record_seconds`.
    pub rx_record: Option<PathBuf>,
    pub rx_record_seconds: f64,
    /// The TX writer never runs further ahead of the wall clock than this. It is the most a packet
    /// waits behind zeros already written; too small and the radio runs dry (underflows).
    pub tx_lead: Duration,
    pub tx_mode: TxMode,
}

impl SdrConfig {
    /// Defaults of the lab's W1 bench: 1 MS/s, 125 kbaud, IF 250 kHz, QPSK, decimate 2.
    pub fn new(tx: PathBuf, rx: PathBuf) -> Self {
        Self {
            tx,
            rx,
            rx_format: IqFormat::Cf32,
            modulation: Modulation::Qpsk,
            code: PayloadCode::None,
            sample_rate: 1e6,
            symbol_rate: 125e3,
            if_hz: 250e3,
            cfo_hz: 0.0,
            track_cfo: true,
            cfo_search: 0.03,
            decimate: 2,
            block: 512,
            coarse_step: 0,
            tx_peak: 0.5,
            mtu: DEFAULT_MTU,
            log: None,
            rx_record: None,
            rx_record_seconds: 60.0,
            tx_lead: Duration::from_millis(20),
            tx_mode: TxMode::Stream,
        }
    }

    /// Samples per symbol of the streams.
    pub fn sps(&self) -> Result<usize, String> {
        let r = self.sample_rate / self.symbol_rate;
        if !(r.is_finite() && (r - r.round()).abs() < 1e-9 && r.round() >= 4.0) {
            return Err(format!(
                "--sample-rate / --symbol-rate must be a whole number of 4 or more (it is {r})"
            ));
        }
        Ok(r.round() as usize)
    }

    pub fn check(&self) -> Result<(), String> {
        let sps = self.sps()?;
        if !(1..=2).contains(&self.decimate) || sps % self.decimate != 0 || sps / self.decimate < 4
        {
            return Err(format!(
                "--decimate {} does not fit {sps} samples per symbol (1 or 2, leaving 4 or more)",
                self.decimate
            ));
        }
        if !(1..=MAX_MTU).contains(&self.mtu) {
            return Err(format!("--mtu must be 1 to {MAX_MTU} (one frame v1)"));
        }
        if !(self.tx_peak > 0.0 && self.tx_peak <= 0.9) {
            return Err("--tx-peak must be above 0 and at most 0.9".into());
        }
        // the signal (RRC roll-off 0.35: +-0.675 symbol rates) must fit inside the band at the IF
        if !self.if_hz.is_finite()
            || self.if_hz.abs() + 0.7 * self.symbol_rate >= self.sample_rate / 2.0
        {
            return Err("--if-hz puts the signal outside the sample rate's band".into());
        }
        if !self.cfo_hz.is_finite() || self.cfo_hz.abs() >= self.sample_rate / 8.0 {
            return Err("--cfo-hz is too large for the sample rate".into());
        }
        if !(64..=1 << 20).contains(&self.block) {
            return Err("--block must be 64 to 1048576".into());
        }
        if !(self.cfo_search > 0.0 && self.cfo_search.is_finite()) {
            return Err("--cfo-search must be above 0 (the live receiver needs it)".into());
        }
        if !(Duration::from_millis(1)..=Duration::from_secs(1)).contains(&self.tx_lead) {
            return Err("--tx-lead-ms must be 1 to 1000".into());
        }
        if !(0.0..=3600.0).contains(&self.rx_record_seconds) {
            return Err("--rx-record-seconds must be 0 to 3600".into());
        }
        Ok(())
    }
}

/// What the threads share with the radio handle.
struct Shared {
    stop: AtomicBool,
    failed: Mutex<Option<String>>,
    tx_queue: Mutex<VecDeque<(Vec<Complex32>, Instant)>>,
    /// Signalled when a frame is queued (burst mode waits on it).
    tx_ready: Condvar,
    tx_frames: AtomicU64,
    bad_frames: AtomicU64,
    rx_frames: AtomicU64,
    tx_underrun_ms: AtomicU64,
    /// Lines for the log thread. The radio's threads never touch the file: a write can block for tens
    /// of milliseconds while the filesystem flushes (measured: 50-60 ms stalls every 10 s on the bench).
    log: Option<SyncSender<LogMsg>>,
    /// Log lines dropped because the log thread fell behind.
    log_dropped: AtomicU64,
}

impl Shared {
    fn fail(&self, why: String) {
        let mut f = self.failed.lock().unwrap();
        if f.is_none() {
            self.event("failed", &format!(",\"what\":{}", json_str(&why)));
            *f = Some(why);
        }
        self.stop.store(true, Ordering::SeqCst);
    }

    fn event(&self, event: &str, fields: &str) {
        if let Some(l) = &self.log {
            let line = format!(
                "{{\"ts\":\"{}\",\"event\":\"{event}\"{fields}}}\n",
                rfc3339(SystemTime::now())
            );
            if l.try_send(LogMsg::Line(line)).is_err() {
                self.log_dropped.fetch_add(1, Ordering::Relaxed);
            }
        }
    }

    /// Wait (at most `wait`) until the log thread has written everything sent so far.
    fn flush_log(&self, wait: Duration) {
        if let Some(l) = &self.log {
            let (tx, rx) = mpsc::sync_channel(1);
            if l.send(LogMsg::Flush(tx)).is_ok() {
                let _ = rx.recv_timeout(wait);
            }
        }
    }
}

enum LogMsg {
    Line(String),
    /// Acknowledged once everything before it is written.
    Flush(SyncSender<()>),
}

/// Writes log lines as they come, flushing whenever the queue is empty.
fn log_thread(mut w: BufWriter<File>, rx: Receiver<LogMsg>) {
    while let Ok(m) = rx.recv() {
        let mut next = Some(m);
        while let Some(m) = next.take() {
            match m {
                LogMsg::Line(l) => {
                    let _ = w.write_all(l.as_bytes());
                }
                LogMsg::Flush(ack) => {
                    let _ = w.flush();
                    let _ = ack.try_send(());
                }
            }
            next = rx.try_recv().ok();
        }
        let _ = w.flush();
    }
}

pub struct SdrRadio {
    cfg: SdrConfig,
    modem: Modem,
    shared: Arc<Shared>,
    rx: Receiver<Vec<u8>>,
    counters: Counters,
    threads: Vec<JoinHandle<()>>,
}

impl SdrRadio {
    /// Start both streams. Opening a FIFO waits for the program on its other end, so this returns at
    /// once and the threads open the files; a failure shows up on the next `send` or `recv`.
    pub fn open(cfg: SdrConfig) -> Result<Self, RadioError> {
        cfg.check().map_err(RadioError::Config)?;
        let sps = cfg.sps().map_err(RadioError::Config)?;
        let tx_modem = Modem::new(ModemConfig {
            sps,
            frame: FrameFormat::V1,
            code: cfg.code,
            ..ModemConfig::new(cfg.modulation)
        })
        .map_err(|e| RadioError::Config(e.to_string()))?;
        let rx_modem = Modem::new(ModemConfig {
            sps: sps / cfg.decimate,
            frame: FrameFormat::V1,
            code: cfg.code,
            ..ModemConfig::new(cfg.modulation)
        })
        .map_err(|e| RadioError::Config(e.to_string()))?;
        let rx_cfg = RxConfig {
            cfo_search: cfg.cfo_search,
            max_len: Some(cfg.mtu),
            only_code: Some(cfg.code),
            coarse_step: cfg.coarse_step,
            ..RxConfig::default()
        };
        let log = match &cfg.log {
            Some(p) => {
                let f = OpenOptions::new()
                    .create(true)
                    .append(true)
                    .open(p)
                    .map_err(|e| RadioError::Config(format!("--sdr-log {}: {e}", p.display())))?;
                let (tx, rx) = mpsc::sync_channel(4096);
                std::thread::Builder::new()
                    .name("sdr-log".into())
                    .spawn(move || log_thread(BufWriter::new(f), rx))?;
                Some(tx)
            }
            None => None,
        };
        let shared = Arc::new(Shared {
            stop: AtomicBool::new(false),
            failed: Mutex::new(None),
            tx_queue: Mutex::new(VecDeque::new()),
            tx_ready: Condvar::new(),
            tx_frames: AtomicU64::new(0),
            bad_frames: AtomicU64::new(0),
            rx_frames: AtomicU64::new(0),
            tx_underrun_ms: AtomicU64::new(0),
            log,
            log_dropped: AtomicU64::new(0),
        });
        let receiver = LiveReceiver::new(rx_modem, rx_cfg).map_err(RadioError::Config)?;
        shared.event(
            "sdr_start",
            &format!(
                ",\"mod\":\"{}\",\"code\":\"{}\",\"sample_rate\":{},\"symbol_rate\":{},\"if_hz\":{},\"cfo_hz\":{},\"decimate\":{},\"block\":{},\"mtu\":{},\"tx_mode\":\"{}\",\"longest_frame_samples\":{}",
                cfg.modulation.name(),
                cfg.code.name(),
                cfg.sample_rate,
                cfg.symbol_rate,
                cfg.if_hz,
                cfg.cfo_hz,
                cfg.decimate,
                cfg.block,
                cfg.mtu,
                match cfg.tx_mode {
                    TxMode::Stream => "stream",
                    TxMode::Burst => "burst",
                },
                receiver.modem().longest_frame_samples(&RxConfig {
                    max_len: Some(cfg.mtu),
                    only_code: Some(cfg.code),
                    ..RxConfig::default()
                })
            ),
        );
        let (tx_pkt, rx_pkt) = mpsc::sync_channel(256);
        let mut threads = Vec::new();
        {
            let (s, c) = (shared.clone(), cfg.clone());
            threads.push(
                std::thread::Builder::new()
                    .name("sdr-tx".into())
                    .spawn(move || tx_thread(&c, &s))?,
            );
        }
        {
            let (s, c) = (shared.clone(), cfg.clone());
            threads.push(
                std::thread::Builder::new()
                    .name("sdr-rx".into())
                    .spawn(move || rx_thread(&c, &s, receiver, tx_pkt))?,
            );
        }
        Ok(Self {
            cfg,
            modem: tx_modem,
            shared,
            rx: rx_pkt,
            counters: Counters::default(),
            threads,
        })
    }

    fn check_failed(&self) -> Result<(), RadioError> {
        match &*self.shared.failed.lock().unwrap() {
            Some(why) => Err(RadioError::Io(std::io::Error::other(why.clone()))),
            None => Ok(()),
        }
    }

    fn sync_counters(&mut self) {
        let bad = self.shared.bad_frames.load(Ordering::Relaxed);
        let have = self.counters.drops.get("bad_frame").copied().unwrap_or(0);
        self.counters
            .add_drop("bad_frame", bad.saturating_sub(have));
    }
}

impl Drop for SdrRadio {
    fn drop(&mut self) {
        self.shared.stop.store(true, Ordering::SeqCst);
        let s = &self.shared;
        s.event(
            "sdr_stats",
            &format!(
                ",\"tx_frames\":{},\"rx_frames\":{},\"bad_frames\":{},\"tx_underrun_ms\":{},\"log_dropped\":{}",
                s.tx_frames.load(Ordering::Relaxed),
                s.rx_frames.load(Ordering::Relaxed),
                s.bad_frames.load(Ordering::Relaxed),
                s.tx_underrun_ms.load(Ordering::Relaxed),
                s.log_dropped.load(Ordering::Relaxed)
            ),
        );
        s.flush_log(Duration::from_secs(1));
        // The threads may be blocked opening a FIFO nobody opened; they are detached then.
        for t in self.threads.drain(..) {
            if t.is_finished() {
                let _ = t.join();
            }
        }
    }
}

impl Radio for SdrRadio {
    fn info(&self) -> RadioInfo {
        let bps = self.cfg.modulation.bits_per_symbol() as f64;
        let rate = match self.cfg.code {
            PayloadCode::None => 1.0,
            PayloadCode::Light => 0.5,
        };
        RadioInfo {
            kind: "sdr",
            target: format!("tx {} rx {}", self.cfg.tx.display(), self.cfg.rx.display()),
            mtu: self.cfg.mtu,
            bitrate: Some((self.cfg.symbol_rate * bps * rate) as u32),
        }
    }

    fn send(&mut self, packet: &[u8]) -> Result<(), RadioError> {
        self.check_failed()?;
        if packet.is_empty() {
            self.counters.add_drop("empty", 1);
            return Ok(());
        }
        if packet.len() > self.cfg.mtu {
            self.counters.add_drop("too_large", 1);
            return Ok(());
        }
        let mut s = self
            .modem
            .modulate(packet)
            .map_err(|e| RadioError::Config(e.to_string()))?;
        let peak = s.iter().map(|v| v.re.hypot(v.im)).fold(0.0f32, f32::max);
        let k = if peak > 0.0 {
            self.cfg.tx_peak / peak
        } else {
            0.0
        };
        for v in &mut s {
            v.re *= k;
            v.im *= k;
        }
        // A short tail of zeros: room for the receiver's matched filter (and, streamed, the next preamble).
        let sps = self.cfg.sps().unwrap_or(8);
        s.extend(std::iter::repeat_n(
            Complex32 { re: 0.0, im: 0.0 },
            GAP_SYMBOLS * sps,
        ));
        let mut q = self.shared.tx_queue.lock().unwrap();
        if q.len() >= MAX_TX_QUEUE {
            drop(q);
            self.counters.add_drop("tx_queue_full", 1);
            return Ok(());
        }
        let ahead: usize = q.iter().map(|(f, _)| f.len()).sum();
        q.push_back((s, Instant::now()));
        drop(q);
        self.shared.tx_ready.notify_one();
        self.counters.sent += 1;
        self.counters.sent_bytes += packet.len() as u64;
        self.shared.event(
            "tx_queued",
            &format!(",\"len\":{},\"ahead_samples\":{ahead}", packet.len()),
        );
        Ok(())
    }

    fn recv(&mut self, timeout: Duration) -> Result<Option<Vec<u8>>, RadioError> {
        self.sync_counters();
        match self.rx.recv_timeout(timeout) {
            Ok(p) => {
                self.counters.received += 1;
                self.counters.received_bytes += p.len() as u64;
                Ok(Some(p))
            }
            Err(RecvTimeoutError::Timeout) => {
                self.check_failed()?;
                Ok(None)
            }
            Err(RecvTimeoutError::Disconnected) => {
                self.check_failed()?;
                Err(RadioError::Io(std::io::Error::other(
                    "the receive stream ended",
                )))
            }
        }
    }

    fn counters(&self) -> &Counters {
        &self.counters
    }

    /// A packet from oasa-link waits at most this long before it is queued for the air.
    fn poll_interval(&self) -> Duration {
        Duration::from_micros(200)
    }
}

/// A numerically controlled oscillator: multiplies by exp(j 2 pi f n / fs), phase kept in f64.
struct Nco {
    phase: f64,
}

impl Nco {
    fn mix(&mut self, x: &mut [Complex32], f_hz: f64, fs: f64) {
        let w = 2.0 * std::f64::consts::PI * f_hz / fs;
        // exact rotation at the start of each block, recursive within (blocks are short)
        let (mut c, mut s) = (self.phase.cos(), self.phase.sin());
        let (cw, sw) = (w.cos(), w.sin());
        for v in x.iter_mut() {
            let (re, im) = (f64::from(v.re), f64::from(v.im));
            v.re = (re * c - im * s) as f32;
            v.im = (re * s + im * c) as f32;
            (c, s) = (c * cw - s * sw, c * sw + s * cw);
        }
        self.phase = (self.phase + w * x.len() as f64).rem_euclid(2.0 * std::f64::consts::PI);
    }
}

/// Windowed-sinc low-pass and keep every `m`-th output, across block boundaries.
struct Decimator {
    taps: Vec<f32>,
    hist: Vec<Complex32>,
    m: usize,
    /// Input samples to skip before the next kept output.
    skip: usize,
}

impl Decimator {
    fn new(m: usize) -> Self {
        let n = FIR_TAPS;
        let half = (n / 2) as f64;
        // cutoff at 0.8 of the output band edge (0.5 / m cycles per input sample): 0.4 / m
        let fc = 0.4 / m as f64;
        let mut taps: Vec<f32> = (0..n)
            .map(|i| {
                let t = i as f64 - half;
                let sinc = if t == 0.0 {
                    2.0 * fc
                } else {
                    (2.0 * std::f64::consts::PI * fc * t).sin() / (std::f64::consts::PI * t)
                };
                let w =
                    0.54 - 0.46 * (2.0 * std::f64::consts::PI * i as f64 / (n - 1) as f64).cos();
                (sinc * w) as f32
            })
            .collect();
        let sum: f32 = taps.iter().sum();
        for t in &mut taps {
            *t /= sum;
        }
        Self {
            taps,
            hist: vec![Complex32 { re: 0.0, im: 0.0 }; n - 1],
            m,
            skip: 0,
        }
    }

    fn run(&mut self, x: &[Complex32], out: &mut Vec<Complex32>) {
        if self.m == 1 {
            out.extend_from_slice(x);
            return;
        }
        let n = self.taps.len();
        let h = self.hist.len();
        self.hist.extend_from_slice(x);
        let mut i = self.skip;
        while i < x.len() {
            // output for input sample i: taps over hist[i .. i + n]
            let w = &self.hist[i..i + n];
            let (mut re, mut im) = (0.0f32, 0.0f32);
            for (v, t) in w.iter().zip(&self.taps) {
                re += v.re * t;
                im += v.im * t;
            }
            out.push(Complex32 { re, im });
            i += self.m;
        }
        self.skip = i - x.len();
        self.hist.drain(..x.len());
        debug_assert_eq!(self.hist.len(), h);
    }
}

fn open_stream(path: &PathBuf, write: bool, shared: &Shared) -> Option<File> {
    let r = if write {
        OpenOptions::new().write(true).open(path)
    } else {
        File::open(path)
    };
    match r {
        Ok(f) => Some(f),
        Err(e) => {
            shared.fail(format!("{}: {e}", path.display()));
            None
        }
    }
}

fn tx_thread(cfg: &SdrConfig, shared: &Shared) {
    let Some(f) = open_stream(&cfg.tx, true, shared) else {
        return;
    };
    if cfg.tx_mode == TxMode::Burst {
        return tx_bursts(cfg, shared, f);
    }
    let mut f = BufWriter::with_capacity(1 << 16, f);
    shared.event("tx_open", "");
    let chunk = ((cfg.sample_rate * TX_CHUNK_SECONDS) as usize).max(64);
    let mut nco = Nco { phase: 0.0 };
    let mut buf = vec![Complex32 { re: 0.0, im: 0.0 }; chunk];
    let mut bytes = Vec::with_capacity(chunk * 8);
    let mut cur: Option<(Vec<Complex32>, usize)> = None;
    let t0 = Instant::now();
    let mut written: u64 = 0;
    while !shared.stop.load(Ordering::SeqCst) {
        // do not run ahead of the wall clock by more than tx_lead
        let due = Duration::from_secs_f64(written as f64 / cfg.sample_rate);
        let now = t0.elapsed();
        if due > now + cfg.tx_lead {
            std::thread::sleep(due - now - cfg.tx_lead);
        } else if now > due + Duration::from_millis(100) {
            // the sink took less than the rate (it was not reading): count it once per gap
            shared
                .tx_underrun_ms
                .fetch_add((now - due).as_millis() as u64, Ordering::Relaxed);
            written = (now.as_secs_f64() * cfg.sample_rate) as u64;
        }
        let mut fill = 0;
        for v in buf.iter_mut() {
            *v = Complex32 { re: 0.0, im: 0.0 };
        }
        while fill < chunk {
            if cur.is_none() {
                let next = shared.tx_queue.lock().unwrap().pop_front();
                match next {
                    Some((s, queued)) => {
                        shared.tx_frames.fetch_add(1, Ordering::Relaxed);
                        shared.event(
                            "tx_air",
                            &format!(
                                ",\"samples\":{},\"queued_ms\":{:.1}",
                                s.len(),
                                queued.elapsed().as_secs_f64() * 1e3
                            ),
                        );
                        cur = Some((s, 0));
                    }
                    None => break,
                }
            }
            let (s, pos) = cur.as_mut().unwrap();
            let k = (chunk - fill).min(s.len() - *pos);
            buf[fill..fill + k].copy_from_slice(&s[*pos..*pos + k]);
            fill += k;
            *pos += k;
            if *pos >= s.len() {
                cur = None;
            }
        }
        nco.mix(&mut buf, cfg.if_hz, cfg.sample_rate);
        bytes.clear();
        for v in &buf {
            bytes.extend_from_slice(&v.re.to_le_bytes());
            bytes.extend_from_slice(&v.im.to_le_bytes());
        }
        if let Err(e) = f.write_all(&bytes).and_then(|_| f.flush()) {
            shared.fail(format!("TX stream {}: {e}", cfg.tx.display()));
            return;
        }
        written += chunk as u64;
    }
}

/// Burst mode: each frame as one record, as soon as it is queued.
fn tx_bursts(cfg: &SdrConfig, shared: &Shared, f: File) {
    let mut f = BufWriter::with_capacity(1 << 16, f);
    shared.event("tx_open", ",\"mode\":\"burst\"");
    let mut nco = Nco { phase: 0.0 };
    let mut bytes = Vec::new();
    while !shared.stop.load(Ordering::SeqCst) {
        let next = {
            let q = shared.tx_queue.lock().unwrap();
            // wake when a frame is queued (or every 50 ms, to see `stop`)
            let (mut q, _) = shared
                .tx_ready
                .wait_timeout_while(q, Duration::from_millis(50), |q| q.is_empty())
                .unwrap();
            q.pop_front()
        };
        let Some((mut s, queued)) = next else {
            continue;
        };
        shared.tx_frames.fetch_add(1, Ordering::Relaxed);
        shared.event(
            "tx_air",
            &format!(
                ",\"samples\":{},\"queued_ms\":{:.1}",
                s.len(),
                queued.elapsed().as_secs_f64() * 1e3
            ),
        );
        nco.mix(&mut s, cfg.if_hz, cfg.sample_rate);
        bytes.clear();
        bytes.extend_from_slice(&(s.len() as u32).to_le_bytes());
        for v in &s {
            bytes.extend_from_slice(&v.re.to_le_bytes());
            bytes.extend_from_slice(&v.im.to_le_bytes());
        }
        if let Err(e) = f.write_all(&bytes).and_then(|_| f.flush()) {
            shared.fail(format!("TX stream {}: {e}", cfg.tx.display()));
            return;
        }
    }
}

fn rx_thread(
    cfg: &SdrConfig,
    shared: &Shared,
    mut receiver: LiveReceiver,
    out: SyncSender<Vec<u8>>,
) {
    let Some(mut f) = open_stream(&cfg.rx, false, shared) else {
        return;
    };
    shared.event("rx_open", "");
    let mut record = match &cfg.rx_record {
        Some(p) => match File::create(p) {
            Ok(f) => Some(BufWriter::new(f)),
            Err(e) => {
                shared.fail(format!("--rx-record {}: {e}", p.display()));
                return;
            }
        },
        None => None,
    };
    let fs_out = cfg.sample_rate / cfg.decimate as f64;
    let record_max = (cfg.rx_record_seconds * fs_out) as u64;
    let mut recorded: u64 = 0;
    let bpsamp = cfg.rx_format.bytes();
    let mut bytes = vec![0u8; RX_READ * bpsamp];
    let mut have = 0usize;
    let mut nco = Nco { phase: 0.0 };
    let mut dec = Decimator::new(cfg.decimate);
    let mut pending: Vec<Complex32> = Vec::with_capacity(cfg.block * 2);
    let mut cfo_hz = cfg.cfo_hz;
    // (decimated sample count at the end of a read, when it was read): to time each decision
    let mut arrivals: VecDeque<(u64, Instant)> = VecDeque::new();
    let mut fed: u64 = 0;
    let sym_rate = cfg.symbol_rate;
    while !shared.stop.load(Ordering::SeqCst) {
        let n = match f.read(&mut bytes[have..]) {
            Ok(0) => {
                shared.fail(format!("RX stream {} ended", cfg.rx.display()));
                return;
            }
            Ok(n) => n,
            Err(e) if e.kind() == std::io::ErrorKind::Interrupted => continue,
            Err(e) => {
                shared.fail(format!("RX stream {}: {e}", cfg.rx.display()));
                return;
            }
        };
        have += n;
        let whole = have / bpsamp * bpsamp;
        if whole == 0 {
            continue;
        }
        let mut x = match cfg.rx_format {
            IqFormat::Cf32 => from_cf32_le(&bytes[..whole]),
            IqFormat::Ci16 => from_ci16_le(&bytes[..whole]),
        };
        bytes.copy_within(whole..have, 0);
        have -= whole;
        nco.mix(&mut x, -(cfg.if_hz + cfo_hz), cfg.sample_rate);
        let before = pending.len();
        dec.run(&x, &mut pending);
        let added = (pending.len() - before) as u64;
        if let Some(w) = record.as_mut() {
            if recorded < record_max {
                let take = (added.min(record_max - recorded)) as usize;
                for v in &pending[before..before + take] {
                    let _ = w.write_all(&v.re.to_le_bytes());
                    let _ = w.write_all(&v.im.to_le_bytes());
                }
                recorded += take as u64;
                if recorded >= record_max {
                    let _ = w.flush();
                }
            }
        }
        if pending.len() < cfg.block {
            continue;
        }
        let now = Instant::now();
        arrivals.push_back((fed + pending.len() as u64, now));
        fed += pending.len() as u64;
        let frames = receiver.push(&pending);
        pending.clear();
        let decided = Instant::now();
        for fr in frames {
            // when the frame's last sample arrived
            let end = fr.end as u64;
            let arrived = arrivals.iter().find(|(k, _)| *k >= end).map(|(_, t)| *t);
            let wait_ms = arrived.map(|t| decided.duration_since(t).as_secs_f64() * 1e3);
            let resid = f64::from(fr.cfo_cycles_per_symbol);
            if fr.crc_ok {
                shared.rx_frames.fetch_add(1, Ordering::Relaxed);
                if cfg.track_cfo {
                    cfo_hz += CFO_TRACK_GAIN * resid * sym_rate;
                    cfo_hz = cfo_hz.clamp(-cfg.sample_rate / 8.0, cfg.sample_rate / 8.0);
                }
                if out.try_send(fr.payload.clone()).is_err() {
                    shared.event("drop", ",\"reason\":\"rx_queue_full\"");
                }
            } else {
                shared.bad_frames.fetch_add(1, Ordering::Relaxed);
            }
            shared.event(
                "rx_frame",
                &format!(
                    ",\"len\":{},\"crc_ok\":{},\"sync_errors\":{},\"metric\":{:.3},\"resid_cycles_per_symbol\":{:.5},\"cfo_hz\":{:.1},\"decide_ms\":{},\"pass_ms\":{:.1}",
                    fr.payload.len(),
                    fr.crc_ok,
                    fr.sync_errors,
                    fr.metric,
                    resid,
                    cfo_hz,
                    wait_ms.map(|v| format!("{v:.1}")).unwrap_or_else(|| "null".into()),
                    decided.duration_since(now).as_secs_f64() * 1e3,
                ),
            );
        }
        // forget arrivals the receiver can no longer report on
        let keep_from = fed.saturating_sub(4 * receiver.window_len() as u64 + cfg.block as u64);
        while arrivals.front().is_some_and(|(k, _)| *k < keep_from) {
            arrivals.pop_front();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn config_checks() {
        let mut c = SdrConfig::new("/tmp/a".into(), "/tmp/b".into());
        assert!(c.check().is_ok());
        c.symbol_rate = 300e3;
        assert!(c.check().is_err());
        c.symbol_rate = 125e3;
        c.decimate = 4;
        assert!(c.check().is_err(), "8 sps / 4 leaves 2");
        c.decimate = 2;
        c.mtu = 256;
        assert!(c.check().is_err());
        c.mtu = 200;
        c.tx_peak = 1.0;
        assert!(c.check().is_err());
        c.tx_peak = 0.5;
        c.if_hz = 420e3;
        assert!(
            c.check().is_err(),
            "250 kHz IF is fine at 1 MS/s, 420 kHz is not"
        );
    }

    #[test]
    fn nco_is_phase_continuous_across_blocks() {
        let fs = 1e6;
        let f = 250e3 + 1234.5;
        let mut a = Nco { phase: 0.0 };
        let mut b = Nco { phase: 0.0 };
        let one = Complex32 { re: 1.0, im: 0.0 };
        let mut whole = vec![one; 10_000];
        a.mix(&mut whole, f, fs);
        let mut parts = Vec::new();
        for n in [1usize, 999, 4096, 3, 4901] {
            let mut p = vec![one; n];
            b.mix(&mut p, f, fs);
            parts.extend(p);
        }
        for (x, y) in whole.iter().zip(&parts) {
            assert!((x.re - y.re).abs() < 1e-4 && (x.im - y.im).abs() < 1e-4);
        }
    }

    #[test]
    fn decimator_matches_across_blocks_and_passes_the_band() {
        let mut a = Decimator::new(2);
        let mut b = Decimator::new(2);
        // a tone at 0.05 cycles per sample (in band) and one at 0.45 (rejected)
        let x: Vec<Complex32> = (0..8000)
            .map(|n| {
                let t = n as f64;
                let p1 = 2.0 * std::f64::consts::PI * 0.05 * t;
                let p2 = 2.0 * std::f64::consts::PI * 0.45 * t;
                Complex32 {
                    re: (p1.cos() + p2.cos()) as f32,
                    im: (p1.sin() + p2.sin()) as f32,
                }
            })
            .collect();
        let mut whole = Vec::new();
        a.run(&x, &mut whole);
        let mut parts = Vec::new();
        let mut i = 0;
        for n in [7usize, 1000, 1, 2, 3333, 3657] {
            b.run(&x[i..i + n], &mut parts);
            i += n;
        }
        assert_eq!(whole.len(), 4000);
        assert_eq!(whole, parts);
        // after the filter's start-up, the output is the in-band tone alone: magnitude near 1
        for v in &whole[100..] {
            let m = v.re.hypot(v.im);
            assert!((m - 1.0).abs() < 0.03, "magnitude {m}");
        }
    }
}

#[cfg(test)]
mod speed {
    //! Cost of each receive stage per second of signal: `cargo test --release --lib speed -- --ignored --nocapture`.
    use super::*;
    use oasa_waveform::channel::{n0_for_ebn0, Channel, Impairments};

    fn secs(f: impl FnOnce()) -> f64 {
        let t = Instant::now();
        f();
        t.elapsed().as_secs_f64()
    }

    /// Frames found against Eb/N0 for each coarse step: 1 MS/s, 250 kbaud (4 samples per symbol),
    /// 32-byte v1 frames, each at a random whole + fractional sample offset and a random carrier offset.
    #[test]
    #[ignore]
    fn detection_vs_coarse_step() {
        let md = Modem::new(ModemConfig {
            sps: 4,
            frame: FrameFormat::V1,
            ..ModemConfig::new(Modulation::Qpsk)
        })
        .unwrap();
        let frame = md.modulate(&[0x5au8; 32]).unwrap();
        let trials = 400;
        for ebn0 in [2.0, 4.0, 6.0, 8.0] {
            let mut line = format!("Eb/N0 {ebn0:>4} dB:");
            for step in [1usize, 2, 4] {
                let rxc = RxConfig {
                    cfo_search: 0.03,
                    max_len: Some(200),
                    only_code: Some(PayloadCode::None),
                    coarse_step: step,
                    ..RxConfig::default()
                };
                let mut ch = Channel::new(11);
                let mut good = 0;
                let mut rng = 12345u64;
                for _ in 0..trials {
                    rng = rng
                        .wrapping_mul(6364136223846793005)
                        .wrapping_add(1442695040888963407);
                    let u = (rng >> 11) as f64 / (1u64 << 53) as f64;
                    let y = ch.apply(
                        &frame,
                        &Impairments {
                            n0: n0_for_ebn0(ebn0, md.energy_per_bit()),
                            frac_delay: u,
                            lead: 300 + (rng >> 40) as usize % 7,
                            tail: 600,
                            cfo_cycles_per_sample: (u - 0.5) * 0.004,
                            phase_rad: u * 6.0,
                        },
                    );
                    let mut r = LiveReceiver::new(md.clone(), rxc.clone()).unwrap();
                    let mut fr = Vec::new();
                    for b in y.chunks(512) {
                        fr.extend(r.push(b));
                    }
                    fr.extend(r.finish());
                    good += fr.iter().filter(|f| f.crc_ok).count();
                }
                line += &format!("  step {step}: {good}/{trials}");
            }
            println!("{line}");
        }
    }

    #[test]
    #[ignore]
    fn stage_costs() {
        for (fs, baud, dec) in [
            (1e6, 125e3, 2usize),
            (2e6, 250e3, 2),
            (1e6, 250e3, 1),
            (4e6, 500e3, 2),
        ] {
            let sps = (fs / baud) as usize;
            let n = fs as usize; // one second
            let mut x: Vec<Complex32> = (0..n)
                .map(|i| Complex32 {
                    re: (i as f32 * 0.37).sin(),
                    im: (i as f32 * 0.11).cos(),
                })
                .collect();
            let t_mix = secs(|| Nco { phase: 0.0 }.mix(&mut x, 250e3, fs));
            let mut out = Vec::with_capacity(n);
            let t_dec = secs(|| Decimator::new(dec).run(&x, &mut out));
            // the receiver on frames at its own rate: 64-byte v1 frames every 10 ms, 12 dB Es/N0
            let md = Modem::new(ModemConfig {
                sps: sps / dec,
                frame: FrameFormat::V1,
                ..ModemConfig::new(Modulation::Qpsk)
            })
            .unwrap();
            let per = (fs / dec as f64 * 0.01) as usize;
            let mut tx = Vec::new();
            while tx.len() < n / dec {
                let mut f = md.modulate(&[7u8; 64]).unwrap();
                f.resize(per, Complex32 { re: 0.0, im: 0.0 });
                tx.extend(f);
            }
            let mut ch = Channel::new(1);
            let y = ch.apply(
                &tx,
                &Impairments {
                    n0: n0_for_ebn0(10.0, md.energy_per_bit()),
                    ..Default::default()
                },
            );
            let rxc = RxConfig {
                cfo_search: 0.03,
                max_len: Some(200),
                only_code: Some(PayloadCode::None),
                ..RxConfig::default()
            };
            // noise only, the same length: what detection alone costs
            let noise = Channel::new(2).apply(
                &vec![Complex32 { re: 0.0, im: 0.0 }; y.len()],
                &Impairments {
                    n0: n0_for_ebn0(10.0, md.energy_per_bit()),
                    ..Default::default()
                },
            );
            let mut rn = LiveReceiver::new(md.clone(), rxc.clone()).unwrap();
            let blk0 = (fs / dec as f64 * 0.001) as usize;
            let t_noise = secs(|| {
                for b in noise.chunks(blk0) {
                    rn.push(b);
                }
            });
            println!("   noise only: receiver {:.0} ms", t_noise * 1e3);
            let ov = 316 + 16;
            let t_cm = secs(|| {
                let mut i = 0;
                while i + blk0 + ov < noise.len() {
                    std::hint::black_box(md.coarse_metric(&noise[i..i + blk0 + ov], &rxc));
                    i += blk0;
                }
            });
            println!("   noise only: coarse_metric alone {:.0} ms", t_cm * 1e3);
            let t_m = secs(|| {
                let mut i = 0;
                let plain = RxConfig::default();
                while i + blk0 + ov < noise.len() {
                    std::hint::black_box(md.metric(&noise[i..i + blk0 + ov], &plain));
                    i += blk0;
                }
            });
            println!(
                "   noise only: plain metric (one frequency, no FFT) {:.0} ms",
                t_m * 1e3
            );
            for step in [2usize, 4] {
                let rs = RxConfig {
                    coarse_step: step,
                    ..rxc.clone()
                };
                let mut r = LiveReceiver::new(md.clone(), rs).unwrap();
                let mut got = 0;
                let t = secs(|| {
                    for b in y.chunks(blk0) {
                        got += r.push(b).len();
                    }
                });
                println!(
                    "   coarse step {step}: receiver {:.0} ms ({got} frames)",
                    t * 1e3
                );
            }
            let mut r = LiveReceiver::new(md, rxc).unwrap();
            let mut got = 0;
            let blk = (fs / dec as f64 * 0.001) as usize; // 1 ms blocks
            let t_rx = secs(|| {
                for b in y.chunks(blk) {
                    got += r.push(b).len();
                }
            });
            println!(
                "fs {:.0} MS/s baud {:.0}k dec {dec}: mix {:.0} ms, decimate {:.0} ms, receiver {:.0} ms ({got} frames) per second of signal",
                fs / 1e6, baud / 1e3, t_mix * 1e3, t_dec * 1e3, t_rx * 1e3
            );
        }
    }
}
