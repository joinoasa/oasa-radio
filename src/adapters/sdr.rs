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
use oasa_waveform::interference::{cancel_tone, ChannelFilter};
use oasa_waveform::iq::{from_cf32_le, from_ci16_le};
use oasa_waveform::stream::LiveReceiver;
use oasa_waveform::{Complex32, Modem, ModemConfig, Modulation, RxConfig, RxFrame};

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
/// Offset changes remembered for frames still in the receiver (one per good frame at most).
const MAX_SHIFTS: usize = 64;

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
    /// The receiver's coarse frequency search, cycles per symbol (0 = off). With `cfo_acquire` on:
    /// the narrow search used once locked.
    pub cfo_search: f32,
    /// Acquisition: search this wide (cycles per symbol) until frames decode, then `cfo_search`
    /// around the tracked offset. 0: off (always `cfo_search`). Needs `track_cfo`.
    pub cfo_acquire: f32,
    /// Good frames in a row that count as locked.
    pub cfo_lock_frames: u32,
    /// Bad frames (CRC failed) in a row that drop the lock.
    pub cfo_unlock_bad: u32,
    /// Stream time without a good frame that drops the lock, ms.
    pub cfo_unlock_ms: u64,
    /// Acquiring, with no good frame for this long (ms), return the tracked offset to `cfo_hz`
    /// (once per such gap). 0: off. Keeps a false good frame (one that passed both CRCs by chance,
    /// say under a jammer) from leaving the wide search centred where the signal is not. Needs
    /// `cfo_acquire`.
    pub cfo_reset_ms: u64,
    /// Channel filter after the shift and decimation (opt-in, B8): a 63-tap low-pass that keeps
    /// the modem's band. With `--decimate 1` nothing else removes a strong signal beside the band,
    /// which hides preambles from the detector.
    pub rx_filter: bool,
    /// Carrier canceller (opt-in, B8): remove the strongest tone in each receiver block when it
    /// stands this many dB over the block's mean FFT bin. 0: off.
    pub rx_notch_db: f64,
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
    /// Receiver channel tracking for fading up to this Doppler, Hz (0: off, the single phase line).
    /// Receiver only; nothing changes on the air. See `docs/phase-tracking.md`.
    pub phase_track_hz: f64,
}

/// Largest Doppler `--phase-track-hz` may give, as a fraction of the symbol rate.
pub const MAX_PHASE_TRACK: f64 = 0.01;

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
            cfo_acquire: 0.0,
            cfo_lock_frames: 2,
            cfo_unlock_bad: 3,
            cfo_unlock_ms: 500,
            cfo_reset_ms: 0,
            rx_filter: false,
            rx_notch_db: 0.0,
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
            phase_track_hz: 0.0,
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
        if self.cfo_acquire != 0.0 {
            if !(self.cfo_acquire > self.cfo_search && self.cfo_acquire.is_finite()) {
                return Err("--cfo-acquire must be wider than --cfo-search (or 0: off)".into());
            }
            if !self.track_cfo {
                return Err("--cfo-acquire needs --track-cfo on".into());
            }
            if !(1..=1000).contains(&self.cfo_lock_frames) {
                return Err("--cfo-lock-frames must be 1 to 1000".into());
            }
            if !(1..=1000).contains(&self.cfo_unlock_bad) {
                return Err("--cfo-unlock-bad must be 1 to 1000".into());
            }
            if !(1..=3_600_000).contains(&self.cfo_unlock_ms) {
                return Err("--cfo-unlock-ms must be 1 to 3600000".into());
            }
        }
        if !(self.rx_notch_db == 0.0 || (10.0..=100.0).contains(&self.rx_notch_db)) {
            return Err("--rx-notch-db must be 0 (off) or 10 to 100".into());
        }
        if self.cfo_reset_ms != 0 {
            if self.cfo_acquire == 0.0 {
                return Err("--cfo-reset-ms needs --cfo-acquire".into());
            }
            if self.cfo_reset_ms > 3_600_000 {
                return Err("--cfo-reset-ms must be 0 to 3600000".into());
            }
        }
        if !(Duration::from_millis(1)..=Duration::from_secs(1)).contains(&self.tx_lead) {
            return Err("--tx-lead-ms must be 1 to 1000".into());
        }
        if !(0.0..=MAX_PHASE_TRACK * self.symbol_rate).contains(&self.phase_track_hz) {
            return Err(format!(
                "--phase-track-hz must be 0 (off) to {MAX_PHASE_TRACK} of the symbol rate"
            ));
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
            // t_us: microseconds since the Unix epoch, so two bridges on one host can time a frame one way
            // (ts has only millisecond resolution)
            let now = SystemTime::now();
            let t_us = now
                .duration_since(SystemTime::UNIX_EPOCH)
                .map(|d| d.as_micros())
                .unwrap_or(0);
            let line = format!(
                "{{\"ts\":\"{}\",\"t_us\":{t_us},\"event\":\"{event}\"{fields}}}\n",
                rfc3339(now)
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
            cfo_search: if cfg.cfo_acquire > 0.0 {
                cfg.cfo_acquire
            } else {
                cfg.cfo_search
            },
            max_len: Some(cfg.mtu),
            only_code: Some(cfg.code),
            coarse_step: cfg.coarse_step,
            phase_track: (cfg.phase_track_hz / cfg.symbol_rate) as f32,
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
                ",\"mod\":\"{}\",\"code\":\"{}\",\"sample_rate\":{},\"symbol_rate\":{},\"if_hz\":{},\"cfo_hz\":{},\"cfo_search\":{},\"cfo_acquire\":{},\"decimate\":{},\"block\":{},\"mtu\":{},\"tx_mode\":\"{}\",\"phase_track_hz\":{},\"longest_frame_samples\":{},\"rx_filter\":{},\"rx_notch_db\":{},\"cfo_reset_ms\":{}",
                cfg.modulation.name(),
                cfg.code.name(),
                cfg.sample_rate,
                cfg.symbol_rate,
                cfg.if_hz,
                cfg.cfo_hz,
                cfg.cfo_search,
                cfg.cfo_acquire,
                cfg.decimate,
                cfg.block,
                cfg.mtu,
                match cfg.tx_mode {
                    TxMode::Stream => "stream",
                    TxMode::Burst => "burst",
                },
                cfg.phase_track_hz,
                receiver.modem().longest_frame_samples(&RxConfig {
                    max_len: Some(cfg.mtu),
                    only_code: Some(cfg.code),
                    ..RxConfig::default()
                }),
                cfg.rx_filter,
                cfg.rx_notch_db,
                cfg.cfo_reset_ms,
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

/// Why the acquisition lock was dropped.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum UnlockReason {
    BadFrames,
    Timeout,
}

impl UnlockReason {
    fn name(self) -> &'static str {
        match self {
            UnlockReason::BadFrames => "bad_frames",
            UnlockReason::Timeout => "timeout",
        }
    }
}

/// A change of the acquisition state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CfoChange {
    Lock,
    Unlock(UnlockReason),
    /// Acquiring with no good frame for `cfo_reset_ms`: the offset goes back to its start.
    Reset,
}

/// Acquisition state (`--cfo-acquire`): wide search until `lock_frames` good frames in a row, then
/// locked (narrow search around the tracked offset) until `unlock_bad` bad frames in a row or
/// `timeout` samples without a good frame. Times are stream sample counts, so the state does not
/// depend on how fast the receiver runs.
#[derive(Debug, Clone)]
struct CfoLock {
    locked: bool,
    /// Good frames in a row while acquiring.
    good: u32,
    /// Bad frames in a row while locked.
    bad: u32,
    /// Stream sample of the last good frame.
    last_good: u64,
    lock_frames: u32,
    unlock_bad: u32,
    timeout: u64,
    /// Samples without a good frame, acquiring, before a [`CfoChange::Reset`] (0: never).
    reset: u64,
    /// The reset for the current gap has happened.
    reset_done: bool,
}

impl CfoLock {
    fn new(lock_frames: u32, unlock_bad: u32, timeout: u64) -> Self {
        Self {
            locked: false,
            good: 0,
            bad: 0,
            last_good: 0,
            lock_frames,
            unlock_bad,
            timeout,
            reset: 0,
            reset_done: false,
        }
    }

    fn with_reset(mut self, reset: u64) -> Self {
        self.reset = reset;
        self
    }

    /// A frame ending at stream sample `at`.
    fn frame(&mut self, crc_ok: bool, at: u64) -> Option<CfoChange> {
        if crc_ok {
            self.last_good = self.last_good.max(at);
            self.bad = 0;
            self.reset_done = false;
            if !self.locked {
                self.good += 1;
                if self.good >= self.lock_frames {
                    self.locked = true;
                    self.good = 0;
                    return Some(CfoChange::Lock);
                }
            }
            return None;
        }
        if !self.locked {
            self.good = 0;
            return None;
        }
        self.bad += 1;
        if self.bad >= self.unlock_bad {
            self.unlock();
            return Some(CfoChange::Unlock(UnlockReason::BadFrames));
        }
        None
    }

    /// The stream has reached sample `at`.
    fn tick(&mut self, at: u64) -> Option<CfoChange> {
        if self.locked && at.saturating_sub(self.last_good) > self.timeout {
            self.unlock();
            return Some(CfoChange::Unlock(UnlockReason::Timeout));
        }
        if !self.locked
            && self.reset > 0
            && !self.reset_done
            && at.saturating_sub(self.last_good) > self.reset
        {
            self.reset_done = true;
            return Some(CfoChange::Reset);
        }
        None
    }

    fn unlock(&mut self) {
        self.locked = false;
        self.good = 0;
        self.bad = 0;
    }
}

/// What one receiver pass gave.
#[derive(Default)]
struct Pass {
    /// Each frame with the tracked offset after it, Hz.
    frames: Vec<(RxFrame, f64)>,
    /// Acquisition changes, each with the offset (Hz) and the search range after it.
    changes: Vec<(CfoChange, f64, f32)>,
}

/// The receive chain after the stream is read: shift back by the IF and the tracked offset,
/// decimate, collect blocks, run the receiver, follow the offset and the acquisition state.
struct RxChain {
    receiver: LiveReceiver,
    nco: Nco,
    dec: Decimator,
    pending: Vec<Complex32>,
    /// The tracked offset, Hz.
    cfo_hz: f64,
    /// Decimated samples handed to the receiver.
    fed: u64,
    lock: Option<CfoLock>,
    /// With acquisition: (first decimated sample, offset it was shifted by), for the samples the
    /// receiver may still hold. A frame's estimate is relative to the shift its own samples got,
    /// which is not the newest one when the offset moved after they were read.
    shifts: VecDeque<(u64, f64)>,
    narrow: f32,
    wide: f32,
    /// The starting offset, Hz (where a [`CfoChange::Reset`] puts the tracker back).
    cfo_start: f64,
    sample_rate: f64,
    symbol_rate: f64,
    if_hz: f64,
    block: usize,
    track: bool,
    filter: Option<ChannelFilter>,
    notch_db: Option<f64>,
    /// Blocks where the carrier canceller removed a tone.
    notched: u64,
}

impl RxChain {
    fn new(cfg: &SdrConfig, receiver: LiveReceiver) -> Self {
        let fs_out = cfg.sample_rate / cfg.decimate as f64;
        let lock = (cfg.cfo_acquire > 0.0).then(|| {
            CfoLock::new(
                cfg.cfo_lock_frames,
                cfg.cfo_unlock_bad,
                (cfg.cfo_unlock_ms as f64 * 1e-3 * fs_out) as u64,
            )
            .with_reset((cfg.cfo_reset_ms as f64 * 1e-3 * fs_out) as u64)
        });
        Self {
            receiver,
            nco: Nco { phase: 0.0 },
            dec: Decimator::new(cfg.decimate),
            pending: Vec::with_capacity(cfg.block * 2),
            cfo_hz: cfg.cfo_hz,
            fed: 0,
            lock,
            shifts: VecDeque::new(),
            narrow: cfg.cfo_search,
            wide: cfg.cfo_acquire,
            cfo_start: cfg.cfo_hz,
            sample_rate: cfg.sample_rate,
            symbol_rate: cfg.symbol_rate,
            if_hz: cfg.if_hz,
            block: cfg.block,
            track: cfg.track_cfo,
            filter: cfg
                .rx_filter
                .then(|| ChannelFilter::for_modem(cfg.sps().unwrap_or(4) / cfg.decimate, 0.35)),
            notch_db: (cfg.rx_notch_db > 0.0).then_some(cfg.rx_notch_db),
            notched: 0,
        }
    }

    /// Shift and decimate stream samples; returns the new decimated samples.
    fn feed(&mut self, x: &mut [Complex32]) -> &[Complex32] {
        if self.lock.is_some() && self.shifts.back().is_none_or(|s| s.1 != self.cfo_hz) {
            let at = self.fed + self.pending.len() as u64;
            self.shifts.push_back((at, self.cfo_hz));
            if self.shifts.len() > MAX_SHIFTS {
                self.shifts.pop_front();
            }
        }
        self.nco
            .mix(x, -(self.if_hz + self.cfo_hz), self.sample_rate);
        let before = self.pending.len();
        self.dec.run(x, &mut self.pending);
        if let Some(f) = self.filter.as_mut() {
            f.run(&mut self.pending[before..]);
        }
        &self.pending[before..]
    }

    /// A block is waiting for the receiver.
    fn ready(&self) -> bool {
        self.pending.len() >= self.block
    }

    /// Run the receiver on what is waiting.
    fn run(&mut self) -> Pass {
        self.fed += self.pending.len() as u64;
        if let Some(db) = self.notch_db {
            self.notched += cancel_tone(&mut self.pending, db).is_some() as u64;
        }
        let frames = self.receiver.push(&self.pending);
        self.pending.clear();
        let mut pass = Pass::default();
        for fr in frames {
            if fr.crc_ok && self.track {
                let resid = f64::from(fr.cfo_cycles_per_symbol) * self.symbol_rate;
                // Acquiring, a good frame's estimate is taken whole, so the narrow search is
                // centred on it at the lock. Locked (or without acquisition), it is smoothed.
                let gain = match &self.lock {
                    Some(l) if !l.locked => 1.0,
                    _ => CFO_TRACK_GAIN,
                };
                // the shift this frame's samples got (without acquisition: the newest, as before)
                let shift = self
                    .shifts
                    .iter()
                    .rev()
                    .find(|s| s.0 <= fr.start)
                    .map_or(self.cfo_hz, |s| s.1);
                self.cfo_hz += gain * (shift + resid - self.cfo_hz);
                self.cfo_hz = self
                    .cfo_hz
                    .clamp(-self.sample_rate / 8.0, self.sample_rate / 8.0);
            }
            let change = self.lock.as_mut().and_then(|l| l.frame(fr.crc_ok, fr.end));
            if let Some(c) = change {
                self.apply(c, &mut pass);
            }
            pass.frames.push((fr, self.cfo_hz));
        }
        if let Some(c) = self.lock.as_mut().and_then(|l| l.tick(self.fed)) {
            self.apply(c, &mut pass);
        }
        pass
    }

    fn apply(&mut self, c: CfoChange, pass: &mut Pass) {
        let search = match c {
            CfoChange::Lock => self.narrow,
            CfoChange::Unlock(_) => self.wide,
            CfoChange::Reset => {
                self.cfo_hz = self.cfo_start;
                self.wide
            }
        };
        // both ranges are above 0 (checked with the config), so this cannot fail
        let _ = self.receiver.set_cfo_search(search);
        pass.changes.push((c, self.cfo_hz, search));
    }
}

fn rx_thread(cfg: &SdrConfig, shared: &Shared, receiver: LiveReceiver, out: SyncSender<Vec<u8>>) {
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
    let mut chain = RxChain::new(cfg, receiver);
    // (decimated sample count at the end of a read, when it was read): to time each decision
    let mut arrivals: VecDeque<(u64, Instant)> = VecDeque::new();
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
        let new = chain.feed(&mut x);
        if let Some(w) = record.as_mut() {
            if recorded < record_max {
                let take = (new.len() as u64).min(record_max - recorded) as usize;
                for v in &new[..take] {
                    let _ = w.write_all(&v.re.to_le_bytes());
                    let _ = w.write_all(&v.im.to_le_bytes());
                }
                recorded += take as u64;
                if recorded >= record_max {
                    let _ = w.flush();
                }
            }
        }
        if !chain.ready() {
            continue;
        }
        let now = Instant::now();
        let fed = chain.fed + chain.pending.len() as u64;
        arrivals.push_back((fed, now));
        let pass = chain.run();
        let decided = Instant::now();
        for (fr, cfo_hz) in pass.frames {
            // when the frame's last sample arrived
            let end = fr.end;
            let arrived = arrivals.iter().find(|(k, _)| *k >= end).map(|(_, t)| *t);
            let wait_ms = arrived.map(|t| decided.duration_since(t).as_secs_f64() * 1e3);
            let resid = f64::from(fr.cfo_cycles_per_symbol);
            if fr.crc_ok {
                shared.rx_frames.fetch_add(1, Ordering::Relaxed);
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
        for (c, cfo_hz, search) in pass.changes {
            let (event, reason) = match c {
                CfoChange::Lock => ("cfo_lock", String::new()),
                CfoChange::Unlock(r) => ("cfo_unlock", format!(",\"reason\":\"{}\"", r.name())),
                CfoChange::Reset => ("cfo_reset", String::new()),
            };
            shared.event(
                event,
                &format!(
                    ",\"cfo_hz\":{cfo_hz:.1},\"cfo_search\":{search}{reason},\"stream_ms\":{:.1}",
                    fed as f64 / fs_out * 1e3
                ),
            );
        }
        // forget arrivals the receiver can no longer report on
        let keep_from =
            fed.saturating_sub(4 * chain.receiver.window_len() as u64 + cfg.block as u64);
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

#[cfg(test)]
mod acquire {
    //! Acquisition (`--cfo-acquire`): the state machine, and the whole receive chain on a simulated
    //! bench stream at the bench's settings (1 MS/s, 250 kbaud, IF 175 kHz, decimate 1, coarse step 4).
    //! The table: `cargo test --release --lib acquire::table -- --ignored --nocapture`.
    use super::*;
    use oasa_waveform::channel::Channel;
    use oasa_waveform::rng::Rng;

    #[test]
    fn locks_after_good_frames_in_a_row() {
        let mut l = CfoLock::new(2, 3, 1000);
        assert_eq!(l.frame(true, 10), None);
        assert_eq!(l.frame(false, 20), None, "a bad frame while acquiring...");
        assert_eq!(l.frame(true, 30), None, "...starts the count again");
        assert_eq!(l.frame(true, 40), Some(CfoChange::Lock));
        assert!(l.locked);
        assert_eq!(l.frame(true, 50), None);
        assert_eq!(l.tick(1000), None);
    }

    #[test]
    fn bad_frames_in_a_row_unlock() {
        let mut l = CfoLock::new(1, 3, 1_000_000);
        assert_eq!(l.frame(true, 0), Some(CfoChange::Lock));
        assert_eq!(l.frame(false, 1), None);
        assert_eq!(l.frame(false, 2), None);
        assert_eq!(l.frame(true, 3), None, "a good frame resets the run");
        assert_eq!(l.frame(false, 4), None);
        assert_eq!(l.frame(false, 5), None);
        assert_eq!(
            l.frame(false, 6),
            Some(CfoChange::Unlock(UnlockReason::BadFrames))
        );
        assert!(!l.locked);
        assert_eq!(l.frame(false, 7), None, "no second unlock");
        assert_eq!(l.frame(true, 8), Some(CfoChange::Lock), "relocks");
    }

    #[test]
    fn no_good_frame_for_the_timeout_unlocks() {
        let mut l = CfoLock::new(1, 3, 100);
        assert_eq!(l.tick(10_000), None, "acquiring: no timeout");
        assert_eq!(l.frame(true, 10_000), Some(CfoChange::Lock));
        assert_eq!(l.tick(10_100), None);
        assert_eq!(l.frame(true, 10_090), None);
        assert_eq!(l.tick(10_190), None);
        assert_eq!(
            l.tick(10_191),
            Some(CfoChange::Unlock(UnlockReason::Timeout))
        );
        assert_eq!(l.tick(20_000), None);
    }

    /// The bench's settings.
    fn bench(acquire: f32, search: f32, unlock_ms: u64) -> SdrConfig {
        let mut c = SdrConfig::new("/dev/null".into(), "/dev/null".into());
        c.sample_rate = 1e6;
        c.symbol_rate = 250e3;
        c.if_hz = 175e3;
        c.decimate = 1;
        c.coarse_step = 4;
        c.cfo_search = search;
        c.cfo_acquire = acquire;
        c.cfo_unlock_ms = unlock_ms;
        c.check().unwrap();
        c
    }

    fn chain(c: &SdrConfig) -> RxChain {
        let md = Modem::new(ModemConfig {
            sps: c.sps().unwrap() / c.decimate,
            frame: FrameFormat::V1,
            code: c.code,
            ..ModemConfig::new(c.modulation)
        })
        .unwrap();
        let rx = RxConfig {
            cfo_search: if c.cfo_acquire > 0.0 {
                c.cfo_acquire
            } else {
                c.cfo_search
            },
            max_len: Some(c.mtu),
            only_code: Some(c.code),
            coarse_step: c.coarse_step,
            ..RxConfig::default()
        };
        RxChain::new(c, LiveReceiver::new(md, rx).unwrap())
    }

    #[derive(Debug, Default)]
    struct Run {
        sent: usize,
        good: usize,
        bad: usize,
        /// Stream ms of each lock and unlock.
        locks: Vec<f64>,
        unlocks: Vec<(f64, UnlockReason)>,
        cfo_hz: f64,
        /// Search work per block while acquiring (or always, without acquisition) and while locked:
        /// (blocks, FFT points, bins, seconds).
        wide: (u64, u64, u64, f64),
        narrow: (u64, u64, u64, f64),
        /// Frames sent after `count_from_ms` and how many of them arrived.
        late_sent: usize,
        late_good: usize,
    }

    /// Bursts of random 40 to 200-byte frames, 0.5 to 3 ms apart, through a cable whose offset at
    /// time t (seconds) is `cfo(t)`, -20 dB, noise -45 dBFS (Es/N0 about 20 dB).
    fn run(c: &SdrConfig, seconds: f64, cfo: impl Fn(f64) -> f64, count_from_ms: f64) -> Run {
        let sps = c.sps().unwrap();
        let tx = Modem::new(ModemConfig {
            sps,
            frame: FrameFormat::V1,
            code: c.code,
            ..ModemConfig::new(c.modulation)
        })
        .unwrap();
        let mut rng = Rng::new(7);
        let mut stream = Vec::new();
        let mut sent = Vec::new();
        let n = (seconds * c.sample_rate) as usize;
        while stream.len() < n {
            let len = 40 + rng.below(161) as usize;
            let p = rng.bytes(len);
            let mut s = tx.modulate(&p).unwrap();
            let peak = s.iter().map(|v| v.re.hypot(v.im)).fold(0.0f32, f32::max);
            let k = c.tx_peak / peak;
            for v in &mut s {
                v.re *= k;
                v.im *= k;
            }
            let gap = GAP_SYMBOLS * sps + 500 + rng.below(2500) as usize;
            sent.push((stream.len(), p));
            stream.extend(s);
            stream.extend(std::iter::repeat_n(Complex32::ZERO, gap));
        }
        // the cable: gain, IF and offset (phase continuous), noise
        let g = 0.1f32;
        let mut ph = 0.0f64;
        for (i, v) in stream.iter_mut().enumerate() {
            let f = c.if_hz + cfo(i as f64 / c.sample_rate);
            let (sn, co) = ph.sin_cos();
            *v = *v * Complex32::new(co as f32 * g, sn as f32 * g);
            ph = (ph + 2.0 * std::f64::consts::PI * f / c.sample_rate)
                .rem_euclid(2.0 * std::f64::consts::PI);
        }
        Channel::new(3).add_awgn(&mut stream, 10f64.powf(-4.5));

        let mut ch = chain(c);
        let fs_out = c.sample_rate / c.decimate as f64;
        let mut r = Run {
            sent: sent.len(),
            ..Run::default()
        };
        let mut got: Vec<Vec<u8>> = Vec::new();
        let mut got_at = Vec::new();
        for chunk in stream.chunks(RX_READ) {
            let mut x = chunk.to_vec();
            ch.feed(&mut x);
            if !ch.ready() {
                continue;
            }
            let locked = ch.lock.as_ref().is_some_and(|l| l.locked);
            let w0 = ch.receiver.search_work();
            let t0 = Instant::now();
            let pass = ch.run();
            let dt = t0.elapsed().as_secs_f64();
            let w1 = ch.receiver.search_work();
            let acc = if locked { &mut r.narrow } else { &mut r.wide };
            acc.0 += 1;
            acc.1 += w1.fft_points - w0.fft_points;
            acc.2 += w1.bins - w0.bins;
            acc.3 += dt;
            let ms = ch.fed as f64 / fs_out * 1e3;
            for (fr, _) in pass.frames {
                if fr.crc_ok {
                    r.good += 1;
                    got_at.push(fr.start);
                    got.push(fr.payload);
                } else {
                    r.bad += 1;
                }
            }
            for (chg, _, _) in pass.changes {
                match chg {
                    CfoChange::Lock => r.locks.push(ms),
                    CfoChange::Unlock(why) => r.unlocks.push((ms, why)),
                    CfoChange::Reset => {}
                }
            }
        }
        for (start, p) in &sent {
            if *start as f64 / c.sample_rate * 1e3 >= count_from_ms {
                r.late_sent += 1;
                if got.iter().any(|g| g == p) {
                    r.late_good += 1;
                }
            }
        }
        // nothing invented, nothing twice
        for g in &got {
            assert!(sent.iter().any(|(_, p)| p == g), "a frame nobody sent");
        }
        let mut s = got_at.clone();
        s.dedup();
        assert_eq!(s.len(), got_at.len(), "a frame delivered twice");
        r.cfo_hz = ch.cfo_hz;
        r
    }

    fn per_block(w: (u64, u64, u64, f64)) -> (f64, f64, f64) {
        let b = w.0.max(1) as f64;
        (w.1 as f64 / b, w.2 as f64 / b, w.3 / b * 1e6)
    }

    /// The search ranges in FFT points and bins at the bench's settings.
    fn sizes(search: f32) -> (usize, usize) {
        let c = chain(&bench(0.0, search, 500));
        (c.receiver.coarse().fft_len, c.receiver.coarse().bins)
    }

    #[test]
    fn locks_at_f20_offsets_and_then_searches_narrow() {
        let c = bench(0.15, 0.03, 500);
        let (narrow_fft, _) = sizes(0.03);
        let (wide_fft, _) = sizes(0.15);
        assert_eq!((narrow_fft, wide_fft), (32, 128));
        // +-18.3 kHz is F20's +-20 ppm at 915 MHz; +-30 kHz is margin
        for hz in [0.0, 18_300.0, -18_300.0, 30_000.0, -30_000.0] {
            let r = run(&c, 0.3, |_| hz, 0.0);
            assert_eq!(r.locks.len(), 1, "{hz} Hz: {r:?}");
            assert!(r.unlocks.is_empty(), "{hz} Hz: {r:?}");
            assert!(r.locks[0] < 20.0, "{hz} Hz: lock at {} ms", r.locks[0]);
            assert_eq!((r.good, r.bad), (r.sent, 0), "{hz} Hz: {r:?}");
            assert!(
                (r.cfo_hz - hz).abs() < 200.0,
                "{hz} Hz: tracked {}",
                r.cfo_hz
            );
            // locked, every position costs the narrow FFT: a quarter of the wide one
            let (pts, _, _) = per_block(r.narrow);
            let (wpts, _, _) = per_block(r.wide);
            assert!(r.narrow.0 > 20 * r.wide.0, "{hz} Hz: {r:?}");
            assert!(pts > 0.0 && wpts / pts > 3.5, "{hz} Hz: {pts} {wpts}");
        }
        // the narrow search alone (the default without --cfo-acquire) does not find +18.3 kHz
        let r = run(&bench(0.0, 0.03, 500), 0.1, |_| 18_300.0, 0.0);
        assert_eq!(r.good, 0, "{r:?}");
    }

    #[test]
    fn a_slow_ramp_is_tracked_without_dropping_the_lock() {
        // 18.3 kHz to 28.3 kHz in one second: far faster than any crystal drifts
        let c = bench(0.15, 0.03, 500);
        let r = run(&c, 1.0, |t| 18_300.0 + 10_000.0 * t, 0.0);
        assert_eq!(r.locks.len(), 1, "{r:?}");
        assert!(r.unlocks.is_empty(), "{r:?}");
        assert_eq!(r.good, r.sent, "{r:?}");
        assert!((r.cfo_hz - 28_300.0).abs() < 500.0, "{}", r.cfo_hz);
    }

    #[test]
    fn a_step_past_the_narrow_window_reacquires_wide() {
        // locked at 18.3 kHz, then a step of 21.7 kHz at 300 ms (the narrow search reaches about
        // +-13 kHz), unlock after 200 ms without a good frame
        let c = bench(0.15, 0.03, 200);
        let r = run(
            &c,
            0.8,
            |t| if t < 0.3 { 18_300.0 } else { 40_000.0 },
            550.0,
        );
        assert_eq!(r.locks.len(), 2, "{r:?}");
        assert_eq!(r.unlocks.len(), 1, "{r:?}");
        let (at, why) = r.unlocks[0];
        assert_eq!(why, UnlockReason::Timeout);
        assert!((490.0..530.0).contains(&at), "unlock at {at} ms");
        assert!(r.locks[1] > at && r.locks[1] < at + 20.0, "{r:?}");
        assert!((r.cfo_hz - 40_000.0).abs() < 200.0, "{}", r.cfo_hz);
        assert!(r.late_sent > 50 && r.late_good == r.late_sent, "{r:?}");
    }

    #[test]
    #[ignore]
    fn table() {
        for search in [0.03f32, 0.1, 0.15] {
            let (f, b) = sizes(search);
            println!("search {search}: FFT {f} points, bins +-{b}");
        }
        println!(
            "| offset | search | lock ms | sent | good | bad | loss % | FFT points/block | bins/block | us/block |"
        );
        println!("|---|---|---|---|---|---|---|---|---|---|");
        for hz in [0.0, 18_300.0, -18_300.0, 30_000.0, -30_000.0] {
            for (name, c) in [
                ("acquire 0.15, locked 0.03", bench(0.15, 0.03, 500)),
                ("always 0.03", bench(0.0, 0.03, 500)),
                ("always 0.1", bench(0.0, 0.1, 500)),
                ("always 0.15", bench(0.0, 0.15, 500)),
            ] {
                let r = run(&c, 2.0, |_| hz, 0.0);
                // locked blocks with acquisition, every block without
                let w = if c.cfo_acquire > 0.0 {
                    r.narrow
                } else {
                    r.wide
                };
                let (p, b, us) = per_block(w);
                println!(
                    "| {hz:+.0} Hz | {name} | {} | {} | {} | {} | {:.2} | {p:.0} | {b:.0} | {us:.0} |",
                    r.locks
                        .first()
                        .map(|v| format!("{v:.1}"))
                        .unwrap_or("-".into()),
                    r.sent,
                    r.good,
                    r.bad,
                    100.0 * (r.sent - r.good) as f64 / r.sent as f64
                );
            }
        }
        let c = bench(0.15, 0.03, 500);
        let r = run(&c, 2.0, |t| 18_300.0 + 5_000.0 * t, 0.0);
        println!(
            "ramp 18.3 -> 28.3 kHz over 2 s: locks {:?} unlocks {:?} sent {} good {} tracked {:.0} Hz",
            r.locks, r.unlocks, r.sent, r.good, r.cfo_hz
        );
        let r = run(
            &c,
            3.0,
            |t| if t < 1.0 { 18_300.0 } else { 40_000.0 },
            1600.0,
        );
        println!(
            "step 18.3 -> 40 kHz at 1 s (unlock 500 ms): locks {:?} unlocks {:?} sent {} good {}; from 1.6 s {} of {}; tracked {:.0} Hz",
            r.locks, r.unlocks, r.sent, r.good, r.late_good, r.late_sent, r.cfo_hz
        );
    }

    /// B8: what a run with a jammer gave.
    #[derive(Debug, Default)]
    struct Jam {
        /// Frames sent wholly before the jam, during it, and wholly after it.
        sent_before: usize,
        good_before: usize,
        /// Frames the receiver returned during the jam (CRC good, CRC bad).
        good_during: usize,
        bad_during: usize,
        /// Frames sent overlapping the jam, and of those delivered.
        sent_during: usize,
        delivered_during: usize,
        sent_after: usize,
        good_after: usize,
        /// Stream ms from the end of the jam to the end of the first good frame after it.
        back_ms: Option<f64>,
        /// The first frame sent after the jam was delivered.
        first_after_ok: bool,
        /// Stream ms of each lock and unlock, from the start of the jam.
        locks: Vec<f64>,
        unlocks: Vec<(f64, UnlockReason)>,
        resets: Vec<f64>,
        /// Samples clipped at full scale.
        clipped: usize,
        cfo_hz: f64,
        /// Seconds of CPU the receiver took per second of stream.
        load: f64,
    }

    /// Frames as in [`run`] (random 40 to 200 bytes, 0.5 to 3 ms apart; signal about -32 dBFS,
    /// Es/N0 about 20 dB, offset `cfo_hz`), and noise `jam_db` above the signal's power, flat over
    /// the signal's band (420 kHz at the IF), from `jam_at` for `jam_s` seconds, then
    /// `after_s` seconds clear. The stream clips at full scale (I and Q at +-1) like a radio's ADC.
    /// `false_frame`: `(s, Hz)`, at `s` seconds the receiver takes a false good frame that puts the
    /// tracked offset at `Hz` (what one noise frame passing both CRCs by chance would do).
    fn jam(
        c: &SdrConfig,
        seed: u64,
        cfo_hz: f64,
        (jam_at, jam_s, after_s): (f64, f64, f64),
        jam_db: f64,
        false_frame: Option<(f64, f64)>,
    ) -> Jam {
        let model = oasa_waveform::interference::Interferer::Noise {
            offset: c.if_hz / c.sample_rate,
            width: 420e3 / c.sample_rate,
        };
        interfere(
            c,
            seed,
            cfo_hz,
            (jam_at, jam_s, after_s),
            jam_db,
            false_frame,
            model,
        )
    }

    /// [`jam`] with any interferer, `model` in cycles per sample of the stream (the signal is at
    /// `if_hz + cfo_hz`).
    fn interfere(
        c: &SdrConfig,
        seed: u64,
        cfo_hz: f64,
        (jam_at, jam_s, after_s): (f64, f64, f64),
        jam_db: f64,
        false_frame: Option<(f64, f64)>,
        model: oasa_waveform::interference::Interferer,
    ) -> Jam {
        use oasa_waveform::interference::{clip, InterfererGen};
        let sps = c.sps().unwrap();
        let tx = Modem::new(ModemConfig {
            sps,
            frame: FrameFormat::V1,
            code: c.code,
            ..ModemConfig::new(c.modulation)
        })
        .unwrap();
        let fs = c.sample_rate;
        let (j0, j1) = ((jam_at * fs) as usize, ((jam_at + jam_s) * fs) as usize);
        let n = ((jam_at + jam_s + after_s) * fs) as usize;
        let mut rng = Rng::new(seed);
        let mut stream = Vec::with_capacity(n + 100_000);
        let mut sent = Vec::new();
        let (mut p_sig, mut n_sig) = (0.0f64, 0usize);
        while stream.len() < n {
            let len = 40 + rng.below(161) as usize;
            let p = rng.bytes(len);
            let mut s = tx.modulate(&p).unwrap();
            let peak = s.iter().map(|v| v.re.hypot(v.im)).fold(0.0f32, f32::max);
            let k = c.tx_peak / peak;
            for v in &mut s {
                *v = *v * k;
                p_sig += f64::from(v.norm_sqr());
            }
            n_sig += s.len();
            let gap = GAP_SYMBOLS * sps + 500 + rng.below(2500) as usize;
            sent.push((stream.len(), stream.len() + s.len(), p));
            stream.extend(s);
            stream.extend(std::iter::repeat_n(Complex32::ZERO, gap));
        }
        stream.truncate(n);
        sent.retain(|(_, e, _)| *e <= n);
        let g = 0.1f32;
        let p_sig = p_sig / n_sig as f64 * f64::from(g * g);
        let mut ph = 0.0f64;
        let mut rng_ph = Rng::new(seed ^ 0xC0FF_EE00);
        ph += 2.0 * std::f64::consts::PI * rng_ph.uniform();
        for v in stream.iter_mut() {
            let (sn, co) = ph.sin_cos();
            *v = *v * Complex32::new(co as f32 * g, sn as f32 * g);
            ph = (ph + 2.0 * std::f64::consts::PI * (c.if_hz + cfo_hz) / fs)
                .rem_euclid(2.0 * std::f64::consts::PI);
        }
        Channel::new(seed ^ 3).add_awgn(&mut stream, 10f64.powf(-4.5));
        let mut jammer =
            InterfererGen::new(model, p_sig * 10f64.powf(jam_db / 10.0), seed ^ 0x7A11);
        jammer.add(&mut stream[j0..j1]);
        let clipped = clip(&mut stream, 1.0);

        let mut ch = chain(c);
        let ms = |k: f64| (k - j0 as f64) / fs * 1e3;
        let mut r = Jam {
            clipped,
            ..Jam::default()
        };
        let mut got: Vec<(usize, usize, Vec<u8>)> = Vec::new();
        let t0 = Instant::now();
        let mut false_frame = false_frame.map(|(t, hz)| ((t * fs) as u64, hz));
        for chunk in stream.chunks(RX_READ) {
            let mut x = chunk.to_vec();
            if let Some((at, hz)) = false_frame {
                if ch.fed >= at {
                    ch.cfo_hz = hz;
                    if let Some(l) = ch.lock.as_mut() {
                        assert_eq!(l.frame(true, ch.fed), None);
                    }
                    false_frame = None;
                }
            }
            ch.feed(&mut x);
            if !ch.ready() {
                continue;
            }
            let pass = ch.run();
            for (fr, _) in pass.frames {
                let during = fr.end > j0 as u64 && fr.start < j1 as u64;
                match (during, fr.crc_ok) {
                    (true, true) => r.good_during += 1,
                    (true, false) => r.bad_during += 1,
                    _ => {}
                }
                if fr.crc_ok {
                    got.push((fr.start as usize, fr.end as usize, fr.payload));
                }
            }
            for (chg, _, _) in pass.changes {
                let at = ms(ch.fed as f64);
                match chg {
                    CfoChange::Lock => r.locks.push(at),
                    CfoChange::Unlock(why) => r.unlocks.push((at, why)),
                    CfoChange::Reset => r.resets.push(at),
                }
            }
        }
        r.load = t0.elapsed().as_secs_f64() / (n as f64 / fs);
        // nothing invented: every delivered frame was sent
        for (_, _, g) in &got {
            assert!(sent.iter().any(|(_, _, p)| p == g), "a frame nobody sent");
        }
        let delivered = |p: &Vec<u8>| got.iter().any(|(_, _, g)| g == p);
        let mut first_after = true;
        for (s0, e0, p) in &sent {
            if *e0 <= j0 {
                r.sent_before += 1;
                r.good_before += delivered(p) as usize;
            } else if *s0 < j1 {
                r.sent_during += 1;
                r.delivered_during += delivered(p) as usize;
            } else {
                r.sent_after += 1;
                let ok = delivered(p);
                r.good_after += ok as usize;
                if first_after {
                    r.first_after_ok = ok;
                    first_after = false;
                }
            }
        }
        r.back_ms = got
            .iter()
            .filter(|(s0, _, _)| *s0 >= j1)
            .map(|(_, e0, _)| (*e0 - j1) as f64 / fs * 1e3)
            .fold(None, |m: Option<f64>, v| Some(m.map_or(v, |m| m.min(v))));
        r.cfo_hz = ch.cfo_hz;
        r
    }

    #[test]
    fn a_jammer_drops_the_lock_and_the_link_comes_back_at_once() {
        // locked at +18.3 kHz (F20's 20 ppm), noise 40 dB over the signal for 0.8 s (the stream
        // clips), then clear. 0.8 s is past the 500 ms unlock timeout; see docs/interference.md
        // for why a longer jam ends in the same state.
        let c = bench(0.15, 0.03, 500);
        let r = jam(&c, 1, 18_300.0, (0.3, 0.8, 0.4), 40.0, None);
        assert!(r.clipped > 100_000, "{r:?}");
        assert_eq!(r.good_before, r.sent_before, "{r:?}");
        assert_eq!(r.good_during, 0, "{r:?}");
        assert_eq!(r.locks.len(), 2, "{r:?}");
        assert!(r.locks[0] < 0.0, "{r:?}");
        // down: no good frame for the timeout
        assert_eq!(r.unlocks.len(), 1, "{r:?}");
        let (at, why) = r.unlocks[0];
        assert_eq!(why, UnlockReason::Timeout, "{r:?}");
        assert!((450.0..560.0).contains(&at), "{r:?}");
        // back: the first frame after the jam, every frame after it, and the offset still right
        assert!(r.first_after_ok, "{r:?}");
        assert!(r.back_ms.unwrap() < 10.0, "{r:?}");
        assert_eq!(r.good_after, r.sent_after, "{r:?}");
        assert!((r.cfo_hz - 18_300.0).abs() < 200.0, "{r:?}");
    }

    #[test]
    fn the_channel_filter_and_the_notch_work_on_the_stream() {
        use oasa_waveform::interference::Interferer;
        let base = bench(0.15, 0.03, 500);
        let fs = base.sample_rate;
        let all = (0.0, 1.0, 0.0);
        // a carrier one channel (420 kHz) below the signal, 20 dB stronger, all the time
        let next = Interferer::Tone {
            offset: (base.if_hz - 420e3) / fs,
        };
        let r = interfere(&base, 2, 300.0, all, 20.0, None, next);
        assert!(r.delivered_during < r.sent_during / 2, "{r:?}");
        let mut c = base.clone();
        c.rx_filter = true;
        let r = interfere(&c, 2, 300.0, all, 20.0, None, next);
        assert_eq!(r.delivered_during, r.sent_during, "{r:?}");
        // a carrier on the signal's centre, as strong as the signal
        let co = Interferer::Tone {
            offset: (base.if_hz + 300.0) / fs,
        };
        let r = interfere(&c, 2, 300.0, all, 0.0, None, co);
        assert_eq!(r.delivered_during, 0, "{r:?}");
        c.rx_notch_db = 25.0;
        let r = interfere(&c, 2, 300.0, all, 0.0, None, co);
        assert!(r.delivered_during + 2 >= r.sent_during, "{r:?}");
    }

    fn improved(mut c: SdrConfig) -> SdrConfig {
        c.rx_filter = true;
        c.rx_notch_db = 25.0;
        c.check().unwrap();
        c
    }

    fn reset(mut c: SdrConfig, ms: u64) -> SdrConfig {
        c.cfo_reset_ms = ms;
        c.check().unwrap();
        c
    }

    #[test]
    fn a_false_frame_under_a_jammer_strands_the_search_unless_reset() {
        // acquiring under the jammer, a false good frame moves the offset to -25 kHz: the signal at
        // +18.3 kHz is then 43 kHz from the centre, outside the wide search (+-37.5 kHz)
        let span = (0.3, 2.0, 1.0);
        let wrong = Some((0.3 + 0.8, -25_000.0));
        let c = bench(0.15, 0.03, 500);
        let r = jam(&c, 4, 18_300.0, span, 40.0, wrong);
        assert!(r.sent_after > 100 && r.good_after == 0, "stuck: {r:?}");
        assert!(r.resets.is_empty());
        // --cfo-reset-ms 600: back to the start (0 Hz, 18.3 kHz away) 600 ms after the false frame
        let c = reset(bench(0.15, 0.03, 500), 600);
        let r = jam(&c, 4, 18_300.0, span, 40.0, wrong);
        assert_eq!(r.resets.len(), 2, "{r:?}");
        assert!((r.resets[1] - 1400.0).abs() < 10.0, "{r:?}");
        assert!(r.first_after_ok, "{r:?}");
        // the first good frame re-centres the offset by 18.3 kHz at once, which can cost the
        // frame already in the receiver (as at start-up)
        assert!(r.good_after + 1 >= r.sent_after, "{r:?}");
        assert!((r.cfo_hz - 18_300.0).abs() < 200.0, "{r:?}");
    }

    #[test]
    fn reset_fires_once_per_gap_and_only_while_acquiring() {
        let mut l = CfoLock::new(1, 3, 100).with_reset(50);
        assert_eq!(l.tick(51), Some(CfoChange::Reset));
        assert_eq!(l.tick(500), None, "once per gap");
        assert_eq!(l.frame(true, 600), Some(CfoChange::Lock));
        assert_eq!(l.tick(690), None, "locked: no reset");
        assert_eq!(l.tick(701), Some(CfoChange::Unlock(UnlockReason::Timeout)));
        assert_eq!(l.tick(702), Some(CfoChange::Reset));
        let mut off = CfoLock::new(1, 3, 100);
        assert_eq!(off.tick(1_000_000), None);
    }

    /// B8 table: 10 seeds, 2 s jam at +40 dB, both receiver settings.
    /// `cargo test --release --lib acquire::jammer_table -- --ignored --nocapture`
    #[test]
    #[ignore]
    fn jammer_table() {
        println!("| setting | seed | jam dB | sent/good before | frames during (good/bad) | unlocks (ms from jam start) | locks | first after ok | back ms | sent/good after | tracked Hz | clipped | load |");
        println!("|---|---|---|---|---|---|---|---|---|---|---|---|---|");
        for (name, c, hz) in [
            (
                "acquire 0.15, locked 0.03, +18.3 kHz",
                bench(0.15, 0.03, 500),
                18_300.0,
            ),
            ("always 0.1, +300 Hz", bench(0.0, 0.1, 500), 300.0),
            (
                "acquire as above, --cfo-reset-ms 1000",
                reset(bench(0.15, 0.03, 500), 1000),
                18_300.0,
            ),
            (
                "acquire as above, --rx-filter on --rx-notch-db 25",
                improved(bench(0.15, 0.03, 500)),
                18_300.0,
            ),
        ] {
            for seed in 1..=10u64 {
                let r = jam(&c, seed, hz, (0.5, 2.0, 1.0), 40.0, None);
                println!(
                    "| {name} | {seed} | 40 | {}/{} | {}/{} | {:?} | {:?} | {} | {} | {}/{} | {:.0} | {} | {:.2} |",
                    r.sent_before, r.good_before, r.good_during, r.bad_during,
                    r.unlocks.iter().map(|(t, w)| format!("{t:.0} {}", w.name())).collect::<Vec<_>>(),
                    r.locks.iter().map(|t| format!("{t:.1}")).collect::<Vec<_>>(),
                    r.first_after_ok,
                    r.back_ms.map(|v| format!("{v:.2}")).unwrap_or("never".into()),
                    r.sent_after, r.good_after, r.cfo_hz, r.clipped, r.load
                );
            }
        }
    }
}
