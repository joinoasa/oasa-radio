//! The modem: [`Modem::modulate`] makes complex baseband samples from a payload, [`Modem::demodulate`]
//! finds frames in a buffer of samples.
//!
//! Nothing here knows about the channel. The samples are what an SDR's `cf32` stream holds.
//!
//! What the receiver does, in order:
//! 1. **Detect.** Correlate the buffer with the known waveform of preamble + sync word at every
//!    sample offset. The metric `|c|^2 / (E_ref * E_window)` is 1 for a perfect match and has the
//!    known distribution `Beta(1, M-1)` in white noise (`M` = reference length in samples), so the
//!    threshold sets the false alarm rate per position: `(1 - t)^(M-1)`. Peaks are local maxima.
//! 2. **Estimate.** From the correlation of 8-symbol chunks of the preamble against the reference:
//!    carrier frequency offset (phase slope between chunks), carrier phase, and amplitude.
//! 3. **Demodulate.** Derotate, matched filter (RRC) at the symbol instants for BPSK and QPSK, or
//!    two tone correlators for 2-FSK. BPSK and QPSK payloads are tracked by a small decision-directed
//!    phase-locked loop.
//! 4. **Check.** The sync word (up to `sync_max_errors` wrong bits), the length byte against its
//!    complement, and the CRC-16.
//!
//! Handled: unknown start sample (integer), unknown carrier phase, a small carrier frequency offset
//! (see `docs/waveform-w0.md` for the range), unknown gain. Not handled: fractional timing, sample clock
//! offset, multipath, a stream in chunks (a call takes one buffer).

use crate::frame::{
    body_bits, body_len_bits, parse_body, parse_header, preamble_bits, sync_bits,
    DEFAULT_SYNC_WORD, MAX_PAYLOAD, PREAMBLE_LEN,
};
use crate::iq::Complex32;
use crate::pulse::{rrc, rrc_shifted};
use std::f64::consts::PI;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Modulation {
    /// Binary FSK, continuous phase, tones at +/- half the symbol rate (orthogonal over a symbol).
    /// Detected noncoherently.
    Fsk2,
    Bpsk,
    /// Gray coded: the first bit of a pair sets the I sign, the second the Q sign (0 is positive).
    Qpsk,
}

impl Modulation {
    pub fn bits_per_symbol(self) -> usize {
        match self {
            Modulation::Qpsk => 2,
            _ => 1,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Modulation::Fsk2 => "fsk2",
            Modulation::Bpsk => "bpsk",
            Modulation::Qpsk => "qpsk",
        }
    }

    pub fn parse(s: &str) -> Option<Self> {
        match s {
            "fsk2" | "fsk" => Some(Modulation::Fsk2),
            "bpsk" => Some(Modulation::Bpsk),
            "qpsk" => Some(Modulation::Qpsk),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct ModemConfig {
    pub modulation: Modulation,
    /// Samples per symbol, 4 to 64.
    pub sps: usize,
    /// Root-raised-cosine roll-off (PSK only), 0.1 to 1.
    pub rolloff: f32,
    /// RRC length in symbols (PSK only), even, 4 to 32.
    pub rrc_span: usize,
    pub sync_word: u16,
}

impl Default for ModemConfig {
    fn default() -> Self {
        Self {
            modulation: Modulation::Bpsk,
            sps: 8,
            rolloff: 0.35,
            rrc_span: 16,
            sync_word: DEFAULT_SYNC_WORD,
        }
    }
}

impl ModemConfig {
    pub fn new(modulation: Modulation) -> Self {
        Self {
            modulation,
            ..Self::default()
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ConfigError(pub String);

impl std::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl std::error::Error for ConfigError {}

/// Receiver settings.
#[derive(Debug, Clone, PartialEq)]
pub struct RxConfig {
    /// Detection threshold on the normalized correlation, in `(0, 1)`. In white noise the false alarm
    /// probability per sample position is `(1 - t)^(M - 1)`, with `M = 79 * sps`.
    pub detect_threshold: f32,
    /// Largest number of wrong sync word bits (of 16) a frame may have and still be accepted.
    pub sync_max_errors: u32,
    /// If set, ignore the length field and decode this many payload bytes (used for BER
    /// measurement, so a damaged length does not hide a payload).
    pub expected_len: Option<usize>,
    /// Refine carrier phase and frequency over the whole frame body after the preamble (BPSK and QPSK).
    /// Off means the preamble's estimate alone is used.
    pub refine_phase: bool,
    /// Extra trial frequencies for detection: `cfo_bins` on each side of zero, `cfo_step` cycles per
    /// symbol apart. 0 searches only zero offset. Not used when `cfo_search` is on.
    pub cfo_bins: usize,
    pub cfo_step: f32,
    /// Coarse frequency search: the largest carrier offset to look for, in cycles per symbol (2.0 is
    /// two symbol rates either way). 0 turns it off. It is capped at half the sample rate
    /// (`sps / 2` cycles per symbol). See [`Modem::coarse_metric`] for the method and its cost.
    pub cfo_search: f32,
}

impl Default for RxConfig {
    fn default() -> Self {
        Self {
            detect_threshold: 0.04,
            sync_max_errors: 3,
            expected_len: None,
            refine_phase: true,
            cfo_bins: 0,
            cfo_step: 0.005,
            cfo_search: 0.0,
        }
    }
}

/// The coarse frequency search for one receiver setting: see [`Modem::coarse_metric`].
#[derive(Debug, Clone)]
pub struct CoarseSearch {
    /// Samples summed into one point before the FFT (1 for wide searches).
    pub chunk: usize,
    /// FFT size.
    pub fft_len: usize,
    /// Bins examined on each side of zero.
    pub bins: usize,
    /// Distance between the time offsets searched, in samples.
    pub step: usize,
    /// Trial frequencies (bins examined). In white noise the chance that a position passes a
    /// threshold `t` is at most about `cells * (1 - t)^(M - 1)`: this sets the threshold.
    pub cells: f64,
    /// Detection threshold on the coarse metric: `detect_threshold` raised so that the false alarm
    /// probability per position stays what it is without the search.
    pub threshold: f32,
    fft: crate::fft::Fft,
}

/// The largest offset searched, as a fraction of the sample rate.
const COARSE_MAX_CYCLES_PER_SAMPLE: f64 = 0.5;
/// Chunk length rule: the searched offset times the chunk length stays under this, so a chunk loses
/// at most `sinc^2(0.2)` = 0.6 dB of the signal at the edge of the range.
const COARSE_CHUNK_TURNS: f64 = 0.2;

/// One frame found by [`Modem::demodulate`].
#[derive(Debug, Clone, PartialEq)]
pub struct RxFrame {
    /// Sample index where the frame's first symbol pulse starts.
    pub start: usize,
    /// Sample index just past the frame's last sample.
    pub end: usize,
    pub payload: Vec<u8>,
    /// The CRC-16 matched. Treat a frame with `crc_ok == false` as damaged.
    pub crc_ok: bool,
    /// Wrong bits in the 16-bit sync word.
    pub sync_errors: u32,
    /// Detection metric at the peak, 0 to 1.
    pub metric: f32,
    /// Sub-sample timing correction, in samples (-0.5 to 0.5), added to `start`. 0 for 2-FSK, which
    /// is timed to whole samples.
    pub timing_frac: f32,
    /// Estimated carrier frequency offset, cycles per symbol.
    pub cfo_cycles_per_symbol: f32,
    /// Estimated carrier phase at the frame start, radians.
    pub phase_rad: f32,
}

#[derive(Debug, Clone)]
pub struct Modem {
    cfg: ModemConfig,
    h: Vec<f32>,
    ref_bits: Vec<u8>,
    refw: Vec<Complex32>,
    ref_energy: f64,
    /// (cos, sin) of pi*n/sps for one symbol: the 2-FSK tone correlators.
    tone: Vec<(f32, f32)>,
}

const CHUNK_SYMBOLS: usize = 8;
/// The preamble's frequency estimate is used only if it raises the correlation power by this many
/// noise units over zero offset.
const FREQ_GAIN_MIN: f64 = 15.0;

impl Modem {
    pub fn new(cfg: ModemConfig) -> Result<Self, ConfigError> {
        if !(4..=64).contains(&cfg.sps) {
            return Err(ConfigError("sps must be 4 to 64".into()));
        }
        if !(0.1..=1.0).contains(&cfg.rolloff) {
            return Err(ConfigError("rolloff must be 0.1 to 1".into()));
        }
        if cfg.rrc_span % 2 != 0 || !(4..=32).contains(&cfg.rrc_span) {
            return Err(ConfigError("rrc_span must be even, 4 to 32".into()));
        }
        let h = rrc(cfg.sps, cfg.rrc_span, cfg.rolloff as f64);
        let mut ref_bits = preamble_bits().to_vec();
        ref_bits.extend(sync_bits(cfg.sync_word));
        let mut m = Modem {
            cfg,
            h,
            ref_bits,
            refw: Vec::new(),
            ref_energy: 0.0,
            tone: Vec::new(),
        };
        m.tone = (0..m.cfg.sps)
            .map(|n| {
                let (sn, cs) = (PI * n as f64 / m.cfg.sps as f64).sin_cos();
                (cs as f32, sn as f32)
            })
            .collect();
        let full = m.waveform(&m.ref_bits, &[]);
        // Only the part before the first body symbol's pulse starts is purely preamble and sync.
        let ref_len = m.ref_bits.len() * m.cfg.sps;
        m.refw = full[..ref_len].to_vec();
        m.ref_energy = m.refw.iter().map(|s| s.norm_sqr() as f64).sum();
        Ok(m)
    }

    pub fn config(&self) -> &ModemConfig {
        &self.cfg
    }

    /// Energy of one symbol: the sum of `|x|^2` over its samples. 1 by construction.
    pub fn energy_per_symbol(&self) -> f64 {
        1.0
    }

    /// Energy of one information bit, `Es / bits_per_symbol`.
    pub fn energy_per_bit(&self) -> f64 {
        self.energy_per_symbol() / self.cfg.modulation.bits_per_symbol() as f64
    }

    /// Number of samples [`Modem::modulate`] makes for a payload of `n` bytes.
    pub fn frame_samples(&self, n: usize) -> usize {
        let nsym = self.ref_bits.len() + body_len_bits(n) / self.cfg.modulation.bits_per_symbol();
        self.waveform_len(nsym)
    }

    fn waveform_len(&self, nsym: usize) -> usize {
        match self.cfg.modulation {
            Modulation::Fsk2 => nsym * self.cfg.sps,
            _ => (nsym - 1) * self.cfg.sps + self.h.len(),
        }
    }

    /// A complete frame as samples: preamble, sync word, length, payload, CRC. The first sample is
    /// the start of the first pulse; nothing is added before or after. Fails if the payload is
    /// longer than 255 bytes.
    pub fn modulate(&self, payload: &[u8]) -> Result<Vec<Complex32>, ConfigError> {
        if payload.len() > MAX_PAYLOAD {
            return Err(ConfigError(format!(
                "payload of {} bytes is over the {MAX_PAYLOAD} byte limit",
                payload.len()
            )));
        }
        Ok(self.waveform(&self.ref_bits, &body_bits(payload)))
    }

    /// Raw symbols without any frame: `bits` (0 or 1) at the payload modulation. For tests and
    /// measurement. A QPSK call needs an even number of bits.
    pub fn modulate_bits(&self, bits: &[u8]) -> Vec<Complex32> {
        self.waveform(&[], bits)
    }

    /// `head_bits` go out one bit per symbol (BPSK or FSK), `body` at the payload modulation.
    fn waveform(&self, head_bits: &[u8], body: &[u8]) -> Vec<Complex32> {
        let sps = self.cfg.sps;
        match self.cfg.modulation {
            Modulation::Fsk2 => {
                let all: Vec<u8> = head_bits.iter().chain(body).copied().collect();
                let a = 1.0 / (sps as f64).sqrt();
                let mut out = Vec::with_capacity(all.len() * sps);
                let mut phi = 0.0f64;
                for &b in &all {
                    let dir = if b & 1 == 1 { 1.0 } else { -1.0 };
                    for n in 0..sps {
                        let ph = phi + dir * PI * n as f64 / sps as f64;
                        out.push(Complex32::new((a * ph.cos()) as f32, (a * ph.sin()) as f32));
                    }
                    phi += dir * PI;
                    phi = phi.rem_euclid(2.0 * PI);
                }
                out
            }
            m => {
                let mut syms: Vec<Complex32> = head_bits
                    .iter()
                    .map(|&b| Complex32::new(1.0 - 2.0 * (b & 1) as f32, 0.0))
                    .collect();
                if m == Modulation::Bpsk {
                    syms.extend(
                        body.iter()
                            .map(|&b| Complex32::new(1.0 - 2.0 * (b & 1) as f32, 0.0)),
                    );
                } else {
                    let k = std::f32::consts::FRAC_1_SQRT_2;
                    for p in body.chunks(2) {
                        let q = p.get(1).copied().unwrap_or(0);
                        syms.push(Complex32::new(
                            (1.0 - 2.0 * (p[0] & 1) as f32) * k,
                            (1.0 - 2.0 * (q & 1) as f32) * k,
                        ));
                    }
                }
                if syms.is_empty() {
                    return Vec::new();
                }
                let mut out = vec![Complex32::ZERO; self.waveform_len(syms.len())];
                for (k, s) in syms.iter().enumerate() {
                    for (i, &t) in self.h.iter().enumerate() {
                        out[k * sps + i] += *s * t;
                    }
                }
                out
            }
        }
    }

    /// Find frames in `samples` with the default receiver settings.
    pub fn demodulate(&self, samples: &[Complex32]) -> Vec<RxFrame> {
        self.demodulate_with(samples, &RxConfig::default())
    }

    /// Find frames in `samples`. Frames come in order of position; a frame with a bad CRC is
    /// returned with `crc_ok == false`. Overlapping frames are not separated.
    pub fn demodulate_with(&self, samples: &[Complex32], rx: &RxConfig) -> Vec<RxFrame> {
        if rx.cfo_search > 0.0 {
            return self.demodulate_coarse(samples, rx);
        }
        let metric = self.metric(samples, rx);
        let sps = self.cfg.sps;
        let mut out: Vec<RxFrame> = Vec::new();
        let mut busy_until = 0usize;
        for tau in 0..metric.len() {
            let m = metric[tau];
            if m < rx.detect_threshold || tau < busy_until {
                continue;
            }
            let lo = tau.saturating_sub(sps);
            let hi = (tau + sps + 1).min(metric.len());
            let is_peak = (lo..hi).all(|j| metric[j] < m || (metric[j] == m && j >= tau));
            if !is_peak {
                continue;
            }
            let mu = parabola_peak(&metric, tau);
            if let Some(f) = self.decode_at(samples, tau, mu, m, rx) {
                if f.crc_ok {
                    busy_until = f.end;
                }
                out.push(f);
            }
        }
        out
    }

    /// The detection metric at every sample offset `tau` where the reference fits:
    /// `max over trial frequencies of |sum r[tau+n] conj(p[n])|^2 / (E_p * sum |r[tau+n]|^2)`.
    pub fn metric(&self, r: &[Complex32], rx: &RxConfig) -> Vec<f32> {
        let m = self.refw.len();
        if r.len() < m {
            return Vec::new();
        }
        let positions = r.len() - m + 1;
        // sliding window energy
        let mut prefix = Vec::with_capacity(r.len() + 1);
        let mut acc = 0.0f64;
        prefix.push(0.0);
        for s in r {
            acc += s.norm_sqr() as f64;
            prefix.push(acc);
        }
        let mut best = vec![0.0f32; positions];
        let bins = rx.cfo_bins as isize;
        let (mut pre, mut pim) = (vec![0.0f32; m], vec![0.0f32; m]);
        for b in -bins..=bins {
            let theta = 2.0 * PI * (b as f64) * rx.cfo_step as f64 / self.cfg.sps as f64;
            for (n, p) in self.refw.iter().enumerate() {
                let rot = Complex32::from_polar(1.0, (theta * n as f64) as f32);
                let v = *p * rot;
                pre[n] = v.re;
                pim[n] = v.im;
            }
            for (tau, slot) in best.iter_mut().enumerate() {
                let win = &r[tau..tau + m];
                let (mut cr, mut ci) = (0.0f32, 0.0f32);
                for ((w, &a), &b) in win.iter().zip(&pre).zip(&pim) {
                    // r * conj(p)
                    cr += w.re * a + w.im * b;
                    ci += w.im * a - w.re * b;
                }
                let e = prefix[tau + m] - prefix[tau];
                if e > 1e-30 {
                    let v = ((cr as f64 * cr as f64 + ci as f64 * ci as f64)
                        / (self.ref_energy * e)) as f32;
                    if v > *slot {
                        *slot = v;
                    }
                }
            }
        }
        best
    }

    /// The coarse frequency search for `rx`, or `None` if `rx.cfo_search` is 0.
    pub fn coarse_search(&self, rx: &RxConfig) -> Option<CoarseSearch> {
        if rx.cfo_search.is_nan() || rx.cfo_search <= 0.0 {
            return None;
        }
        let m = self.refw.len();
        let range = (rx.cfo_search as f64 / self.cfg.sps as f64).min(COARSE_MAX_CYCLES_PER_SAMPLE);
        let chunk = ((COARSE_CHUNK_TURNS / range).floor() as usize).clamp(1, m / 16);
        let points = m.div_ceil(chunk);
        let fft_len = (points + points / 2).next_power_of_two().max(8);
        let bins = (((range * (fft_len * chunk) as f64).ceil() as usize) + 1).min(fft_len / 2 - 1);
        // Each bin examined is one trial frequency. The bins are closer than independent (the FFT is
        // zero padded), but the largest of a continuous spectrum beats the largest of independent
        // cells, and counting every bin matches what is measured (a test checks it) or is above it.
        let cells = (2 * bins + 1) as f64;
        let t = rx.detect_threshold as f64;
        let threshold = (1.0 - (1.0 - t) * cells.powf(-1.0 / (m as f64 - 1.0))) as f32;
        Some(CoarseSearch {
            chunk,
            fft_len,
            bins,
            step: (self.cfg.sps / 4).max(1),
            cells,
            threshold,
            fft: crate::fft::Fft::new(fft_len),
        })
    }

    /// The coarse search's metric: one value every `step` samples (position `i` is sample
    /// `i * step`), each `(metric, frequency in cycles per symbol)`. Empty if the search is off.
    ///
    /// Method: at a trial position, multiply the received window by the conjugate of the known
    /// preamble and sync waveform. This removes the chips: what is left is a tone at the carrier
    /// offset. Sum it in chunks of `chunk` samples (1 for wide searches), take an FFT of
    /// `fft_len` points (zero padded to about 1.5 times the window), and keep the strongest bin
    /// within the range. Each bin is the normal correlation metric for a reference shifted to that
    /// bin's frequency, so in white noise it is `Beta(1, M-1)` like [`Modem::metric`]. Taking the
    /// largest of about `cells` independent ones raises the false alarm rate by that factor; the
    /// threshold is raised to cancel it.
    pub fn coarse_metric(&self, r: &[Complex32], rx: &RxConfig) -> Vec<(f32, f32)> {
        match self.coarse_search(rx) {
            Some(cs) => self.coarse_metric_with(r, &cs),
            None => Vec::new(),
        }
    }

    fn coarse_metric_with(&self, r: &[Complex32], cs: &CoarseSearch) -> Vec<(f32, f32)> {
        let m = self.refw.len();
        if r.len() < m {
            return Vec::new();
        }
        let positions = r.len() - m + 1;
        let mut prefix = Vec::with_capacity(r.len() + 1);
        let mut acc = 0.0f64;
        prefix.push(0.0);
        for s in r {
            acc += s.norm_sqr() as f64;
            prefix.push(acc);
        }
        let n = cs.fft_len;
        let span = (n * cs.chunk) as f64;
        let mut buf = vec![Complex32::ZERO; n];
        let mut out = Vec::with_capacity(positions.div_ceil(cs.step));
        for tau in (0..positions).step_by(cs.step) {
            buf.fill(Complex32::ZERO);
            let win = &r[tau..tau + m];
            for (slot, (w, p)) in buf
                .iter_mut()
                .zip(win.chunks(cs.chunk).zip(self.refw.chunks(cs.chunk)))
            {
                let (mut cr, mut ci) = (0.0f32, 0.0f32);
                for (a, b) in w.iter().zip(p) {
                    // a * conj(b)
                    cr += a.re * b.re + a.im * b.im;
                    ci += a.im * b.re - a.re * b.im;
                }
                *slot = Complex32::new(cr, ci);
            }
            cs.fft.forward(&mut buf);
            let (mut best, mut best_k) = (-1.0f32, 0isize);
            for k in -(cs.bins as isize)..=cs.bins as isize {
                let v = buf[k.rem_euclid(n as isize) as usize].norm_sqr();
                if v > best {
                    best = v;
                    best_k = k;
                }
            }
            let mag = |k: isize| buf[k.rem_euclid(n as isize) as usize].norm_sqr().sqrt() as f64;
            let (a, b, c) = (mag(best_k - 1), mag(best_k), mag(best_k + 1));
            let den = a - 2.0 * b + c;
            let delta = if den < 0.0 {
                (0.5 * (a - c) / den).clamp(-0.5, 0.5)
            } else {
                0.0
            };
            let e = prefix[tau + m] - prefix[tau];
            let metric = if e > 1e-30 {
                (best as f64 / (self.ref_energy * e)) as f32
            } else {
                0.0
            };
            let f = (best_k as f64 + delta) / span * self.cfg.sps as f64;
            out.push((metric, f as f32));
        }
        out
    }

    /// [`Modem::demodulate_with`] with the coarse frequency search: find candidates on the coarse
    /// grid, then for each one turn the buffer back by its frequency and run the normal detector
    /// (one frequency) and decoder in a small window around it.
    fn demodulate_coarse(&self, r: &[Complex32], rx: &RxConfig) -> Vec<RxFrame> {
        let Some(cs) = self.coarse_search(rx) else {
            return Vec::new();
        };
        let cm = self.coarse_metric_with(r, &cs);
        if cm.is_empty() {
            return Vec::new();
        }
        let sps = self.cfg.sps;
        let m = self.refw.len();
        let last = r.len() - m;
        // A candidate must be the strongest within one reference length. Taking the best of many
        // frequencies lifts the time side peaks too (for 2-FSK the preamble one symbol off still
        // matches well), and a one-symbol window lets them through as false starts. Real frames
        // are always further apart than this; overlapping frames are not separated anyway.
        let reach = m.div_ceil(cs.step);
        let frame_len = self.frame_samples(rx.expected_len.unwrap_or(MAX_PAYLOAD).min(MAX_PAYLOAD));
        let fine = RxConfig {
            cfo_bins: 0,
            cfo_search: 0.0,
            ..rx.clone()
        };
        let mut out: Vec<RxFrame> = Vec::new();
        let mut busy_until = 0usize;
        for i in 0..cm.len() {
            let (v, f) = cm[i];
            let tau_c = i * cs.step;
            if v < cs.threshold || tau_c < busy_until {
                continue;
            }
            let lo_i = i.saturating_sub(reach);
            let hi_i = (i + reach + 1).min(cm.len());
            let is_peak = (lo_i..hi_i).all(|j| cm[j].0 < v || (cm[j].0 == v && j >= i));
            if !is_peak {
                continue;
            }
            let lo = tau_c.saturating_sub(sps + cs.step).max(busy_until);
            let hi = (tau_c + sps + cs.step).min(last);
            let end = (hi + frame_len + 1).min(r.len());
            let w = f as f64 / sps as f64;
            let d: Vec<Complex32> = r[lo..end]
                .iter()
                .enumerate()
                .map(|(j, s)| {
                    let turns = (w * (lo + j) as f64).rem_euclid(1.0);
                    let (sn, co) = (-2.0 * PI * turns).sin_cos();
                    *s * Complex32::new(co as f32, sn as f32)
                })
                .collect();
            let fm = self.metric(&d[..hi - lo + m], &fine);
            let Some(j) = (0..fm.len()).max_by(|&a, &b| fm[a].total_cmp(&fm[b]).then(b.cmp(&a)))
            else {
                continue;
            };
            let mu = parabola_peak(&fm, j);
            if let Some(mut fr) = self.decode_at(&d, j, mu, fm[j], &fine) {
                fr.start += lo;
                fr.end += lo;
                fr.cfo_cycles_per_symbol += f;
                let turns = (w * fr.start as f64).rem_euclid(1.0);
                let ph = fr.phase_rad as f64 + 2.0 * PI * turns;
                fr.phase_rad = ph.sin().atan2(ph.cos()) as f32;
                if fr.crc_ok {
                    busy_until = fr.end;
                }
                out.push(fr);
            }
        }
        out
    }

    fn decode_at(
        &self,
        r: &[Complex32],
        tau: usize,
        mu: f64,
        metric: f32,
        rx: &RxConfig,
    ) -> Option<RxFrame> {
        let sps = self.cfg.sps;
        let chunk = CHUNK_SYMBOLS * sps;
        let k_chunks = self.refw.len() / chunk;
        // 1. chunk correlations against the reference
        let mut c = Vec::with_capacity(k_chunks);
        for k in 0..k_chunks {
            let (mut cr, mut ci) = (0.0f64, 0.0f64);
            for n in k * chunk..(k + 1) * chunk {
                let a = r[tau + n];
                let p = self.refw[n];
                cr += (a.re * p.re + a.im * p.im) as f64;
                ci += (a.im * p.re - a.re * p.im) as f64;
            }
            c.push((cr, ci));
        }
        // frequency: phase slope between neighbouring chunks (radians per sample)
        let (mut sr, mut si) = (0.0f64, 0.0f64);
        for k in 0..k_chunks - 1 {
            let (a, b) = (c[k], c[k + 1]);
            sr += b.0 * a.0 + b.1 * a.1;
            si += b.1 * a.0 - b.0 * a.1;
        }
        let mut omega = si.atan2(sr) / chunk as f64;
        // phase at the frame start and amplitude
        let (mut pr, mut pi) = (0.0f64, 0.0f64);
        for (k, &(cr, ci)) in c.iter().enumerate() {
            let t = (k * chunk) as f64 + chunk as f64 / 2.0;
            let (s, co) = (-omega * t).sin_cos();
            pr += cr * co - ci * s;
            pi += cr * s + ci * co;
        }
        let e_chunks: f64 = self.refw[..k_chunks * chunk]
            .iter()
            .map(|s| s.norm_sqr() as f64)
            .sum();
        let amp = ((pr * pr + pi * pi).sqrt() / e_chunks).max(1e-9);
        // Use the frequency estimate only if it explains the preamble clearly better than zero
        // offset does (a generalized likelihood ratio test). Under "no offset" the gain in
        // correlation power from the best of many trial frequencies is about exponential with mean
        // 1 in units of the noise power in the sum; `FREQ_GAIN_MIN` is far out in that tail. Without
        // this, noise-made estimates put a phase ramp on the whole frame.
        let window: f64 = r[tau..tau + k_chunks * chunk]
            .iter()
            .map(|s| s.norm_sqr() as f64)
            .sum();
        let noise_per_sample = (window - (pr * pr + pi * pi) / e_chunks).max(1e-9 * window)
            / (k_chunks * chunk) as f64;
        let (s0r, s0i) = c
            .iter()
            .fold((0.0f64, 0.0f64), |a, &(x, y)| (a.0 + x, a.1 + y));
        let gain = ((pr * pr + pi * pi) - (s0r * s0r + s0i * s0i)) / (noise_per_sample * e_chunks);
        if gain < FREQ_GAIN_MIN {
            omega = 0.0;
        }
        let psi = {
            let (mut qr, mut qi) = (0.0f64, 0.0f64);
            for (k, &(cr, ci)) in c.iter().enumerate() {
                let t = (k * chunk) as f64 + chunk as f64 / 2.0;
                let (s, co) = (-omega * t).sin_cos();
                qr += cr * co - ci * s;
                qi += cr * s + ci * co;
            }
            qi.atan2(qr)
        };
        // 2. how much of the buffer we need
        let bps = self.cfg.modulation.bits_per_symbol();
        let n_ref = self.ref_bits.len();
        let max_bits = match rx.expected_len {
            Some(n) => body_len_bits(n),
            None => body_len_bits(MAX_PAYLOAD),
        };
        let want_syms = n_ref + max_bits / bps;
        let avail = r.len() - tau;
        let end_needed = |nsym: usize| self.waveform_len(nsym);
        let usable_syms = (n_ref..=want_syms)
            .rev()
            .find(|&s| end_needed(s) <= avail)
            .unwrap_or(0);
        if usable_syms < n_ref + 16 / bps {
            return None;
        }
        let take = end_needed(usable_syms);
        // derotate and normalize the part we use
        let rr: Vec<Complex32> = (0..take)
            .map(|n| {
                let (s, co) = (-(psi + omega * n as f64)).sin_cos();
                r[tau + n] * Complex32::new((co / amp) as f32, (s / amp) as f32)
            })
            .collect();

        let taps = if self.cfg.modulation == Modulation::Fsk2 {
            Vec::new()
        } else {
            rrc_shifted(sps, self.cfg.rrc_span, self.cfg.rolloff as f64, mu)
        };
        let dem = SymbolDemod::new(self, &taps, &rr);
        // sync word: symbols n_ref-16 .. n_ref, one bit each
        let mut sync_errors = 0;
        let want_sync = &self.ref_bits[PREAMBLE_LEN..];
        for (i, &want) in want_sync.iter().enumerate() {
            if dem.one_bit_symbol(PREAMBLE_LEN + i) != want {
                sync_errors += 1;
            }
        }
        if sync_errors > rx.sync_max_errors {
            return None;
        }

        // 3. body. The header (16 bits) is read first, to learn the length.
        let hdr_syms = 16 / bps;
        let mut z: Vec<Complex32> = Vec::new();
        let mut bits: Vec<u8> = Vec::new();
        let n = match rx.expected_len {
            Some(n) => n,
            None => {
                dem.body_symbols(n_ref, hdr_syms, &mut z, &mut bits);
                parse_header(&bits)?
            }
        };
        let total_syms = n_ref + body_len_bits(n) / bps;
        if end_needed(total_syms) > avail || (rx.expected_len.is_none() && total_syms > usable_syms)
        {
            return None;
        }
        z.clear();
        bits.clear();
        dem.body_symbols(n_ref, total_syms - n_ref, &mut z, &mut bits);
        let mut mu_final = mu;
        if self.cfg.modulation != Modulation::Fsk2 && rx.refine_phase {
            let rho = amp * amp / noise_per_sample;
            let (mut a, mut b) = self.track(&z, rho);
            bits = decide_psk(self.cfg.modulation, &z, a, b);
            // Timing: the preamble fixes it to a fraction of a sample at best. The frame's own
            // symbols, compared with the decisions just made, do better: pick the shift of the
            // matched filter (in steps of half a sample, then a parabola) that gives the largest
            // signal. This is where a noisy parabola on the preamble metric loses 1 to 4 %.
            let d = unit_symbols(self.cfg.modulation, &bits);
            let grid = [-1.0f64, -0.5, 0.0, 0.5, 1.0];
            let q: Vec<f64> = grid
                .iter()
                .map(|&shift| {
                    let t =
                        rrc_shifted(sps, self.cfg.rrc_span, self.cfg.rolloff as f64, mu + shift);
                    let dm = SymbolDemod::new(self, &t, &rr);
                    d.iter()
                        .enumerate()
                        .map(|(k, dk)| {
                            let (s, c) = (-(a + b * k as f64)).sin_cos();
                            let y = dm.z(n_ref + k) * Complex32::new(c as f32, s as f32);
                            (y.re * dk.re + y.im * dk.im) as f64
                        })
                        .sum()
                })
                .collect();
            let best = (0..grid.len())
                .max_by(|&i, &j| q[i].total_cmp(&q[j]))
                .unwrap_or(2);
            let mut shift = grid[best];
            if best > 0 && best + 1 < grid.len() {
                let (y0, y1, y2) = (q[best - 1], q[best], q[best + 1]);
                let den = y0 - 2.0 * y1 + y2;
                if den < 0.0 {
                    shift += 0.5 * (y0 - y2) / den * 0.5; // the grid step is 0.5 samples
                }
            }
            if shift.abs() > 0.05 {
                mu_final = mu + shift;
                let t = rrc_shifted(sps, self.cfg.rrc_span, self.cfg.rolloff as f64, mu_final);
                let dm = SymbolDemod::new(self, &t, &rr);
                z.clear();
                bits.clear();
                dm.body_symbols(n_ref, total_syms - n_ref, &mut z, &mut bits);
                (a, b) = self.track(&z, rho);
                bits = decide_psk(self.cfg.modulation, &z, a, b);
            }
        }
        let (payload, crc_ok) = parse_body(&bits, n);
        Some(RxFrame {
            start: tau,
            end: tau + end_needed(total_syms),
            payload,
            crc_ok,
            sync_errors,
            metric,
            timing_frac: if self.cfg.modulation == Modulation::Fsk2 {
                0.0
            } else {
                mu_final as f32
            },
            cfo_cycles_per_symbol: (omega * sps as f64 / (2.0 * PI)) as f32,
            phase_rad: psi as f32,
        })
    }

    /// Carrier phase and frequency of the frame body: the power-law fit, then decision-directed polish.
    /// Returns `(a, b)`: turn symbol `k` back by `a + b * k` radians.
    fn track(&self, z: &[Complex32], rho: f64) -> (f64, f64) {
        let m = self.cfg.modulation;
        let (mut a, b) = refine_phase(m, z, rho);
        // Decision-directed polish: with the data mostly right it uses the whole frame without
        // the power law's noise enhancement. Two passes.
        for _ in 0..2 {
            a += dd_phase_error(m, z, a, b);
        }
        (a, b)
    }

    /// Payload bits of a frame body as the receiver sees them, for tests: modulate `bits`,
    /// then demodulate them at known timing and phase (no preamble, no estimation).
    pub fn demod_bits_known_timing(&self, samples: &[Complex32], nbits: usize) -> Vec<u8> {
        let nsym = nbits.div_ceil(self.cfg.modulation.bits_per_symbol());
        let dem = SymbolDemod::new(self, &self.h, samples);
        let (mut z, mut bits) = (Vec::new(), Vec::new());
        dem.body_symbols(0, nsym, &mut z, &mut bits);
        bits.truncate(nbits);
        bits
    }

    /// The symbols-per-sample ratio the modem was built with.
    pub fn sps(&self) -> usize {
        self.cfg.sps
    }
}

/// Symbol-by-symbol demodulation of a derotated, amplitude-normalized buffer that starts at the frame start.
struct SymbolDemod<'a> {
    m: &'a Modem,
    h: &'a [f32],
    rr: &'a [Complex32],
}

impl<'a> SymbolDemod<'a> {
    fn new(m: &'a Modem, h: &'a [f32], rr: &'a [Complex32]) -> Self {
        Self { m, h, rr }
    }

    /// Matched filter output for symbol `k` (PSK): amplitude 1 for a clean symbol, noise variance N0.
    fn z(&self, k: usize) -> Complex32 {
        let base = k * self.m.cfg.sps;
        let (mut re, mut im) = (0.0f32, 0.0f32);
        for (&t, s) in self.h.iter().zip(&self.rr[base..]) {
            re += t * s.re;
            im += t * s.im;
        }
        Complex32::new(re, im)
    }

    /// Energies of the "+" tone (bit 1) and the "-" tone (bit 0) in symbol `k` (2-FSK).
    fn tones(&self, k: usize) -> (f32, f32) {
        let sps = self.m.cfg.sps;
        let (mut pr, mut pi, mut mr, mut mi) = (0.0f32, 0.0f32, 0.0f32, 0.0f32);
        for (s, &(cs, sn)) in self.rr[k * sps..(k + 1) * sps].iter().zip(&self.m.tone) {
            // multiply by e^{-j pi n/sps} (plus tone) and e^{+j pi n/sps} (minus tone)
            pr += s.re * cs + s.im * sn;
            pi += s.im * cs - s.re * sn;
            mr += s.re * cs - s.im * sn;
            mi += s.im * cs + s.re * sn;
        }
        let k = 1.0 / sps as f32;
        ((pr * pr + pi * pi) * k, (mr * mr + mi * mi) * k)
    }

    /// The bit of a one-bit-per-symbol symbol (preamble and sync, or BPSK and FSK payload).
    fn one_bit_symbol(&self, k: usize) -> u8 {
        if self.m.cfg.modulation == Modulation::Fsk2 {
            let (p, m) = self.tones(k);
            (p > m) as u8
        } else {
            (self.z(k).re < 0.0) as u8
        }
    }

    /// Demodulate `count` body symbols starting at symbol `first`. PSK: appends the matched filter
    /// outputs to `z` and the hard decisions (no phase correction beyond the preamble's) to `bits`.
    fn body_symbols(&self, first: usize, count: usize, z: &mut Vec<Complex32>, bits: &mut Vec<u8>) {
        for k in first..first + count {
            match self.m.cfg.modulation {
                Modulation::Fsk2 => bits.push(self.one_bit_symbol(k)),
                m => {
                    let v = self.z(k);
                    z.push(v);
                    bits.extend(slice_symbol(m, v));
                }
            }
        }
    }
}

/// Hard decision on one PSK symbol: BPSK one bit, QPSK two (I sign then Q sign; 0 is positive).
fn slice_symbol(m: Modulation, v: Complex32) -> impl Iterator<Item = u8> {
    let second = (m == Modulation::Qpsk).then_some((v.im < 0.0) as u8);
    std::iter::once((v.re < 0.0) as u8).chain(second)
}

/// Decisions on PSK matched-filter outputs after turning each symbol back by `a + b * k` radians.
fn decide_psk(m: Modulation, z: &[Complex32], a: f64, b: f64) -> Vec<u8> {
    let mut bits = Vec::with_capacity(z.len() * 2);
    for (k, &v) in z.iter().enumerate() {
        let (s, c) = (-(a + b * k as f64)).sin_cos();
        let y = v * Complex32::new(c as f32, s as f32);
        bits.extend(slice_symbol(m, y));
    }
    bits
}

/// Hard decisions as unit-energy symbols (`+-1`, or `(+-1 +-j)/sqrt 2`), to compare matched filter outputs with.
fn unit_symbols(m: Modulation, bits: &[u8]) -> Vec<Complex32> {
    let sign = |b: u8| 1.0 - 2.0 * (b & 1) as f32;
    if m == Modulation::Bpsk {
        bits.iter().map(|&b| Complex32::new(sign(b), 0.0)).collect()
    } else {
        let k = std::f32::consts::FRAC_1_SQRT_2;
        bits.chunks_exact(2)
            .map(|p| Complex32::new(sign(p[0]) * k, sign(p[1]) * k))
            .collect()
    }
}

/// The phase left over after turning the symbols back by `a + b * k`, from the symbols and their
/// own hard decisions: the angle of `sum z conj(decision)`.
fn dd_phase_error(m: Modulation, z: &[Complex32], a: f64, b: f64) -> f64 {
    let (mut sr, mut si) = (0.0f64, 0.0f64);
    for (k, &v) in z.iter().enumerate() {
        let (s, c) = (-(a + b * k as f64)).sin_cos();
        let y = v * Complex32::new(c as f32, s as f32);
        let d = if m == Modulation::Bpsk {
            Complex32::new(if y.re < 0.0 { -1.0 } else { 1.0 }, 0.0)
        } else {
            Complex32::new(
                if y.re < 0.0 { -1.0 } else { 1.0 },
                if y.im < 0.0 { -1.0 } else { 1.0 },
            )
        };
        // y * conj(d)
        sr += (y.re * d.re + y.im * d.im) as f64;
        si += (y.im * d.re - y.re * d.im) as f64;
    }
    si.atan2(sr)
}

/// Residual carrier phase and frequency of the whole frame body, from the symbols themselves.
///
/// Viterbi and Viterbi: raising a symbol to the power `M` (2 for BPSK, 4 for QPSK, with QPSK's
/// constant -1 removed) strips the data and leaves `M` times the phase error. The body is cut into
/// segments (32 symbols or more, see the code); each gives a phase; a weighted straight-line fit over the segments gives
/// the phase `a` at symbol 0 and the slope `b` (radians per symbol). The slope is used only if it is
/// four standard errors from zero. No decisions are used, so there is no decision bias. The input
/// must already be within a quarter turn (BPSK) or an eighth (QPSK) of right, which the preamble's
/// estimate gives. Returns `(a, b)`.
fn refine_phase(m: Modulation, z: &[Complex32], rho: f64) -> (f64, f64) {
    let order = if m == Modulation::Bpsk { 2 } else { 4 };
    // Longer segments at low SNR, so that a segment's phase noise stays well inside the 1/order
    // turn that the unwrapping can resolve: segment length times Es/N0 at least 64 (BPSK) or 256 (QPSK).
    let target = if order == 2 { 64.0 } else { 256.0 };
    let seg = ((target / rho.max(1e-3)).ceil() as usize).clamp(32, z.len().max(32));
    let nseg = (z.len() / seg).max(1);
    let mut centre = Vec::with_capacity(nseg);
    let mut theta: Vec<f64> = Vec::with_capacity(nseg);
    let mut weight = Vec::with_capacity(nseg);
    for j in 0..nseg {
        let lo = j * seg;
        let hi = if j + 1 == nseg { z.len() } else { lo + seg };
        let (mut sr, mut si) = (0.0f64, 0.0f64);
        for v in &z[lo..hi] {
            let (x, y) = (v.re as f64, v.im as f64);
            let (mut pr, mut pi) = (x, y);
            for _ in 1..order {
                (pr, pi) = (pr * x - pi * y, pr * y + pi * x);
            }
            sr += pr;
            si += pi;
        }
        if order == 4 {
            sr = -sr;
            si = -si;
        }
        let step = std::f64::consts::PI / order as f64;
        let mut t = si.atan2(sr) / order as f64;
        if let Some(&prev) = theta.last() {
            // the segment-to-segment turn is small: take the nearest of the order-fold ambiguities
            let d: f64 = t - prev;
            t = prev + (d + step).rem_euclid(2.0 * step) - step;
        }
        theta.push(t);
        weight.push(sr * sr + si * si);
        centre.push((lo + hi - 1) as f64 / 2.0);
    }
    let wsum: f64 = weight.iter().sum();
    if wsum <= 0.0 {
        return (0.0, 0.0);
    }
    let mean_t: f64 = theta.iter().zip(&weight).map(|(t, w)| t * w).sum::<f64>() / wsum;
    if nseg < 4 {
        return (mean_t, 0.0);
    }
    let mean_c: f64 = centre.iter().zip(&weight).map(|(c, w)| c * w).sum::<f64>() / wsum;
    let sxx: f64 = centre
        .iter()
        .zip(&weight)
        .map(|(c, w)| w * (c - mean_c).powi(2))
        .sum();
    let sxy: f64 = (0..nseg)
        .map(|j| weight[j] * (centre[j] - mean_c) * (theta[j] - mean_t))
        .sum();
    let slope = sxy / sxx;
    let icpt = mean_t - slope * mean_c;
    // Weights are |S|^2, proportional to 1 / (phase variance of a segment), so the weighted
    // residual sum over (segments - 2) estimates the unknown scale of that variance, and the slope
    // has variance scale / sxx.
    let rss: f64 = (0..nseg)
        .map(|j| weight[j] * (theta[j] - icpt - slope * centre[j]).powi(2))
        .sum();
    let se = (rss / (nseg as f64 - 2.0) / sxx).sqrt();
    if slope.abs() > 4.0 * se {
        (icpt, slope)
    } else {
        (mean_t, 0.0)
    }
}

/// Where the peak of three neighbouring metric values lies, as an offset from `i` in samples
/// (a parabola through the three points; clamped to half a sample).
fn parabola_peak(y: &[f32], i: usize) -> f64 {
    if i == 0 || i + 1 >= y.len() {
        return 0.0;
    }
    let (a, b, c) = (y[i - 1] as f64, y[i] as f64, y[i + 1] as f64);
    let den = a - 2.0 * b + c;
    if den >= 0.0 {
        return 0.0;
    }
    (0.5 * (a - c) / den).clamp(-0.5, 0.5)
}
