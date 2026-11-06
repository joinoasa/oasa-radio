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
//! Frame format v1 (`ModemConfig::frame`, see `docs/frame-v1.md`) changes step 4: after the sync word
//! the receiver takes soft values for the 152 header symbols, decodes the protected header, and
//! applies the field rules. The known header then serves as more preamble: carrier phase, frequency
//! and amplitude are estimated again over preamble, sync word and header, since the body starts
//! that much later. The CRC-16 covers the header bytes.
//!
//! With the light code (`payload_code` 1, `docs/fec-light.md`) the body symbols become soft values
//! (log-likelihood ratios, see [`fsk_llr`] and `decode_at`), are de-interleaved and Viterbi decoded.
//!
//! Handled: unknown start sample (integer), unknown carrier phase, a small carrier frequency offset
//! (see `docs/waveform-w0.md` for the range), unknown gain. Not handled: fractional timing, sample clock
//! offset, multipath, a stream in chunks (a call takes one buffer).

use crate::frame::{
    body_bits, body_bits_light, body_bits_v1, body_len_bits, body_len_bits_coded, header_symbols,
    parse_body, parse_body_light, parse_body_v1, parse_header, parse_header_v1, preamble_bits,
    sync_bits, FrameFormat, HeaderFields, PayloadCode, DEFAULT_SYNC_WORD, HEADER_SYMBOLS,
    MAX_PAYLOAD, PREAMBLE_LEN,
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

    /// The 2-bit `modulation` field of the v1 header: 0 BPSK, 1 QPSK, 2 2-FSK.
    pub fn code(self) -> u8 {
        match self {
            Modulation::Bpsk => 0,
            Modulation::Qpsk => 1,
            Modulation::Fsk2 => 2,
        }
    }

    pub fn from_code(code: u8) -> Option<Self> {
        match code {
            0 => Some(Modulation::Bpsk),
            1 => Some(Modulation::Qpsk),
            2 => Some(Modulation::Fsk2),
            _ => None,
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
    /// Frame format sent and expected. W0 by default.
    pub frame: FrameFormat,
    /// Error correction on the body (v1 only). The receiver takes the code from each frame's
    /// header; this setting is what the modem sends, and what it assumes when the length is given
    /// (`RxConfig::expected_len`).
    pub code: PayloadCode,
}

impl Default for ModemConfig {
    fn default() -> Self {
        Self {
            modulation: Modulation::Bpsk,
            sps: 8,
            rolloff: 0.35,
            rrc_span: 16,
            sync_word: DEFAULT_SYNC_WORD,
            frame: FrameFormat::W0,
            code: PayloadCode::None,
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
    /// v1 without `expected_len`: frames whose header announces more payload bytes than this are
    /// rejected at the header, and the receiver needs (and keeps) no more of the buffer than a frame
    /// of this length takes. A radio sets it to its MTU. `None`: 255.
    pub max_len: Option<usize>,
    /// v1 without `expected_len`: accept only frames with this body code; any other is rejected at
    /// the header, and the longest frame is counted with this code. A radio that sends one code sets
    /// it. `None`: any code a v1 header can announce.
    pub only_code: Option<PayloadCode>,
    /// Distance between the time offsets the coarse search tests, in samples (at most `sps`). 0: a
    /// quarter symbol (`sps / 4`, at least 1). Larger steps cost proportionally less; the fine search around each
    /// candidate covers `sps + step` samples either way, so timing is still found to the sample.
    pub coarse_step: usize,
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
            max_len: None,
            only_code: None,
            coarse_step: 0,
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
    /// The reference, real and imaginary parts apart (for the vectorized products).
    ref_re: Vec<f32>,
    ref_im: Vec<f32>,
}

/// The largest offset searched, as a fraction of the sample rate.
const COARSE_MAX_CYCLES_PER_SAMPLE: f64 = 0.5;
/// Chunk length rule: the searched offset times the chunk length stays under this, so a chunk loses
/// at most `sinc^2(0.2)` = 0.6 dB of the signal at the edge of the range.
const COARSE_CHUNK_TURNS: f64 = 0.2;

fn check_len(payload: &[u8]) -> Result<(), ConfigError> {
    if payload.len() > MAX_PAYLOAD {
        return Err(ConfigError(format!(
            "payload of {} bytes is over the {MAX_PAYLOAD} byte limit",
            payload.len()
        )));
    }
    Ok(())
}

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
    /// The decoded v1 header (its CRC-8 matched). `None` for W0, and for v1 when the CRC-8 did not
    /// match (only possible with `expected_len`, which skips the header checks).
    pub header: Option<HeaderFields>,
}

/// What the receiver did with the candidates it found, from [`Modem::demodulate_report`].
///
/// These count *candidates*, not frames: peaks of the detection metric past the threshold. A real
/// frame is one candidate; its own body and noise add more (random data correlates with the
/// preamble now and then). Every candidate ends in exactly one of the five other counts or in a
/// returned frame:
/// `preambles == frames + sync_rejected + header_rejected + too_short + inside_frame`.
///
/// `inside_frame` keeps the others honest. A candidate that lies inside the span of a frame already
/// accepted, or inside the span a header failure reserves, is counted there and not as a rejection:
/// it is not a lost frame. An accepted frame's span is its length from the header. A header failure
/// gives no length, so it reserves the longest possible frame (255 bytes) after its start. A frame
/// that fails at the sync word reserves nothing, so its own body candidates count as `sync_rejected`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RxReport {
    /// Candidates past the detection threshold.
    pub preambles: usize,
    /// Candidates inside the span of an accepted frame or of a header failure, that gave no frame.
    pub inside_frame: usize,
    /// The sync word had too many wrong bits.
    pub sync_rejected: usize,
    /// The header was refused: v1 CRC-8 or field rules; W0 length complement.
    pub header_rejected: usize,
    /// The frame ran off the end of the buffer.
    pub too_short: usize,
}

impl RxReport {
    /// Add another report's counts (for totals over many buffers).
    pub fn add(&mut self, o: &RxReport) {
        self.preambles += o.preambles;
        self.inside_frame += o.inside_frame;
        self.sync_rejected += o.sync_rejected;
        self.header_rejected += o.header_rejected;
        self.too_short += o.too_short;
    }
}

/// Counts and spans kept while scanning a buffer.
#[derive(Default)]
pub(crate) struct Tally {
    pub(crate) rep: RxReport,
    /// Candidates before this sample are not tried: a frame is surely there.
    pub(crate) busy_until: usize,
    /// Rejections of candidates before this sample count as `inside_frame`.
    pub(crate) span_until: usize,
}

impl Tally {
    /// A candidate inside a sure frame: counted, not tried.
    pub(crate) fn skipped(&mut self) {
        self.rep.preambles += 1;
        self.rep.inside_frame += 1;
    }
}

/// Why `decode_at` gave no frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Reject {
    Sync,
    Header,
    TooShort,
}

/// Carrier estimate of one frame: frequency (radians per sample), phase at the frame start, amplitude
/// and the noise power per sample left after taking the signal out.
struct Estimate {
    omega: f64,
    psi: f64,
    amp: f64,
    noise_per_sample: f64,
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
        if cfg.frame == FrameFormat::W0 && cfg.code != PayloadCode::None {
            return Err(ConfigError(
                "error correction needs frame format v1 (W0 has no payload_code)".into(),
            ));
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

    /// Energy of one information bit: `Es / (bits_per_symbol * code rate)`. With the light code
    /// (rate 1/2) a bit costs two coded bits, so `Eb = 2 Es` for BPSK: Es/N0 per coded symbol is
    /// Eb/N0 - 3 dB. The tail and the CRC are overhead and not counted, as in W0.
    pub fn energy_per_bit(&self) -> f64 {
        self.energy_per_symbol()
            / (self.cfg.modulation.bits_per_symbol() as f64 * self.cfg.code.rate())
    }

    /// Length in samples of the known preamble and sync waveform the detector correlates with (the
    /// detection window, and how far apart two candidates must be to be told apart).
    pub fn reference_len(&self) -> usize {
        self.refw.len()
    }

    /// Number of samples [`Modem::modulate`] makes for a payload of `n` bytes.
    pub fn frame_samples(&self, n: usize) -> usize {
        self.frame_samples_coded(n, self.cfg.code)
    }

    /// Symbols [`Modem::modulate`] makes for a payload of `n` bytes: the airtime in symbol periods.
    pub fn frame_symbols(&self, n: usize) -> usize {
        self.ref_bits.len() + self.header_syms() + self.body_len_bits(n, self.cfg.code) / self.bps()
    }

    fn frame_samples_coded(&self, n: usize, code: PayloadCode) -> usize {
        let nsym =
            self.ref_bits.len() + self.header_syms() + self.body_len_bits(n, code) / self.bps();
        self.waveform_len(nsym)
    }

    /// The longest frame the receiver can be given with `rx`, in samples: the expected length if
    /// set, otherwise 255 bytes, with the code that makes it longest that a v1 header can announce
    /// (light). Used for how much of the buffer a candidate may need.
    pub fn longest_frame_samples(&self, rx: &RxConfig) -> usize {
        match rx.expected_len {
            Some(n) => self.frame_samples(n.min(MAX_PAYLOAD)),
            None => self.frame_samples_coded(self.longest_len(rx), self.longest_code(rx)),
        }
    }

    /// The code of the longest frame a header can announce: light for v1 (or `only_code`), none for W0.
    fn longest_code(&self, rx: &RxConfig) -> PayloadCode {
        match self.cfg.frame {
            FrameFormat::W0 => PayloadCode::None,
            FrameFormat::V1 => rx.only_code.unwrap_or(PayloadCode::Light),
        }
    }

    /// The most payload bytes a header may announce: 255, or `max_len` for v1.
    fn longest_len(&self, rx: &RxConfig) -> usize {
        match self.cfg.frame {
            FrameFormat::W0 => MAX_PAYLOAD,
            FrameFormat::V1 => rx.max_len.unwrap_or(MAX_PAYLOAD).clamp(1, MAX_PAYLOAD),
        }
    }

    fn bps(&self) -> usize {
        self.cfg.modulation.bits_per_symbol()
    }

    /// Symbols between the sync word and the body: the v1 header, none in W0.
    fn header_syms(&self) -> usize {
        match self.cfg.frame {
            FrameFormat::W0 => 0,
            FrameFormat::V1 => HEADER_SYMBOLS,
        }
    }

    /// Bits of the body (everything after the header symbols) for a payload of `n` bytes.
    fn body_len_bits(&self, n: usize, code: PayloadCode) -> usize {
        match self.cfg.frame {
            FrameFormat::W0 => body_len_bits(n),
            FrameFormat::V1 => body_len_bits_coded(n, code),
        }
    }

    fn waveform_len(&self, nsym: usize) -> usize {
        match self.cfg.modulation {
            Modulation::Fsk2 => nsym * self.cfg.sps,
            _ => (nsym - 1) * self.cfg.sps + self.h.len(),
        }
    }

    /// A complete frame as samples: W0: preamble, sync word, length, payload, CRC. V1: preamble,
    /// sync word, protected header, payload, CRC. The first sample is the start of the first
    /// pulse; nothing is added before or after. Fails if the payload is longer than 255 bytes.
    pub fn modulate(&self, payload: &[u8]) -> Result<Vec<Complex32>, ConfigError> {
        check_len(payload)?;
        match self.cfg.frame {
            FrameFormat::W0 => Ok(self.waveform(&self.ref_bits, &body_bits(payload))),
            FrameFormat::V1 => {
                let h = HeaderFields {
                    payload_code: self.cfg.code.code(),
                    ..HeaderFields::new(self.cfg.modulation.code(), payload.len() as u8)
                };
                self.modulate_v1(payload, &h)
            }
        }
    }

    /// A v1 frame with the given header fields, written as they are (the length is not checked
    /// against the payload). The body is coded as `payload_code` says (1: light; any other value:
    /// none). For tests: this is how reserved fields and bad headers are sent. Fails for a W0 modem.
    // Hidden from the docs: it is for tests and can send a length that does not match the payload.
    #[doc(hidden)]
    pub fn modulate_v1(
        &self,
        payload: &[u8],
        header: &HeaderFields,
    ) -> Result<Vec<Complex32>, ConfigError> {
        check_len(payload)?;
        let body = match PayloadCode::from_code(header.payload_code) {
            Some(PayloadCode::Light) => body_bits_light(header, payload),
            _ => body_bits_v1(header, payload),
        };
        self.modulate_v1_parts(&header_symbols(header), &body)
    }

    /// A v1 frame from ready-made parts: the 152 header symbols (bits, see
    /// [`crate::frame::header_symbols`]) and the whitened body bits (see
    /// [`crate::frame::body_bits_v1`]; QPSK needs an even number). For tests that damage one part.
    // Hidden from the docs: it is for tests and can send a header that does not match the body.
    #[doc(hidden)]
    pub fn modulate_v1_parts(
        &self,
        header_symbols: &[u8],
        body: &[u8],
    ) -> Result<Vec<Complex32>, ConfigError> {
        if self.cfg.frame != FrameFormat::V1 {
            return Err(ConfigError(
                "the modem is not set to frame format v1".into(),
            ));
        }
        if header_symbols.len() != HEADER_SYMBOLS {
            return Err(ConfigError(format!("need {HEADER_SYMBOLS} header symbols")));
        }
        let mut head = self.ref_bits.clone();
        head.extend_from_slice(header_symbols);
        Ok(self.waveform(&head, body))
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
        self.demodulate_report(samples, rx).0
    }

    /// [`Modem::demodulate_with`], and a count of what happened to every candidate: how many
    /// were tried and at which stage the others were lost.
    pub fn demodulate_report(
        &self,
        samples: &[Complex32],
        rx: &RxConfig,
    ) -> (Vec<RxFrame>, RxReport) {
        let mut tally = Tally::default();
        if rx.cfo_search > 0.0 {
            let frames = self.demodulate_coarse(samples, rx, &mut tally);
            return (frames, tally.rep);
        }
        let metric = self.metric(samples, rx);
        let sps = self.cfg.sps;
        let mut out: Vec<RxFrame> = Vec::new();
        for tau in 0..metric.len() {
            let m = metric[tau];
            if m < rx.detect_threshold {
                continue;
            }
            let lo = tau.saturating_sub(sps);
            let hi = (tau + sps + 1).min(metric.len());
            let is_peak = (lo..hi).all(|j| metric[j] < m || (metric[j] == m && j >= tau));
            if !is_peak {
                continue;
            }
            if tau < tally.busy_until {
                tally.skipped();
                continue;
            }
            let mu = parabola_peak(&metric, tau);
            let res = self.decode_at(samples, tau, mu, m, rx);
            if let Some(f) = self.record(res, tau, rx, &mut tally) {
                out.push(f);
            }
        }
        (out, tally.rep)
    }

    /// Count the outcome of one candidate at absolute sample `tau`, update the spans, and return
    /// the frame if there is one.
    pub(crate) fn record(
        &self,
        res: Result<RxFrame, Reject>,
        tau: usize,
        rx: &RxConfig,
        tally: &mut Tally,
    ) -> Option<RxFrame> {
        tally.rep.preambles += 1;
        let inside = tau < tally.span_until;
        let reserve = |tally: &mut Tally| {
            // no length known: reserve the longest frame (or the expected one)
            tally.span_until = tally.span_until.max(tau + self.longest_frame_samples(rx));
        };
        let count = |tally: &mut Tally, field: fn(&mut RxReport) -> &mut usize| {
            if inside {
                tally.rep.inside_frame += 1;
            } else {
                *field(&mut tally.rep) += 1;
            }
        };
        match res {
            Ok(f) => {
                tally.span_until = tally.span_until.max(f.end);
                // A v1 header that passed its CRC-8 and rules gives the length: the frame is
                // there whatever its CRC-16 says. W0 trusts only a good CRC-16.
                // With the length known, a frame whose sync word matched is skipped past even
                // with a bad CRC-16, so its body is not searched again for false starts.
                let sure = skip_past(&f, rx)
                    || (self.cfg.frame == FrameFormat::V1 && rx.expected_len.is_none());
                if sure {
                    tally.busy_until = f.end;
                }
                Some(f)
            }
            Err(Reject::Sync) => {
                count(tally, |r| &mut r.sync_rejected);
                None
            }
            Err(Reject::Header) => {
                count(tally, |r| &mut r.header_rejected);
                reserve(tally);
                None
            }
            Err(Reject::TooShort) => {
                count(tally, |r| &mut r.too_short);
                None
            }
        }
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
        let bins = rx.cfo_bins as isize;
        // the reference turned to each frequency bin, as separate real and imaginary parts
        let refs: Vec<(Vec<f32>, Vec<f32>)> = (-bins..=bins)
            .map(|b| {
                let theta = 2.0 * PI * (b as f64) * rx.cfo_step as f64 / self.cfg.sps as f64;
                self.refw
                    .iter()
                    .enumerate()
                    .map(|(n, p)| {
                        let v = *p * Complex32::from_polar(1.0, (theta * n as f64) as f32);
                        (v.re, v.im)
                    })
                    .unzip()
            })
            .collect();
        // Every position is independent: split them over the CPUs. The result is the same, bit for bit.
        let one = |range: std::ops::Range<usize>| -> Vec<f32> {
            let mut best = vec![0.0f32; range.len()];
            for (pre, pim) in &refs {
                for (slot, tau) in best.iter_mut().zip(range.clone()) {
                    let win = &r[tau..tau + m];
                    let (mut cr, mut ci) = (0.0f32, 0.0f32);
                    for ((w, &a), &b) in win.iter().zip(pre).zip(pim) {
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
        };
        in_parallel(positions, 4096, one)
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
            step: if rx.coarse_step > 0 {
                rx.coarse_step.min(self.cfg.sps)
            } else {
                (self.cfg.sps / 4).max(1)
            },
            cells,
            threshold,
            fft: crate::fft::Fft::new(fft_len),
            ref_re: self.refw.iter().map(|v| v.re).collect(),
            ref_im: self.refw.iter().map(|v| v.im).collect(),
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

    pub(crate) fn coarse_metric_with(&self, r: &[Complex32], cs: &CoarseSearch) -> Vec<(f32, f32)> {
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
        let count = positions.div_ceil(cs.step);
        // Every position is independent: split them over the CPUs. The result is the same, bit for bit.
        let one = |range: std::ops::Range<usize>| -> Vec<(f32, f32)> {
            let mut buf = vec![Complex32::ZERO; n];
            let mut out = Vec::with_capacity(range.len());
            // window * conj(reference), every product on its own (so the loop vectorizes), then the
            // chunk sums; one running sum per chunk would be bound by the latency of its additions
            let (mut pr, mut pi) = (vec![0.0f32; m], vec![0.0f32; m]);
            // this range's samples with real and imaginary parts apart: the products then vectorize
            let lo = range.start * cs.step;
            let hi = ((range.end.max(1) - 1) * cs.step + m).min(r.len()).max(lo);
            let xr: Vec<f32> = r[lo..hi].iter().map(|v| v.re).collect();
            let xi: Vec<f32> = r[lo..hi].iter().map(|v| v.im).collect();
            let (br, bi) = (&cs.ref_re[..m], &cs.ref_im[..m]);
            for tau in range.map(|i| i * cs.step) {
                buf.fill(Complex32::ZERO);
                let (wr, wi) = (&xr[tau - lo..tau - lo + m], &xi[tau - lo..tau - lo + m]);
                for k in 0..m {
                    // window * conj(reference)
                    pr[k] = wr[k] * br[k] + wi[k] * bi[k];
                    pi[k] = wi[k] * br[k] - wr[k] * bi[k];
                }
                for ((slot, cr), ci) in buf
                    .iter_mut()
                    .zip(pr.chunks(cs.chunk))
                    .zip(pi.chunks(cs.chunk))
                {
                    *slot = Complex32::new(cr.iter().sum(), ci.iter().sum());
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
                let mag =
                    |k: isize| buf[k.rem_euclid(n as isize) as usize].norm_sqr().sqrt() as f64;
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
        };
        in_parallel(count, 2048, one)
    }

    /// [`Modem::demodulate_with`] with the coarse frequency search: find candidates on the coarse
    /// grid, then for each one turn the buffer back by its frequency and run the normal detector
    /// (one frequency) and decoder in a small window around it.
    fn demodulate_coarse(&self, r: &[Complex32], rx: &RxConfig, tally: &mut Tally) -> Vec<RxFrame> {
        let Some(cs) = self.coarse_search(rx) else {
            return Vec::new();
        };
        let cm = self.coarse_metric_with(r, &cs);
        if cm.is_empty() {
            return Vec::new();
        }
        let m = self.refw.len();
        // A candidate must be the strongest within one reference length. Taking the best of many
        // frequencies lifts the time side peaks too (for 2-FSK the preamble one symbol off still
        // matches well), and a one-symbol window lets them through as false starts. Real frames
        // are always further apart than this; overlapping frames are not separated anyway.
        let reach = m.div_ceil(cs.step);
        let frame_len = self.longest_frame_samples(rx);
        let fine = RxConfig {
            cfo_bins: 0,
            cfo_search: 0.0,
            ..rx.clone()
        };
        let mut out: Vec<RxFrame> = Vec::new();
        for i in 0..cm.len() {
            let (v, f) = cm[i];
            let tau_c = i * cs.step;
            if v < cs.threshold {
                continue;
            }
            let lo_i = i.saturating_sub(reach);
            let hi_i = (i + reach + 1).min(cm.len());
            let is_peak = (lo_i..hi_i).all(|j| cm[j].0 < v || (cm[j].0 == v && j >= i));
            if !is_peak {
                continue;
            }
            if tau_c < tally.busy_until {
                tally.skipped();
                continue;
            }
            let Some((res, tau, _)) =
                self.coarse_candidate(r, tau_c, f, &cs, &fine, frame_len, tally.busy_until)
            else {
                continue;
            };
            if let Some(fr) = self.record(res, tau, rx, tally) {
                out.push(fr);
            }
        }
        out
    }

    /// One coarse-search candidate at `tau_c` with frequency `f` (cycles per symbol): turn a small
    /// window of `r` back by `f`, find the fine peak and decode there. Returns the decode, the
    /// sample the candidate counts at, and whether `r` held all the samples the longest frame
    /// could need (if not, `TooShort` may only mean "wait for more"). `None`: no fine peak.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn coarse_candidate(
        &self,
        r: &[Complex32],
        tau_c: usize,
        f: f32,
        cs: &CoarseSearch,
        fine: &RxConfig,
        frame_len: usize,
        busy_until: usize,
    ) -> Option<(Result<RxFrame, Reject>, usize, bool)> {
        let sps = self.cfg.sps;
        let m = self.refw.len();
        let last = r.len().checked_sub(m)?;
        let lo = tau_c.saturating_sub(sps + cs.step).max(busy_until);
        let hi = (tau_c + sps + cs.step).min(last);
        let full = hi + frame_len < r.len();
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
        let fm = self.metric(&d[..hi - lo + m], fine);
        let j = (0..fm.len()).max_by(|&a, &b| fm[a].total_cmp(&fm[b]).then(b.cmp(&a)))?;
        let mu = parabola_peak(&fm, j);
        let res = self.decode_at(&d, j, mu, fm[j], fine).map(|mut fr| {
            fr.start += lo;
            fr.end += lo;
            fr.cfo_cycles_per_symbol += f;
            let turns = (w * fr.start as f64).rem_euclid(1.0);
            let ph = fr.phase_rad as f64 + 2.0 * PI * turns;
            fr.phase_rad = ph.sin().atan2(ph.cos()) as f32;
            fr
        });
        Some((res, lo + j, full))
    }

    /// The known waveform of preamble, sync word and v1 header `h`, up to where the body's first
    /// pulse starts.
    fn header_reference(&self, h: &HeaderFields) -> Vec<Complex32> {
        let mut bits = self.ref_bits.clone();
        bits.extend(header_symbols(h));
        let mut w = self.waveform(&bits, &[]);
        w.truncate(bits.len() * self.cfg.sps);
        w
    }

    /// Carrier phase, frequency and amplitude of the frame at `tau`, from the chunk correlations of
    /// the received samples against the known waveform `refw` (it must start at the frame start and
    /// be at least `CHUNK_SYMBOLS` symbols long).
    fn estimate(&self, r: &[Complex32], tau: usize, refw: &[Complex32]) -> Estimate {
        let sps = self.cfg.sps;
        let chunk = CHUNK_SYMBOLS * sps;
        let k_chunks = refw.len() / chunk;
        // 1. chunk correlations against the reference
        let mut c = Vec::with_capacity(k_chunks);
        for k in 0..k_chunks {
            let (mut cr, mut ci) = (0.0f64, 0.0f64);
            for n in k * chunk..(k + 1) * chunk {
                let a = r[tau + n];
                let p = refw[n];
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
        let e_chunks: f64 = refw[..k_chunks * chunk]
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
        Estimate {
            omega,
            psi,
            amp,
            noise_per_sample,
        }
    }

    fn decode_at(
        &self,
        r: &[Complex32],
        tau: usize,
        mu: f64,
        metric: f32,
        rx: &RxConfig,
    ) -> Result<RxFrame, Reject> {
        let sps = self.cfg.sps;
        let Estimate {
            mut omega,
            mut psi,
            mut amp,
            mut noise_per_sample,
        } = self.estimate(r, tau, &self.refw);
        // 2. how much of the buffer we need
        let bps = self.bps();
        let n_ref = self.ref_bits.len();
        // the body starts after the preamble, the sync word and (v1) the header symbols
        let body_start = n_ref + self.header_syms();
        let max_bits = match rx.expected_len {
            Some(n) => self.body_len_bits(n, self.cfg.code),
            None => self.body_len_bits(self.longest_len(rx), self.longest_code(rx)),
        };
        let want_syms = body_start + max_bits / bps;
        let avail = r.len() - tau;
        let end_needed = |nsym: usize| self.waveform_len(nsym);
        let usable_syms = (n_ref..=want_syms)
            .rev()
            .find(|&s| end_needed(s) <= avail)
            .unwrap_or(0);
        if usable_syms < body_start + 16 / bps {
            return Err(Reject::TooShort);
        }
        // derotate and normalize the part we use: W0 all of it now; v1 up to the end of the header
        // now, and the body once the header has given its length and code
        let take = end_needed(match self.cfg.frame {
            FrameFormat::W0 => usable_syms,
            FrameFormat::V1 => body_start,
        });
        let derotate = |omega: f64, psi: f64, amp: f64, take: usize| -> Vec<Complex32> {
            (0..take)
                .map(|n| {
                    let (s, co) = (-(psi + omega * n as f64)).sin_cos();
                    r[tau + n] * Complex32::new((co / amp) as f32, (s / amp) as f32)
                })
                .collect()
        };
        let mut rr = derotate(omega, psi, amp, take);
        let taps = if self.cfg.modulation == Modulation::Fsk2 {
            Vec::new()
        } else {
            rrc_shifted(sps, self.cfg.rrc_span, self.cfg.rolloff as f64, mu)
        };
        // sync word: symbols n_ref-16 .. n_ref, one bit each
        let mut sync_errors = 0;
        let want_sync = &self.ref_bits[PREAMBLE_LEN..];
        let dem = SymbolDemod::new(self, &taps, &rr);
        for (i, &want) in want_sync.iter().enumerate() {
            if dem.one_bit_symbol(PREAMBLE_LEN + i) != want {
                sync_errors += 1;
            }
        }
        if sync_errors > rx.sync_max_errors {
            return Err(Reject::Sync);
        }

        // 3. body. The length comes first: W0 reads its 16 header bits, v1 decodes the header
        // symbols. With `expected_len` the length is given and a header is not checked (v1 still
        // decodes it, for the report in the frame).
        let mut z: Vec<Complex32> = Vec::new();
        let mut bits: Vec<u8> = Vec::new();
        let mut header = None;
        let n = match self.cfg.frame {
            FrameFormat::W0 => match rx.expected_len {
                Some(n) => n,
                None => {
                    dem.body_symbols(n_ref, 16 / bps, &mut z, &mut bits);
                    parse_header(&bits).ok_or(Reject::Header)?
                }
            },
            FrameFormat::V1 => {
                let soft: Vec<f32> = (n_ref..body_start).map(|k| dem.header_soft(k)).collect();
                header = parse_header_v1(&soft);
                match (rx.expected_len, &header) {
                    (Some(n), _) => n,
                    (None, Some(h))
                        if h.acceptable(self.cfg.modulation.code())
                            && h.length as usize <= self.longest_len(rx)
                            && rx.only_code.is_none_or(|c| c.code() == h.payload_code) =>
                    {
                        h.length as usize
                    }
                    _ => return Err(Reject::Header),
                }
            }
        };
        // The header is now known: another 152 symbols to estimate frequency and phase from. The
        // body starts that much further from the preamble, and a small frequency error that the
        // preamble alone cannot see (it is below its noise) would turn the phase too far by then.
        if let Some(h) = &header {
            let e = self.estimate(r, tau, &self.header_reference(h));
            (omega, psi, amp, noise_per_sample) = (e.omega, e.psi, e.amp, e.noise_per_sample);
        }
        // The body's code: from the header, or the modem's own setting when the length is given.
        let code = match (self.cfg.frame, rx.expected_len, &header) {
            (FrameFormat::W0, _, _) => PayloadCode::None,
            (FrameFormat::V1, None, Some(h)) => {
                PayloadCode::from_code(h.payload_code).unwrap_or(PayloadCode::None)
            }
            (FrameFormat::V1, _, _) => self.cfg.code,
        };
        let total_syms = body_start + self.body_len_bits(n, code) / bps;
        if end_needed(total_syms) > avail || (rx.expected_len.is_none() && total_syms > usable_syms)
        {
            return Err(Reject::TooShort);
        }
        if self.cfg.frame == FrameFormat::V1 {
            rr = derotate(omega, psi, amp, end_needed(total_syms));
        }
        let dem = SymbolDemod::new(self, &taps, &rr);
        z.clear();
        bits.clear();
        dem.body_symbols(body_start, total_syms - body_start, &mut z, &mut bits);
        let mut mu_final = mu;
        // Es/N0 of this frame: the matched filter outputs have amplitude 1 and noise variance 1 / rho
        let rho = amp * amp / noise_per_sample;
        // the phase line the PSK body is turned back by (none without the refinement)
        let (mut line_a, mut line_b) = (0.0f64, 0.0f64);
        if self.cfg.modulation != Modulation::Fsk2 && rx.refine_phase {
            // The preamble and sync word are known: their matched filter outputs with the data
            // taken off, at their symbol index counted from the body's first symbol (negative).
            // A light-coded body runs at half the Es/N0 and twice the length: there the decoded
            // header counts as known too, which narrows the slope search (the bound shrinks about
            // five times) and anchors the phase better. Uncoded frames keep the preamble only.
            let known_bits = match (code, &header) {
                (PayloadCode::Light, Some(h)) => {
                    let mut k = self.ref_bits.clone();
                    k.extend(header_symbols(h));
                    k
                }
                _ => self.ref_bits.clone(),
            };
            let known_at = |dm: &SymbolDemod| -> Vec<(f64, Complex32)> {
                known_bits
                    .iter()
                    .enumerate()
                    .map(|(j, &bit)| {
                        let s = 1.0 - 2.0 * (bit & 1) as f32;
                        (j as f64 - body_start as f64, dm.z(j) * s)
                    })
                    .collect()
            };
            let known = known_at(&dem);
            let (mut a, mut b) = self.track(&z, rho, &known);
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
                            let y = dm.z(body_start + k) * Complex32::new(c as f32, s as f32);
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
                dm.body_symbols(body_start, total_syms - body_start, &mut z, &mut bits);
                (a, b) = self.track(&z, rho, &known_at(&dm));
                bits = decide_psk(self.cfg.modulation, &z, a, b);
            }
            (line_a, line_b) = (a, b);
        }
        let (payload, crc_ok) = match self.cfg.frame {
            FrameFormat::W0 => parse_body(&bits, n),
            // v1: the CRC-16 covers the header bytes. If the header was not decoded (only with
            // `expected_len`), the frame cannot be trusted.
            FrameFormat::V1 => {
                let h = header.unwrap_or_default();
                let (payload, ok) = match code {
                    PayloadCode::None => parse_body_v1(&bits, &h, n),
                    PayloadCode::Light => {
                        let soft = match self.cfg.modulation {
                            Modulation::Fsk2 => (body_start..total_syms)
                                .map(|k| dem.fsk_llr(k, rho))
                                .collect(),
                            m => psk_llr(m, &z, line_a, line_b, rho),
                        };
                        parse_body_light(&soft, &h, n)
                    }
                };
                (payload, ok && header.is_some())
            }
        };
        Ok(RxFrame {
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
            header,
        })
    }

    /// Carrier phase and frequency of the frame body. Returns `(a, b)`: turn body symbol `k` back by
    /// `a + b * k` radians.
    ///
    /// First the power-law fit ([`refine_phase`]) with decision-directed polish, as before. Then, if
    /// the known symbols (`known`: preamble and sync word, data taken off, at their index from the
    /// body's first symbol) and the body together are more likely under another phase line, that line
    /// is used ([`ml_line`]). The power law knows the phase only to a quarter turn (QPSK) and its
    /// slope is noisy at low SNR; a drift left by the preamble's frequency estimate then put the
    /// body on the wrong quarter, or let it turn away, in a few percent of QPSK frames at 0 dB
    /// Eb/N0 (oasa-radio#14). The known symbols fix the phase near the frame start without ambiguity.
    fn track(&self, z: &[Complex32], rho: f64, known: &[(f64, Complex32)]) -> (f64, f64) {
        let m = self.cfg.modulation;
        let (mut a, b) = refine_phase(m, z, rho);
        // Decision-directed polish: with the data mostly right it uses the whole frame without
        // the power law's noise enhancement. Two passes.
        for _ in 0..2 {
            a += dd_phase_error(m, z, a, b);
        }
        if known.is_empty() {
            return (a, b);
        }
        let ll = |a: f64, b: f64| line_log_likelihood(m, z, known, rho, a, b);
        let (ma, mb) = ml_line(m, z, known, rho);
        if ll(ma, mb) > ll(a, b) {
            (ma, mb)
        } else {
            (a, b)
        }
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

    /// Soft value of symbol `k` as one coded header bit: positive means bit 0 (see [`crate::fec`]).
    /// PSK: the real part of the matched filter output. 2-FSK: energy of the "-" tone minus
    /// energy of the "+" tone.
    fn header_soft(&self, k: usize) -> f32 {
        if self.m.cfg.modulation == Modulation::Fsk2 {
            let (p, m) = self.tones(k);
            m - p
        } else {
            self.z(k).re
        }
    }

    /// Log-likelihood ratio of 2-FSK symbol `k` as one coded bit (positive: bit 0), at Es/N0 `rho`.
    /// Noncoherent: `ln I0(2 rho |c-|) - ln I0(2 rho |c+|)`, with `c-` and `c+` the tone
    /// correlations scaled so that the sent tone has magnitude 1 (their energies are what
    /// [`SymbolDemod::tones`] returns). See [`fsk_llr`].
    fn fsk_llr(&self, k: usize, rho: f64) -> f32 {
        let (p, m) = self.tones(k);
        fsk_llr(m as f64, p as f64, rho)
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

/// Log-likelihood ratios (positive: bit 0) of PSK matched filter outputs `z`, each turned back by
/// `a + b * k` radians, at Es/N0 `rho`. The outputs have amplitude 1 and complex noise variance
/// `1 / rho` (`1 / (2 rho)` per dimension). BPSK: `4 rho Re(y)`. QPSK (each axis carries `1/sqrt 2`):
/// `2 sqrt(2) rho Re(y)` for the first bit and the same of `Im(y)` for the second.
fn psk_llr(m: Modulation, z: &[Complex32], a: f64, b: f64, rho: f64) -> Vec<f32> {
    let mut out = Vec::with_capacity(z.len() * 2);
    let g = if m == Modulation::Bpsk {
        4.0 * rho
    } else {
        2.0 * std::f64::consts::SQRT_2 * rho
    };
    for (k, &v) in z.iter().enumerate() {
        let (s, c) = (-(a + b * k as f64)).sin_cos();
        let (x, y) = (v.re as f64, v.im as f64);
        out.push((g * (x * c - y * s)) as f32);
        if m == Modulation::Qpsk {
            out.push((g * (x * s + y * c)) as f32);
        }
    }
    out
}

/// Noncoherent 2-FSK log-likelihood ratio for bit 0 from the energies of the bit-0 tone `e0` and the
/// bit-1 tone `e1` (scaled so that the sent tone has energy 1 without noise), at Es/N0 `rho`. With
/// unknown phase the sent tone's correlation `c` has likelihood `exp(-rho (|c|^2 + 1)) I0(2 rho |c|)`
/// and the other `exp(-rho |c|^2)`; the ratio is `ln I0(2 rho sqrt e0) - ln I0(2 rho sqrt e1)`.
pub fn fsk_llr(e0: f64, e1: f64, rho: f64) -> f32 {
    (ln_i0(2.0 * rho * e0.max(0.0).sqrt()) - ln_i0(2.0 * rho * e1.max(0.0).sqrt())) as f32
}

/// `ln I0(x)` for `x >= 0`, the modified Bessel function of order 0: the polynomial approximations
/// of Abramowitz and Stegun 9.8.1 (`x <= 3.75`) and 9.8.2 (above), relative error under 2e-7.
pub fn ln_i0(x: f64) -> f64 {
    let x = x.abs();
    if x <= 3.75 {
        let t = (x / 3.75).powi(2);
        (1.0 + t
            * (3.515_622_9
                + t * (3.089_942_4
                    + t * (1.206_749_2 + t * (0.265_973_2 + t * (0.036_076_8 + t * 0.004_581_3))))))
            .ln()
    } else {
        let t = 3.75 / x;
        let p = 0.398_942_28
            + t * (0.013_285_92
                + t * (0.002_253_19
                    + t * (-0.001_575_65
                        + t * (0.009_162_81
                            + t * (-0.020_577_06
                                + t * (0.026_355_37 + t * (-0.016_476_33 + t * 0.003_923_77)))))));
        x - 0.5 * x.ln() + p.ln()
    }
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
        return power_law_slope(m, z, mean_t);
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

/// `f` over `0..count`, split into contiguous ranges run on up to the machine's CPUs (at most 8), the
/// results joined in order. Below `min_each` items per thread it runs on the calling thread only.
fn in_parallel<T: Send>(
    count: usize,
    min_each: usize,
    f: impl Fn(std::ops::Range<usize>) -> Vec<T> + Sync,
) -> Vec<T> {
    let cpus = std::thread::available_parallelism()
        .map_or(1, |n| n.get())
        .min(8);
    let threads = cpus.min(count / min_each.max(1)).max(1);
    if threads == 1 {
        return f(0..count);
    }
    let each = count.div_ceil(threads);
    let f = &f;
    std::thread::scope(|sc| {
        let handles: Vec<_> = (0..threads)
            .map(|t| {
                let range = (t * each).min(count)..((t + 1) * each).min(count);
                sc.spawn(move || f(range))
            })
            .collect();
        let mut out = Vec::with_capacity(count);
        for h in handles {
            out.extend(h.join().expect("detection thread panicked"));
        }
        out
    })
}

/// `ln cosh(u)`, without overflow for large `|u|`.
fn ln_cosh(u: f64) -> f64 {
    let u = u.abs();
    if u > 18.0 {
        // exp(-36) is below f64 resolution next to u
        return u - std::f64::consts::LN_2;
    }
    u + (-2.0 * u).exp().ln_1p() - std::f64::consts::LN_2
}

/// Log-likelihood (up to a constant) of the phase line `a + b * k` given the body symbols `z`
/// (unknown, equally likely data) and the known symbols `known` (data taken off; one bit per symbol,
/// unit energy). `rho` is Es/N0: the matched filter outputs have amplitude 1 and noise variance
/// `1 / rho`. Exact for BPSK and Gray QPSK in white noise.
fn line_log_likelihood(
    m: Modulation,
    z: &[Complex32],
    known: &[(f64, Complex32)],
    rho: f64,
    a: f64,
    b: f64,
) -> f64 {
    let turn = |v: Complex32, (c, s): (f64, f64)| {
        let (x, y) = (v.re as f64, v.im as f64);
        (x * c - y * s, x * s + y * c)
    };
    let mut ll = 0.0;
    for &(k, w) in known {
        let (s, c) = (-(a + b * k)).sin_cos();
        ll += 2.0 * rho * turn(w, (c, s)).0;
    }
    let g = if m == Modulation::Bpsk {
        2.0 * rho
    } else {
        std::f64::consts::SQRT_2 * rho
    };
    // the rotation for symbol k, stepped by e^{-jb} (re-anchored every 64 symbols against drift)
    let (s1, c1) = (-b).sin_cos();
    let mut rot = (1.0f64, 0.0f64);
    for (k, &v) in z.iter().enumerate() {
        if k % 64 == 0 {
            let (s, c) = (-(a + b * k as f64)).sin_cos();
            rot = (c, s);
        }
        let (x, y) = turn(v, rot);
        ll += ln_cosh(g * x);
        if m == Modulation::Qpsk {
            ll += ln_cosh(g * y);
        }
        rot = (rot.0 * c1 - rot.1 * s1, rot.0 * s1 + rot.1 * c1);
    }
    ll
}

/// The phase line `(a, b)` that maximizes [`line_log_likelihood`]: slopes within 5 times the
/// Cramer-Rao bound of the preamble's frequency estimate (at most +-[`SLOPE_SEARCH_CYCLES`] cycles
/// per symbol), and for each the phase within 16 degrees of what the
/// known symbols alone give at that slope (they are long and strong enough that the truth is well
/// inside that); then a finer search around the best.
fn ml_line(m: Modulation, z: &[Complex32], known: &[(f64, Complex32)], rho: f64) -> (f64, f64) {
    let anchor = |b: f64| {
        let (mut sr, mut si) = (0.0f64, 0.0f64);
        for &(k, w) in known {
            let (s, c) = (-b * k).sin_cos();
            sr += w.re as f64 * c - w.im as f64 * s;
            si += w.re as f64 * s + w.im as f64 * c;
        }
        si.atan2(sr)
    };
    let ll = |a: f64, b: f64| line_log_likelihood(m, z, known, rho, a, b);
    let span = (z.len() as f64 + known.len() as f64).max(1.0);
    // What the preamble's frequency estimate can leave: its Cramer-Rao bound from the known symbols,
    // sqrt(6 / (rho N (N^2 - 1))) radians per symbol, 5 of those (the estimate is near the bound,
    // and an offset the preamble did not take as real is under about 4). Never more than
    // SLOPE_SEARCH_CYCLES. Narrower at high SNR: fewer false peaks to choose from, and faster.
    let n = known.len().max(2) as f64;
    let crb = (6.0 / (rho.max(1e-3) * n * (n * n - 1.0))).sqrt();
    let b_max = (5.0 * crb).min(2.0 * PI * SLOPE_SEARCH_CYCLES);
    // coarse: slope step = half a radian over the whole span; phase step 8 degrees
    let b_step = 0.5 / span;
    let a_step = 8f64.to_radians();
    let nb = (b_max / b_step).ceil() as i64;
    let (mut best, mut ba, mut bb) = (f64::NEG_INFINITY, 0.0, 0.0);
    for i in -nb..=nb {
        let b = i as f64 * b_step;
        let a0 = anchor(b);
        for j in -2..=2 {
            let a = a0 + j as f64 * a_step;
            let v = ll(a, b);
            if v > best {
                (best, ba, bb) = (v, a, b);
            }
        }
    }
    // fine: shrink the steps around the best, a few rounds
    let (mut sa, mut sb) = (a_step / 2.0, b_step / 2.0);
    for _ in 0..5 {
        let (ca, cb) = (ba, bb);
        for i in -1..=1 {
            for j in -1..=1 {
                let (a, b) = (ca + j as f64 * sa, cb + i as f64 * sb);
                let v = ll(a, b);
                if v > best {
                    (best, ba, bb) = (v, a, b);
                }
            }
        }
        sa /= 2.0;
        sb /= 2.0;
    }
    (ba, bb)
}

/// Largest residual frequency [`power_law_slope`] searches, in cycles per symbol. After the
/// preamble's estimate, what is left at 0 dB Eb/N0 is within about +-0.0015 (3 sigma; the
/// preamble is near its Cramer-Rao bound there).
const SLOPE_SEARCH_CYCLES: f64 = 0.0025;

/// Gain over slope 0, in units of the noise power of the sum, that [`power_law_slope`] needs before
/// it uses a slope. With no slope the gain of the best of the search is about the largest of a few
/// independent exponentials of mean 1, so this keeps false slopes to a few percent of frames, and a
/// false slope is never larger than the search range.
const SLOPE_GAIN_MIN: f64 = 5.0;

/// Phase and slope of the frame body at low SNR, where [`refine_phase`] has too few segments to
/// fit a line: the slope `b` (radians per symbol) that makes `|sum_k z_k^M exp(-j M b k)|` largest,
/// searched over the whole body (so no unwrapping), then the phase at that slope. `M` is the
/// power-law order. The slope is used only if it raises the sum by [`SLOPE_GAIN_MIN`] over slope 0;
/// otherwise the result is `(mean_t, 0)`.
///
/// Why: with the coarse frequency search on, the preamble leaves up to about 0.0015 cycles per
/// symbol at 0 dB, which is over 100 degrees across a 64-byte QPSK frame. Without this, QPSK lost
/// 0.6 to 1.4 dB at 1 to 2 dB Eb/N0 (oasa-radio#14). Returns `(a, b)` like [`refine_phase`].
fn power_law_slope(m: Modulation, z: &[Complex32], mean_t: f64) -> (f64, f64) {
    let order = if m == Modulation::Bpsk { 2 } else { 4 };
    let n = z.len();
    if n < 32 {
        return (mean_t, 0.0);
    }
    let p: Vec<(f64, f64)> = z
        .iter()
        .map(|v| {
            let (x, y) = (v.re as f64, v.im as f64);
            let (mut pr, mut pi) = (x, y);
            for _ in 1..order {
                (pr, pi) = (pr * x - pi * y, pr * y + pi * x);
            }
            (pr, pi)
        })
        .collect();
    let centre = (n - 1) as f64 / 2.0;
    // the power-law sum at slope b, about the body's centre
    let sum_at = |b: f64| -> (f64, f64) {
        let (mut sr, mut si) = (0.0f64, 0.0f64);
        for (k, &(pr, pi)) in p.iter().enumerate() {
            let (s, c) = (-(order as f64) * b * (k as f64 - centre)).sin_cos();
            sr += pr * c - pi * s;
            si += pr * s + pi * c;
        }
        (sr, si)
    };
    let pw = |b: f64| {
        let (r, i) = sum_at(b);
        r * r + i * i
    };
    // grid: a quarter of the resolution, 1 / (order * n) cycles per symbol
    let step = 2.0 * PI / (4.0 * order as f64 * n as f64);
    let reach = (2.0 * PI * SLOPE_SEARCH_CYCLES / step).ceil() as i64;
    let (mut best, mut best_j) = (-1.0f64, 0i64);
    for j in -reach..=reach {
        let v = pw(j as f64 * step);
        if v > best {
            best = v;
            best_j = j;
        }
    }
    let mut b = best_j as f64 * step;
    if best_j.abs() < reach {
        let (y0, y1, y2) = (pw(b - step).sqrt(), best.sqrt(), pw(b + step).sqrt());
        let den = y0 - 2.0 * y1 + y2;
        if den < 0.0 {
            b += 0.5 * (y0 - y2) / den * step;
        }
    }
    // Noise power of the sum: the total power of the terms less the signal's share of it.
    let e: f64 = p.iter().map(|&(r, i)| r * r + i * i).sum();
    let noise = (e - best / n as f64).max(1e-30);
    if (pw(b) - pw(0.0)) / noise < SLOPE_GAIN_MIN {
        return (mean_t, 0.0);
    }
    let (mut sr, mut si) = sum_at(b);
    if order == 4 {
        sr = -sr;
        si = -si;
    }
    // the phase at the centre, on the ambiguity the segments chose (nearest their mean)
    let st = PI / order as f64;
    let a_c = si.atan2(sr) / order as f64;
    let a_c = mean_t + (a_c - mean_t + st).rem_euclid(2.0 * st) - st;
    (a_c - b * centre, b)
}

/// Where the peak of three neighbouring metric values lies, as an offset from `i` in samples
/// (a parabola through the three points; clamped to half a sample).
/// Sync errors up to which a damaged frame of known length is still taken as a real frame (the
/// default `sync_max_errors`). A false start in noise has about 8 of 16 wrong.
const SKIP_SYNC_MAX_ERRORS: u32 = 3;

/// Whether the search skips past this frame. A frame's extent is known when its CRC passes, or when
/// the length is given and its sync word matched: then the body of a damaged frame is not searched
/// again for false starts. With sync rejection off (BER measurement), a false start in the noise
/// before a frame must not hide the frame, so its sync word must match.
fn skip_past(f: &RxFrame, rx: &RxConfig) -> bool {
    f.crc_ok || (rx.expected_len.is_some() && f.sync_errors <= SKIP_SYNC_MAX_ERRORS)
}

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

#[cfg(test)]
mod llr_tests {
    use super::*;

    #[test]
    fn ln_i0_matches_its_series() {
        // I0(x) = sum (x^2/4)^k / (k!)^2
        for x in [0.0f64, 0.1, 1.0, 3.0, 3.75, 3.76, 5.0, 10.0, 30.0, 100.0] {
            let (mut term, mut sum) = (1.0f64, 1.0f64);
            for k in 1..400 {
                term *= x * x / 4.0 / (k as f64 * k as f64);
                sum += term;
            }
            let want = sum.ln();
            assert!(
                (ln_i0(x) - want).abs() <= 2e-7 * want.abs().max(1.0),
                "x {x}: {} vs {want}",
                ln_i0(x)
            );
        }
    }

    #[test]
    fn llr_signs_and_scale() {
        // PSK: positive for bit 0, and QPSK gives two values per symbol
        let z = [Complex32::new(0.5, -0.25)];
        let b = psk_llr(Modulation::Bpsk, &z, 0.0, 0.0, 2.0);
        assert_eq!(b, vec![4.0]);
        let q = psk_llr(Modulation::Qpsk, &z, 0.0, 0.0, 2.0);
        let r2 = std::f32::consts::SQRT_2;
        assert!((q[0] - 2.0 * r2).abs() < 1e-5 && (q[1] + r2).abs() < 1e-5);
        // a quarter turn back moves Im onto Re
        let t = psk_llr(
            Modulation::Bpsk,
            &[Complex32::new(0.0, 1.0)],
            PI / 2.0,
            0.0,
            1.0,
        );
        assert!((t[0] - 4.0).abs() < 1e-5);
        // FSK: more energy in the bit-0 tone means bit 0
        assert!(fsk_llr(1.0, 0.1, 3.0) > 0.0 && fsk_llr(0.1, 1.0, 3.0) < 0.0);
        assert_eq!(fsk_llr(0.5, 0.5, 3.0), 0.0);
    }
}

#[cfg(test)]
mod parallel_tests {
    use super::in_parallel;

    #[test]
    fn in_parallel_keeps_every_item_in_order() {
        for (count, min_each) in [
            (0, 10),
            (1, 10),
            (9, 10),
            (10_000, 100),
            (10_007, 1),
            (65_537, 4096),
        ] {
            let got = in_parallel(count, min_each, |r: std::ops::Range<usize>| {
                r.collect::<Vec<_>>()
            });
            assert_eq!(
                got,
                (0..count).collect::<Vec<_>>(),
                "count {count}, min_each {min_each}"
            );
        }
    }
}
