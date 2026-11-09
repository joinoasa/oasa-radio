//! `oasa-radio waveform ...`: OASA's own waveform (milestone W0), in simulation.
//!
//! * `ber`: Monte Carlo bit error rate against theory, one JSON line per Eb/N0 point.
//! * `per`: packet loss of the real receiver (length and code read from each frame), one line per point.
//! * `iq`: write a modulated, noisy frame as a SigMF pair (`cf32_le`) for the lab.
//! * `decode`: read such a file back and print the frames found and what happened to the candidates.
//! * `df`: two receive channels' phase difference and the direction of arrival (T-RF3).
//!
//! `--frame w0|v1` picks the frame format (default w0; v1 has the protected header). `--code
//! none|light` picks the error correction on the v1 body (default none; see docs/fec-light.md).
//!
//! The payloads here are test data made from a seed. Payload bytes are printed only when asked for (`--hex`).

use std::collections::BTreeMap;
use std::io::Write;
use std::path::PathBuf;
use std::time::SystemTime;

use oasa_waveform::channel::{n0_for_ebn0, snr_per_sample_db, Channel, Impairments};
use oasa_waveform::df;
use oasa_waveform::diversity::Combine;
use oasa_waveform::frame::{FrameFormat, HeaderFields, PayloadCode};
use oasa_waveform::iq::{from_cf32_le, from_ci16_le, to_cf32_le};
use oasa_waveform::rng::Rng;
use oasa_waveform::sim::{run_ber, run_found, BerOptions};
use oasa_waveform::stream::StreamReceiver;
use oasa_waveform::{Modem, ModemConfig, Modulation, RxConfig, RxFrame};

use crate::log::{json_str, rfc3339};

pub const USAGE: &str = "\
WAVEFORM (W0, simulation):
  oasa-radio waveform ber --mod bpsk|qpsk|fsk2 --ebn0-db 0,2,4,6 [--bits N] [--seed S]
                          [--sps 8] [--payload-len 64] [--cfo CYCLES_PER_SYMBOL]
                          [--cfo-random MAX] [--random-frac-delay] [--cfo-bins 0]
                          [--cfo-search MAX] [--frame w0|v1] [--code none|light] [--json]
                          [--channel awgn|tworay|rayleigh] [--doppler-hz 0] [--ray-delay-us 1]
                          [--ray-gain-db -3] [--symbol-rate 250000] [--frame-interval-ms 10]
                          [--phase-track-hz 0]
  oasa-radio waveform per --mod bpsk|qpsk|fsk2 --ebn0-db 3,3.5,4 [--frames 400] [--sync-max-errors 3]
                          [same options as ber, except --bits] [--diversity off|select|snr|mrc]
  oasa-radio waveform iq --mod bpsk|qpsk|fsk2 --out BASE [--ebn0-db X] [--payload-len 32 | --payload-hex HEX]
                         [--seed S] [--sps 8] [--symbol-rate 25000] [--center-hz 0]
                         [--phase-deg 0] [--delay 64] [--tail 64] [--cfo 0] [--frac-delay 0]
                         [--frame w0|v1] [--code none|light]
  oasa-radio waveform decode --in BASE --mod bpsk|qpsk|fsk2 [--sps 8] [--cfo-bins 0]
                             [--cfo-search MAX] [--frame w0|v1] [--code none|light] [--hex]
                             [--payload-len N] [--sync-max-errors 3]
  oasa-radio waveform rx-stream --in -|FILE --mod bpsk|qpsk|fsk2 [--block 65536]
                                [same receiver options as decode]
  oasa-radio waveform cable --in FILE --out FILE [--gain-db 0] [--noise-dbfs -60] [--cfo-hz 0]
                            [--sample-rate 1000000] [--seed 1]
                            [--channel awgn|tworay|rayleigh] [--doppler-hz 0] [--ray-delay-us 1]
                            [--ray-gain-db -3]
  oasa-radio waveform df --in1 FILE --in2 FILE [--format ci16|cf32] [--block N]
                         [--burst-db 10] [--win 256] [--min-len 4096] [--skip 0]
                         [--cal-deg 0] [--spacing-wl D]

  ber     Sends random frames through the modem and the simulated channel at each Eb/N0 and prints one
          JSON line per point: errors, bits, BER, the theory value (BPSK and QPSK: Q(sqrt(2Eb/N0));
          2-FSK, noncoherent: 0.5 exp(-Eb/2N0)), a 3-sigma Wilson interval and the binomial z-score.
          Default 100000 bits, seed 1. The same seed gives the same lines. (--json is accepted; output
          is always JSON lines.) --cfo-random gives each frame its own offset, uniform in +-MAX.
          With --frame v1 the line also has header_ok: frames whose header decoded, through the
          real header path, to what was sent (the BER measurement itself skips the header checks).
          In v1 BER mode a header that fails its CRC-8 marks the frame crc_ok false.
          With --code light, Eb is per information bit (the coded body runs at Eb/N0 - 3 dB per
          BPSK symbol); theory and z are null (no closed form).
  per     Sends --frames random frames at each Eb/N0 and runs the receiver as it runs on the air
          (no known length; header, length and code read from each frame). One JSON line per
          point: frames, found (header accepted at the right place), delivered (CRC-16 good),
          packet_loss = 1 - delivered/frames with a 3-sigma Wilson interval, and the candidate
          counts. Default 400 frames of 64 bytes, seed 1. The simulated reference for the bench.
  iq      Writes BASE.sigmf-data (cf32_le, little endian float32 I and Q) and BASE.sigmf-meta: one frame
          with BASE delay samples of noise in front and TAIL after. No --ebn0-db means no noise.
  decode  Finds frames in BASE.sigmf-data. Prints sizes and CRC status (and, for v1, the header
          fields); the payload only with --hex. The last line, decode_done, has the counts of
          what happened to each candidate: preambles, sync_rejected, header_rejected, too_short,
          inside_frame (candidates inside a frame already found; not lost frames).
          --payload-len N takes the length as known instead of reading the header, and
          --sync-max-errors 16 keeps frames with a damaged sync word: what `ber` does, for measuring
          BER on recordings (the W0 header has no error correction, so at 0 dB most W0 headers
          have a bit error and those frames are otherwise lost).

  rx-stream  The receiver for a continuous stream: reads raw cf32_le (no SigMF wrapper) from stdin
          (--in -) or a file, --block samples at a time, and prints each frame as soon as it is
          decided (flushed), with start and end counted from the stream's first sample. A frame is
          decided once one longest frame has arrived after its start; the result is the same as
          decode on the whole stream. Ends with stream_done (samples, frames, crc_ok_frames,
          trailing_bytes: an incomplete last sample, ignored). Memory stays about two frames.

  cable   A simulated cable between two `oasa-radio bridge --radio sdr` streams: reads cf32 from
          --in (one radio's --tx FIFO), turns it by --cfo-hz, scales it by --gain-db, adds
          receiver noise of --noise-dbfs (power per sample, full scale 1) and writes it to --out
          (the other radio's --rx FIFO), until --in ends. For tests and the lab's simulated bench.
  df      Two channels recorded together (one multi-channel stream; raw files, no SigMF wrapper,
          default ci16 as UHD's --type short writes). Measures r = mean(x2 conj(x1)) per burst (runs
          --burst-db over the noise floor in --win sample windows, at least --min-len long) or, with
          --block N, per fixed block (a continuous tone). One JSON line per segment: phase of channel 2
          relative to channel 1 (degrees, positive: channel 2 leads), coherence, power ratio, SNR per
          channel; then df_done with the circular mean and standard deviation. --cal-deg subtracts the
          receiver offset; with --spacing-wl (antenna spacing in wavelengths) each line also has
          doa_deg, from broadside, positive toward channel 2. --skip drops samples at the start (the
          radio settling). Carrier offset and modulation cancel, so any source works.

  --frame w0 is the original format (length byte and complement). --frame v1 protects the header
  with a code and a CRC-8 (docs/frame-v1.md). The sender and the receiver must use the same one.
  --code light (v1 only) codes the body: K=7 rate 1/2 and an interleaver (docs/fec-light.md). A v1
  receiver reads the code from each header, so decode and rx-stream need --code only with
  --payload-len (the length, and then the code, taken as known).

  Fading (--channel, B9; waveform/src/fading.rs): tworay adds a second ray --ray-delay-us later and
  --ray-gain-db weaker (with no --doppler-hz, a fixed ray and a random phase per frame; with Doppler,
  each ray fades on its own: two-ray Rayleigh). rayleigh is flat Rayleigh fading with a Clarke/Jakes
  spectrum, --doppler-hz = speed / wavelength. Mean power gain 1, so --ebn0-db is the mean. ber and
  per take --symbol-rate (for the sample rate) and --frame-interval-ms (time between frames: the
  fading moves on by this from frame to frame); the cable uses --sample-rate and rounds the second
  ray's delay to whole samples. awgn (the default) leaves everything as before.

  --diversity off|select|snr|mrc (per only; B14, docs/diversity.md) receives on two antennas, each
  with its own fading (--channel, independent) and noise at the same mean --ebn0-db, and combines
  them: select decodes both and keeps the frame with a good CRC, snr decodes only the channel with
  the stronger preamble, mrc adds the channels (maximal-ratio) after estimating each one's carrier
  and phase. Not given (the default): one antenna. off: one antenna, and like the others it adds
  rx_us_per_frame, the receiver's time per frame. Receiver only, nothing changes on the air.

  --phase-track-hz HZ (bpsk, qpsk; B12, docs/phase-tracking.md) turns on the receiver's channel
  tracker for fading up to HZ Doppler at --symbol-rate: the gain is followed block by block through
  the frame instead of one phase line. Receiver only, nothing changes on the air. 0 (default): off.

  Frequency offsets (--cfo, --cfo-random, --cfo-search) are in cycles per symbol: 1.0 is one symbol
  rate. --cfo-search MAX turns on the coarse frequency search (an FFT over the preamble) for offsets
  up to +-MAX (see docs/waveform-w0.md for its cost). --cfo-bins is the older narrow search; give
  one or the other.
";

#[derive(Debug, Clone, PartialEq)]
pub struct BerArgs {
    pub modulation: Modulation,
    pub frame: FrameFormat,
    pub code: PayloadCode,
    pub ebn0_db: Vec<f64>,
    pub bits: u64,
    pub seed: u64,
    pub sps: usize,
    pub payload_len: usize,
    pub cfo: f64,
    pub cfo_random: f64,
    pub random_frac_delay: bool,
    pub cfo_bins: usize,
    pub cfo_search: f64,
    /// Fading model and its settings (B9): see `ChannelArgs`.
    pub channel: ChannelArgs,
    /// Receiver channel tracking for fading up to this Doppler, Hz (0: off; B12, docs/phase-tracking.md).
    pub phase_track_hz: f64,
}

/// `--channel awgn|tworay|rayleigh` and its options, in physical units (the sample rate comes from
/// `--symbol-rate` times `--sps` for `ber`/`per`, from `--sample-rate` for `cable`).
#[derive(Debug, Clone, PartialEq)]
pub struct ChannelArgs {
    pub kind: String,
    pub doppler_hz: f64,
    pub ray_delay_us: f64,
    pub ray_gain_db: f64,
    /// Symbols per second (`ber`/`per` only; the cable has its own `--sample-rate`).
    pub symbol_rate: f64,
    /// Time from one frame's start to the next, ms (`ber`/`per` only).
    pub frame_interval_ms: f64,
}

impl Default for ChannelArgs {
    fn default() -> Self {
        Self {
            kind: "awgn".into(),
            doppler_hz: 0.0,
            ray_delay_us: 1.0,
            ray_gain_db: -3.0,
            symbol_rate: 250_000.0,
            frame_interval_ms: 10.0,
        }
    }
}

impl ChannelArgs {
    /// The model at `sample_rate` samples per second.
    pub fn fading(&self, sample_rate: f64) -> oasa_waveform::fading::Fading {
        use oasa_waveform::fading::Fading;
        let doppler = self.doppler_hz / sample_rate;
        match self.kind.as_str() {
            "tworay" => Fading::TwoRay {
                delay: self.ray_delay_us * 1e-6 * sample_rate,
                gain_db: self.ray_gain_db,
                doppler,
            },
            "rayleigh" => Fading::Rayleigh { doppler },
            _ => Fading::None,
        }
    }

    /// `,"channel":...` for a JSON line, or nothing for AWGN (so the default output is unchanged).
    pub fn json(&self) -> String {
        match self.kind.as_str() {
            "awgn" => String::new(),
            "tworay" => format!(
                ",\"channel\":\"tworay\",\"ray_delay_us\":{},\"ray_gain_db\":{},\"doppler_hz\":{},\"symbol_rate\":{},\"frame_interval_ms\":{}",
                num(self.ray_delay_us), num(self.ray_gain_db), num(self.doppler_hz), num(self.symbol_rate), num(self.frame_interval_ms)
            ),
            _ => format!(
                ",\"channel\":\"rayleigh\",\"doppler_hz\":{},\"symbol_rate\":{},\"frame_interval_ms\":{}",
                num(self.doppler_hz), num(self.symbol_rate), num(self.frame_interval_ms)
            ),
        }
    }
}

/// Reads `--channel` and its options (shared by `ber`, `per` and `cable`).
fn parse_channel(named: &mut BTreeMap<String, String>, with_rate: bool) -> R<ChannelArgs> {
    let d = ChannelArgs::default();
    let kind = named.remove("channel").unwrap_or_else(|| "awgn".into());
    if !matches!(kind.as_str(), "awgn" | "tworay" | "rayleigh") {
        return Err(format!("--channel {kind:?}: use awgn, tworay or rayleigh"));
    }
    let c = ChannelArgs {
        doppler_hz: opt_float("doppler-hz", named.remove("doppler-hz"), 0.0, 0.0, 1e6)?,
        ray_delay_us: opt_float(
            "ray-delay-us",
            named.remove("ray-delay-us"),
            d.ray_delay_us,
            0.0,
            1e4,
        )?,
        ray_gain_db: opt_float(
            "ray-gain-db",
            named.remove("ray-gain-db"),
            d.ray_gain_db,
            -60.0,
            0.0,
        )?,
        symbol_rate: if with_rate {
            opt_float(
                "symbol-rate",
                named.remove("symbol-rate"),
                d.symbol_rate,
                1.0,
                1e9,
            )?
        } else {
            d.symbol_rate
        },
        frame_interval_ms: if with_rate {
            opt_float(
                "frame-interval-ms",
                named.remove("frame-interval-ms"),
                d.frame_interval_ms,
                0.0,
                1e6,
            )?
        } else {
            d.frame_interval_ms
        },
        kind,
    };
    if c.kind == "rayleigh" && c.doppler_hz <= 0.0 {
        return Err("--channel rayleigh needs --doppler-hz above 0".into());
    }
    Ok(c)
}

/// `per`: the options of `ber` (its `bits` unused), the number of frames and the sync tolerance.
#[derive(Debug, Clone, PartialEq)]
pub struct PerArgs {
    pub ber: BerArgs,
    pub frames: u64,
    pub sync_max_errors: u32,
    /// `--diversity`: None, not given (one antenna, output as before); `Some(None)`, `off` (one
    /// antenna, with the receiver's time per frame in the output); `Some(Some(c))`, two antennas.
    pub diversity: Option<Option<Combine>>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct IqArgs {
    pub modulation: Modulation,
    pub frame: FrameFormat,
    pub code: PayloadCode,
    pub out: String,
    pub ebn0_db: Option<f64>,
    pub payload: Payload,
    pub seed: u64,
    pub sps: usize,
    pub symbol_rate: f64,
    pub center_hz: f64,
    pub phase_deg: f64,
    pub delay: usize,
    pub tail: usize,
    pub cfo: f64,
    pub frac_delay: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Payload {
    Random(usize),
    Bytes(Vec<u8>),
}

#[derive(Debug, Clone, PartialEq)]
pub struct DecodeArgs {
    pub modulation: Modulation,
    pub frame: FrameFormat,
    /// What the sender codes the body with; used only with `payload_len` (a receiver reads the
    /// code from each header).
    pub code: PayloadCode,
    pub input: String,
    pub sps: usize,
    pub cfo_bins: usize,
    pub cfo_search: f64,
    pub hex: bool,
    /// Take the payload length as known instead of reading it from the header (BER measurement).
    pub payload_len: Option<usize>,
    pub sync_max_errors: u32,
}

#[derive(Debug, Clone, PartialEq)]
pub enum WaveformCommand {
    Help,
    Ber(BerArgs),
    Per(PerArgs),
    Iq(IqArgs),
    Decode(DecodeArgs),
    /// Decode a continuous stream in blocks of `block` samples (`input` is "-" for stdin or a raw
    /// cf32_le file).
    RxStream(DecodeArgs, usize),
    Df(DfArgs),
    /// A simulated cable between two `sdr` radios' streams (src/cable.rs).
    Cable {
        input: String,
        output: String,
        cfg: crate::cable::CableConfig,
    },
}

#[derive(Debug, Clone, PartialEq)]
pub struct DfArgs {
    pub in1: String,
    pub in2: String,
    /// true: cf32_le; false: ci16_le.
    pub cf32: bool,
    /// Fixed blocks of this many samples instead of finding bursts.
    pub block: Option<usize>,
    pub burst_db: f64,
    pub win: usize,
    pub min_len: usize,
    pub skip: usize,
    pub cal_deg: f64,
    pub spacing_wl: Option<f64>,
}

type R<T> = Result<T, String>;

/// Largest Doppler the channel tracker is set up for, as a fraction of the symbol rate.
pub const MAX_TRACK_CYCLES: f64 = crate::adapters::sdr::MAX_PHASE_TRACK;

/// Parse the arguments after `waveform`.
pub fn parse(args: &[String]) -> R<WaveformCommand> {
    let sub = match args.first().map(String::as_str) {
        None | Some("-h" | "--help" | "help") => return Ok(WaveformCommand::Help),
        Some(s @ ("ber" | "per" | "iq" | "decode" | "rx-stream" | "df" | "cable")) => s,
        Some(other) => {
            return Err(format!(
            "unknown waveform command {other:?}; use ber, per, iq, decode, rx-stream, df or cable"
        ))
        }
    };
    let mut named: BTreeMap<String, String> = BTreeMap::new();
    let mut flags: Vec<String> = Vec::new();
    let mut it = args[1..].iter();
    while let Some(a) = it.next() {
        let k = a
            .strip_prefix("--")
            .ok_or(format!("unexpected argument {a:?}"))?;
        match (sub, k) {
            (_, "help") => return Ok(WaveformCommand::Help),
            ("ber" | "per", "json" | "random-frac-delay") | ("decode" | "rx-stream", "hex") => {
                flags.push(k.to_string())
            }
            _ => {
                let v = it.next().ok_or(format!("--{k} needs a value"))?;
                if named.insert(k.to_string(), v.clone()).is_some() {
                    return Err(format!("--{k} given twice"));
                }
            }
        }
    }
    if sub == "df" {
        return parse_df(named);
    }
    if sub == "cable" {
        return parse_cable(named);
    }
    let modulation = {
        let v = named.remove("mod").ok_or("missing --mod")?;
        Modulation::parse(&v).ok_or(format!("--mod {v:?}: use bpsk, qpsk or fsk2"))?
    };
    let frame = match named.remove("frame") {
        None => FrameFormat::W0,
        Some(v) => FrameFormat::parse(&v).ok_or(format!("--frame {v:?}: use w0 or v1"))?,
    };
    let code = match named.remove("code") {
        None => PayloadCode::None,
        Some(v) => PayloadCode::parse(&v).ok_or(format!("--code {v:?}: use none or light"))?,
    };
    if code != PayloadCode::None && frame != FrameFormat::V1 {
        return Err("--code light needs --frame v1 (W0 has no payload_code)".into());
    }
    let sps = int("sps", named.remove("sps"), 8, 4, 64)? as usize;
    // Frequencies in cycles per symbol stay inside the sampled band, +-sps/2.
    let half = sps as f64 / 2.0;
    let cmd = match sub {
        "ber" | "per" => {
            let list = named.remove("ebn0-db").ok_or("missing --ebn0-db")?;
            let ebn0_db = list
                .split(',')
                .map(|x| float("ebn0-db", x, -30.0, 60.0))
                .collect::<R<Vec<f64>>>()?;
            if ebn0_db.is_empty() || ebn0_db.len() > 64 {
                return Err("--ebn0-db takes 1 to 64 values".into());
            }
            let (bits, frames, sync_max_errors) = if sub == "ber" {
                (
                    int("bits", named.remove("bits"), 100_000, 1, 10_000_000_000)?,
                    0,
                    3,
                )
            } else {
                (
                    0,
                    int("frames", named.remove("frames"), 400, 1, 100_000_000)?,
                    int("sync-max-errors", named.remove("sync-max-errors"), 3, 0, 16)? as u32,
                )
            };
            let diversity = match (sub, named.remove("diversity")) {
                (_, None) => None,
                ("per", Some(v)) if v == "off" => Some(None),
                ("per", Some(v)) => Some(Some(
                    Combine::parse(&v)
                        .ok_or(format!("--diversity {v:?}: use off, select, snr or mrc"))?,
                )),
                (_, Some(_)) => return Err("--diversity is for per only".into()),
            };
            let a = BerArgs {
                modulation,
                frame,
                code,
                ebn0_db,
                bits,
                seed: int("seed", named.remove("seed"), 1, 0, u64::MAX)?,
                sps,
                payload_len: int("payload-len", named.remove("payload-len"), 64, 1, 255)? as usize,
                cfo: opt_float("cfo", named.remove("cfo"), 0.0, -half, half)?,
                cfo_random: opt_float("cfo-random", named.remove("cfo-random"), 0.0, 0.0, half)?,
                random_frac_delay: flags.iter().any(|f| f == "random-frac-delay"),
                cfo_bins: int("cfo-bins", named.remove("cfo-bins"), 0, 0, 20)? as usize,
                cfo_search: opt_float("cfo-search", named.remove("cfo-search"), 0.0, 0.0, half)?,
                channel: parse_channel(&mut named, true)?,
                phase_track_hz: opt_float(
                    "phase-track-hz",
                    named.remove("phase-track-hz"),
                    0.0,
                    0.0,
                    1e6,
                )?,
            };
            if sub == "ber" {
                WaveformCommand::Ber(a)
            } else {
                WaveformCommand::Per(PerArgs {
                    ber: a,
                    frames,
                    sync_max_errors,
                    diversity,
                })
            }
        }
        "iq" => {
            let payload = match (named.remove("payload-hex"), named.remove("payload-len")) {
                (Some(_), Some(_)) => {
                    return Err("give --payload-hex or --payload-len, not both".into())
                }
                (Some(h), None) => Payload::Bytes(unhex(&h)?),
                (None, l) => Payload::Random(int("payload-len", l, 32, 0, 255)? as usize),
            };
            if let Payload::Bytes(b) = &payload {
                if b.len() > 255 {
                    return Err("--payload-hex is longer than 255 bytes".into());
                }
            }
            WaveformCommand::Iq(IqArgs {
                modulation,
                frame,
                code,
                out: named.remove("out").ok_or("missing --out")?,
                ebn0_db: match named.remove("ebn0-db") {
                    None => None,
                    Some(v) => Some(float("ebn0-db", &v, -30.0, 80.0)?),
                },
                payload,
                seed: int("seed", named.remove("seed"), 1, 0, u64::MAX)?,
                sps,
                symbol_rate: opt_float(
                    "symbol-rate",
                    named.remove("symbol-rate"),
                    25_000.0,
                    1.0,
                    1e9,
                )?,
                center_hz: opt_float("center-hz", named.remove("center-hz"), 0.0, 0.0, 1e12)?,
                phase_deg: opt_float("phase-deg", named.remove("phase-deg"), 0.0, -360.0, 360.0)?,
                delay: int("delay", named.remove("delay"), 64, 0, 10_000_000)? as usize,
                tail: int("tail", named.remove("tail"), 64, 0, 10_000_000)? as usize,
                cfo: opt_float("cfo", named.remove("cfo"), 0.0, -half, half)?,
                frac_delay: opt_float("frac-delay", named.remove("frac-delay"), 0.0, 0.0, 0.999)?,
            })
        }
        _ => {
            let block = if sub == "rx-stream" {
                Some(int("block", named.remove("block"), 65_536, 1024, 16_777_216)? as usize)
            } else {
                None
            };
            let d = DecodeArgs {
                modulation,
                frame,
                code,
                input: named.remove("in").ok_or("missing --in")?,
                sps,
                cfo_bins: int("cfo-bins", named.remove("cfo-bins"), 0, 0, 20)? as usize,
                cfo_search: opt_float("cfo-search", named.remove("cfo-search"), 0.0, 0.0, half)?,
                hex: flags.iter().any(|f| f == "hex"),
                payload_len: match named.remove("payload-len") {
                    None => None,
                    v => Some(int("payload-len", v, 0, 0, 255)? as usize),
                },
                sync_max_errors: int("sync-max-errors", named.remove("sync-max-errors"), 3, 0, 16)?
                    as u32,
            };
            match block {
                Some(b) => WaveformCommand::RxStream(d, b),
                None => WaveformCommand::Decode(d),
            }
        }
    };
    match &cmd {
        WaveformCommand::Ber(a) | WaveformCommand::Per(PerArgs { ber: a, .. }) => {
            if a.cfo.abs() + a.cfo_random > half {
                return Err(format!(
                    "--cfo plus --cfo-random must stay within +-{half} (half the sample rate)"
                ));
            }
            if a.cfo_bins > 0 && a.cfo_search > 0.0 {
                return Err("give --cfo-bins or --cfo-search, not both".into());
            }
            if a.phase_track_hz > 0.0 && a.modulation == Modulation::Fsk2 {
                return Err("--phase-track-hz is for bpsk and qpsk".into());
            }
            if a.phase_track_hz > MAX_TRACK_CYCLES * a.channel.symbol_rate {
                return Err(format!(
                    "--phase-track-hz must stay within {MAX_TRACK_CYCLES} of --symbol-rate"
                ));
            }
        }
        WaveformCommand::Decode(a) | WaveformCommand::RxStream(a, _)
            if a.cfo_bins > 0 && a.cfo_search > 0.0 =>
        {
            return Err("give --cfo-bins or --cfo-search, not both".into());
        }
        _ => {}
    }
    if let Some(k) = named.keys().next() {
        return Err(format!("unknown option --{k} for waveform {sub}"));
    }
    Ok(cmd)
}

fn parse_df(mut named: BTreeMap<String, String>) -> R<WaveformCommand> {
    let cf32 = match named.remove("format").as_deref() {
        None | Some("ci16") => false,
        Some("cf32") => true,
        Some(v) => return Err(format!("--format {v:?}: use ci16 or cf32")),
    };
    let a = DfArgs {
        in1: named.remove("in1").ok_or("missing --in1")?,
        in2: named.remove("in2").ok_or("missing --in2")?,
        cf32,
        block: match named.remove("block") {
            None => None,
            v => Some(int("block", v, 0, 16, 1 << 28)? as usize),
        },
        burst_db: opt_float("burst-db", named.remove("burst-db"), 10.0, 1.0, 60.0)?,
        win: int("win", named.remove("win"), 256, 8, 1 << 20)? as usize,
        min_len: int("min-len", named.remove("min-len"), 4096, 16, 1 << 28)? as usize,
        skip: int("skip", named.remove("skip"), 0, 0, 1 << 32)? as usize,
        cal_deg: opt_float("cal-deg", named.remove("cal-deg"), 0.0, -360.0, 360.0)?,
        spacing_wl: match named.remove("spacing-wl") {
            None => None,
            Some(v) => Some(float("spacing-wl", &v, 0.01, 10.0)?),
        },
    };
    if let Some(k) = named.keys().next() {
        return Err(format!("unknown option --{k} for waveform df"));
    }
    Ok(WaveformCommand::Df(a))
}

fn parse_cable(mut named: BTreeMap<String, String>) -> R<WaveformCommand> {
    let input = named.remove("in").ok_or("missing --in")?;
    let output = named.remove("out").ok_or("missing --out")?;
    let d = crate::cable::CableConfig::default();
    let cfg = crate::cable::CableConfig {
        gain_db: opt_float("gain-db", named.remove("gain-db"), d.gain_db, -200.0, 60.0)?,
        noise_dbfs: opt_float(
            "noise-dbfs",
            named.remove("noise-dbfs"),
            d.noise_dbfs,
            -200.0,
            0.0,
        )?,
        cfo_hz: opt_float("cfo-hz", named.remove("cfo-hz"), d.cfo_hz, -1e6, 1e6)?,
        sample_rate: opt_float(
            "sample-rate",
            named.remove("sample-rate"),
            d.sample_rate,
            1.0,
            1e9,
        )?,
        seed: int("seed", named.remove("seed"), d.seed, 0, u64::MAX)?,
        fading: Default::default(),
    };
    let ch = parse_channel(&mut named, false)?;
    let cfg = crate::cable::CableConfig {
        fading: ch.fading(cfg.sample_rate),
        ..cfg
    };
    if let Some(k) = named.keys().next() {
        return Err(format!("unknown option --{k} for cable"));
    }
    Ok(WaveformCommand::Cable { input, output, cfg })
}

fn run_cable(
    input: &str,
    output: &str,
    cfg: &crate::cable::CableConfig,
    out: &mut dyn Write,
) -> R<()> {
    let mut i = std::fs::File::open(input).map_err(|e| format!("{input}: {e}"))?;
    let mut o = std::fs::OpenOptions::new()
        .write(true)
        .open(output)
        .map_err(|e| format!("{output}: {e}"))?;
    let n = match crate::cable::run(cfg, &mut i, &mut o) {
        Ok(n) => n,
        // the receiving side went away: the end of the run, not an error
        Err(e) if e.kind() == std::io::ErrorKind::BrokenPipe => 0,
        Err(e) => return Err(format!("cable: {e}")),
    };
    writeln!(
        out,
        "{{\"ts\":\"{}\",\"event\":\"cable_done\",\"samples\":{n}}}",
        now()
    )
    .map_err(io)
}

fn int(k: &str, v: Option<String>, default: u64, min: u64, max: u64) -> R<u64> {
    match v {
        None => Ok(default),
        Some(v) => v
            .parse::<u64>()
            .ok()
            .filter(|n| (min..=max).contains(n))
            .ok_or(format!("--{k} must be a whole number, {min} to {max}")),
    }
}

fn float(k: &str, v: &str, min: f64, max: f64) -> R<f64> {
    v.trim()
        .parse::<f64>()
        .ok()
        .filter(|x| x.is_finite() && (min..=max).contains(x))
        .ok_or(format!("--{k}: {v:?} is not a number from {min} to {max}"))
}

fn opt_float(k: &str, v: Option<String>, default: f64, min: f64, max: f64) -> R<f64> {
    match v {
        None => Ok(default),
        Some(v) => float(k, &v, min, max),
    }
}

fn unhex(s: &str) -> R<Vec<u8>> {
    if s.len() % 2 != 0 || !s.is_ascii() {
        return Err("--payload-hex needs an even number of hex digits".into());
    }
    (0..s.len() / 2)
        .map(|i| {
            u8::from_str_radix(&s[2 * i..2 * i + 2], 16)
                .map_err(|_| "--payload-hex: not hex".to_string())
        })
        .collect()
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02x}")).collect()
}

/// A JSON number, or `null` for NaN and infinity.
fn num(x: f64) -> String {
    if x.is_finite() {
        format!("{x:e}")
    } else {
        "null".to_string()
    }
}

fn now() -> String {
    rfc3339(SystemTime::now())
}

pub fn run(cmd: &WaveformCommand, out: &mut dyn Write) -> R<()> {
    match cmd {
        WaveformCommand::Help => write!(out, "{USAGE}").map_err(|e| e.to_string()),
        WaveformCommand::Ber(a) => run_ber_cmd(a, out),
        WaveformCommand::Per(a) => run_per_cmd(a, out),
        WaveformCommand::Iq(a) => run_iq(a, out),
        WaveformCommand::Decode(a) => run_decode(a, out),
        WaveformCommand::RxStream(a, block) => run_rx_stream(a, *block, out),
        WaveformCommand::Df(a) => run_df(a, out),
        WaveformCommand::Cable { input, output, cfg } => run_cable(input, output, cfg, out),
    }
}

fn opt_num(x: Option<f64>) -> String {
    x.map_or("null".to_string(), num)
}

/// Phase difference per burst (or block) of a two-channel capture, then the summary.
fn run_df(a: &DfArgs, out: &mut dyn Write) -> R<()> {
    let load = |p: &str| -> R<Vec<oasa_waveform::Complex32>> {
        let b = std::fs::read(p).map_err(|e| format!("{p}: {e}"))?;
        let mut v = if a.cf32 {
            from_cf32_le(&b)
        } else {
            from_ci16_le(&b)
        };
        v.drain(..a.skip.min(v.len()));
        Ok(v)
    };
    let (x1, x2) = (load(&a.in1)?, load(&a.in2)?);
    if x1.len() != x2.len() {
        return Err(format!(
            "the two channels differ in length ({} and {} samples after --skip): not one stream?",
            x1.len(),
            x2.len()
        ));
    }
    let noise = (df::noise_floor(&x1, a.win), df::noise_floor(&x2, a.win));
    let segments = match a.block {
        Some(b) => df::fixed_blocks(x1.len(), b),
        None => df::find_bursts(&x1, &x2, a.win, a.burst_db, a.min_len),
    };
    let mut phases = Vec::with_capacity(segments.len());
    let mut ratios = Vec::with_capacity(segments.len());
    for &(s, len) in &segments {
        // with fixed blocks the noise floor is not known (all signal)
        let m = df::measure(&x1, &x2, s, len, a.block.is_none().then_some(noise));
        let corrected = df::wrap_deg(m.phase_deg - a.cal_deg);
        let doa = a
            .spacing_wl
            .and_then(|d| df::doa_deg(m.phase_deg, a.cal_deg, d));
        writeln!(
            out,
            "{{\"ts\":\"{}\",\"event\":\"segment\",\"start\":{},\"len\":{},\"phase_deg\":{},\"phase_cal_deg\":{},\"coherence\":{},\"ratio_db\":{},\"snr1_db\":{},\"snr2_db\":{},\"doa_deg\":{}}}",
            now(),
            m.start + a.skip,
            m.len,
            num(m.phase_deg),
            num(corrected),
            num(m.coherence),
            num(m.ratio_db),
            opt_num(m.snr1_db),
            opt_num(m.snr2_db),
            opt_num(doa),
        )
        .map_err(io)?;
        phases.push(corrected);
        ratios.push(m.ratio_db);
    }
    let stats = df::circ_stats_deg(&phases);
    let mean_ratio = (!ratios.is_empty()).then(|| ratios.iter().sum::<f64>() / ratios.len() as f64);
    let doa = match (stats, a.spacing_wl) {
        (Some((m, _)), Some(d)) => df::doa_deg(m, 0.0, d),
        _ => None,
    };
    writeln!(
        out,
        "{{\"ts\":\"{}\",\"event\":\"df_done\",\"samples\":{},\"segments\":{},\"phase_mean_deg\":{},\"phase_std_deg\":{},\"ratio_mean_db\":{},\"noise1\":{},\"noise2\":{},\"cal_deg\":{},\"spacing_wl\":{},\"doa_mean_deg\":{}}}",
        now(),
        x1.len(),
        segments.len(),
        opt_num(stats.map(|s| s.0)),
        opt_num(stats.map(|s| s.1)),
        opt_num(mean_ratio),
        num(noise.0),
        num(noise.1),
        num(a.cal_deg),
        opt_num(a.spacing_wl),
        opt_num(doa),
    )
    .map_err(io)?;
    out.flush().map_err(io)
}

fn io(e: std::io::Error) -> String {
    e.to_string()
}

fn ber_options(a: &BerArgs) -> BerOptions {
    BerOptions {
        sps: a.sps,
        frame: a.frame,
        code: a.code,
        payload_len: a.payload_len,
        cfo_cycles_per_symbol: a.cfo,
        cfo_random: a.cfo_random,
        random_frac_delay: a.random_frac_delay,
        rx: RxConfig {
            cfo_bins: a.cfo_bins,
            cfo_search: a.cfo_search as f32,
            phase_track: (a.phase_track_hz / a.channel.symbol_rate) as f32,
            ..RxConfig::default()
        },
        fading: a.channel.fading(a.channel.symbol_rate * a.sps as f64),
        frame_interval: a.channel.frame_interval_ms * 1e-3 * a.channel.symbol_rate * a.sps as f64,
        ..BerOptions::default()
    }
}

/// `,"phase_track_hz":...` when the tracker is on (so the default output is unchanged).
fn track_json(a: &BerArgs) -> String {
    if a.phase_track_hz > 0.0 {
        format!(",\"phase_track_hz\":{}", num(a.phase_track_hz))
    } else {
        String::new()
    }
}

fn run_ber_cmd(a: &BerArgs, out: &mut dyn Write) -> R<()> {
    let opts = ber_options(a);
    let theory = match (a.code, a.modulation) {
        (PayloadCode::Light, _) => "none: coded, Eb per information bit (docs/fec-light.md)",
        (_, Modulation::Fsk2) => "noncoherent_bfsk: 0.5*exp(-Eb/2N0)",
        _ => "coherent_psk: Q(sqrt(2Eb/N0))",
    };
    for &db in &a.ebn0_db {
        let p = run_ber(a.modulation, db, a.bits, a.seed, &opts).map_err(|e| e.to_string())?;
        writeln!(
            out,
            "{{\"ts\":\"{}\",\"event\":\"ber\",\"mod\":\"{}\",\"ebn0_db\":{},\"bits\":{},\"errors\":{},\"ber\":{},\"theory\":{},\"ci_lo\":{},\"ci_hi\":{},\"z\":{},\"frame\":\"{}\",\"code\":\"{}\",\"frames\":{},\"detected\":{},\"crc_ok\":{}{},\"seed\":{},\"sps\":{},\"payload_len\":{},\"cfo_cycles_per_symbol\":{},\"cfo_random\":{},\"random_frac_delay\":{},\"cfo_bins\":{},\"cfo_search\":{},\"theory_kind\":{}{}{}}}",
            now(),
            a.modulation.name(),
            num(db),
            p.bits,
            p.errors,
            num(p.ber),
            num(p.theory),
            num(p.ci_lo),
            num(p.ci_hi),
            num(p.z),
            a.frame.name(),
            a.code.name(),
            p.frames,
            p.detected,
            p.crc_ok,
            if a.frame == FrameFormat::V1 {
                format!(",\"header_ok\":{}", p.header_ok)
            } else {
                String::new()
            },
            a.seed,
            a.sps,
            a.payload_len,
            num(a.cfo),
            num(a.cfo_random),
            a.random_frac_delay,
            a.cfo_bins,
            num(a.cfo_search),
            json_str(theory),
            a.channel.json(),
            track_json(a),
        )
        .map_err(io)?;
        out.flush().map_err(io)?;
    }
    Ok(())
}

fn run_per_cmd(p: &PerArgs, out: &mut dyn Write) -> R<()> {
    let a = &p.ber;
    let mut opts = ber_options(a);
    opts.rx.sync_max_errors = p.sync_max_errors;
    opts.diversity = p.diversity.flatten();
    for &db in &a.ebn0_db {
        let r = run_found(a.modulation, db, p.frames, a.seed, &opts).map_err(|e| e.to_string())?;
        let lost = r.frames - r.crc_ok;
        let (lo, hi) = oasa_waveform::theory::wilson(lost, r.frames, 3.0);
        let c = &r.report;
        writeln!(
            out,
            "{{\"ts\":\"{}\",\"event\":\"per\",\"mod\":\"{}\",\"frame\":\"{}\",\"code\":\"{}\",\"ebn0_db\":{},\"frames\":{},\"found\":{},\"delivered\":{},\"packet_loss\":{},\"ci_lo\":{},\"ci_hi\":{},\"preambles\":{},\"sync_rejected\":{},\"header_rejected\":{},\"too_short\":{},\"inside_frame\":{},\"seed\":{},\"sps\":{},\"payload_len\":{},\"frame_symbols\":{},\"cfo_cycles_per_symbol\":{},\"cfo_random\":{},\"random_frac_delay\":{},\"cfo_bins\":{},\"cfo_search\":{},\"sync_max_errors\":{}{}{}{}}}",
            now(),
            a.modulation.name(),
            a.frame.name(),
            a.code.name(),
            num(db),
            r.frames,
            r.found,
            r.crc_ok,
            num(lost as f64 / r.frames as f64),
            num(lo),
            num(hi),
            c.preambles,
            c.sync_rejected,
            c.header_rejected,
            c.too_short,
            c.inside_frame,
            a.seed,
            a.sps,
            a.payload_len,
            modem_for(a.modulation, a.frame, a.code, a.sps)?.frame_symbols(a.payload_len),
            num(a.cfo),
            num(a.cfo_random),
            a.random_frac_delay,
            a.cfo_bins,
            num(a.cfo_search),
            p.sync_max_errors,
            a.channel.json(),
            track_json(a),
            match p.diversity {
                None => String::new(),
                Some(d) => format!(
                    ",\"diversity\":\"{}\",\"rx_us_per_frame\":{}",
                    d.map_or("off", |c| c.name()),
                    num(r.rx_seconds * 1e6 / r.frames as f64)
                ),
            },
        )
        .map_err(io)?;
        out.flush().map_err(io)?;
    }
    Ok(())
}

fn modem_for(m: Modulation, frame: FrameFormat, code: PayloadCode, sps: usize) -> R<Modem> {
    Modem::new(ModemConfig {
        sps,
        frame,
        code,
        ..ModemConfig::new(m)
    })
    .map_err(|e| e.to_string())
}

/// `BASE`, from `BASE`, `BASE.sigmf-meta` or `BASE.sigmf-data`.
fn base_path(s: &str) -> PathBuf {
    let s = s
        .strip_suffix(".sigmf-meta")
        .or_else(|| s.strip_suffix(".sigmf-data"))
        .unwrap_or(s);
    PathBuf::from(s)
}

fn with_ext(base: &std::path::Path, ext: &str) -> PathBuf {
    let mut p = base.as_os_str().to_owned();
    p.push(ext);
    PathBuf::from(p)
}

fn run_iq(a: &IqArgs, out: &mut dyn Write) -> R<()> {
    let modem = modem_for(a.modulation, a.frame, a.code, a.sps)?;
    let mut rng = Rng::new(a.seed);
    let payload = match &a.payload {
        Payload::Random(n) => rng.bytes(*n),
        Payload::Bytes(b) => b.clone(),
    };
    let tx = modem.modulate(&payload).map_err(|e| e.to_string())?;
    let n0 = a
        .ebn0_db
        .map(|db| n0_for_ebn0(db, modem.energy_per_bit()))
        .unwrap_or(0.0);
    let mut ch = Channel::new(a.seed ^ 0x5eed_5eed);
    let imp = Impairments {
        n0,
        phase_rad: a.phase_deg.to_radians(),
        cfo_cycles_per_sample: a.cfo / a.sps as f64,
        frac_delay: a.frac_delay,
        lead: a.delay,
        tail: a.tail,
    };
    let y = ch.apply(&tx, &imp);
    let base = base_path(&a.out);
    let data_path = with_ext(&base, ".sigmf-data");
    let meta_path = with_ext(&base, ".sigmf-meta");
    std::fs::write(&data_path, to_cf32_le(&y))
        .map_err(|e| format!("{}: {e}", data_path.display()))?;
    let rate = a.symbol_rate * a.sps as f64;
    let ts = now();
    let desc = format!(
        "OASA waveform test frame ({}, code {}): {} with RRC (or tones for fsk2), {} bytes, simulated channel",
        a.frame.name(),
        a.code.name(),
        a.modulation.name(),
        payload.len()
    );
    let meta = format!(
        "{{\n  \"global\": {{\n    \"core:datatype\": \"cf32_le\",\n    \"core:sample_rate\": {rate:?},\n    \"core:version\": \"1.2.0\",\n    \"core:num_channels\": 1,\n    \"core:description\": {},\n    \"core:recorder\": \"oasa-radio waveform iq\",\n    \"oasa:modulation\": \"{}\",\n    \"oasa:frame\": \"{}\",\n    \"oasa:payload_code\": \"{}\",\n    \"oasa:samples_per_symbol\": {},\n    \"oasa:symbol_rate\": {:?},\n    \"oasa:ebn0_db\": {},\n    \"oasa:simulated\": true\n  }},\n  \"captures\": [\n    {{ \"core:sample_start\": 0, \"core:frequency\": {:?}, \"core:datetime\": \"{}\" }}\n  ],\n  \"annotations\": [\n    {{ \"core:sample_start\": {}, \"core:sample_count\": {}, \"core:label\": {} }}\n  ]\n}}\n",
        json_str(&desc),
        a.modulation.name(),
        a.frame.name(),
        a.code.name(),
        a.sps,
        a.symbol_rate,
        a.ebn0_db.map(num).unwrap_or_else(|| "null".into()),
        a.center_hz,
        ts,
        a.delay,
        tx.len(),
        json_str(&format!("frame {} payload {} bytes", a.modulation.name(), payload.len())),
    );
    std::fs::write(&meta_path, meta).map_err(|e| format!("{}: {e}", meta_path.display()))?;
    // Eb is per information bit: a code of rate r puts r times fewer of them in a symbol
    let snr = a
        .ebn0_db
        .map(|db| {
            num(snr_per_sample_db(db, a.modulation.bits_per_symbol(), a.sps)
                + 10.0 * a.code.rate().log10())
        })
        .unwrap_or_else(|| "null".into());
    writeln!(
        out,
        "{{\"ts\":\"{ts}\",\"event\":\"iq\",\"mod\":\"{}\",\"frame\":\"{}\",\"code\":\"{}\",\"data\":{},\"meta\":{},\"samples\":{},\"sample_rate\":{rate:?},\"frame_start\":{},\"frame_samples\":{},\"frame_symbols\":{},\"payload_len\":{},\"ebn0_db\":{},\"snr_per_sample_db\":{snr},\"seed\":{}}}",
        a.modulation.name(),
        a.frame.name(),
        a.code.name(),
        json_str(&data_path.display().to_string()),
        json_str(&meta_path.display().to_string()),
        y.len(),
        a.delay,
        tx.len(),
        modem.frame_symbols(payload.len()),
        payload.len(),
        a.ebn0_db.map(num).unwrap_or_else(|| "null".into()),
        a.seed,
    )
    .map_err(io)?;
    Ok(())
}

/// `,"header":{...}` for a decoded v1 header.
fn header_json(h: &HeaderFields) -> String {
    let m = Modulation::from_code(h.modulation).map_or("reserved", Modulation::name);
    format!(
        ",\"header\":{{\"version\":{},\"extension\":{},\"modulation\":\"{m}\",\"payload_code\":{},\"pilots\":{},\"length\":{},\"channel_plan\":{},\"coexistence\":{}}}",
        h.version, h.extension, h.payload_code, h.pilots, h.length, h.channel_plan, h.coexistence
    )
}

/// One `frame` JSON line.
fn write_frame(out: &mut dyn Write, f: &RxFrame, hex_payload: bool) -> R<()> {
    let payload = if hex_payload {
        format!(",\"payload_hex\":\"{}\"", hex(&f.payload))
    } else {
        String::new()
    };
    let header = f.header.as_ref().map(header_json).unwrap_or_default();
    writeln!(
        out,
        "{{\"ts\":\"{}\",\"event\":\"frame\",\"start\":{},\"end\":{},\"len\":{},\"crc_ok\":{},\"sync_errors\":{},\"metric\":{},\"cfo_cycles_per_symbol\":{},\"phase_rad\":{},\"timing_frac\":{}{header}{payload}}}",
        now(),
        f.start,
        f.end,
        f.payload.len(),
        f.crc_ok,
        f.sync_errors,
        num(f64::from(f.metric)),
        num(f64::from(f.cfo_cycles_per_symbol)),
        num(f64::from(f.phase_rad)),
        num(f64::from(f.timing_frac)),
    )
    .map_err(io)
}

fn rx_config(a: &DecodeArgs) -> RxConfig {
    RxConfig {
        cfo_bins: a.cfo_bins,
        cfo_search: a.cfo_search as f32,
        expected_len: a.payload_len,
        sync_max_errors: a.sync_max_errors,
        ..RxConfig::default()
    }
}

/// Decode a continuous stream: read raw cf32_le from stdin or a file in blocks, print each frame as
/// soon as it is decided (flushed), then `stream_done`. Frame positions count from the stream's
/// first sample.
fn run_rx_stream(a: &DecodeArgs, block: usize, out: &mut dyn Write) -> R<()> {
    use std::io::Read;
    let modem = modem_for(a.modulation, a.frame, a.code, a.sps)?;
    let mut rx = StreamReceiver::new(modem, rx_config(a));
    let mut input: Box<dyn Read> = if a.input == "-" {
        Box::new(std::io::stdin().lock())
    } else {
        Box::new(std::fs::File::open(&a.input).map_err(|e| format!("{}: {e}", a.input))?)
    };
    let mut bytes = vec![0u8; block * 8];
    let (mut have, mut frames, mut good) = (0usize, 0u64, 0u64);
    loop {
        let n = input.read(&mut bytes[have..]).map_err(io)?;
        have += n;
        // a whole block, or the end: hand over the complete samples, keep a partial one
        if have == bytes.len() || (n == 0 && have >= 8) {
            let whole = have / 8 * 8;
            for f in rx.push(&from_cf32_le(&bytes[..whole])) {
                write_frame(out, &f, a.hex)?;
                frames += 1;
                good += f.crc_ok as u64;
            }
            out.flush().map_err(io)?;
            bytes.copy_within(whole..have, 0);
            have -= whole;
        }
        if n == 0 {
            break;
        }
    }
    for f in rx.finish() {
        write_frame(out, &f, a.hex)?;
        frames += 1;
        good += f.crc_ok as u64;
    }
    writeln!(
        out,
        "{{\"ts\":\"{}\",\"event\":\"stream_done\",\"mod\":\"{}\",\"frame\":\"{}\",\"samples\":{},\"frames\":{},\"crc_ok_frames\":{},\"trailing_bytes\":{}}}",
        now(),
        a.modulation.name(),
        a.frame.name(),
        rx.samples_seen(),
        frames,
        good,
        have,
    )
    .map_err(io)?;
    out.flush().map_err(io)
}

fn run_decode(a: &DecodeArgs, out: &mut dyn Write) -> R<()> {
    let data_path = with_ext(&base_path(&a.input), ".sigmf-data");
    let bytes = std::fs::read(&data_path).map_err(|e| format!("{}: {e}", data_path.display()))?;
    let samples = from_cf32_le(&bytes);
    let modem = modem_for(a.modulation, a.frame, a.code, a.sps)?;
    let rx = rx_config(a);
    let (frames, rep) = modem.demodulate_report(&samples, &rx);
    for f in &frames {
        write_frame(out, f, a.hex)?;
    }
    writeln!(
        out,
        "{{\"ts\":\"{}\",\"event\":\"decode_done\",\"mod\":\"{}\",\"frame\":\"{}\",\"samples\":{},\"frames\":{},\"crc_ok_frames\":{},\"preambles\":{},\"sync_rejected\":{},\"header_rejected\":{},\"too_short\":{},\"inside_frame\":{}}}",
        now(),
        a.modulation.name(),
        a.frame.name(),
        samples.len(),
        frames.len(),
        frames.iter().filter(|f| f.crc_ok).count(),
        rep.preambles,
        rep.sync_rejected,
        rep.header_rejected,
        rep.too_short,
        rep.inside_frame,
    )
    .map_err(io)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn v(s: &str) -> Vec<String> {
        s.split_whitespace().map(String::from).collect()
    }

    #[test]
    fn rx_stream_parses() {
        match parse(&v(
            "rx-stream --in - --mod qpsk --cfo-search 0.1 --payload-len 64 --hex",
        ))
        .unwrap()
        {
            WaveformCommand::RxStream(a, block) => {
                assert_eq!(
                    (a.input.as_str(), block, a.payload_len, a.hex),
                    ("-", 65_536, Some(64), true)
                );
                assert_eq!(a.cfo_search, 0.1);
            }
            other => panic!("{other:?}"),
        }
        match parse(&v("rx-stream --in x --mod bpsk --block 4096")).unwrap() {
            WaveformCommand::RxStream(_, block) => assert_eq!(block, 4096),
            other => panic!("{other:?}"),
        }
        assert!(parse(&v("rx-stream --in - --mod bpsk --block 10")).is_err());
        assert!(parse(&v("rx-stream --mod bpsk")).is_err());
        assert!(parse(&v("decode --in x --mod bpsk --block 4096")).is_err());
        assert!(parse(&v(
            "rx-stream --in - --mod bpsk --cfo-bins 2 --cfo-search 0.1"
        ))
        .is_err());
    }

    #[test]
    fn ber_parses_with_defaults() {
        match parse(&v("ber --mod qpsk --ebn0-db 0,2.5,4")).unwrap() {
            WaveformCommand::Ber(a) => {
                assert_eq!(a.modulation, Modulation::Qpsk);
                assert_eq!(a.ebn0_db, vec![0.0, 2.5, 4.0]);
                assert_eq!((a.bits, a.seed, a.sps, a.payload_len), (100_000, 1, 8, 64));
            }
            other => panic!("{other:?}"),
        }
        assert!(matches!(
            parse(&v("ber --mod fsk2 --ebn0-db 3 --json --random-frac-delay")).unwrap(),
            WaveformCommand::Ber(BerArgs {
                random_frac_delay: true,
                ..
            })
        ));
    }

    #[test]
    fn phase_track_option() {
        match parse(&v("per --mod qpsk --ebn0-db 30 --phase-track-hz 200")).unwrap() {
            WaveformCommand::Per(p) => assert_eq!(p.ber.phase_track_hz, 200.0),
            other => panic!("{other:?}"),
        }
        match parse(&v("ber --mod bpsk --ebn0-db 3")).unwrap() {
            WaveformCommand::Ber(a) => {
                assert_eq!(a.phase_track_hz, 0.0);
                assert_eq!(ber_options(&a).rx.phase_track, 0.0);
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn diversity_option() {
        let div = |s: &str| match parse(&v(s)).unwrap() {
            WaveformCommand::Per(p) => p.diversity,
            other => panic!("{other:?}"),
        };
        assert_eq!(div("per --mod qpsk --ebn0-db 10"), None);
        assert_eq!(
            div("per --mod qpsk --ebn0-db 10 --diversity off"),
            Some(None)
        );
        assert_eq!(
            div("per --mod qpsk --ebn0-db 10 --diversity mrc"),
            Some(Some(Combine::Mrc))
        );
        assert_eq!(
            div("per --mod qpsk --ebn0-db 10 --diversity snr"),
            Some(Some(Combine::PreambleSnr))
        );
        for bad in [
            "per --mod qpsk --ebn0-db 10 --diversity both",
            "ber --mod qpsk --ebn0-db 10 --diversity mrc",
            "decode --in x --mod qpsk --diversity mrc",
        ] {
            assert!(parse(&v(bad)).is_err(), "{bad}");
        }
    }

    #[test]
    fn bad_arguments_are_refused() {
        for bad in [
            "ber --ebn0-db 3",
            "ber --mod bpsk",
            "ber --mod 8psk --ebn0-db 3",
            "ber --mod bpsk --ebn0-db x",
            "ber --mod bpsk --ebn0-db 3 --sps 2",
            "ber --mod bpsk --ebn0-db 3 --bits 0",
            "ber --mod bpsk --ebn0-db 3 --bogus 1",
            "ber --mod bpsk --mod qpsk --ebn0-db 3",
            "ber --mod bpsk --ebn0-db 3 --cfo 4.5",
            "ber --mod bpsk --ebn0-db 3 --sps 4 --cfo-search 2.5",
            "ber --mod bpsk --ebn0-db 3 --cfo-random -1",
            "ber --mod bpsk --ebn0-db 3 --cfo 3 --cfo-random 2",
            "ber --mod bpsk --ebn0-db 3 --cfo-bins 2 --cfo-search 1",
            "per --mod qpsk --ebn0-db 3 --phase-track-hz -1",
            "per --mod fsk2 --ebn0-db 3 --phase-track-hz 200",
            "per --mod qpsk --ebn0-db 3 --symbol-rate 250000 --phase-track-hz 2501",
            "decode --mod bpsk --in x --cfo-bins 2 --cfo-search 1",
            "iq --mod bpsk",
            "iq --mod bpsk --out x --payload-hex abc",
            "iq --mod bpsk --out x --payload-hex 00 --payload-len 4",
            "decode --mod bpsk",
            "frobnicate",
        ] {
            assert!(parse(&v(bad)).is_err(), "{bad:?} was accepted");
        }
        assert_eq!(parse(&[]).unwrap(), WaveformCommand::Help);
    }

    #[test]
    fn iq_and_decode_parse() {
        match parse(&v(
            "iq --mod bpsk --out /tmp/x --payload-hex 0aff --ebn0-db 12 --delay 10",
        ))
        .unwrap()
        {
            WaveformCommand::Iq(a) => {
                assert_eq!(a.payload, Payload::Bytes(vec![0x0a, 0xff]));
                assert_eq!(a.ebn0_db, Some(12.0));
                assert_eq!(a.delay, 10);
            }
            other => panic!("{other:?}"),
        }
        assert!(matches!(
            parse(&v("decode --mod qpsk --in /tmp/x --hex --cfo-bins 3")).unwrap(),
            WaveformCommand::Decode(DecodeArgs {
                hex: true,
                cfo_bins: 3,
                ..
            })
        ));
        match parse(&v(
            "decode --mod bpsk --in /tmp/x --payload-len 64 --sync-max-errors 16",
        ))
        .unwrap()
        {
            WaveformCommand::Decode(a) => {
                assert_eq!((a.payload_len, a.sync_max_errors), (Some(64), 16))
            }
            other => panic!("{other:?}"),
        }
        match parse(&v("decode --mod bpsk --in /tmp/x")).unwrap() {
            WaveformCommand::Decode(a) => assert_eq!((a.payload_len, a.sync_max_errors), (None, 3)),
            other => panic!("{other:?}"),
        }
        assert!(parse(&v("decode --mod bpsk --in /tmp/x --sync-max-errors 17")).is_err());
        assert!(parse(&v("decode --mod bpsk --in /tmp/x --payload-len 256")).is_err());
        match parse(&v(
            "ber --mod bpsk --ebn0-db 8 --cfo-random 2 --cfo-search 2.05",
        ))
        .unwrap()
        {
            WaveformCommand::Ber(a) => assert_eq!((a.cfo_random, a.cfo_search), (2.0, 2.05)),
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn frame_option_parses_and_defaults_to_w0() {
        match parse(&v("ber --mod bpsk --ebn0-db 2 --frame v1")).unwrap() {
            WaveformCommand::Ber(a) => assert_eq!(a.frame, FrameFormat::V1),
            other => panic!("{other:?}"),
        }
        match parse(&v("iq --mod bpsk --out x")).unwrap() {
            WaveformCommand::Iq(a) => assert_eq!(a.frame, FrameFormat::W0),
            other => panic!("{other:?}"),
        }
        match parse(&v("decode --mod qpsk --in x --frame v1")).unwrap() {
            WaveformCommand::Decode(a) => assert_eq!(a.frame, FrameFormat::V1),
            other => panic!("{other:?}"),
        }
        assert!(parse(&v("ber --mod bpsk --ebn0-db 2 --frame v2")).is_err());
        assert!(parse(&v("decode --mod bpsk --in x --frame")).is_err());
    }

    #[test]
    fn code_option_parses_and_needs_v1() {
        for cmd in [
            "ber --mod bpsk --ebn0-db 2 --frame v1 --code light",
            "per --mod bpsk --ebn0-db 2 --frame v1 --code light",
            "iq --mod qpsk --out x --frame v1 --code light",
            "decode --mod qpsk --in x --frame v1 --code light",
            "rx-stream --mod qpsk --in x --frame v1 --code light",
        ] {
            let code = match parse(&v(cmd)).unwrap() {
                WaveformCommand::Ber(a) | WaveformCommand::Per(PerArgs { ber: a, .. }) => a.code,
                WaveformCommand::Iq(a) => a.code,
                WaveformCommand::Decode(a) | WaveformCommand::RxStream(a, _) => a.code,
                other => panic!("{other:?}"),
            };
            assert_eq!(code, PayloadCode::Light, "{cmd}");
        }
        match parse(&v("ber --mod bpsk --ebn0-db 2 --frame v1")).unwrap() {
            WaveformCommand::Ber(a) => assert_eq!(a.code, PayloadCode::None),
            other => panic!("{other:?}"),
        }
        for bad in [
            "ber --mod bpsk --ebn0-db 2 --code light",
            "iq --mod bpsk --out x --frame w0 --code light",
            "decode --mod bpsk --in x --code light",
            "ber --mod bpsk --ebn0-db 2 --frame v1 --code heavy",
            "per --mod bpsk --ebn0-db 2 --bits 100",
            "ber --mod bpsk --ebn0-db 2 --frames 100",
            "per --mod bpsk --ebn0-db 2 --frames 0",
        ] {
            assert!(parse(&v(bad)).is_err(), "{bad:?} was accepted");
        }
        match parse(&v(
            "per --mod qpsk --ebn0-db 3,4 --frame v1 --code light --frames 50 --sync-max-errors 5",
        ))
        .unwrap()
        {
            WaveformCommand::Per(p) => {
                assert_eq!((p.frames, p.sync_max_errors), (50, 5));
                assert_eq!(p.ber.ebn0_db, vec![3.0, 4.0]);
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn header_json_names_the_fields() {
        let h = HeaderFields {
            channel_plan: 3,
            coexistence: 1,
            ..HeaderFields::new(1, 64)
        };
        assert_eq!(
            header_json(&h),
            ",\"header\":{\"version\":0,\"extension\":0,\"modulation\":\"qpsk\",\"payload_code\":0,\"pilots\":0,\"length\":64,\"channel_plan\":3,\"coexistence\":1}"
        );
    }

    #[test]
    fn json_numbers() {
        assert_eq!(num(f64::NAN), "null");
        assert_eq!(num(0.0), "0e0");
        assert!(num(1.5e-4).parse::<f64>().is_ok());
    }
}
