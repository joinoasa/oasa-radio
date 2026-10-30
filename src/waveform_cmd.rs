//! `oasa-radio waveform ...`: OASA's own waveform (milestone W0), in simulation.
//!
//! * `ber`: Monte Carlo bit error rate against theory, one JSON line per Eb/N0 point.
//! * `iq`: write a modulated, noisy frame as a SigMF pair (`cf32_le`) for the lab.
//! * `decode`: read such a file back and print the frames found.
//!
//! The payloads here are test data made from a seed. Payload bytes are printed only when asked for (`--hex`).

use std::collections::BTreeMap;
use std::io::Write;
use std::path::PathBuf;
use std::time::SystemTime;

use oasa_waveform::channel::{n0_for_ebn0, snr_per_sample_db, Channel, Impairments};
use oasa_waveform::iq::{from_cf32_le, to_cf32_le};
use oasa_waveform::rng::Rng;
use oasa_waveform::sim::{run_ber, BerOptions};
use oasa_waveform::{Modem, ModemConfig, Modulation, RxConfig};

use crate::log::{json_str, rfc3339};

pub const USAGE: &str = "\
WAVEFORM (W0, simulation):
  oasa-radio waveform ber --mod bpsk|qpsk|fsk2 --ebn0-db 0,2,4,6 [--bits N] [--seed S]
                          [--sps 8] [--payload-len 64] [--cfo CYCLES_PER_SYMBOL]
                          [--cfo-random MAX] [--random-frac-delay] [--cfo-bins 0]
                          [--cfo-search MAX] [--json]
  oasa-radio waveform iq --mod bpsk|qpsk|fsk2 --out BASE [--ebn0-db X] [--payload-len 32 | --payload-hex HEX]
                         [--seed S] [--sps 8] [--symbol-rate 25000] [--center-hz 0]
                         [--phase-deg 0] [--delay 64] [--tail 64] [--cfo 0] [--frac-delay 0]
  oasa-radio waveform decode --in BASE --mod bpsk|qpsk|fsk2 [--sps 8] [--cfo-bins 0]
                             [--cfo-search MAX] [--hex]

  ber     Sends random frames through the modem and the simulated channel at each Eb/N0 and prints one
          JSON line per point: errors, bits, BER, the theory value (BPSK and QPSK: Q(sqrt(2Eb/N0));
          2-FSK, noncoherent: 0.5 exp(-Eb/2N0)), a 3-sigma Wilson interval and the binomial z-score.
          Default 100000 bits, seed 1. The same seed gives the same lines. (--json is accepted; output
          is always JSON lines.) --cfo-random gives each frame its own offset, uniform in +-MAX.
  iq      Writes BASE.sigmf-data (cf32_le, little endian float32 I and Q) and BASE.sigmf-meta: one frame
          with BASE delay samples of noise in front and TAIL after. No --ebn0-db means no noise.
  decode  Finds frames in BASE.sigmf-data. Prints sizes and CRC status; the payload only with --hex.

  Frequency offsets (--cfo, --cfo-random, --cfo-search) are in cycles per symbol: 1.0 is one symbol
  rate. --cfo-search MAX turns on the coarse frequency search (an FFT over the preamble) for offsets
  up to +-MAX; it costs about 8 times the plain search. --cfo-bins is the older narrow search.
";

#[derive(Debug, Clone, PartialEq)]
pub struct BerArgs {
    pub modulation: Modulation,
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
}

#[derive(Debug, Clone, PartialEq)]
pub struct IqArgs {
    pub modulation: Modulation,
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
    pub input: String,
    pub sps: usize,
    pub cfo_bins: usize,
    pub cfo_search: f64,
    pub hex: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub enum WaveformCommand {
    Help,
    Ber(BerArgs),
    Iq(IqArgs),
    Decode(DecodeArgs),
}

type R<T> = Result<T, String>;

/// Parse the arguments after `waveform`.
pub fn parse(args: &[String]) -> R<WaveformCommand> {
    let sub = match args.first().map(String::as_str) {
        None | Some("-h" | "--help" | "help") => return Ok(WaveformCommand::Help),
        Some(s @ ("ber" | "iq" | "decode")) => s,
        Some(other) => {
            return Err(format!(
                "unknown waveform command {other:?}; use ber, iq or decode"
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
            ("ber", "json" | "random-frac-delay") | ("decode", "hex") => flags.push(k.to_string()),
            _ => {
                let v = it.next().ok_or(format!("--{k} needs a value"))?;
                if named.insert(k.to_string(), v.clone()).is_some() {
                    return Err(format!("--{k} given twice"));
                }
            }
        }
    }
    let modulation = {
        let v = named.remove("mod").ok_or("missing --mod")?;
        Modulation::parse(&v).ok_or(format!("--mod {v:?}: use bpsk, qpsk or fsk2"))?
    };
    let sps = int("sps", named.remove("sps"), 8, 4, 64)? as usize;
    // Frequencies in cycles per symbol stay inside the sampled band, +-sps/2.
    let half = sps as f64 / 2.0;
    let cmd = match sub {
        "ber" => {
            let list = named.remove("ebn0-db").ok_or("missing --ebn0-db")?;
            let ebn0_db = list
                .split(',')
                .map(|x| float("ebn0-db", x, -30.0, 60.0))
                .collect::<R<Vec<f64>>>()?;
            if ebn0_db.is_empty() || ebn0_db.len() > 64 {
                return Err("--ebn0-db takes 1 to 64 values".into());
            }
            WaveformCommand::Ber(BerArgs {
                modulation,
                ebn0_db,
                bits: int("bits", named.remove("bits"), 100_000, 1, 10_000_000_000)?,
                seed: int("seed", named.remove("seed"), 1, 0, u64::MAX)?,
                sps,
                payload_len: int("payload-len", named.remove("payload-len"), 64, 1, 255)? as usize,
                cfo: opt_float("cfo", named.remove("cfo"), 0.0, -half, half)?,
                cfo_random: opt_float("cfo-random", named.remove("cfo-random"), 0.0, 0.0, half)?,
                random_frac_delay: flags.iter().any(|f| f == "random-frac-delay"),
                cfo_bins: int("cfo-bins", named.remove("cfo-bins"), 0, 0, 20)? as usize,
                cfo_search: opt_float("cfo-search", named.remove("cfo-search"), 0.0, 0.0, half)?,
            })
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
        _ => WaveformCommand::Decode(DecodeArgs {
            modulation,
            input: named.remove("in").ok_or("missing --in")?,
            sps,
            cfo_bins: int("cfo-bins", named.remove("cfo-bins"), 0, 0, 20)? as usize,
            cfo_search: opt_float("cfo-search", named.remove("cfo-search"), 0.0, 0.0, half)?,
            hex: flags.iter().any(|f| f == "hex"),
        }),
    };
    if let Some(k) = named.keys().next() {
        return Err(format!("unknown option --{k} for waveform {sub}"));
    }
    Ok(cmd)
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
        WaveformCommand::Iq(a) => run_iq(a, out),
        WaveformCommand::Decode(a) => run_decode(a, out),
    }
}

fn io(e: std::io::Error) -> String {
    e.to_string()
}

fn run_ber_cmd(a: &BerArgs, out: &mut dyn Write) -> R<()> {
    let opts = BerOptions {
        sps: a.sps,
        payload_len: a.payload_len,
        cfo_cycles_per_symbol: a.cfo,
        cfo_random: a.cfo_random,
        random_frac_delay: a.random_frac_delay,
        rx: RxConfig {
            cfo_bins: a.cfo_bins,
            cfo_search: a.cfo_search as f32,
            ..RxConfig::default()
        },
        ..BerOptions::default()
    };
    let theory = match a.modulation {
        Modulation::Fsk2 => "noncoherent_bfsk: 0.5*exp(-Eb/2N0)",
        _ => "coherent_psk: Q(sqrt(2Eb/N0))",
    };
    for &db in &a.ebn0_db {
        let p = run_ber(a.modulation, db, a.bits, a.seed, &opts).map_err(|e| e.to_string())?;
        writeln!(
            out,
            "{{\"ts\":\"{}\",\"event\":\"ber\",\"mod\":\"{}\",\"ebn0_db\":{},\"bits\":{},\"errors\":{},\"ber\":{},\"theory\":{},\"ci_lo\":{},\"ci_hi\":{},\"z\":{},\"frames\":{},\"detected\":{},\"crc_ok\":{},\"seed\":{},\"sps\":{},\"payload_len\":{},\"cfo_cycles_per_symbol\":{},\"cfo_random\":{},\"random_frac_delay\":{},\"cfo_bins\":{},\"cfo_search\":{},\"theory_kind\":{}}}",
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
            p.frames,
            p.detected,
            p.crc_ok,
            a.seed,
            a.sps,
            a.payload_len,
            num(a.cfo),
            num(a.cfo_random),
            a.random_frac_delay,
            a.cfo_bins,
            num(a.cfo_search),
            json_str(theory),
        )
        .map_err(io)?;
        out.flush().map_err(io)?;
    }
    Ok(())
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
    let modem = Modem::new(ModemConfig {
        sps: a.sps,
        ..ModemConfig::new(a.modulation)
    })
    .map_err(|e| e.to_string())?;
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
        "OASA waveform W0 test frame: {} with RRC (or tones for fsk2), {} bytes, simulated channel",
        a.modulation.name(),
        payload.len()
    );
    let meta = format!(
        "{{\n  \"global\": {{\n    \"core:datatype\": \"cf32_le\",\n    \"core:sample_rate\": {rate:?},\n    \"core:version\": \"1.2.0\",\n    \"core:num_channels\": 1,\n    \"core:description\": {},\n    \"core:recorder\": \"oasa-radio waveform iq\",\n    \"oasa:modulation\": \"{}\",\n    \"oasa:samples_per_symbol\": {},\n    \"oasa:symbol_rate\": {:?},\n    \"oasa:ebn0_db\": {},\n    \"oasa:simulated\": true\n  }},\n  \"captures\": [\n    {{ \"core:sample_start\": 0, \"core:frequency\": {:?}, \"core:datetime\": \"{}\" }}\n  ],\n  \"annotations\": [\n    {{ \"core:sample_start\": {}, \"core:sample_count\": {}, \"core:label\": {} }}\n  ]\n}}\n",
        json_str(&desc),
        a.modulation.name(),
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
    let snr = a
        .ebn0_db
        .map(|db| num(snr_per_sample_db(db, a.modulation.bits_per_symbol(), a.sps)))
        .unwrap_or_else(|| "null".into());
    writeln!(
        out,
        "{{\"ts\":\"{ts}\",\"event\":\"iq\",\"mod\":\"{}\",\"data\":{},\"meta\":{},\"samples\":{},\"sample_rate\":{rate:?},\"frame_start\":{},\"frame_samples\":{},\"payload_len\":{},\"ebn0_db\":{},\"snr_per_sample_db\":{snr},\"seed\":{}}}",
        a.modulation.name(),
        json_str(&data_path.display().to_string()),
        json_str(&meta_path.display().to_string()),
        y.len(),
        a.delay,
        tx.len(),
        payload.len(),
        a.ebn0_db.map(num).unwrap_or_else(|| "null".into()),
        a.seed,
    )
    .map_err(io)?;
    Ok(())
}

fn run_decode(a: &DecodeArgs, out: &mut dyn Write) -> R<()> {
    let data_path = with_ext(&base_path(&a.input), ".sigmf-data");
    let bytes = std::fs::read(&data_path).map_err(|e| format!("{}: {e}", data_path.display()))?;
    let samples = from_cf32_le(&bytes);
    let modem = Modem::new(ModemConfig {
        sps: a.sps,
        ..ModemConfig::new(a.modulation)
    })
    .map_err(|e| e.to_string())?;
    let rx = RxConfig {
        cfo_bins: a.cfo_bins,
        cfo_search: a.cfo_search as f32,
        ..RxConfig::default()
    };
    let frames = modem.demodulate_with(&samples, &rx);
    for f in &frames {
        let payload = if a.hex {
            format!(",\"payload_hex\":\"{}\"", hex(&f.payload))
        } else {
            String::new()
        };
        writeln!(
            out,
            "{{\"ts\":\"{}\",\"event\":\"frame\",\"start\":{},\"end\":{},\"len\":{},\"crc_ok\":{},\"sync_errors\":{},\"metric\":{},\"cfo_cycles_per_symbol\":{},\"phase_rad\":{},\"timing_frac\":{}{payload}}}",
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
        .map_err(io)?;
    }
    writeln!(
        out,
        "{{\"ts\":\"{}\",\"event\":\"decode_done\",\"mod\":\"{}\",\"samples\":{},\"frames\":{},\"crc_ok_frames\":{}}}",
        now(),
        a.modulation.name(),
        samples.len(),
        frames.len(),
        frames.iter().filter(|f| f.crc_ok).count()
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
            "ber --mod bpsk --ebn0-db 8 --cfo-random 2 --cfo-search 2.05",
        ))
        .unwrap()
        {
            WaveformCommand::Ber(a) => assert_eq!((a.cfo_random, a.cfo_search), (2.0, 2.05)),
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn json_numbers() {
        assert_eq!(num(f64::NAN), "null");
        assert_eq!(num(0.0), "0e0");
        assert!(num(1.5e-4).parse::<f64>().is_ok());
    }
}
