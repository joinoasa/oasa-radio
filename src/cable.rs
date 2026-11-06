//! A simulated cable between two `sdr` radios: reads one radio's TX stream, writes the other's RX
//! stream (cf32, little endian), with the loss, clock offset and receiver noise of a real bench. For
//! tests and the lab's simulated bench (`oasa-radio waveform cable`). Not part of any radio path.
//!
//! ```text
//!   radio A --tx FIFO--> cable (gain, offset, noise) --rx FIFO--> radio B
//! ```
//!
//! The TX stream is paced by its writer (`sdr` never runs more than `tx_lead`, 20 ms by default, ahead of the wall clock),
//! so the cable needs no clock of its own: it passes samples on as they come.

use std::io::{Read, Write};

use oasa_waveform::rng::Rng;

#[derive(Debug, Clone, PartialEq)]
pub struct CableConfig {
    /// Gain of the path, dB (a 60 dB cable is -60, plus what the radios add).
    pub gain_db: f64,
    /// Noise added at the receiver, dB full scale per complex sample (power).
    pub noise_dbfs: f64,
    /// Frequency offset between the two radios' clocks, Hz.
    pub cfo_hz: f64,
    pub sample_rate: f64,
    pub seed: u64,
}

impl Default for CableConfig {
    fn default() -> Self {
        Self {
            gain_db: 0.0,
            noise_dbfs: -60.0,
            cfo_hz: 0.0,
            sample_rate: 1e6,
            seed: 1,
        }
    }
}

/// Pass samples from `input` to `output` until `input` ends or `output` closes. Returns the
/// number of samples passed.
pub fn run(
    cfg: &CableConfig,
    input: &mut dyn Read,
    output: &mut dyn Write,
) -> std::io::Result<u64> {
    let g = 10f64.powf(cfg.gain_db / 20.0);
    let sigma = (10f64.powf(cfg.noise_dbfs / 10.0) / 2.0).sqrt();
    let w = 2.0 * std::f64::consts::PI * cfg.cfo_hz / cfg.sample_rate;
    let mut rng = Rng::new(cfg.seed);
    let mut phase = 0.0f64;
    let mut buf = vec![0u8; 8 * 4096];
    let mut have = 0usize;
    let mut out = Vec::with_capacity(buf.len());
    let mut n: u64 = 0;
    loop {
        let k = match input.read(&mut buf[have..]) {
            Ok(0) => return Ok(n),
            Ok(k) => k,
            Err(e) if e.kind() == std::io::ErrorKind::Interrupted => continue,
            Err(e) => return Err(e),
        };
        have += k;
        let whole = have / 8 * 8;
        out.clear();
        for c in buf[..whole].chunks_exact(8) {
            let re = f64::from(f32::from_le_bytes([c[0], c[1], c[2], c[3]]));
            let im = f64::from(f32::from_le_bytes([c[4], c[5], c[6], c[7]]));
            let (s, co) = phase.sin_cos();
            let yr = g * (re * co - im * s) + sigma * rng.gaussian();
            let yi = g * (re * s + im * co) + sigma * rng.gaussian();
            phase = (phase + w).rem_euclid(2.0 * std::f64::consts::PI);
            out.extend_from_slice(&(yr as f32).to_le_bytes());
            out.extend_from_slice(&(yi as f32).to_le_bytes());
            n += 1;
        }
        buf.copy_within(whole..have, 0);
        have -= whole;
        output.write_all(&out)?;
        output.flush()?;
    }
}
