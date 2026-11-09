//! Receive diversity (test plan B14): two or more antennas, each with its own receive chain, sampled
//! together (one multi-channel stream: the channels share the sample clock and the local oscillator, so
//! timing and carrier offset are common and only each antenna's channel gain differs). Receiver only;
//! nothing changes on the air. With one channel every method is exactly [`Modem::demodulate_report`].
//!
//! Three ways to use the second antenna, cheapest to dearest in what they need from the receiver:
//!
//! - [`Combine::Select`]: frame selection. Run the whole receiver on every channel and keep, for each
//!   frame, the copy whose CRC-16 matched. Costs one full receiver per channel.
//! - [`Combine::PreambleSnr`]: selection per frame before decoding. The detection metric is computed on
//!   every channel; at each candidate the channel whose preamble correlates best (the normalized
//!   metric is a monotone function of the preamble's SNR) is the only one decoded. Costs one detector
//!   per channel and one decoder.
//! - [`Combine::Mrc`]: maximal-ratio combining. At each candidate (found on the best channel's metric),
//!   the frame timing is taken from the sum of the channels' fine metrics; each channel's carrier
//!   frequency, phase, amplitude and noise are estimated from its own preamble, and the channels are
//!   added sample by sample with the weights `conj(h_i) / N_i` (`h_i` the channel gain at the middle
//!   of the preamble, `N_i` its noise power). The sum is then decoded as one channel: the receiver's
//!   own estimates (and the channel tracker, if on) take out what is common. With slow fading
//!   the gains hold through a frame and this is the optimum combiner; when they turn within a frame
//!   (50 Hz and up at 250 kbaud), the weights from the preamble go stale towards its end.
//!
//! Without the coarse frequency search (`cfo_search == 0`) the detection metric is the plain one,
//! one value per sample.

use crate::iq::Complex32;
use crate::modem::{parabola_peak, Modem, Reject, RxConfig, RxFrame, RxReport, Tally};
use std::f64::consts::PI;

/// How to use more than one receive channel.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Combine {
    /// Decode every channel, keep the frame with a good CRC.
    Select,
    /// Decode only the channel whose preamble is strongest, frame by frame.
    PreambleSnr,
    /// Maximal-ratio combining after per-channel carrier and phase estimation.
    Mrc,
}

impl Combine {
    pub fn name(self) -> &'static str {
        match self {
            Combine::Select => "select",
            Combine::PreambleSnr => "snr",
            Combine::Mrc => "mrc",
        }
    }

    pub fn parse(s: &str) -> Option<Self> {
        match s {
            "select" => Some(Combine::Select),
            "snr" => Some(Combine::PreambleSnr),
            "mrc" => Some(Combine::Mrc),
            _ => None,
        }
    }
}

/// The detection metric of every channel on one grid.
struct Grid {
    /// Samples between grid points.
    step: usize,
    threshold: f32,
    /// A candidate must be the largest within this many grid points.
    reach: usize,
    /// Per channel, per grid point: (metric, frequency in cycles per symbol).
    vals: Vec<Vec<(f32, f32)>>,
}

impl Modem {
    /// Find frames in `chans`, two or more channels of the same length sampled together, using them
    /// as `how` says. One channel: exactly [`Modem::demodulate_report`]. The report counts the
    /// candidates of the combined search (for [`Combine::Select`], those of the first channel).
    pub fn demodulate_diversity(
        &self,
        chans: &[&[Complex32]],
        rx: &RxConfig,
        how: Combine,
    ) -> (Vec<RxFrame>, RxReport) {
        match chans {
            [] => (Vec::new(), RxReport::default()),
            [one] => self.demodulate_report(one, rx),
            _ if how == Combine::Select => self.select_frames(chans, rx),
            _ => self.combined_search(chans, rx, how),
        }
    }

    fn select_frames(&self, chans: &[&[Complex32]], rx: &RxConfig) -> (Vec<RxFrame>, RxReport) {
        let sps = self.config().sps;
        let mut rep = RxReport::default();
        let mut all: Vec<RxFrame> = Vec::new();
        for (i, c) in chans.iter().enumerate() {
            let (f, r) = self.demodulate_report(c, rx);
            if i == 0 {
                rep = r;
            }
            all.extend(f);
        }
        // stable: at equal starts the first channel's copy comes first
        all.sort_by_key(|f| f.start);
        let mut out: Vec<RxFrame> = Vec::new();
        for f in all {
            match out.last_mut() {
                Some(last) if f.start.abs_diff(last.start) <= sps as u64 => {
                    // the same frame on another channel: a good CRC wins, then the stronger preamble
                    if (f.crc_ok && !last.crc_ok)
                        || (f.crc_ok == last.crc_ok && f.metric > last.metric)
                    {
                        *last = f;
                    }
                }
                _ => out.push(f),
            }
        }
        (out, rep)
    }

    fn grid(&self, chans: &[&[Complex32]], rx: &RxConfig) -> Grid {
        match self.coarse_search(rx) {
            Some(cs) => Grid {
                step: cs.step,
                threshold: cs.threshold,
                reach: self.reference_len().div_ceil(cs.step),
                vals: chans
                    .iter()
                    .map(|c| self.coarse_metric_with(c, &cs))
                    .collect(),
            },
            None => Grid {
                step: 1,
                threshold: rx.detect_threshold,
                reach: self.config().sps,
                vals: chans
                    .iter()
                    .map(|c| self.metric(c, rx).into_iter().map(|m| (m, 0.0)).collect())
                    .collect(),
            },
        }
    }

    fn combined_search(
        &self,
        chans: &[&[Complex32]],
        rx: &RxConfig,
        how: Combine,
    ) -> (Vec<RxFrame>, RxReport) {
        let mut tally = Tally::default();
        let g = self.grid(chans, rx);
        let len = g.vals.iter().map(Vec::len).min().unwrap_or(0);
        // the best channel at every grid point
        let best: Vec<(f32, f32, usize)> = (0..len)
            .map(|i| {
                (0..g.vals.len())
                    .map(|c| (g.vals[c][i].0, g.vals[c][i].1, c))
                    .fold((f32::MIN, 0.0, 0), |a, b| if b.0 > a.0 { b } else { a })
            })
            .collect();
        let frame_len = self.longest_frame_samples(rx);
        let fine = RxConfig {
            cfo_bins: if g.step == 1 { rx.cfo_bins } else { 0 },
            cfo_search: 0.0,
            ..rx.clone()
        };
        let mut out = Vec::new();
        for i in 0..len {
            let (v, f, ch) = best[i];
            if v < g.threshold {
                continue;
            }
            let lo_i = i.saturating_sub(g.reach);
            let hi_i = (i + g.reach + 1).min(len);
            if !(lo_i..hi_i).all(|j| best[j].0 < v || (best[j].0 == v && j >= i)) {
                continue;
            }
            let tau_c = i * g.step;
            if (tau_c as u64) < tally.busy_until {
                tally.skipped();
                continue;
            }
            // Positions here count from the start of the channels, so the span fits a usize.
            let busy = tally.busy_until as usize;
            let got = match how {
                Combine::Mrc => self.mrc_candidate(chans, tau_c, f, g.step, &fine, frame_len, busy),
                _ => self
                    .coarse_candidate(chans[ch], tau_c, f, g.step, &fine, frame_len, busy)
                    .map(|(res, tau, _)| (res, tau)),
            };
            let Some((res, tau)) = got else {
                continue;
            };
            if let Some(fr) = self.record(res, tau as u64, rx, &mut tally) {
                out.push(fr);
            }
        }
        (out, tally.rep)
    }

    /// One candidate, maximal-ratio combined: see the module notes. Returns the decode and the sample
    /// the candidate counts at; `None` when there is no fine peak.
    #[allow(clippy::too_many_arguments)]
    fn mrc_candidate(
        &self,
        chans: &[&[Complex32]],
        tau_c: usize,
        f: f32,
        step: usize,
        fine: &RxConfig,
        frame_len: usize,
        busy_until: usize,
    ) -> Option<(Result<RxFrame, Reject>, usize)> {
        let sps = self.config().sps;
        let refw = self.reference();
        let m = refw.len();
        let n = chans.iter().map(|c| c.len()).min()?;
        let last = n.checked_sub(m)?;
        let lo = tau_c.saturating_sub(sps + step).max(busy_until);
        let hi = (tau_c + sps + step).min(last);
        if hi < lo {
            return None;
        }
        let end = (hi + frame_len + 1).min(n);
        // the common coarse frequency off every channel
        let w = f as f64 / sps as f64;
        let rot: Vec<Complex32> = (lo..end)
            .map(|k| {
                let turns = (w * k as f64).rem_euclid(1.0);
                let (sn, co) = (-2.0 * PI * turns).sin_cos();
                Complex32::new(co as f32, sn as f32)
            })
            .collect();
        let d: Vec<Vec<Complex32>> = chans
            .iter()
            .map(|c| c[lo..end].iter().zip(&rot).map(|(s, r)| *s * *r).collect())
            .collect();
        // timing from the sum of the channels' fine metrics (each is normalized by its own power)
        let mut fm = vec![0f32; hi - lo + 1];
        for di in &d {
            for (a, b) in fm.iter_mut().zip(self.metric(&di[..hi - lo + m], fine)) {
                *a += b;
            }
        }
        let j = (0..fm.len()).max_by(|&a, &b| fm[a].total_cmp(&fm[b]).then(b.cmp(&a)))?;
        let mu = parabola_peak(&fm, j);
        // each channel's gain at the middle of the preamble, and its noise
        let mid = (m / 2) as f64;
        let weights: Vec<Complex32> = d
            .iter()
            .map(|di| {
                let e = self.estimate(di, j, refw);
                let ph = e.psi + e.omega * mid;
                let k = (e.amp / e.noise_per_sample) as f32;
                Complex32::new(ph.cos() as f32, -ph.sin() as f32) * k
            })
            .collect();
        // scale the sum back to about the size of one channel (the decoder does not care)
        let norm = 1.0
            / weights
                .iter()
                .map(|w| w.norm_sqr())
                .sum::<f32>()
                .sqrt()
                .max(1e-30);
        let y: Vec<Complex32> = (0..end - lo)
            .map(|k| {
                let mut s = Complex32::ZERO;
                for (di, wi) in d.iter().zip(&weights) {
                    s += di[k] * *wi;
                }
                s * norm
            })
            .collect();
        let metric = fm[j] / chans.len() as f32;
        let res = self.decode_at(&y, j, mu, metric, fine).map(|mut fr| {
            fr.start += lo as u64;
            fr.end += lo as u64;
            fr.cfo_cycles_per_symbol += f;
            let turns = (w * fr.start as f64).rem_euclid(1.0);
            let ph = fr.phase_rad as f64 + 2.0 * PI * turns;
            fr.phase_rad = ph.sin().atan2(ph.cos()) as f32;
            fr
        });
        Some((res, lo + j))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::channel::{n0_for_ebn0, Channel, Impairments};
    use crate::frame::FrameFormat;
    use crate::modem::{ModemConfig, Modulation};

    const ALL: [Combine; 3] = [Combine::Select, Combine::PreambleSnr, Combine::Mrc];

    fn modem() -> Modem {
        Modem::new(ModemConfig {
            sps: 4,
            frame: FrameFormat::V1,
            ..ModemConfig::new(Modulation::Qpsk)
        })
        .unwrap()
    }

    fn rx() -> RxConfig {
        RxConfig {
            cfo_search: 0.1,
            ..RxConfig::default()
        }
    }

    /// One frame through channel gain `g` (complex), the carrier offset `cfo` (cycles per symbol)
    /// and noise at `ebn0_db` (0: none).
    fn through(
        m: &Modem,
        tx: &[Complex32],
        g: Complex32,
        cfo: f64,
        ebn0_db: f64,
        seed: u64,
    ) -> Vec<Complex32> {
        let mut ch = Channel::new(seed);
        let faded: Vec<Complex32> = tx.iter().map(|s| *s * g).collect();
        ch.apply(
            &faded,
            &Impairments {
                n0: if ebn0_db > 0.0 {
                    n0_for_ebn0(ebn0_db, m.energy_per_bit())
                } else {
                    0.0
                },
                phase_rad: 0.0,
                cfo_cycles_per_sample: cfo / 4.0,
                frac_delay: 0.0,
                lead: 37,
                tail: 8,
            },
        )
    }

    #[test]
    fn names_round_trip() {
        for c in ALL {
            assert_eq!(Combine::parse(c.name()), Some(c));
        }
        assert_eq!(Combine::parse("max"), None);
    }

    #[test]
    fn one_channel_is_the_plain_receiver() {
        let m = modem();
        let tx = m.modulate(&[7u8; 54]).unwrap();
        let y = through(&m, &tx, Complex32::new(1.0, 0.0), 0.01, 8.0, 3);
        let plain = m.demodulate_report(&y, &rx());
        for c in ALL {
            assert_eq!(m.demodulate_diversity(&[&y], &rx(), c), plain);
        }
    }

    #[test]
    fn a_dead_channel_does_not_hurt() {
        // channel 2 is noise only (a deep fade): every method still finds the frame on channel 1
        let m = modem();
        let payload: Vec<u8> = (0..54).collect();
        let tx = m.modulate(&payload).unwrap();
        let y1 = through(&m, &tx, Complex32::new(0.6, 0.8), 0.02, 14.0, 5);
        let y2 = through(&m, &tx, Complex32::ZERO, 0.02, 14.0, 6);
        for c in ALL {
            for chans in [[&y1[..], &y2[..]], [&y2[..], &y1[..]]] {
                let (f, _) = m.demodulate_diversity(&chans, &rx(), c);
                assert_eq!(f.len(), 1, "{c:?}");
                assert!(f[0].crc_ok, "{c:?}");
                assert_eq!(f[0].payload, payload, "{c:?}");
                assert!(f[0].start.abs_diff(37) <= 1, "{c:?} at {}", f[0].start);
            }
        }
    }

    #[test]
    fn mrc_adds_the_channels_in_phase() {
        // Two channels at the same weak SNR and opposite phases: a plain sum would cancel, and
        // each alone loses many frames. MRC must beat the better single channel clearly.
        let m = modem();
        let r = rx();
        let (mut single, mut mrc, mut sel) = (0, 0, 0);
        for s in 0..60u64 {
            let payload: Vec<u8> = (0..54).map(|k| (k as u64 * 31 + s) as u8).collect();
            let tx = m.modulate(&payload).unwrap();
            let y1 = through(&m, &tx, Complex32::new(1.0, 0.0), 0.01, 5.0, 100 + s);
            let y2 = through(&m, &tx, Complex32::new(-1.0, 0.0), 0.01, 5.0, 900 + s);
            let ok = |fs: &[RxFrame]| fs.iter().any(|f| f.crc_ok && f.payload == payload) as u32;
            single += ok(&m.demodulate_report(&y1, &r).0);
            sel += ok(&m.demodulate_diversity(&[&y1, &y2], &r, Combine::Select).0);
            mrc += ok(&m.demodulate_diversity(&[&y1, &y2], &r, Combine::Mrc).0);
        }
        // at 5 dB one channel loses a large share; MRC is at about 8 dB on the sum
        assert!(single < 50, "single {single}");
        assert!(sel >= single, "select {sel} single {single}");
        assert!(
            mrc >= 57 && mrc > sel,
            "mrc {mrc} select {sel} single {single}"
        );
    }
}
