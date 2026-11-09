//! Channel tracking through a frame, for fast fading (test plan B12). Off by default
//! ([`crate::modem::RxConfig::phase_track`]); without it the receiver turns the body back by one
//! straight phase line.
//!
//! Under flat fading the channel gain `h(t)` turns during a frame. Its phase is not a line: it bends
//! where the fading changes direction and turns by up to half a turn, fast, where `|h|` passes near
//! zero. A line fitted to a 1.8 ms frame at 200 Hz Doppler leaves parts of the frame a quarter turn
//! or more off, and those symbols decode wrong.
//!
//! This tracker follows the complex gain `h` itself, not its phase. `h` is smooth (its spectrum
//! ends at the Doppler frequency), and near a deep fade it passes close to zero almost on a straight
//! line; the phase jump there is what a straight line through the origin looks like in polar form.
//! So a smoother on `h` crosses a fade with the right phase on the other side, where a phase
//! tracker has to guess the jump.
//!
//! Method:
//! 1. Cut the frame into blocks of a few symbols (more at low SNR, see [`block_len`]).
//! 2. Going forward from the frame start, predict each block's gain with a Kalman filter (gain and
//!    its rate of change, a random walk on the rate; the same filter for I and Q). Measure the
//!    block's gain against the data: the known symbols (preamble, sync word and, once decoded, the
//!    v1 header) or the receiver's own decisions made with the prediction (one bit per symbol for
//!    the header before it is decoded, the payload modulation for the body).
//! 3. Smooth backward (Rauch-Tung-Striebel), so every block's gain uses the blocks after it too.
//! 4. Measure every block again against decisions made with the smoothed gain, filter and smooth
//!    once more, and interpolate between block centres: one gain per symbol.
//!
//! The receiver then decides on `z conj(h)` and, with the light code, uses it as the soft value:
//! a symbol in a fade counts in proportion to `|h|^2`, as it should.
//!
//! The known symbols at the frame start fix the absolute phase; after that the decisions only
//! resolve it to a quarter turn (QPSK) or half a turn (BPSK). A fade deep and long enough can still
//! leave the rest of the frame turned (a cycle slip); see `docs/phase-tracking.md` for how often.
//!
//! Cost: a few complex multiplies per symbol per pass and a 2x2 filter per block; no search.

use crate::iq::Complex32;

/// What the receiver knows about one symbol.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum Sym {
    /// Known data, unit magnitude (`+-1` for the one-bit-per-symbol preamble, sync and header).
    Known(Complex32),
    /// Unknown, one bit per symbol (BPSK, and the v1 header before it is decoded).
    Bpsk,
    /// Unknown QPSK (Gray, `(+-1 +-j) / sqrt 2`).
    Qpsk,
}

/// Block length in symbols for a frame at Es/N0 `rho`: short at high SNR, so fast fading is
/// followed; longer at low SNR, so each block's gain is still measured well.
pub(crate) fn block_len(rho: f64) -> usize {
    ((BLOCK_SNR / rho.max(1e-3)).ceil() as usize).clamp(MIN_BLOCK, MAX_BLOCK)
}

/// Smallest block, symbols.
const MIN_BLOCK: usize = 8;
/// Largest block, symbols.
const MAX_BLOCK: usize = 64;
/// Block length times Es/N0 that the block length aims for.
const BLOCK_SNR: f64 = 500.0;

/// The gain's rate of change, random walk per symbol (per I or Q, unit mean power, 1 / symbol^3),
/// for fading with Doppler `fd` (cycles per symbol). The rate of a unit-power Rayleigh gain is about
/// `2 pi fd` and changes over about `1 / (2 pi fd)` symbols, which suggests `(2 pi fd)^3`;
/// [`Q_SCALE`] times that was best in simulation (B12: Rayleigh 50 and 200 Hz at 250 kbaud, 30 dB;
/// 0.03 to 0.3 within the noise, 3 and more clearly worse: a filter that believes in faster
/// fading than there is follows the noise).
pub(crate) fn rate_noise(fd: f64) -> f64 {
    Q_SCALE * (2.0 * std::f64::consts::PI * fd).powi(3)
}

const Q_SCALE: f64 = 0.1;

/// Gain of every symbol of `z` (matched filter outputs from the frame start, mean power about 1,
/// noise variance `1 / rho`): symbol `k` is about `gain[k] d_k` plus noise. `kind(k)` says what is
/// known about symbol `k`; `q` is the rate noise from [`rate_noise`].
pub(crate) fn track(
    z: &[Complex32],
    kind: impl Fn(usize) -> Sym,
    rho: f64,
    q: f64,
) -> Vec<Complex32> {
    let n = z.len();
    if n == 0 {
        return Vec::new();
    }
    // Es/N0 from the known symbols, block by block, so the fading's own turn does not count as
    // noise (against a line fit, at 200 Hz Doppler, it can look like 20 dB of noise); the
    // caller's value when there are too few known symbols.
    let rho = local_rho(z, &kind).unwrap_or(rho).max(1e-3);
    let noise = 1.0 / rho;
    let blen = block_len(rho);
    let blocks: Vec<(usize, usize)> = (0..n.div_ceil(blen))
        .map(|j| (j * blen, ((j + 1) * blen).min(n)))
        .collect();
    let centre = |&(lo, hi): &(usize, usize)| (lo + hi - 1) as f64 / 2.0;
    // Variance (per I or Q) of a block's gain measurement `g`. Noise alone gives `noise / len`.
    // Decisions made on noise line up with it, so in a deep fade the measurement does not go to
    // zero but stays about `sqrt(2 / pi)` noise amplitudes long, pointing where the decisions were
    // steered: count that as noise too, in proportion to the decided symbols, fading out once the
    // block is clearly above the noise.
    let variance = |g: (f64, f64), (lo, hi): (usize, usize)| {
        let len = (hi - lo) as f64;
        let decided = (lo..hi)
            .filter(|&k| !matches!(kind(k), Sym::Known(_)))
            .count() as f64
            / len;
        let snr = (g.0 * g.0 + g.1 * g.1) / noise;
        0.5 * (noise / len + decided * std::f64::consts::FRAC_2_PI * noise * (-snr).exp())
    };

    // pass 1: forward, the prediction steers the decisions
    let mut kf = Kalman::new(q);
    for b in &blocks {
        let c = centre(b);
        let (h0, r0) = kf.predict_at(c);
        let pred = |k: usize| {
            let t = k as f64 - c;
            (h0.0 + r0.0 * t, h0.1 + r0.1 * t)
        };
        let g = block_gain(z, &kind, *b, &pred, true);
        kf.update(c, g, variance(g, *b));
    }
    let smooth = kf.smooth();
    // pass 2: measure again against decisions made with the smoothed gain
    let mut kf = Kalman::new(q);
    for b in &blocks {
        let c = centre(b);
        let g = block_gain(z, &kind, *b, &|k| interp_at(&smooth, k as f64), false);
        kf.predict_at(c);
        kf.update(c, g, variance(g, *b));
    }
    let smooth = kf.smooth();
    (0..n)
        .map(|k| {
            let (re, im) = interp_at(&smooth, k as f64);
            Complex32::new(re as f32, im as f32)
        })
        .collect()
}

/// Es/N0 of `z`: signal power over the frame, noise from the known symbols. With the data taken
/// off, neighbouring known symbols differ by the noise (twice its variance) plus the little the
/// fading moves in one symbol, so a gain that turns, even fast, does not count as noise.
/// `None` with fewer than [`NOISE_PAIRS`] neighbouring known pairs.
fn local_rho(z: &[Complex32], kind: &impl Fn(usize) -> Sym) -> Option<f64> {
    let (mut sum, mut pairs) = (0.0f64, 0usize);
    let mut prev: Option<(f64, f64)> = None;
    for (k, v) in z.iter().enumerate() {
        let cur = match kind(k) {
            Sym::Known(d) => {
                let (x, y, dr, di) = (v.re as f64, v.im as f64, d.re as f64, d.im as f64);
                Some((x * dr + y * di, y * dr - x * di))
            }
            _ => None,
        };
        if let (Some(a), Some(b)) = (prev, cur) {
            sum += (b.0 - a.0).powi(2) + (b.1 - a.1).powi(2);
            pairs += 1;
        }
        prev = cur;
    }
    if pairs < NOISE_PAIRS {
        return None;
    }
    let noise = (sum / (2.0 * pairs as f64)).max(1e-12);
    let power = z.iter().map(|v| v.norm_sqr() as f64).sum::<f64>() / z.len() as f64;
    Some(((power - noise) / noise).max(1e-3))
}

/// Fewest neighbouring known pairs [`local_rho`] estimates the noise from.
const NOISE_PAIRS: usize = 16;

/// The gain of the block `lo..hi`: the mean of `z_k conj(d_k)`, with `d_k` the known data or the
/// decision on `z_k conj(guide(k))`. With `refine`, decide once more guided by the first mean (a
/// guide up to about 45 degrees off then still decides well).
fn block_gain(
    z: &[Complex32],
    kind: &impl Fn(usize) -> Sym,
    (lo, hi): (usize, usize),
    guide: &impl Fn(usize) -> (f64, f64),
    refine: bool,
) -> (f64, f64) {
    let len = (hi - lo) as f64;
    let mean = |guide: &dyn Fn(usize) -> (f64, f64)| {
        let (mut sr, mut si) = (0.0f64, 0.0f64);
        for (k, v) in z.iter().enumerate().take(hi).skip(lo) {
            let (x, y) = (v.re as f64, v.im as f64);
            let (dr, di) = match kind(k) {
                Sym::Known(d) => (d.re as f64, d.im as f64),
                sym => {
                    // z conj(guide)
                    let (gr, gi) = guide(k);
                    let (tx, ty) = (x * gr + y * gi, y * gr - x * gi);
                    if sym == Sym::Bpsk {
                        (if tx < 0.0 { -1.0 } else { 1.0 }, 0.0)
                    } else {
                        let h = std::f64::consts::FRAC_1_SQRT_2;
                        (if tx < 0.0 { -h } else { h }, if ty < 0.0 { -h } else { h })
                    }
                }
            };
            // z conj(d)
            sr += x * dr + y * di;
            si += y * dr - x * di;
        }
        (sr / len, si / len)
    };
    let g = mean(guide);
    if !refine {
        return g;
    }
    mean(&|_| g)
}

/// A smoothed block: (centre, gain, rate), the gain and rate as (I, Q).
type Smoothed = (f64, (f64, f64), (f64, f64));

/// A Kalman filter on a complex gain and its rate over block centres (I and Q share the
/// covariance), with the backward smoother.
struct Kalman {
    q: f64,
    steps: Vec<Step>,
    /// state: gain and rate, each (I, Q)
    x: [(f64, f64); 2],
    p: [[f64; 2]; 2],
    t: f64,
}

#[derive(Clone, Copy)]
struct Step {
    t: f64,
    xp: [(f64, f64); 2],
    pp: [[f64; 2]; 2],
    xf: [(f64, f64); 2],
    pf: [[f64; 2]; 2],
}

/// Before the first block, per I or Q: the gain is about 1 in magnitude (the receiver normalized
/// it) at any phase; its rate is small (the derotation took the carrier offset off).
const P0_GAIN: f64 = 2.0;
const P0_RATE: f64 = 1e-4;

impl Kalman {
    fn new(q: f64) -> Self {
        Self {
            q,
            steps: Vec::new(),
            x: [(0.0, 0.0); 2],
            p: [[P0_GAIN, 0.0], [0.0, P0_RATE]],
            t: f64::NAN,
        }
    }

    /// Move the prediction to time `c` and return it (gain, rate).
    fn predict_at(&mut self, c: f64) -> ((f64, f64), (f64, f64)) {
        if self.t.is_finite() {
            let d = c - self.t;
            let [g, r] = self.x;
            self.x = [(g.0 + d * r.0, g.1 + d * r.1), r];
            let p = self.p;
            // F P F^T + Q, F = [[1, d], [0, 1]], Q = q [[d^3/3, d^2/2], [d^2/2, d]]
            let p00 = p[0][0] + 2.0 * d * p[0][1] + d * d * p[1][1] + self.q * d * d * d / 3.0;
            let p01 = p[0][1] + d * p[1][1] + self.q * d * d / 2.0;
            let p11 = p[1][1] + self.q * d;
            self.p = [[p00, p01], [p01, p11]];
        }
        self.t = c;
        (self.x[0], self.x[1])
    }

    /// Take the measurement `g` (variance `r` per I or Q) at centre `c`; [`Kalman::predict_at`] it first.
    fn update(&mut self, c: f64, g: (f64, f64), r: f64) {
        let (xp, pp) = (self.x, self.p);
        let s = pp[0][0] + r;
        let k = [pp[0][0] / s, pp[0][1] / s];
        let innov = (g.0 - xp[0].0, g.1 - xp[0].1);
        self.x = [
            (xp[0].0 + k[0] * innov.0, xp[0].1 + k[0] * innov.1),
            (xp[1].0 + k[1] * innov.0, xp[1].1 + k[1] * innov.1),
        ];
        self.p = [
            [(1.0 - k[0]) * pp[0][0], (1.0 - k[0]) * pp[0][1]],
            [(1.0 - k[0]) * pp[0][1], pp[1][1] - k[1] * pp[0][1]],
        ];
        self.steps.push(Step {
            t: c,
            xp,
            pp,
            xf: self.x,
            pf: self.p,
        });
    }

    /// Smoothed (centre, gain, rate) for every step.
    fn smooth(&self) -> Vec<Smoothed> {
        let n = self.steps.len();
        if n == 0 {
            return Vec::new();
        }
        let mut xs = vec![[(0.0f64, 0.0f64); 2]; n];
        xs[n - 1] = self.steps[n - 1].xf;
        for j in (0..n - 1).rev() {
            let (cur, next) = (&self.steps[j], &self.steps[j + 1]);
            let d = next.t - cur.t;
            // C = Pf F^T Pp(next)^-1
            let pf = cur.pf;
            let a = [
                [pf[0][0] + d * pf[0][1], pf[0][1]],
                [pf[0][1] + d * pf[1][1], pf[1][1]],
            ];
            let pp = next.pp;
            let det = pp[0][0] * pp[1][1] - pp[0][1] * pp[0][1];
            if det.abs() < 1e-300 {
                xs[j] = cur.xf;
                continue;
            }
            let inv = [
                [pp[1][1] / det, -pp[0][1] / det],
                [-pp[0][1] / det, pp[0][0] / det],
            ];
            let c = [
                [
                    a[0][0] * inv[0][0] + a[0][1] * inv[1][0],
                    a[0][0] * inv[0][1] + a[0][1] * inv[1][1],
                ],
                [
                    a[1][0] * inv[0][0] + a[1][1] * inv[1][0],
                    a[1][0] * inv[0][1] + a[1][1] * inv[1][1],
                ],
            ];
            let dg = (xs[j + 1][0].0 - next.xp[0].0, xs[j + 1][0].1 - next.xp[0].1);
            let dr = (xs[j + 1][1].0 - next.xp[1].0, xs[j + 1][1].1 - next.xp[1].1);
            xs[j] = [
                (
                    cur.xf[0].0 + c[0][0] * dg.0 + c[0][1] * dr.0,
                    cur.xf[0].1 + c[0][0] * dg.1 + c[0][1] * dr.1,
                ),
                (
                    cur.xf[1].0 + c[1][0] * dg.0 + c[1][1] * dr.0,
                    cur.xf[1].1 + c[1][0] * dg.1 + c[1][1] * dr.1,
                ),
            ];
        }
        self.steps
            .iter()
            .zip(&xs)
            .map(|(s, x)| (s.t, x[0], x[1]))
            .collect()
    }
}

/// Gain at time `t` from smoothed block states: linear between centres, the rate beyond the ends.
fn interp_at(s: &[Smoothed], t: f64) -> (f64, f64) {
    let j = s.partition_point(|&(c, _, _)| c <= t);
    let ext = |(c, g, r): (f64, (f64, f64), (f64, f64))| (g.0 + r.0 * (t - c), g.1 + r.1 * (t - c));
    if j == 0 {
        ext(s[0])
    } else if j == s.len() {
        ext(s[s.len() - 1])
    } else {
        let (c0, g0, _) = s[j - 1];
        let (c1, g1, _) = s[j];
        let w = (t - c0) / (c1 - c0);
        (g0.0 + (g1.0 - g0.0) * w, g0.1 + (g1.1 - g0.1) * w)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fading::RayleighProcess;
    use crate::rng::Rng;
    use std::f64::consts::PI;

    /// Known symbols in a v1 frame before the body (preamble, sync word, header).
    const KNOWN: usize = 231;

    fn wrap(x: f64) -> f64 {
        (x + PI).rem_euclid(2.0 * PI) - PI
    }

    fn qpsk(rng: &mut Rng) -> Complex32 {
        let s = std::f32::consts::FRAC_1_SQRT_2;
        let mut b = || if rng.uniform() < 0.5 { s } else { -s };
        Complex32::new(b(), b())
    }

    fn bit(rng: &mut Rng) -> Complex32 {
        Complex32::new(if rng.uniform() < 0.5 { 1.0 } else { -1.0 }, 0.0)
    }

    /// `KNOWN` one-bit symbols then `body` QPSK symbols through the gains `h`, plus noise at `rho`.
    fn frame(h: &[Complex32], rho: f64, rng: &mut Rng) -> (Vec<Complex32>, Vec<Complex32>) {
        let d: Vec<Complex32> = (0..h.len())
            .map(|k| if k < KNOWN { bit(rng) } else { qpsk(rng) })
            .collect();
        let sd = (0.5 / rho).sqrt();
        let z = (0..h.len())
            .map(|k| {
                h[k] * d[k]
                    + Complex32::new((rng.gaussian() * sd) as f32, (rng.gaussian() * sd) as f32)
            })
            .collect();
        (d, z)
    }

    fn run(z: &[Complex32], d: &[Complex32], rho: f64, fd: f64) -> Vec<Complex32> {
        track(
            z,
            |k| {
                if k < KNOWN {
                    Sym::Known(d[k])
                } else {
                    Sym::Qpsk
                }
            },
            rho,
            rate_noise(fd),
        )
    }

    fn errors(z: &[Complex32], d: &[Complex32], g: &[Complex32]) -> usize {
        (KNOWN..z.len())
            .filter(|&k| {
                let y = z[k] * g[k].conj();
                (y.re < 0.0, y.im < 0.0) != (d[k].re < 0.0, d[k].im < 0.0)
            })
            .count()
    }

    /// Rayleigh at 200 Hz Doppler and 250 kbaud (0.0008 cycles per symbol), 33 dB Es/N0, a light-coded
    /// 54-byte body (454 QPSK symbols): the tracker decides about as well as the true gain does, and
    /// cycle slips (32 symbols in a row a quarter turn off where the signal is up) stay rare.
    #[test]
    fn follows_200_hz_fading_about_as_well_as_the_true_gain() {
        let (fd, rho, n, frames) = (0.0008, 2000.0, KNOWN + 454, 400);
        let mut rng = Rng::new(3);
        let p = RayleighProcess::new(fd, &mut rng);
        let (mut bad, mut bad_true, mut slips) = (0, 0, 0);
        for f in 0..frames {
            let h: Vec<Complex32> = (0..n).map(|k| p.at((f * 2500 + k) as f64)).collect();
            let (d, z) = frame(&h, rho, &mut rng);
            let g = run(&z, &d, rho, fd);
            bad += (errors(&z, &d, &g) > 0) as usize;
            bad_true += (errors(&z, &d, &h) > 0) as usize;
            let mut run_len = 0;
            let mut slipped = false;
            for k in KNOWN..n {
                if h[k].norm_sqr() > 0.01 {
                    if wrap(g[k].arg() as f64 - h[k].arg() as f64).abs() > PI / 4.0 {
                        run_len += 1;
                        slipped |= run_len >= 32;
                    } else {
                        run_len = 0;
                    }
                }
            }
            slips += slipped as usize;
        }
        // measured: 18 frames with an error (the true gain: 16), 0 slips
        assert!(
            bad <= bad_true + 4,
            "{bad} frames with errors, {bad_true} with the true gain"
        );
        assert!(slips <= 2, "{slips} slips");
    }

    /// A fixed gain in white noise at 12.3 dB Es/N0 (9.3 dB Eb/N0 QPSK, the 1 % loss point): the
    /// tracked phase is within about 1.5 degrees rms, about what one line over the frame gives.
    #[test]
    fn white_noise_phase_error_is_small() {
        let (rho, n) = (17.0, KNOWN + 224);
        let mut rng = Rng::new(9);
        let h = vec![Complex32::from_polar(1.0, 0.7); n];
        let mut sq = 0.0;
        for _ in 0..200 {
            let (d, z) = frame(&h, rho, &mut rng);
            let g = run(&z, &d, rho, 0.0008);
            sq += (KNOWN..n)
                .map(|k| wrap(g[k].arg() as f64 - 0.7).powi(2))
                .sum::<f64>()
                / (n - KNOWN) as f64;
        }
        let rms = (sq / 200.0).sqrt();
        assert!(rms < 0.03, "rms phase error {rms} rad");
    }

    /// The gain passes 30 dB under its mean (about 0.03 from zero) on a straight line in the middle
    /// of the body: its phase turns by about half a turn within a few symbols. The tracker crosses
    /// with the right phase on the other side: no wrong decision where the signal is 20 dB or more
    /// above the noise.
    #[test]
    fn crosses_a_deep_fade() {
        let (rho, n) = (2000.0, KNOWN + 454);
        let mut rng = Rng::new(4);
        let mut wrong = 0;
        for trial in 0..50 {
            let t0 = (KNOWN + 100 + 5 * trial) as f64;
            let dir = Complex32::from_polar(1.0, trial as f32);
            let off = Complex32::from_polar(0.03, trial as f32 + 1.57);
            // 1 at the frame start, through `off` at t0 (a speed like 200 Hz Doppler gives)
            let speed = 1.0 / t0 as f32;
            let h: Vec<Complex32> = (0..n)
                .map(|k| off + dir * (speed * (k as f32 - t0 as f32)))
                .collect();
            let (d, z) = frame(&h, rho, &mut rng);
            let g = run(&z, &d, rho, 0.0008);
            wrong += (KNOWN..n)
                .filter(|&k| h[k].norm_sqr() as f64 * rho > 100.0)
                .filter(|&k| {
                    let y = z[k] * g[k].conj();
                    (y.re < 0.0, y.im < 0.0) != (d[k].re < 0.0, d[k].im < 0.0)
                })
                .count();
        }
        assert_eq!(wrong, 0);
    }

    /// The noise estimate comes from short blocks of known symbols, so a gain that turns fast
    /// (0.02 rad per symbol, four times what 200 Hz Doppler gives at 250 kbaud) does not count as noise.
    #[test]
    fn local_noise_ignores_a_turning_gain() {
        let (rho, n) = (1000.0, KNOWN + 100);
        let mut rng = Rng::new(5);
        let h: Vec<Complex32> = (0..n)
            .map(|k| Complex32::from_polar(1.0, 0.02 * k as f32))
            .collect();
        let (d, z) = frame(&h, rho, &mut rng);
        let kind = |k: usize| {
            if k < KNOWN {
                Sym::Known(d[k])
            } else {
                Sym::Qpsk
            }
        };
        let est = local_rho(&z, &kind).unwrap();
        let db = 10.0 * (est / rho).log10();
        assert!(db.abs() < 1.5, "estimate {db} dB off");
        // and with no known symbols there is no estimate
        assert!(local_rho(&z, &|_| Sym::Qpsk).is_none());
    }

    #[test]
    fn block_length_follows_the_snr() {
        assert_eq!(block_len(2000.0), MIN_BLOCK);
        assert_eq!(block_len(17.0), 30);
        assert_eq!(block_len(0.5), MAX_BLOCK);
        assert!(track(&[], |_| Sym::Qpsk, 10.0, 0.0).is_empty());
    }
}
