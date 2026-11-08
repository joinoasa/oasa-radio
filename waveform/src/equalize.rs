//! Equalizer for strong echoes (test plan B15). Off by default
//! ([`crate::modem::RxConfig::equalizer`]); without it the receiver decides each symbol on its
//! matched filter output alone.
//!
//! A second ray a quarter to a whole symbol late and 3 dB down puts so much of each symbol into its
//! neighbours that the eye closes for part of the echo phases (see `docs/equalizer.md`). The frame
//! is still found and its header (BPSK, coded) still decodes; the QPSK body does not.
//!
//! Method: a decision feedback equalizer (DFE) whose taps are solved, not adapted:
//! 1. The feed-forward part takes matched filter outputs at half-symbol spacing ([`PRE`] before
//!    and [`POST`] after the symbol instant), so it does not depend on where in the symbol the
//!    preamble put the timing, and it absorbs the carrier phase and amplitude.
//! 2. The feedback part subtracts [`FEEDBACK`] earlier symbols times their taps.
//! 3. The taps are the least-squares (MMSE) solution over the symbols the receiver knows: the
//!    preamble and sync word for the v1 header (feed-forward only, since the header symbols are
//!    unknown at that point), then preamble, sync word and the decoded header for the body.
//!    Feedback in training uses the known symbols; on the body, the receiver's own decisions.
//! 4. A second-order decision-directed phase loop inside the feedback loop follows the phase
//!    drift the carrier estimate leaves over the frame (without it, the taps solved at the frame
//!    start are a few degrees off by its end, and white-noise loss at 9.5 dB triples).
//! 5. The receiver (`modem.rs`) runs the body twice: the second time with taps solved over the
//!    whole frame, the first pass's decisions counting as known.
//!
//! The left-over error power on the known symbols gives the Es/N0 after the equalizer, which the
//! light code's soft values use.
//!
//! Cost per frame: matched filter outputs at twice the symbol rate, three Hermitian solves of at
//! most `FFE + FEEDBACK` (9) unknowns, and `FFE + FEEDBACK` complex multiplies per symbol per pass. Nothing is added on the air and no samples are needed beyond the frame.

use crate::iq::Complex32;

/// Feed-forward taps before the symbol instant, in half symbols.
pub(crate) const PRE: usize = 2;
/// Feed-forward taps after the symbol instant, in half symbols.
pub(crate) const POST: usize = 4;
/// Feed-forward taps.
pub(crate) const FFE: usize = PRE + POST + 1;
/// Feedback taps (earlier symbols).
pub(crate) const FEEDBACK: usize = 2;
/// Gain of the decision-directed phase loop, per symbol.
const PLL_GAIN: f64 = 0.03;
/// Frequency gain of the phase loop (second order), per symbol.
const PLL_FREQ_GAIN: f64 = 0.0002;
/// Ridge on the normal equations, relative to their mean diagonal: keeps the solve well posed when
/// a tap sees almost nothing (frame edge, deep notch).
const RIDGE: f64 = 1e-3;

#[derive(Debug, Clone, Copy, Default, PartialEq)]
struct C {
    re: f64,
    im: f64,
}

impl C {
    fn new(re: f64, im: f64) -> Self {
        Self { re, im }
    }
    fn of(z: Complex32) -> Self {
        Self::new(z.re as f64, z.im as f64)
    }
    fn conj(self) -> Self {
        Self::new(self.re, -self.im)
    }
    fn mul(self, o: C) -> C {
        C::new(
            self.re * o.re - self.im * o.im,
            self.re * o.im + self.im * o.re,
        )
    }
    fn add(self, o: C) -> C {
        C::new(self.re + o.re, self.im + o.im)
    }
    fn sub(self, o: C) -> C {
        C::new(self.re - o.re, self.im - o.im)
    }
    fn scale(self, k: f64) -> C {
        C::new(self.re * k, self.im * k)
    }
    fn norm_sqr(self) -> f64 {
        self.re * self.re + self.im * self.im
    }
    fn to32(self) -> Complex32 {
        Complex32::new(self.re as f32, self.im as f32)
    }
}

/// Solved taps.
#[derive(Debug, Clone)]
pub(crate) struct Taps {
    ffe: [C; FFE],
    fb: [C; FEEDBACK],
    /// Whether the feedback taps are used.
    with_fb: bool,
    /// Mean squared error on the training symbols (unit-energy symbols): about `1 / (Es/N0)`.
    pub(crate) mse: f64,
}

/// The feed-forward input of symbol `k`. `half` holds matched filter outputs at half-symbol
/// spacing: `half[2 k + i]` is the output `(i - PRE)` half symbols from symbol `k`'s instant, so
/// symbol `k`'s input is `half[2k..2k + FFE]` (zero past the end), turned back by `phase(k)`
/// radians (`phase` empty: not turned).
fn input(half: &[Complex32], k: usize, phase: &[f64]) -> [C; FFE] {
    let mut x = [C::default(); FFE];
    let rot = match phase.get(k) {
        Some(&p) => {
            let (s, c) = (-p).sin_cos();
            C::new(c, s)
        }
        None => C::new(1.0, 0.0),
    };
    for (i, v) in x.iter_mut().enumerate() {
        if let Some(&s) = half.get(2 * k + i) {
            *v = C::of(s).mul(rot);
        }
    }
    x
}

/// The regressor of symbol `k`: its feed-forward input, then minus the `FEEDBACK` symbols before it
/// (`sym(j)` for `j < k`, zero before the frame).
fn regressor(
    half: &[Complex32],
    k: usize,
    nfb: usize,
    sym: &impl Fn(usize) -> Complex32,
    phase: &[f64],
) -> Vec<C> {
    let mut u: Vec<C> = input(half, k, phase).to_vec();
    for j in 1..=nfb {
        u.push(if k >= j {
            C::of(sym(k - j)).scale(-1.0)
        } else {
            C::default()
        });
    }
    u
}

/// Least-squares taps over the training symbols `0..sym.len()` (all known, unit energy).
/// `feedback`: solve feedback taps too (else feed-forward only). `phase`: the carrier phase of each
/// symbol to take off first (as [`Taps::run`] returns it; empty for none), so that a frame whose
/// phase drifts does not average its taps down.
pub(crate) fn train(half: &[Complex32], sym: &[Complex32], feedback: bool, phase: &[f64]) -> Taps {
    let nfb = if feedback { FEEDBACK } else { 0 };
    let n = FFE + nfb;
    let mut r = vec![C::default(); n * n];
    let mut p = vec![C::default(); n];
    let at = |j: usize| sym[j];
    for (k, &d) in sym.iter().enumerate() {
        let u = regressor(half, k, nfb, &at, phase);
        let d = C::of(d);
        for a in 0..n {
            let ua = u[a].conj();
            p[a] = p[a].add(ua.mul(d));
            for b in a..n {
                r[a * n + b] = r[a * n + b].add(ua.mul(u[b]));
            }
        }
    }
    for a in 0..n {
        for b in 0..a {
            r[a * n + b] = r[b * n + a].conj();
        }
    }
    let diag: f64 = (0..n).map(|a| r[a * n + a].re).sum::<f64>() / n as f64;
    for a in 0..n {
        r[a * n + a].re += RIDGE * diag.max(1e-12);
    }
    let w = solve(r, p, n);
    let mut t = Taps {
        ffe: [C::default(); FFE],
        fb: [C::default(); FEEDBACK],
        with_fb: feedback,
        mse: 0.0,
    };
    t.ffe.copy_from_slice(&w[..FFE]);
    if feedback {
        t.fb.copy_from_slice(&w[FFE..]);
    }
    let mut e = 0.0;
    for (k, &d) in sym.iter().enumerate() {
        e += t.output(half, k, &at, phase).sub(C::of(d)).norm_sqr();
    }
    t.mse = (e / sym.len().max(1) as f64).max(1e-6);
    t
}

impl Taps {
    /// Output for symbol `k` given the symbols before it.
    fn output(
        &self,
        half: &[Complex32],
        k: usize,
        sym: &impl Fn(usize) -> Complex32,
        phase: &[f64],
    ) -> C {
        let x = input(half, k, phase);
        let mut y = C::default();
        for (w, v) in self.ffe.iter().zip(&x) {
            y = y.add(w.mul(*v));
        }
        if self.with_fb {
            for (j, f) in self.fb.iter().enumerate() {
                if k > j {
                    y = y.sub(f.mul(C::of(sym(k - j - 1))));
                }
            }
        }
        y
    }

    /// Feed-forward output only (for the header before it is decoded), symbols `range`.
    pub(crate) fn linear(
        &self,
        half: &[Complex32],
        range: std::ops::Range<usize>,
    ) -> Vec<Complex32> {
        let zero = |_| Complex32::ZERO;
        let lin = Taps {
            with_fb: false,
            ..self.clone()
        };
        range
            .map(|k| lin.output(half, k, &zero, &[]).to32())
            .collect()
    }

    /// Equalize symbols `first..first + count`. Symbols before `first` are `known`; after that each
    /// decision (`decide`: equalized value to the nearest unit-energy symbol) feeds back. `base`:
    /// the phase the taps were trained with, per symbol (empty: none); the phase loop follows what
    /// is left. Returns the equalized values (phase loop applied) and the total phase of every
    /// symbol `0..first + count` (0 before `first`), for [`train`].
    pub(crate) fn run(
        &self,
        half: &[Complex32],
        known: &[Complex32],
        first: usize,
        count: usize,
        base: &[f64],
        decide: impl Fn(Complex32) -> Complex32,
    ) -> (Vec<Complex32>, Vec<f64>) {
        let mut d: Vec<Complex32> = known[..first.min(known.len())].to_vec();
        d.resize(first, Complex32::ZERO);
        let mut out = Vec::with_capacity(count);
        let mut phases = vec![0.0f64; first];
        let (mut phi, mut freq) = (0.0f64, 0.0f64);
        for k in first..first + count {
            let x = input(half, k, &[]);
            let mut y = C::default();
            for (w, v) in self.ffe.iter().zip(&x) {
                y = y.add(w.mul(*v));
            }
            let total = phi + base.get(k).copied().unwrap_or(0.0);
            phases.push(total);
            let (s, c) = (-total).sin_cos();
            y = y.mul(C::new(c, s));
            if self.with_fb {
                for (j, f) in self.fb.iter().enumerate() {
                    if k > j {
                        y = y.sub(f.mul(C::of(d[k - j - 1])));
                    }
                }
            }
            let yc = y.to32();
            let dk = decide(yc);
            // phase error: angle of y conj(d), small-angle form
            let e = y.mul(C::of(dk).conj());
            let err = e.im / C::of(dk).norm_sqr().max(1e-9);
            freq += PLL_FREQ_GAIN * err;
            phi += PLL_GAIN * err + freq;
            d.push(dk);
            out.push(yc);
        }
        (out, phases)
    }
}

/// Solve the `n x n` complex system `a w = b` (row major) by Gaussian elimination with partial
/// pivoting. A singular column gives a zero tap.
fn solve(mut a: Vec<C>, mut b: Vec<C>, n: usize) -> Vec<C> {
    for col in 0..n {
        let piv = (col..n)
            .max_by(|&i, &j| {
                a[i * n + col]
                    .norm_sqr()
                    .total_cmp(&a[j * n + col].norm_sqr())
            })
            .unwrap_or(col);
        if a[piv * n + col].norm_sqr() < 1e-30 {
            continue;
        }
        if piv != col {
            for j in 0..n {
                a.swap(piv * n + j, col * n + j);
            }
            b.swap(piv, col);
        }
        let inv = {
            let p = a[col * n + col];
            p.conj().scale(1.0 / p.norm_sqr())
        };
        for row in col + 1..n {
            let f = a[row * n + col].mul(inv);
            if f.norm_sqr() == 0.0 {
                continue;
            }
            for j in col..n {
                let v = a[col * n + j];
                a[row * n + j] = a[row * n + j].sub(f.mul(v));
            }
            let v = b[col];
            b[row] = b[row].sub(f.mul(v));
        }
    }
    let mut w = vec![C::default(); n];
    for row in (0..n).rev() {
        let p = a[row * n + row];
        if p.norm_sqr() < 1e-30 {
            continue;
        }
        let mut s = b[row];
        for j in row + 1..n {
            s = s.sub(a[row * n + j].mul(w[j]));
        }
        w[row] = s.mul(p.conj().scale(1.0 / p.norm_sqr()));
    }
    w
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A symbol-spaced two-tap channel `y_k = x_k + 0.7 e^{j 2} x_{k-1}` (as half-symbol samples: the
    /// odd ones carry the mean of the neighbours): the trained DFE gives the symbols back.
    #[test]
    fn dfe_removes_a_postcursor() {
        let mut rng = crate::rng::Rng::new(4);
        let k = std::f32::consts::FRAC_1_SQRT_2;
        let n = 300;
        let sym: Vec<Complex32> = (0..n)
            .map(|_| {
                Complex32::new(
                    if rng.uniform() < 0.5 { k } else { -k },
                    if rng.uniform() < 0.5 { k } else { -k },
                )
            })
            .collect();
        let echo = Complex32::new(0.7 * 2f32.cos(), 0.7 * 2f32.sin());
        let y: Vec<Complex32> = (0..n)
            .map(|i| {
                sym[i]
                    + if i > 0 {
                        sym[i - 1] * echo
                    } else {
                        Complex32::ZERO
                    }
            })
            .collect();
        // half[2k + PRE] is symbol k's instant
        let mut half = vec![Complex32::ZERO; 2 * n + FFE];
        for i in 0..n {
            half[2 * i + PRE] = y[i];
            if i + 1 < n {
                half[2 * i + PRE + 1] = (y[i] + y[i + 1]) * 0.5;
            }
        }
        let t = train(&half, &sym[..80], true, &[]);
        assert!(t.mse < 1e-3, "mse {}", t.mse);
        let qpsk = |v: Complex32| {
            Complex32::new(
                if v.re < 0.0 { -k } else { k },
                if v.im < 0.0 { -k } else { k },
            )
        };
        let (out, _) = t.run(&half, &sym, 80, n - 80, &[], qpsk);
        for (i, v) in out.iter().enumerate() {
            assert_eq!(qpsk(*v), sym[80 + i], "symbol {}", 80 + i);
        }
    }

    #[test]
    fn solve_matches_a_known_system() {
        let a = vec![
            C::new(4.0, 0.0),
            C::new(1.0, 1.0),
            C::new(1.0, -1.0),
            C::new(3.0, 0.0),
        ];
        let w = [C::new(1.0, 2.0), C::new(-0.5, 0.25)];
        let b = vec![
            a[0].mul(w[0]).add(a[1].mul(w[1])),
            a[2].mul(w[0]).add(a[3].mul(w[1])),
        ];
        let got = solve(a, b, 2);
        for (g, w) in got.iter().zip(&w) {
            assert!(g.sub(*w).norm_sqr() < 1e-20);
        }
    }
}
