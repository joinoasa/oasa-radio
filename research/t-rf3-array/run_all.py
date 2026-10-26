#!/usr/bin/env python3
"""Regenerate every figure in figures/ and results.md from fixed seeds.

    python run_all.py            # about 1-2 minutes
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from t_rf3 import array_model as am
from t_rf3 import beamforming as bf
from t_rf3 import diversity as dv
from t_rf3 import doa
from t_rf3 import lora_check as lc

HERE = Path(__file__).resolve().parent
FIG = HERE / "figures"
FIG.mkdir(exist_ok=True)
SEED = 20261026

BLUE, ORANGE, GREEN, RED, PURPLE, GRAY = "#0072B2", "#E69F00", "#009E73", "#D55E00", "#CC79A7", "#666666"
plt.rcParams.update({"figure.dpi": 100, "savefig.dpi": 100, "font.size": 9, "axes.grid": True,
                     "grid.alpha": 0.3, "axes.spines.top": False, "axes.spines.right": False,
                     "legend.frameon": False})
SIDES = [(1 / 3, "λ/3"), (0.5, "λ/2"), (1 / np.sqrt(3), "0.577λ (R=λ/3)"), (1.0, "λ")]
MD: list[str] = []


def md(s=""):
    MD.append(s)


def table(header, rows):
    md("| " + " | ".join(header) + " |")
    md("|" + "|".join("---" for _ in header) + "|")
    for r in rows:
        md("| " + " | ".join(str(c) for c in r) + " |")
    md()


def save(fig, name):
    p = FIG / name
    fig.savefig(p, bbox_inches="tight")
    plt.close(fig)
    kb = p.stat().st_size / 1024
    assert kb < 300, f"{name} is {kb:.0f} KB"


def f(x, n=2):
    return f"{x:.{n}f}"


# =========================================================== 1. geometry / ambiguity
def exp_geometry():
    md("## 1. Geometry and ambiguity")
    md()
    thr = am.triangle_exact_ambiguity_threshold()
    md(f"Analytic threshold for exact ambiguity: side >= 1/sqrt(3) = {thr:.4f} wavelengths "
       f"(circumradius 1/3 wavelength). Derivation in `array_model.triangle_exact_ambiguity_threshold`.")
    md()
    lam915, lam58 = am.wavelength_m(915e6), am.wavelength_m(5.8e9)
    table(["side", "at 915 MHz", "at 5.8 GHz"],
          [[lab, f"{100 * s * lam915:.1f} cm", f"{100 * s * lam58:.2f} cm"] for s, lab in SIDES])
    ss = np.arange(0.2, 1.2001, 0.01)
    D = np.array([am.distinguishability(am.triangle_positions(s), 45.0, 0.5) for s in ss])
    steer = np.arange(0, 61, 2.5)
    psl = []
    for s in ss[::4]:
        pos = am.triangle_positions(s)
        psl.append(max(am.peak_sidelobe_db(am.conventional_weights(pos, a), pos, a) for a in steer))
    psl = np.array(psl)

    fig, ax = plt.subplots(1, 3, figsize=(11, 3.4))
    pos = am.triangle_positions(0.5)
    ax[0].plot(*np.vstack([pos, pos[:1]]).T, "-o", color=BLUE)
    for i, p in enumerate(pos):
        ax[0].annotate(f"{i + 1}", p, textcoords="offset points", xytext=(5, 5))
    ax[0].plot(0, 0, "+", color=GRAY)
    ax[0].set_aspect("equal"); ax[0].set_title("Triangle, side λ/2 (R=0.289λ)\nx to the right, azimuth CCW")
    ax[0].set_xlabel("x (λ)"); ax[0].set_ylabel("y (λ)")
    ax[1].semilogy(ss, np.maximum(D, 1e-9), color=BLUE)
    ax[1].axvline(thr, color=RED, ls="--", label="1/√3 = 0.577λ")
    ax[1].set_xlabel("side length (λ)"); ax[1].set_ylabel("D = 1 − max |a₁ᴴa₂|/N")
    ax[1].set_title("Can two azimuths (≥45° apart)\nlook identical? D→0: yes"); ax[1].legend()
    ax[2].plot(ss[::4], psl, "-o", ms=3, color=BLUE)
    ax[2].axvline(thr, color=RED, ls="--")
    ax[2].set_xlabel("side length (λ)"); ax[2].set_ylabel("highest other lobe (dB)")
    ax[2].set_title("Worst-case secondary lobe,\nany steer angle (0 dB = grating lobe)")
    fig.tight_layout(); save(fig, "fig1_geometry_ambiguity.png")

    below = D[ss < thr - 0.02]; above = D[ss > thr + 0.03]
    md(f"Numerical check (pairs >= 45 deg apart, 0.5 deg grid): min D for side < 0.55 lambda is {below.min():.3f}; "
       f"max D for side > 0.60 lambda is {above.max():.1e}. Worst-case secondary lobe at side = lambda/2: "
       f"{psl[np.argmin(abs(ss[::4] - 0.5))]:.1f} dB; at side = lambda: {psl[np.argmin(abs(ss[::4] - 1.0))]:.2f} dB.")
    md()
    rows = []
    for s in (0.30, 1 / 3, 0.40, 0.45, 0.50, 0.55, thr, 0.65):
        pos = am.triangle_positions(s)
        worst = max(am.peak_sidelobe_db(am.conventional_weights(pos, a), pos, a) for a in steer)
        rows.append([f"{s:.3f}", f"{worst:.1f}", f"{am.distinguishability(pos, 45.0, 0.25):.2e}"])
    table(["side (lambda)", "worst other lobe over steer (dB)", "D (min distinguishability)"], rows)
    md("A side of lambda/2 is not ambiguous, but at some steer angles a lobe comes within about 1 dB of the main lobe, so it is close to ambiguous in noise (see 4a). Smaller sides trade beamwidth for margin.")
    md()
    md("Fig 1: `figures/fig1_geometry_ambiguity.png`")
    md()


# =========================================================== 2. patterns & steering
def exp_patterns():
    md("## 2. Beam patterns and 360 degree steering (conventional, w = a(steer))")
    md()
    fig = plt.figure(figsize=(10, 2.9))
    rows = []
    az = np.linspace(0, 360, 1441)
    for i, (s, lab) in enumerate(SIDES):
        pos = am.triangle_positions(s)
        ax = fig.add_subplot(1, 4, i + 1, projection="polar")
        for st, col, ls in [(0, BLUE, "-"), (45, ORANGE, "--")]:
            w = am.conventional_weights(pos, st)
            ax.plot(np.deg2rad(az), np.maximum(am.power_db(am.response(w, pos, az)) - 10 * np.log10(9), -30), color=col, ls=ls)
        ax.set_ylim(-30, 0); ax.set_yticks([-30, -20, -10, 0]); ax.set_yticklabels([]); ax.set_title(lab, fontsize=9, pad=10)
        steers = np.arange(0, 360, 5.0)
        bw = np.array([am.hpbw_deg(am.conventional_weights(pos, a), pos, a) for a in steers])
        sl = np.array([am.peak_sidelobe_db(am.conventional_weights(pos, a), pos, a) for a in steers])
        g = np.array([10 * np.log10(abs(am.response(am.conventional_weights(pos, a), pos, a)) ** 2) for a in steers])
        rows.append([lab, f(bw[0], 1), f"{np.nanmin(bw):.1f} – {np.nanmax(bw):.1f}", f(sl[0], 1),
                     f(np.max(sl), 1), f(g.min(), 3), f(g.max(), 3)])
    fig.suptitle("Conventional beam, steer 0° (solid) and 45° (dashed); radial scale −30…0 dB", fontsize=9, y=1.06)
    save(fig, "fig2_patterns.png")
    table(["side", "HPBW @0° (deg)", "HPBW over all steer (deg)", "peak other lobe @0° (dB)", "worst over steer (dB)",
           "gain min over steer (dB)", "gain max (dB)"], rows)
    md("Gain toward the steer direction is N^2 = 9 (9.54 dB over one element) at every steer angle: "
       "isotropic elements, so no scan loss in azimuth. HPBW varies with steer direction (3-fold symmetry). "
       "'Other lobe' = highest local maximum that is not the main lobe, relative to the main peak; it is a broad "
       "back lobe for small arrays, not a narrow sidelobe. Patterns depend only on side/lambda, so they are the same at 915 MHz and 5.8 GHz.")
    md()
    # fig3: steering
    fig, ax = plt.subplots(1, 2, figsize=(8, 3.2))
    steers = np.arange(0, 360.5, 2.0)
    pos = am.triangle_positions(0.5)
    ax[0].plot(steers, [10 * np.log10(abs(am.response(am.conventional_weights(pos, a), pos, a)) ** 2) for a in steers], color=BLUE)
    ax[0].axhline(10 * np.log10(9), color=GRAY, ls=":"); ax[0].set_ylim(8, 11)
    ax[0].set_xlabel("steer azimuth (deg)"); ax[0].set_ylabel("power gain toward steer (dB re 1 element)")
    ax[0].set_title("Steering over 360°, side λ/2")
    for (s, lab), c in zip(SIDES, [BLUE, ORANGE, GREEN, RED]):
        p = am.triangle_positions(s)
        ax[1].plot(steers, [am.hpbw_deg(am.conventional_weights(p, a), p, a) for a in steers], color=c, label=lab)
    ax[1].set_xlabel("steer azimuth (deg)"); ax[1].set_ylabel("HPBW (deg)"); ax[1].legend(fontsize=7)
    ax[1].set_title("Beamwidth vs steer direction")
    fig.tight_layout(); save(fig, "fig3_steering.png")
    md("Figs 2, 3: `figures/fig2_patterns.png`, `figures/fig3_steering.png`")
    md()


# =========================================================== 3. null steering
def exp_nulls():
    md("## 3. Null steering")
    md()
    md("Null depth = 10 log10(|w^H a_jammer|^2 / |w^H a_target|^2): the same weights' response toward the jammer relative to the target. "
       "Weights are computed for the ideal array; errors are applied to the array they are evaluated on. "
       "Target 0 deg, jammers 70 and 200 deg unless stated. Numerical floor is -300 dB.")
    md()
    rng = np.random.default_rng(SEED)
    t, jams = 0.0, [70.0, 200.0]
    # table by side length, ideal
    rows = []
    for s, lab in SIDES:
        pos = am.triangle_positions(s)
        r = [lab]
        for nl in (1, 2):
            w = bf.lcmv_weights(pos, t, jams[:nl])
            g = bf.element_errors(rng, (4000, 3), 5.0, 0.0)
            nd = bf.null_depth_db(np.broadcast_to(w, (4000, 3)), pos, t, jams[:nl], gains=g)
            r += [f(10 * np.log10(np.sum(abs(w) ** 2)), 1), f(np.median(nd.max(0)), 1)]
        rows.append(r)
    table(["side", "1 null: noise gain (dB)", "1 null: median depth @5 deg rms (dB)",
           "2 nulls: noise gain (dB)", "2 nulls: median worst-null depth @5 deg rms (dB)"], rows)
    md("Noise gain is the white-noise power gain of the weights, sum |w_n|^2, with unit response toward the target (conventional beam: 1/3 = -4.8 dB). Nulling costs SNR when a jammer is near the target.")
    md()
    # pattern example
    pos = am.triangle_positions(0.5)
    az = np.linspace(0, 360, 1441)
    fig, ax = plt.subplots(1, 3, figsize=(11, 3.4))
    w0 = bf.lcmv_weights(pos, t, jams[:0]) if False else am.conventional_weights(pos, t) / 3
    w2 = bf.lcmv_weights(pos, t, jams)
    ax[0].plot(az, am.power_db(am.response(w0, pos, az), -60), color=GRAY, label="conventional")
    ax[0].plot(az, am.power_db(am.response(w2, pos, az), -60), color=BLUE, label="2 nulls (LCMV)")
    for j in jams:
        ax[0].axvline(j, color=RED, ls=":")
    ax[0].axvline(t, color=GREEN, ls=":"); ax[0].set_ylim(-60, 5)
    ax[0].set_xlabel("azimuth (deg)"); ax[0].set_ylabel("power re target response (dB)")
    ax[0].set_title("Side λ/2: target 0°, jammers 70° and 200°"); ax[0].legend(fontsize=7)
    # null depth vs phase error
    pe = np.array([0.1, 0.3, 1, 2, 3, 5, 7, 10, 15, 20])
    n = 40000
    for nl, col in [(1, ORANGE), (2, BLUE)]:
        w = bf.lcmv_weights(pos, t, jams[:nl])
        med, mean, th = [], [], []
        for p in pe:
            g = bf.element_errors(rng, (n, 3), p, 0.0)
            lin = 10 ** (bf.null_depth_db(np.broadcast_to(w, (n, 3)), pos, t, jams[:nl], gains=g) / 10).max(0)
            med.append(10 * np.log10(np.median(lin))); mean.append(10 * np.log10(lin.mean()))
            th.append(10 * np.log10(bf.predicted_null_power(w, p, 0.0)))
        ax[1].plot(pe, med, "o-", color=col, ms=3, label=f"{nl} null: median")
        ax[1].plot(pe, mean, "s--", color=col, ms=3, label=f"{nl} null: mean power")
        ax[1].plot(pe, th, ":", color="k", lw=1)
    ax[1].set_xscale("log"); ax[1].set_xlabel("element phase error, rms (deg)"); ax[1].set_ylabel("null depth (dB)")
    ax[1].set_title("Null depth vs phase error\n(dotted: ||w||²σ² prediction)"); ax[1].legend(fontsize=6)
    # error table
    cases = [("ideal", 0, 0), ("1 deg phase", 1, 0), ("5 deg phase", 5, 0), ("10 deg phase", 10, 0),
             ("0.5 dB amplitude", 0, 0.5), ("1 dB amplitude", 0, 1.0), ("5 deg + 1 dB", 5, 1.0)]
    rows = []
    for nl in (1, 2):
        w = bf.lcmv_weights(pos, t, jams[:nl])
        for name, p, a in cases:
            if p == 0 and a == 0:
                nd = bf.null_depth_db(w, pos, t, jams[:nl])
                rows.append([nl, name, f"{nd.max():.0f} (floor)", f"{nd.max():.0f}", f"{nd.max():.0f}", "-"])
                continue
            g = bf.element_errors(rng, (n, 3), p, a)
            lin = 10 ** (bf.null_depth_db(np.broadcast_to(w, (n, 3)), pos, t, jams[:nl], gains=g) / 10).max(0)
            rows.append([nl, name, f(10 * np.log10(np.median(lin)), 1), f(10 * np.log10(lin.mean()), 1),
                         f(10 * np.log10(np.quantile(lin, 0.9)), 1), f(10 * np.log10(bf.predicted_null_power(w, p, a)), 1)])
    table(["nulls", "element errors (rms)", "median depth (dB)", "mean-power depth (dB)", "90th pct worst (dB)", "theory (dB)"], rows)
    md("Theory = sum|w_n|^2 (sigma_phase^2 + sigma_amp^2), the small-error prediction of mean residual power. Mean-power depth is 10 log10 of the average residual power (dominated by bad draws); 90th pct = the depth that 10% of arrays do worse than. "
       "With two nulls the worst of the two is reported.")
    md()
    # quantisation, random scenarios
    n = 3000
    bits_list = [3, 4, 5, 6, 7, 8]
    qrows = []
    ang = []
    while len(ang) < n:
        a = rng.uniform(0, 360, 3)
        d = np.abs(am.wrap_deg(a[:, None] - a[None, :]) if hasattr(am, "wrap_deg") else doa.wrap_deg(a[:, None] - a[None, :]))
        if np.all(d[np.triu_indices(3, 1)] > 25):
            ang.append(a)
    ang = np.array(ang)
    qmed = {}
    for nl in (1, 2):
        for b in bits_list:
            depths, th = [], []
            for a in ang:
                w = bf.lcmv_weights(pos, a[0], a[1:1 + nl])
                wq = bf.quantize_phase(w, b)
                depths.append(bf.null_depth_db(wq, pos, a[0], a[1:1 + nl]).max())
                step = 2 * np.pi / 2 ** b
                th.append(np.sum(abs(w) ** 2) * step ** 2 / 12)
            lin = 10 ** (np.array(depths) / 10)
            qmed[(nl, b)] = 10 * np.log10(np.median(lin))
            qrows.append([nl, b, f"{360 / 2 ** b:.2f}", f(10 * np.log10(np.median(lin)), 1), f(10 * np.log10(lin.mean()), 1),
                          f(10 * np.log10(np.quantile(lin, 0.9)), 1), f(10 * np.log10(np.mean(th)), 1)])
    for nl, col in [(1, ORANGE), (2, BLUE)]:
        ax[2].plot(bits_list, [qmed[(nl, b)] for b in bits_list], "o-", color=col, label=f"{nl} null, median")
    ax[2].set_xlabel("phase-shifter bits"); ax[2].set_ylabel("null depth (dB)")
    ax[2].set_title("Quantised phase (random scenes)"); ax[2].legend(fontsize=7)
    fig.tight_layout(); save(fig, "fig4_nulls.png")
    md("### Quantised phase shifters (weights' phases rounded, amplitudes ideal; 3000 random target/jammer scenes, >25 deg apart, side lambda/2)")
    md()
    table(["nulls", "bits", "step (deg)", "median depth (dB)", "mean-power depth (dB)", "90th pct worst (dB)", "theory mean (dB)"], qrows)
    md("Theory = mean of sum|w_n|^2 step^2/12. With two nulls the mean is dominated by scenes where a jammer is close to the target (huge weights), where the small-error theory fails; use the median and 90th percentile.")
    md()
    md("Fig 4: `figures/fig4_nulls.png`")
    md()


# =========================================================== 3b. MVDR
def exp_mvdr():
    md("## 3b. MVDR with sample covariance")
    md()
    md("Side lambda/2, target 0 deg (steering vector known exactly), two jammers (60, 200 deg) at 30 dB INR each, unit noise, "
       "target SNR 10 dB per element. SINR improvement = output SINR / single-element input SINR (input SINR = 10 dB - 33 dB = about -23 dB). "
       "Training either excludes the target (signal-free) or contains it (signal-present, the realistic case).")
    md()
    pos = am.triangle_positions(0.5)
    t, jams, jnr, snr = 0.0, [60.0, 200.0], [30.0, 30.0], 10.0
    a_t = am.steering(pos, t)
    Ks = [3, 4, 6, 10, 20, 50, 100, 300, 1000]
    trials = 300
    rng = np.random.default_rng(SEED + 1)
    _, R_in = bf.simulate_snapshots(rng, pos, 4, t, snr, jams, jnr, False)
    sinr_opt = bf.optimum_sinr(a_t, R_in, 10 ** (snr / 10))
    sinr_in = 10 ** (snr / 10) / (np.trace(R_in).real / 3)
    w_c = a_t / 3
    sinr_conv = bf.output_sinr(w_c, a_t, R_in, 10 ** (snr / 10))
    res = {k: [] for k in ("free", "present")}
    for K in Ks:
        acc = {k: [] for k in res}
        for _ in range(trials):
            X, _r = bf.simulate_snapshots(rng, pos, K, t, snr, jams, jnr, False)
            Xp, _r = bf.simulate_snapshots(rng, pos, K, t, snr, jams, jnr, True)
            R, Rp = X @ X.conj().T / K, Xp @ Xp.conj().T / K
            for key, RR in (("free", R), ("present", Rp)):
                w = bf.mvdr_weights(RR, a_t)
                acc[key].append(bf.output_sinr(w, a_t, R_in, 10 ** (snr / 10)))
        for key in res:
            res[key].append(np.mean(acc[key]) / sinr_opt)
    fig, ax = plt.subplots(1, 2, figsize=(9, 3.4))
    ax[0].semilogx(Ks, [10 * np.log10(x) for x in res["free"]], "o-", color=BLUE, label="signal-free training")
    ax[0].semilogx(Ks, [10 * np.log10(bf.smi_loss_theory(K, 3)) for K in Ks], "k:", label="theory (K−1)/(K+1)")
    ax[0].semilogx(Ks, [10 * np.log10(x) for x in res["present"]], "s-", color=RED, label="target in training")
    ax[0].set_xlabel("snapshots K"); ax[0].set_ylabel("mean SINR / optimum (dB)"); ax[0].legend(fontsize=7)
    ax[0].set_title("SMI loss vs snapshots")
    az = np.linspace(0, 360, 1441)
    Xp, _r = bf.simulate_snapshots(rng, pos, 100, t, snr, jams, jnr, False)
    w = bf.mvdr_weights(Xp @ Xp.conj().T / 100, a_t)
    ax[1].plot(az, am.power_db(am.response(w_c, pos, az), -60), color=GRAY, label="conventional")
    ax[1].plot(az, am.power_db(am.response(w, pos, az), -60), color=BLUE, label="MVDR, K=100")
    for j in jams:
        ax[1].axvline(j, color=RED, ls=":")
    ax[1].set_ylim(-60, 5); ax[1].set_xlabel("azimuth (deg)"); ax[1].set_ylabel("power (dB), unit target gain for MVDR")
    ax[1].set_title("MVDR pattern, one realisation"); ax[1].legend(fontsize=7)
    fig.tight_layout(); save(fig, "fig5_mvdr.png")
    rows = [[K, f(10 * np.log10(res["free"][i]), 2), f(10 * np.log10(bf.smi_loss_theory(K, 3)), 2),
             f(10 * np.log10(res["present"][i]), 2),
             f(10 * np.log10(res["free"][i] * sinr_opt / sinr_in), 1), f(10 * np.log10(res["present"][i] * sinr_opt / sinr_in), 1)]
            for i, K in enumerate(Ks)]
    table(["K", "free: loss (dB)", "theory (dB)", "target in training: loss (dB)",
           "free: SINR improvement (dB)", "target in training: improvement (dB)"], rows)
    md(f"Optimum SINR {10 * np.log10(sinr_opt):.1f} dB; improvement of the optimum over one element {10 * np.log10(sinr_opt / sinr_in):.1f} dB; "
       f"conventional beam improvement {10 * np.log10(sinr_conv / sinr_in):.1f} dB (white-noise array gain is 4.8 dB; the rest is the jammers landing in low-gain directions). "
       "Theory: Reed, Mallett, Brennan 1974 (signal-free training). Self-nulling with the target in training is the known failure of "
       "SMI-MVDR (signal-free training avoids it; diagonal loading of 1% to 100% of the mean diagonal was tried once and did not help in this jammer-dominated scene, not part of this script). Losses are averages in linear power, shown in dB.")
    md()
    md("Fig 5: `figures/fig5_mvdr.png`")
    md()
    return res


# =========================================================== 4a. DOA triangle
def exp_doa_triangle():
    md("## 4a. DOA with the 3-element triangle (single source)")
    md()
    md("Gaussian source signal, unit-power white noise, SNR per element; azimuth uniform on [0, 360) per trial; 500 trials per point; "
       "Bartlett and MUSIC on the sample covariance, grid search plus zoom refinement. CRB = stochastic, closed form (Stoica & Nehorai 1990), "
       "checked against numerical Fisher information in the tests. The CRB does not depend on azimuth for this triangle.")
    md()
    rng = np.random.default_rng(SEED + 2)
    snrs = np.arange(-10, 31, 5)
    T = 500
    fig, ax = plt.subplots(1, 3, figsize=(11.5, 3.5))
    rows = []
    for (s, lab), col in zip(SIDES[:2], [ORANGE, BLUE]):
        pos = am.triangle_positions(s)
        for method, mk, ls in [("music", "o", "-"), ("bartlett", "x", "--")]:
            r = []
            for snr in snrs:
                az = rng.uniform(0, 360, T)
                est = doa.estimate_azimuth(doa.sample_cov(doa.gen_snapshots(rng, pos, az, snr, 100, T)), pos, method)
                r.append(doa.rmse_deg(est, az))
            ax[0].semilogy(snrs, r, mk + ls, color=col, ms=4, label=f"{lab} {method}")
            if method == "music":
                rows += [[lab, snr, f(x, 3 if x < 10 else 1), f(doa.crb_azimuth_deg(pos, 0, snr, 100), 3), f(x / doa.crb_azimuth_deg(pos, 0, snr, 100), 2)]
                         for snr, x in zip(snrs, r) if snr in (-5, 0, 10, 20, 30)]
        ax[0].semilogy(snrs, [doa.crb_azimuth_deg(pos, 0, x, 100) for x in snrs], ":", color=col, lw=2)
    ax[0].set_xlabel("SNR per element (dB)"); ax[0].set_ylabel("RMSE (deg)"); ax[0].set_title("K = 100; dotted = CRB")
    ax[0].legend(fontsize=6)
    Ks = [5, 10, 20, 50, 100, 300, 1000]
    pos = am.triangle_positions(0.5)
    rk, ck = [], []
    for K in Ks:
        az = rng.uniform(0, 360, T)
        est = doa.estimate_azimuth(doa.sample_cov(doa.gen_snapshots(rng, pos, az, 10, K, T)), pos, "music")
        rk.append(doa.rmse_deg(est, az)); ck.append(doa.crb_azimuth_deg(pos, 0, 10, K))
    ax[1].loglog(Ks, rk, "o-", color=BLUE, label="MUSIC, λ/2")
    ax[1].loglog(Ks, ck, ":", color=BLUE, lw=2, label="CRB")
    ax[1].set_xlabel("snapshots K"); ax[1].set_ylabel("RMSE (deg)"); ax[1].set_title("SNR 10 dB"); ax[1].legend(fontsize=7)
    table(["side", "SNR (dB)", "MUSIC RMSE (deg)", "CRB (deg)", "RMSE/CRB"], rows)
    md("K = 100 snapshots. Below about -5 dB the estimator is in its threshold region (outliers dominate RMSE); "
       "at 0 dB and above RMSE is within a few percent of the CRB.")
    md()
    md("Snapshots at SNR 10 dB:")
    md()
    table(["K", "MUSIC RMSE (deg)", "CRB (deg)", "ratio"], [[K, f(a, 3), f(b, 3), f(a / b, 2)] for K, a, b in zip(Ks, rk, ck)])
    # side length / ambiguity effect
    rows = []
    sides = [0.3, 1 / 3, 0.4, 0.5, 1 / np.sqrt(3), 0.65, 0.8, 1.0]
    for s in sides:
        pos = am.triangle_positions(s)
        r = [f"{s:.3f}"]
        for snr in (-5, 0, 10, 30):
            az = rng.uniform(0, 360, 800)
            est = doa.estimate_azimuth(doa.sample_cov(doa.gen_snapshots(rng, pos, az, snr, 100, 800)), pos, "music")
            err = np.abs(doa.wrap_deg(est - az)); out = err > 5 * doa.crb_azimuth_deg(pos, 0, snr, 100)
            r.append(f"{100 * out.mean():.1f}%")
            if snr == 10:
                r.append(f(np.sqrt(np.mean(err[~out] ** 2)) / doa.crb_azimuth_deg(pos, 0, snr, 100), 2))
        rows.append(r)
    md("Outliers (error > 5 x the CRB standard deviation, which a Gaussian error would essentially never exceed), MUSIC, K = 100, 800 trials:")
    md()
    table(["side (λ)", "-5 dB", "0 dB", "10 dB", "10 dB: inlier RMSE / CRB", "30 dB"], rows)
    md("Larger apertures have a lower CRB. Up to 0.4 lambda there are no outliers down to -5 dB; at lambda/2 a few appear at -5 dB (a lobe within 1 dB of the main lobe), "
       "and once side >= 0.577 lambda the manifold crosses itself so some azimuths are mistaken for others even at 30 dB. "
       "That is the ambiguity of section 1 seen in estimation. Inlier RMSE still follows the CRB.")
    md()
    # panel 3: outlier vs azimuth for side 1.0
    pos = am.triangle_positions(1.0)
    azg = np.arange(0, 360, 1.0)
    X = doa.gen_snapshots(rng, pos, azg, 30, 100, len(azg))
    est = doa.estimate_azimuth(doa.sample_cov(X), pos, "music")
    ax[2].plot(azg, doa.wrap_deg(est - azg), ".", ms=3, color=RED)
    ax[2].set_xlabel("true azimuth (deg)"); ax[2].set_ylabel("error (deg)"); ax[2].set_title("Side λ, 30 dB: errors by azimuth\n(ambiguous directions)")
    fig.tight_layout(); save(fig, "fig6_doa_triangle.png")
    md("Fig 6: `figures/fig6_doa_triangle.png`")
    md()


# =========================================================== 4b. interferometer
def exp_interferometer():
    md("## 4b. Two-channel phase interferometer (E310 case)")
    md()
    d = 0.5
    lam = am.wavelength_m(915e6)
    md(f"915 MHz: lambda = {100 * lam:.2f} cm, spacing d = lambda/2 = {100 * lam / 2:.1f} cm. Broadside angle theta: 0 = perpendicular to the baseline. "
       "Phase difference psi = 2 pi (d/lambda) sin(theta) + eps, eps = unknown inter-channel offset.")
    md()
    rng = np.random.default_rng(SEED + 3)
    fig, ax = plt.subplots(1, 4, figsize=(14, 3.4))
    # (a) front/back
    phi = np.arange(0, 360.5, 1.0)
    th = np.rad2deg(np.arcsin(np.cos(np.deg2rad(phi))))
    X = doa.gen_pair_snapshots(rng, d, th, 200, 4, len(phi))
    th_hat = doa.interferometer_theta_deg(X, d)
    phi_hat = np.rad2deg(np.arccos(np.sin(np.deg2rad(th_hat))))  # assume source in front (y > 0)
    ax[0].plot(phi, phi_hat, ".", ms=3, color=BLUE)
    ax[0].plot(phi, phi, color=GRAY, lw=0.8)
    ax[0].set_xlabel("true azimuth (deg, baseline along 0°–180°)"); ax[0].set_ylabel("reported azimuth (front assumed)")
    ax[0].set_title("Front/back fold: 180°–360° mirrors 0°–180°")
    # exact mirror
    m1 = doa.gen_pair_snapshots(np.random.default_rng(0), d, 30.0, 40, 8, 1)
    # mirror-source has identical steering vector: show by steering vectors
    p2 = am.pair_positions(d)
    mirr = float(np.max(np.abs(am.steering(p2, 60.0) - am.steering(p2, -60.0))))
    # (b) rmse vs SNR
    snrs = np.arange(-10, 31, 5)
    for tdeg, col in [(0, BLUE), (30, ORANGE), (60, RED)]:
        r = []
        for snr in snrs:
            X = doa.gen_pair_snapshots(rng, d, tdeg, snr, 128, 3000)
            r.append(np.sqrt(np.mean((doa.interferometer_theta_deg(X, d) - tdeg) ** 2)))
        ax[1].semilogy(snrs, r, "o-", ms=3, color=col, label=f"θ={tdeg}°")
        ax[1].semilogy(snrs, [doa.crb_interferometer_deg(d, tdeg, x, 128) for x in snrs], ":", color=col, lw=2)
    ax[1].set_xlabel("SNR per channel (dB)"); ax[1].set_ylabel("RMSE (deg)"); ax[1].set_title("d=λ/2, K=128; dotted = CRB"); ax[1].legend(fontsize=7)
    rows = []
    for tdeg in (0, 30, 60):
        for snr in (0, 10, 20):
            X = doa.gen_pair_snapshots(rng, d, tdeg, snr, 128, 4000)
            e = doa.interferometer_theta_deg(X, d)
            r = np.sqrt(np.mean((e - tdeg) ** 2)); c = doa.crb_interferometer_deg(d, tdeg, snr, 128)
            rows.append([tdeg, snr, f(r, 2), f(c, 2), f(r / c, 2)])
    table(["theta (deg)", "SNR (dB)", "RMSE (deg)", "CRB (deg)", "ratio"], rows)
    # (c) offset bias
    eps = np.linspace(-30, 30, 121)
    for tdeg, col in [(0, BLUE), (30, ORANGE), (60, RED)]:
        ax[2].plot(eps, doa.offset_bias_deg(d, tdeg, eps), color=col, label=f"θ={tdeg}°")
    ax[2].set_xlabel("uncorrected channel phase offset (deg)"); ax[2].set_ylabel("DOA error (deg)")
    ax[2].set_title("Why calibration is required (d=λ/2)"); ax[2].legend(fontsize=7)
    rows = []
    for e0 in (1, 2, 5, 10, 30):
        rows.append([e0] + [f(doa.offset_bias_deg(d, t, e0), 2) for t in (0, 30, 60)])
    table(["offset eps (deg)", "error @ theta=0", "@30", "@60"], rows)
    md("Rule of thumb at d = lambda/2: error = eps/(pi cos theta) radians, i.e. about 0.32 deg per degree of offset at broadside, growing as 1/cos theta.")
    md()
    # (d) LoRa cross-correlation
    sf, nsym = 7, 30
    snrs2 = np.array([-20, -15, -10, -7.5, -5, 0, 5, 10])
    r2, c2 = [], []
    for snr in snrs2:
        est, K = lc.lora_pair_doa(rng, sf, nsym, snr, d, 30.0, 1500)
        r2.append(np.sqrt(np.mean((est - 30.0) ** 2))); c2.append(doa.crb_interferometer_deg(d, 30.0, snr, K, stochastic=False))
    ax[3].semilogy(snrs2, r2, "o-", ms=3, color=BLUE, label="simulated")
    ax[3].semilogy(snrs2, c2, ":", color=BLUE, lw=2, label="CRB (K=3840)")
    ax[3].set_xlabel("SNR per sample in 125 kHz (dB)"); ax[3].set_ylabel("RMSE (deg)")
    ax[3].set_title("LoRa SF7, 30 symbols, θ=30°"); ax[3].legend(fontsize=7)
    fig.tight_layout(); save(fig, "fig7_interferometer.png")
    md(f"### LoRa symbol train through the interferometer (SF7, BW 125 kHz, 30 symbols = {30 * 128} samples at fs = BW, cross-correlation of the two channels, theta = 30 deg)")
    md()
    table(["SNR per sample (dB)", "RMSE (deg)", "CRB (deg)", "ratio"], [[s, f(a, 3), f(b, 3), f(a / b, 2)] for s, a, b in zip(snrs2, r2, c2)])
    md("SNR here is per complex sample in the 125 kHz noise bandwidth (SF7 demodulation threshold is about -7.5 dB on this scale). "
       "Sample noise is assumed white and independent at fs = BW.")
    md()
    # calibration residual budget
    md("### Error budget with residual calibration error (K = 3840, SNR 0 dB, d = lambda/2)")
    md()
    rows = []
    for sc in (0, 1, 2, 5):
        r = [sc]
        for t in (0, 30, 60):
            stat = doa.crb_interferometer_deg(d, t, 0, 3840, stochastic=False)
            sysb = abs(doa.offset_bias_deg(d, t, sc))
            r.append(f(np.hypot(stat, sysb), 2))
        rows.append(r)
    table(["cal. residual (deg)", "total @theta=0 (deg)", "@30", "@60"], rows)
    md("Statistical part is the CRB; the calibration residual is treated as a fixed bias added in quadrature.")
    md()
    md(f"Front/back: with both elements on the x axis, azimuths phi and -phi have identical steering vectors (max difference {mirr:.1e}), "
       "so a 2-element array cannot tell front from back, and only sin(theta) is observed: theta is limited to +/-90 deg, and with d = lambda/2 the phase just reaches +/-180 deg at endfire. "
       "d > lambda/2 would also wrap the phase and add more ambiguities.")
    md()
    md("Fig 7: `figures/fig7_interferometer.png`")
    md()


# =========================================================== 4c. diversity
def exp_diversity():
    md("## 4c. Diversity gain, 2 branches, Rayleigh fading")
    md()
    rng = np.random.default_rng(SEED + 4)
    n = 2_000_000
    h1, h2 = dv.rayleigh_branches(rng, n, 0.0)
    g1, gsc, gmrc = dv.combine(h1, h2)
    rows = [["single", "0.00", f(dv.db(g1.mean()), 2), "1.00"],
            ["selection (SC)", f(dv.db(1.5), 2), f(dv.db(gsc.mean()), 2), f(gsc.mean(), 3)],
            ["MRC", f(dv.db(2.0), 2), f(dv.db(gmrc.mean()), 2), f(gmrc.mean(), 3)]]
    md("Mean output SNR relative to the mean SNR of one branch (independent branches):")
    md()
    table(["scheme", "theory (dB)", "simulated (dB)", "simulated ratio"], rows)
    rows = []
    for p in (0.1, 0.01):
        base = dv.outage_snr_theory(p, "single")
        row = [f"{100 * p:g}%"]
        for sch, samp in (("sc", gsc), ("mrc", gmrc)):
            th = dv.db(dv.outage_snr_theory(p, sch) / base)
            si = dv.db(dv.outage_snr_sim(samp, p) / dv.outage_snr_sim(g1, p))
            row += [f(th, 2), f(si, 2)]
        rows.append(row)
    md("Outage diversity gain = how much lower the mean SNR can be for the same outage probability than with one branch (ratio of the SNR thresholds at which P[SNR < threshold] equals the outage probability):")
    md()
    table(["outage prob", "SC theory (dB)", "SC sim", "MRC theory (dB)", "MRC sim"], rows)
    fig, ax = plt.subplots(1, 3, figsize=(11.5, 3.4))
    xs = np.logspace(-3, 1.2, 200)
    for samp, th, lab, col in [(g1, lambda x: 1 - np.exp(-x), "single", GRAY),
                               (gsc, lambda x: (1 - np.exp(-x)) ** 2, "SC", ORANGE),
                               (gmrc, lambda x: 1 - np.exp(-x) * (1 + x), "MRC", BLUE)]:
        srt = np.sort(samp)
        ax[0].semilogy(10 * np.log10(xs), th(xs), "-", color=col, label=f"{lab} theory")
        pts = np.logspace(-3.5, -0.05, 25)
        ax[0].semilogy(10 * np.log10(np.quantile(srt, pts)), pts, "o", ms=3, color=col, label=f"{lab} sim" if lab == "MRC" else None)
    ax[0].set_xlim(-30, 10); ax[0].set_ylim(1e-4, 1)
    ax[0].set_xlabel("SNR / mean branch SNR (dB)"); ax[0].set_ylabel("P(SNR < x)"); ax[0].set_title("Outage CDF"); ax[0].legend(fontsize=7)
    # spacing
    ds = [0.05, 0.125, 0.25, 0.5, 1.0]
    rows = []
    gm, gs = [], []
    for dd in ds:
        rho = dv.isotropic_scatter_corr(dd)
        a, b = dv.rayleigh_branches(rng, 1_000_000, rho)
        s1, ssc, smrc = dv.combine(a, b)
        base = dv.outage_snr_sim(s1, 0.01)
        gm.append(dv.db(dv.outage_snr_sim(smrc, 0.01) / base)); gs.append(dv.db(dv.outage_snr_sim(ssc, 0.01) / base))
        th = dv.db(dv.outage_snr_theory(0.01, "mrc", rho) / dv.outage_snr_theory(0.01, "single"))
        rows.append([dd, f(rho, 3), f(gs[-1], 2), f(gm[-1], 2), f(th, 2), f(dv.db(smrc.mean() / s1.mean()), 2)])
    ax[1].semilogx(ds, gm, "o-", color=BLUE, label="MRC"); ax[1].semilogx(ds, gs, "s-", color=ORANGE, label="SC")
    ax[1].set_xlabel("antenna spacing (λ)"); ax[1].set_ylabel("1% outage gain (dB)")
    ax[1].set_title("Effect of branch correlation\n(isotropic scattering, |J0|)"); ax[1].legend(fontsize=7)
    md("Spacing between the two antennas, isotropic 2-D scattering (field correlation |J0(2 pi d/lambda)|):")
    md()
    table(["d (lambda)", "|rho|", "SC 1% gain (dB)", "MRC 1% gain (dB)", "MRC theory (dB)", "MRC mean gain (dB)"], rows)
    md(f"At 915 MHz lambda/2 is {100 * am.wavelength_m(915e6) / 2:.1f} cm: branches are already nearly uncorrelated (|rho| = {dv.isotropic_scatter_corr(.5):.2f}).")
    md()
    # estimated-weight MRC
    L = 1024
    rows = []
    for gdb in (-7.5, 0, 10):
        gb = 10 ** (gdb / 10)
        h = np.sqrt(gb / 2) * (rng.standard_normal((2, 400_000)) + 1j * rng.standard_normal((2, 400_000)))
        e = np.sqrt(1 / (2 * L)) * (rng.standard_normal(h.shape) + 1j * rng.standard_normal(h.shape))
        hh = h + e
        out = np.abs(np.sum(np.conj(hh) * h, axis=0)) ** 2 / np.sum(np.abs(hh) ** 2, axis=0)
        ideal = np.sum(np.abs(h) ** 2, axis=0)
        rows.append([gdb, f(-dv.db(out.mean() / ideal.mean()), 3)])
    md(f"MRC with weights estimated from a {L}-sample preamble (8 SF7 symbols) instead of the true channel; loss vs ideal MRC:")
    md()
    table(["mean SNR per sample per branch (dB)", "loss (dB)"], rows)
    ax[2].axis("off")
    ax[2].text(0, 0.9, "Theory used\n\nsingle: P = 1 − e^(−x)\nSC: P = (1 − e^(−x))²\nMRC: P = 1 − e^(−x)(1 + x)\n\nmean gain: SC 1.5, MRC 2.0\n(x = SNR / mean branch SNR)", va="top", fontsize=9)
    fig.tight_layout(); save(fig, "fig8_diversity.png")
    md("Fig 8: `figures/fig8_diversity.png`")
    md()


# =========================================================== 5. LoRa narrowband
def exp_lora():
    md("## 5. Narrowband validity for the LoRa source")
    md()
    md("Triangle of side lambda/2. 'B*tau' is bandwidth times the longest inter-element delay (needs << 1). "
       "'Edge loss' is the worst-case beamforming loss (over 360 deg) at the band edge when phases are set for the carrier. "
       "'Edge phase' is the phase error across the longest baseline at the band edge.")
    md()
    rows = [[r["name"], f"{r['frac_bw']:.2e}", f"{r['tau_ps']:.0f} ps", f"{r['bt']:.1e}", f"{r['loss_db']:.1e} dB", f"{r['edge_phase_deg']:.3f} deg"]
            for r in lc.narrowband_table(0.5)]
    table(["signal", "BW/fc", "max delay", "B*tau", "edge loss", "edge phase"], rows)
    rel, perr = lc.chirp_delay_error()
    md(f"Waveform check: an SF7 BW125k chirp delayed by the full lambda/2 baseline delay (theta = 40 deg) differs from the phase-only model by "
       f"{rel:.1e} relative amplitude, and the cross-correlation phase differs from the true carrier phase by {perr:.1e} deg. "
       "A full chirp is spectrally centred, so the delay-induced phase bias is about zero.")
    md()
    md("Practical consequence: the 2-channel DOA needs no dechirping. Cross-correlating the two channels (sum of x2 * conj(x1)) cancels the chirp "
       "and keeps only the inter-channel phase. Dechirp/FFT is still useful to find packet start and to pick the samples that contain signal.")
    md()


def main():
    t0 = time.time()
    md("# T-RF3 simulation results")
    md()
    md(f"Generated by `run_all.py`, seed {SEED}, numpy {np.__version__}. Do not edit by hand.")
    md()
    for fn in (exp_geometry, exp_patterns, exp_nulls, exp_mvdr, exp_doa_triangle, exp_interferometer, exp_diversity, exp_lora):
        t = time.time()
        fn()
        print(f"{fn.__name__}: {time.time() - t:.1f}s", flush=True)
    (HERE / "results.md").write_text("\n".join(MD) + "\n")
    print(f"done in {time.time() - t0:.0f}s")


if __name__ == "__main__":
    sys.exit(main())
