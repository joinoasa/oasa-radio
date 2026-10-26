"""Null steering (LCMV / projection) and MVDR from sample covariance.

Null depth definition used everywhere in this package:

    null_depth_dB = 10 log10( |w^H a_j|^2 / |w^H a_t|^2 )

the response of the SAME weights toward the jammer direction, relative to the
response toward the target. Both are evaluated on the array that actually
exists (with element errors), while the weights are computed from the ideal
model. It is not relative to a single element or to the unconstrained pattern.
"""
from __future__ import annotations

import numpy as np

from .array_model import steering


def lcmv_weights(pos, target_deg, null_degs) -> np.ndarray:
    """Minimum-norm weights with w^H a_t = 1 and w^H a_j = 0 for each null.

    Equivalent to w = P a_t / (a_t^H P a_t)^... for one null (projection), and the
    unique solution for two nulls on 3 elements. Min-norm = best white-noise gain.
    """
    nulls = np.atleast_1d(np.asarray(null_degs, dtype=float))
    if len(nulls) > len(pos) - 1:
        raise ValueError("at most N-1 nulls")
    C = np.column_stack([steering(pos, target_deg)] + [steering(pos, n) for n in nulls])
    f = np.zeros(C.shape[1], dtype=complex)
    f[0] = 1.0
    return C @ np.linalg.solve(C.conj().T @ C, f)


def null_depth_db(w, pos, target_deg, jammer_deg, gains=None, floor_db=-300.0) -> np.ndarray:
    """Null depth (see module docstring). `gains` is an optional per-element complex
    error vector g (array manifold becomes g * a), shape (N,) or (T, N) with w (T, N)."""
    a_t = steering(pos, target_deg)
    jam = np.atleast_1d(jammer_deg)
    out = []
    for j in jam:
        a_j = steering(pos, j)
        if gains is not None:
            a_t_e, a_j_e = gains * a_t, gains * a_j
        else:
            a_t_e, a_j_e = a_t, a_j
        num = np.abs(np.sum(np.conj(w) * a_j_e, axis=-1)) ** 2
        den = np.abs(np.sum(np.conj(w) * a_t_e, axis=-1)) ** 2
        out.append(np.maximum(10 * np.log10(np.maximum(num / den, 1e-300)), floor_db))
    return np.array(out)


def element_errors(rng, shape, phase_rms_deg=0.0, amp_rms_db=0.0) -> np.ndarray:
    """Complex per-element gain errors g = (1+da) e^{j dphi}; Gaussian, zero mean."""
    ph = np.deg2rad(phase_rms_deg) * rng.standard_normal(shape)
    amp = 10 ** (amp_rms_db * rng.standard_normal(shape) / 20.0)
    return amp * np.exp(1j * ph)


def quantize_phase(w, bits: int) -> np.ndarray:
    """Quantise weight phases to 2^bits uniform states; amplitudes kept ideal
    (a real phase-shifter + attenuator front end would also quantise amplitude)."""
    step = 2 * np.pi / (2 ** bits)
    return np.abs(w) * np.exp(1j * step * np.round(np.angle(w) / step))


def predicted_null_power(w, phase_rms_deg, amp_rms_db) -> np.ndarray:
    """Mean residual null power (linear, relative to target response 1) for small
    zero-mean errors: ||w||^2 (sigma_phi^2 + sigma_a^2). The mean error of g does
    not contribute because the ideal response at the null is zero."""
    var = np.deg2rad(phase_rms_deg) ** 2 + (amp_rms_db * np.log(10) / 20.0) ** 2
    return np.sum(np.abs(w) ** 2, axis=-1) * var


# ---------------------------------------------------------------- MVDR / SMI

def simulate_snapshots(rng, pos, K, target_deg, snr_db, jam_degs=(), jnr_db=(), signal_present=True):
    """Complex Gaussian snapshots (N, K): unit-power noise + jammers (+ target).
    Returns X and the true interference+noise covariance."""
    N = len(pos)
    X = (rng.standard_normal((N, K)) + 1j * rng.standard_normal((N, K))) / np.sqrt(2)
    R_in = np.eye(N, dtype=complex)
    for az, jnr in zip(jam_degs, jnr_db):
        a = steering(pos, az)
        p = 10 ** (jnr / 10)
        s = np.sqrt(p / 2) * (rng.standard_normal(K) + 1j * rng.standard_normal(K))
        X += np.outer(a, s)
        R_in += p * np.outer(a, a.conj())
    if signal_present:
        a = steering(pos, target_deg)
        p = 10 ** (snr_db / 10)
        s = np.sqrt(p / 2) * (rng.standard_normal(K) + 1j * rng.standard_normal(K))
        X += np.outer(a, s)
    return X, R_in


def mvdr_weights(R, a, loading=0.0):
    """w = R^-1 a / (a^H R^-1 a); optional diagonal loading (loading x mean diag)."""
    Rl = R + loading * np.real(np.trace(R)) / len(R) * np.eye(len(R))
    Ri_a = np.linalg.solve(Rl, a)
    return Ri_a / (a.conj() @ Ri_a)


def output_sinr(w, a_t, R_in, snr_lin=1.0):
    """SINR at the beamformer output: sigma_s^2 |w^H a|^2 / (w^H R_in w)."""
    num = snr_lin * np.abs(np.vdot(w, a_t)) ** 2
    den = np.real(np.vdot(w, R_in @ w))
    return num / den


def optimum_sinr(a_t, R_in, snr_lin=1.0):
    return snr_lin * np.real(a_t.conj() @ np.linalg.solve(R_in, a_t))


def smi_loss_theory(K, N):
    """E[SINR/SINR_opt] for MVDR/SMI with signal-free training:
    (K+2-N)/(K+1). Reed, Mallett, Brennan, IEEE Trans. AES 10(6), 1974."""
    return (K + 2 - N) / (K + 1)
