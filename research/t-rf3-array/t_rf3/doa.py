"""DOA estimation (Bartlett, MUSIC), 2-element interferometer and Cramer-Rao bounds."""
from __future__ import annotations

import numpy as np

from .array_model import steering, steering_deriv


# ---------------------------------------------------------------- data model

def gen_snapshots(rng, pos, az_deg, snr_db, K, trials=1, gains=None):
    """Single source, stochastic (Gaussian) signal. SNR = signal / noise power per
    element. Returns (trials, N, K). az_deg may be an array of length `trials`."""
    N = len(pos)
    az = np.broadcast_to(np.asarray(az_deg, dtype=float), (trials,))
    a = steering(pos, az)  # (T, N)
    if gains is not None:
        a = a * gains
    amp = 10 ** (snr_db / 20.0)
    s = amp * (rng.standard_normal((trials, K)) + 1j * rng.standard_normal((trials, K))) / np.sqrt(2)
    n = (rng.standard_normal((trials, N, K)) + 1j * rng.standard_normal((trials, N, K))) / np.sqrt(2)
    return a[:, :, None] * s[:, None, :] + n


def sample_cov(X):
    return np.einsum("tnk,tmk->tnm", X, X.conj()) / X.shape[-1]


# ---------------------------------------------------------------- spectra

def _bartlett_cost(R, A):
    # A: (T, G, N) steering at candidate angles; returns -a^H R a / N (minimise)
    return -np.real(np.einsum("tgi,tij,tgj->tg", A.conj(), R, A)) / A.shape[-1]


def _music_cost(R, A, n_sources=1):
    w, V = np.linalg.eigh(R)
    En = V[:, :, : R.shape[-1] - n_sources]  # noise subspace (smallest eigenvalues)
    proj = np.einsum("tni,tgn->tgi", En.conj(), A)  # En^H a
    return np.sum(np.abs(proj) ** 2, axis=-1)  # 1 / MUSIC pseudo-spectrum


def estimate_azimuth(R, pos, method="music", coarse_deg=1.0, zooms=(1.0, 0.05, 0.0025)):
    """Grid search + local zoom, vectorised over trials. Returns azimuth in degrees [0, 360)."""
    cost = {"music": _music_cost, "bartlett": _bartlett_cost}[method]
    T = R.shape[0]
    grid = np.arange(0.0, 360.0, coarse_deg)
    A = np.broadcast_to(steering(pos, grid), (T, len(grid), len(pos)))
    best = grid[np.argmin(cost(R, A), axis=1)]
    for h in zooms:
        loc = np.linspace(-h, h, 21)
        ang = best[:, None] + loc[None, :]
        best = ang[np.arange(T), np.argmin(cost(R, steering(pos, ang)), axis=1)]
    return np.mod(best, 360.0)


def wrap_deg(x):
    return (np.asarray(x) + 180.0) % 360.0 - 180.0


def rmse_deg(est, truth):
    return float(np.sqrt(np.mean(wrap_deg(est - truth) ** 2)))


# ---------------------------------------------------------------- CRB

def crb_single_source(a, da, snr_lin, K, stochastic=True):
    """CRB on the angle parameter (rad^2) for one source in white noise.

    a, da: steering vector and its derivative with respect to the angle.
    Closed form from Stoica & Nehorai, "MUSIC, maximum likelihood, and
    Cramer-Rao bound", IEEE Trans. ASSP 37(5), 1989 (deterministic signals)
    and "Performance study of conditional and unconditional direction-of-arrival
    estimation", IEEE Trans. ASSP 38(10), 1990 (Gaussian signals):

        CRB_det = 1 / (2 K SNR  d^H P_perp d),   P_perp = I - a a^H / (a^H a)
        CRB_sto = CRB_det (1 + 1/(N SNR))     (single source, N = number of elements)

    The Gaussian-signal formula is verified against the numerically evaluated
    Slepian-Bangs Fisher information in the tests.
    """
    N = len(a)
    dPd = np.real(np.vdot(da, da) - np.abs(np.vdot(a, da)) ** 2 / np.real(np.vdot(a, a)))
    crb = 1.0 / (2 * K * snr_lin * dPd)
    if stochastic:
        crb *= 1.0 + 1.0 / (N * snr_lin)
    return crb


def crb_azimuth_deg(pos, az_deg, snr_db, K, stochastic=True):
    """CRB on azimuth, returned as a standard deviation in degrees."""
    a = steering(pos, az_deg)
    da = steering_deriv(pos, az_deg)
    return float(np.rad2deg(np.sqrt(crb_single_source(a, da, 10 ** (snr_db / 10), K, stochastic))))


def fisher_numeric(a, da, snr_lin, K):
    """Slepian-Bangs FIM for (angle, signal power, noise power) with R = P a a^H + s2 I,
    FIM_ij = K tr(R^-1 dR_i R^-1 dR_j). Noise power fixed at 1, P = snr_lin.
    Returns the 3x3 FIM; the angle CRB is inv(FIM)[0, 0]."""
    N = len(a)
    P, s2 = snr_lin, 1.0
    R = P * np.outer(a, a.conj()) + s2 * np.eye(N)
    Ri = np.linalg.inv(R)
    dR = [P * (np.outer(da, a.conj()) + np.outer(a, da.conj())),
          np.outer(a, a.conj()), np.eye(N)]
    F = np.empty((3, 3))
    for i in range(3):
        for j in range(3):
            F[i, j] = K * np.real(np.trace(Ri @ dR[i] @ Ri @ dR[j]))
    return F


# ---------------------------------------------------------------- 2-element interferometer
# Broadside angle theta: 0 = perpendicular to the baseline, + toward +x.
# Phase difference between element 2 (+d/2) and 1 (-d/2): psi = 2 pi d sin(theta) + eps.

def interferometer_theta_deg(X, spacing, offset_rad=0.0):
    """Estimate broadside angle from 2-channel snapshots X (T, 2, K).
    Uses the phase of the cross-correlation (the ML phase estimate for one source)."""
    r12 = np.mean(X[:, 1, :] * np.conj(X[:, 0, :]), axis=-1)
    psi = np.angle(r12 * np.exp(-1j * offset_rad))
    s = np.clip(psi / (2 * np.pi * spacing), -1.0, 1.0)
    return np.rad2deg(np.arcsin(s))


def gen_pair_snapshots(rng, spacing, theta_deg, snr_db, K, trials, offset_rad=0.0):
    """Two-channel snapshots; channel 2 carries an extra fixed phase `offset_rad`
    (unknown inter-channel offset)."""
    th = np.broadcast_to(np.asarray(theta_deg, dtype=float), (trials,))
    psi = 2 * np.pi * spacing * np.sin(np.deg2rad(th)) + offset_rad
    a = np.stack([np.exp(-0.5j * psi), np.exp(0.5j * psi)], axis=1)  # (T, 2)
    amp = 10 ** (snr_db / 20.0)
    s = amp * (rng.standard_normal((trials, K)) + 1j * rng.standard_normal((trials, K))) / np.sqrt(2)
    n = (rng.standard_normal((trials, 2, K)) + 1j * rng.standard_normal((trials, 2, K))) / np.sqrt(2)
    return a[:, :, None] * s[:, None, :] + n


def crb_interferometer_deg(spacing, theta_deg, snr_db, K, stochastic=True):
    """CRB on broadside angle (deg, std) for 2 elements at `spacing` wavelengths.
    a_n = exp(j 2 pi p_n sin(theta)), da/dtheta = j 2 pi p_n cos(theta) a_n."""
    p = np.array([-spacing / 2, spacing / 2])
    th = np.deg2rad(theta_deg)
    a = np.exp(2j * np.pi * p * np.sin(th))
    da = 2j * np.pi * p * np.cos(th) * a
    return float(np.rad2deg(np.sqrt(crb_single_source(a, da, 10 ** (snr_db / 10), K, stochastic))))


def offset_bias_deg(spacing, theta_deg, offset_deg):
    """DOA error (deg) caused by an uncorrected inter-channel phase offset (exact)."""
    s = np.sin(np.deg2rad(theta_deg)) + np.deg2rad(offset_deg) / (2 * np.pi * spacing)
    return np.rad2deg(np.arcsin(np.clip(s, -1, 1))) - theta_deg
