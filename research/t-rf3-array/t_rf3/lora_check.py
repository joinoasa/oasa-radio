"""Is the narrowband model valid for LoRa? And DOA from a 2-channel LoRa symbol train."""
from __future__ import annotations

import numpy as np

from .array_model import C0, steering, triangle_positions


def lora_symbol(sf: int, sym: int = 0) -> np.ndarray:
    """Baseband LoRa upchirp symbol sampled at fs = BW (2^SF samples), cyclically shifted by `sym`.
    Periodic in 2^SF samples, constant envelope, sweeps -BW/2 .. +BW/2."""
    N = 2 ** sf
    n = np.arange(N)
    base = np.exp(1j * np.pi * (n ** 2 / N - n))
    return np.roll(base, -sym)


def fractional_delay(x: np.ndarray, delay_samples: float) -> np.ndarray:
    """Exact delay of a periodic band-limited sequence via the FFT."""
    f = np.fft.fftfreq(len(x))
    return np.fft.ifft(np.fft.fft(x) * np.exp(-2j * np.pi * f * delay_samples))


def max_array_delay_s(pos_wl: np.ndarray, freq_hz: float) -> float:
    """Largest possible inter-element propagation delay (longest baseline / c)."""
    lam = C0 / freq_hz
    d = np.linalg.norm(pos_wl[:, None, :] - pos_wl[None, :, :], axis=-1).max() * lam
    return d / C0


def narrowband_table(side_wl=0.5):
    rows = []
    for name, fc, bw in [("LoRa BW125k @ 915 MHz", 915e6, 125e3),
                         ("LoRa BW500k @ 915 MHz", 915e6, 500e3),
                         ("20 MHz data @ 5.8 GHz", 5.8e9, 20e6),
                         ("80 MHz data @ 5.8 GHz", 5.8e9, 80e6)]:
        pos = triangle_positions(side_wl)
        tau = max_array_delay_s(pos, fc)
        # worst array-gain loss at the band edge when weights are set for fc:
        # phases are scaled by (1 + df/fc); loss of |a(f)^H a(fc)|^2 / N^2 at df = BW/2, worst azimuth.
        az = np.arange(0, 360, 1.0)
        a0 = steering(pos, az)
        a1 = np.exp(1j * np.angle(a0) * (1 + bw / 2 / fc))
        gain = np.abs(np.sum(np.conj(a0) * a1, axis=1)) ** 2 / len(pos) ** 2
        rows.append(dict(name=name, frac_bw=bw / fc, tau_ps=tau * 1e12,
                         bt=bw * tau, loss_db=float(-10 * np.log10(gain.min())),
                         edge_phase_deg=float(np.rad2deg(2 * np.pi * (bw / 2) * tau))))
    return rows


def chirp_delay_error(sf=7, side_wl=0.5, freq_hz=915e6, bw=125e3, theta_deg=40.0):
    """Compare the true delayed chirp with the narrowband (phase-only) model.

    Returns (relative waveform error, phase-estimate error in degrees) for the
    longest baseline of a triangle of the given side at azimuth theta_deg. Channel 2
    is x(t - tau) e^{-j 2 pi fc tau}; the narrowband model uses x(t) e^{-j 2 pi fc tau}.
    The phase is estimated from the cross-correlation (no dechirp needed)."""
    lam = C0 / freq_hz
    x = lora_symbol(sf, sym=37)
    tau = side_wl * lam * np.cos(np.deg2rad(theta_deg)) / C0  # baseline along u
    ph = np.exp(-2j * np.pi * freq_hz * tau)
    y_true = fractional_delay(x, tau * bw) * ph
    y_nb = x * ph
    rel_err = np.linalg.norm(y_true - y_nb) / np.linalg.norm(x)
    est = np.angle(np.sum(y_true * np.conj(x)))
    truth = np.angle(ph)
    err = np.rad2deg(np.angle(np.exp(1j * (est - truth))))
    return float(rel_err), float(err)


def lora_pair_doa(rng, sf, n_symbols, snr_db_per_sample, spacing, theta_deg, trials, offset_rad=0.0):
    """Two-channel interferometer on a LoRa symbol train, cross-correlation estimator.
    SNR is per complex sample at fs = BW (noise bandwidth = BW). Returns theta estimates (deg)."""
    N = 2 ** sf
    syms = np.concatenate([lora_symbol(sf, int(s)) for s in rng.integers(0, N, n_symbols)])
    K = len(syms)
    psi = 2 * np.pi * spacing * np.sin(np.deg2rad(theta_deg)) + offset_rad
    amp = 10 ** (snr_db_per_sample / 20)
    noise = lambda: (rng.standard_normal((trials, K)) + 1j * rng.standard_normal((trials, K))) / np.sqrt(2)
    x1 = amp * syms[None, :] * np.exp(-0.5j * psi) + noise()
    x2 = amp * syms[None, :] * np.exp(+0.5j * psi) + noise()
    r = np.mean(x2 * np.conj(x1), axis=1)
    s = np.angle(r * np.exp(-1j * offset_rad)) / (2 * np.pi * spacing)
    return np.rad2deg(np.arcsin(np.clip(s, -1, 1))), K
