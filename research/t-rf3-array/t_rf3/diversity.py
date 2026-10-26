"""Two-branch diversity over Rayleigh fading: selection (SC) and maximal-ratio (MRC).

Branch SNR gamma_i = gbar * |h_i|^2, h_i ~ CN(0,1). Noise is independent per branch.
"""
from __future__ import annotations

import numpy as np
from scipy.optimize import brentq
from scipy.special import j0


def rayleigh_branches(rng, n, corr=0.0):
    """Two unit-power complex Gaussian gains with complex correlation `corr` (real, >=0 magnitude)."""
    z = (rng.standard_normal((2, n)) + 1j * rng.standard_normal((2, n))) / np.sqrt(2)
    h1 = z[0]
    h2 = corr * z[0] + np.sqrt(1 - corr ** 2) * z[1]
    return h1, h2


def isotropic_scatter_corr(spacing_wl):
    """Field correlation between two isotropic antennas in 2-D isotropic scattering:
    J0(2 pi d / lambda) (Clarke/Jakes). Magnitude is what matters here."""
    return float(abs(j0(2 * np.pi * spacing_wl)))


def combine(h1, h2, gbar=1.0):
    g1, g2 = gbar * np.abs(h1) ** 2, gbar * np.abs(h2) ** 2
    return g1, np.maximum(g1, g2), g1 + g2  # single, SC, MRC


def outage_snr_theory(prob, scheme, rho=0.0):
    """SNR threshold x (relative to mean branch SNR) with P(gamma < x) = prob."""
    def cdf(x):
        if scheme == "single":
            return 1 - np.exp(-x)
        if scheme == "sc":  # correlated SC for bivariate exponential is more involved; iid only
            if rho != 0.0:
                raise ValueError("SC theory implemented for independent branches only")
            return (1 - np.exp(-x)) ** 2
        if scheme == "mrc":  # sum of two independent exponentials with means 1 +/- rho
            l1, l2 = 1 + rho, 1 - rho
            if rho < 1e-9:
                return 1 - np.exp(-x) * (1 + x)
            return 1 - (l1 * np.exp(-x / l1) - l2 * np.exp(-x / l2)) / (l1 - l2)
        raise ValueError(scheme)
    return brentq(lambda x: cdf(x) - prob, 1e-9, 100.0)


def mean_gain_theory(scheme):
    """Mean output SNR / mean single-branch SNR, iid Rayleigh."""
    return {"single": 1.0, "sc": 1.5, "mrc": 2.0}[scheme]


def outage_snr_sim(samples, prob):
    return float(np.quantile(samples, prob))


def db(x):
    return 10 * np.log10(x)
