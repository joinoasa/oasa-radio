"""Array geometry, steering vectors, patterns and pattern metrics.

Conventions
- Positions are in wavelengths (so results hold at any carrier; see `wavelength_m`).
- Azimuth phi is measured in the horizontal plane from +x, counter-clockwise, degrees.
- A plane wave from azimuth phi gives element signals a_n = exp(j 2 pi p_n . u(phi)),
  u = (cos phi, sin phi). Elements are isotropic, no mutual coupling.
- Weights w act as y = w^H x, so the response to a plane wave is w^H a(phi).
"""
from __future__ import annotations

import numpy as np

C0 = 299_792_458.0


def wavelength_m(freq_hz: float) -> float:
    return C0 / freq_hz


def triangle_positions(side: float) -> np.ndarray:
    """Equilateral triangle, centroid at origin. `side` in wavelengths. Returns (3, 2)."""
    radius = side / np.sqrt(3.0)
    ang = np.deg2rad([90.0, 210.0, 330.0])
    return radius * np.column_stack([np.cos(ang), np.sin(ang)])


def pair_positions(spacing: float) -> np.ndarray:
    """Two elements on the x axis, centred. `spacing` in wavelengths. Returns (2, 2)."""
    return np.array([[-spacing / 2.0, 0.0], [spacing / 2.0, 0.0]])


def steering(pos: np.ndarray, az_deg) -> np.ndarray:
    """Steering vectors. az_deg scalar -> (N,), array of shape S -> (*S, N)."""
    az = np.deg2rad(np.asarray(az_deg, dtype=float))
    u = np.stack([np.cos(az), np.sin(az)], axis=-1)
    return np.exp(2j * np.pi * (u @ pos.T))


def steering_deriv(pos: np.ndarray, az_deg) -> np.ndarray:
    """d a / d phi (per radian of azimuth)."""
    az = np.deg2rad(np.asarray(az_deg, dtype=float))
    du = np.stack([-np.sin(az), np.cos(az)], axis=-1)
    return 2j * np.pi * (du @ pos.T) * steering(pos, az_deg)


def response(w: np.ndarray, pos: np.ndarray, az_deg) -> np.ndarray:
    """Complex response w^H a(phi)."""
    return steering(pos, az_deg) @ np.conj(w)


def power_db(x, floor_db: float = -300.0) -> np.ndarray:
    p = np.abs(x) ** 2
    return np.maximum(10 * np.log10(np.maximum(p, 1e-300)), floor_db)


def conventional_weights(pos: np.ndarray, steer_deg: float) -> np.ndarray:
    """Delay-and-sum with unit-modulus weights: w = a(steer). Peak power gain N^2."""
    return steering(pos, steer_deg)


# ---------------------------------------------------------------- ambiguity

def distinguishability(pos: np.ndarray, min_sep_deg: float = 30.0, step_deg: float = 0.25) -> float:
    """D = 1 - max |a1^H a2| / N over all azimuth pairs separated by >= min_sep_deg.

    D = 0 means two different azimuths give the same steering vector (up to a
    common phase): the array cannot tell them apart whatever the SNR.
    """
    az = np.arange(0.0, 360.0, step_deg)
    A = steering(pos, az) / np.sqrt(len(pos))
    G = np.abs(A.conj() @ A.T)  # (M, M)
    sep = np.abs(az[:, None] - az[None, :])
    sep = np.minimum(sep, 360.0 - sep)
    G = np.where(sep >= min_sep_deg, G, 0.0)
    return float(1.0 - G.max())


def triangle_exact_ambiguity_threshold() -> float:
    """Side length (wavelengths) at and above which exact ambiguities exist.

    Two azimuths are indistinguishable iff for every baseline b: b.(u1-u2) is an
    integer number of wavelengths. Baselines of the triangle form a lattice with
    spacing s; the condition says u1-u2 lies on the reciprocal lattice, whose
    shortest vector has length 2/(sqrt(3) s) (in 1/wavelength units). Since
    |u1-u2| <= 2 for unit vectors, a solution exists iff 2/(sqrt(3) s) <= 2,
    i.e. s >= 1/sqrt(3) = 0.577 wavelengths (circumradius 1/3 wavelength).
    """
    return 1.0 / np.sqrt(3.0)


def _offsets(n=7201):
    return np.linspace(-180.0, 180.0, n)


def pattern_vs_offset(w, pos, steer_deg, n=7201):
    """Power pattern |w^H a|^2 at steer_deg + offset, offset in [-180, 180]."""
    off = _offsets(n)
    return off, np.abs(response(w, pos, steer_deg + off)) ** 2


def hpbw_deg(w, pos, steer_deg) -> float:
    """-3 dB (half-power) full width of the lobe containing the steer direction.

    nan if the pattern never falls below -3 dB on one side (lobe wider than 360).
    """
    off, p = pattern_vs_offset(w, pos, steer_deg)
    p = p / p[len(p) // 2]  # relative to the response at steer
    mid = len(p) // 2
    edges = []
    for sign in (+1, -1):
        seq = p[mid:] if sign > 0 else p[mid::-1]
        offs = off[mid:] if sign > 0 else off[mid::-1]
        idx = np.nonzero(seq < 0.5)[0]
        if len(idx) == 0:
            return float("nan")
        i = idx[0]
        x0, x1, y0, y1 = offs[i - 1], offs[i], seq[i - 1], seq[i]
        edges.append(abs(x0 + (0.5 - y0) * (x1 - x0) / (y1 - y0)))
    return float(sum(edges))


def peak_sidelobe_db(w, pos, steer_deg) -> float:
    """Highest local maximum other than the main lobe, in dB relative to the
    response at the steer direction. -inf if the pattern has a single lobe."""
    off, p = pattern_vs_offset(w, pos, steer_deg)
    p = p[:-1]  # drop duplicate 180/-180 point for circular logic
    left, right = np.roll(p, 1), np.roll(p, -1)
    peaks = np.nonzero((p > left) & (p >= right))[0]
    mid = len(p) // 2
    p0 = p[mid]
    peaks = [i for i in peaks if abs(i - mid) > 2]
    if not peaks:
        return float("-inf")
    return float(10 * np.log10(max(p[i] for i in peaks) / p0))
