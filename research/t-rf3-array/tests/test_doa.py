import numpy as np

from t_rf3 import array_model as am
from t_rf3 import diversity as dv
from t_rf3 import doa
from t_rf3 import lora_check as lc


def test_crb_matches_fisher_information():
    pos = am.triangle_positions(0.5)
    for az in (0.0, 33.0, 200.0):
        a, da = am.steering(pos, az), am.steering_deriv(pos, az)
        for snr in (0.1, 1.0, 100.0):
            F = doa.fisher_numeric(a, da, snr, 50)
            np.testing.assert_allclose(np.linalg.inv(F)[0, 0], doa.crb_single_source(a, da, snr, 50), rtol=1e-9)


def test_crb_isotropic_closed_form():
    s = 0.5
    R = s / np.sqrt(3)
    pos = am.triangle_positions(s)
    snr = 10.0
    expect = 1 / (2 * 100 * snr * (2 * np.pi) ** 2 * 3 * R ** 2 / 2)
    a, da = am.steering(pos, 77.0), am.steering_deriv(pos, 77.0)
    np.testing.assert_allclose(doa.crb_single_source(a, da, snr, 100, stochastic=False), expect, rtol=1e-9)


def test_triangle_doa_converges_and_reaches_crb():
    rng = np.random.default_rng(10)
    pos = am.triangle_positions(0.5)
    T = 400
    az = rng.uniform(0, 360, T)
    for method in ("music", "bartlett"):
        R = doa.sample_cov(doa.gen_snapshots(rng, pos, az, 30, 100, T))
        est = doa.estimate_azimuth(R, pos, method)
        r = doa.rmse_deg(est, az)
        assert r < 0.2
        assert r / doa.crb_azimuth_deg(pos, 0, 30, 100) < 1.25
    R = doa.sample_cov(doa.gen_snapshots(rng, pos, az, 10, 100, T))
    r = doa.rmse_deg(doa.estimate_azimuth(R, pos, "music"), az)
    assert r / doa.crb_azimuth_deg(pos, 0, 10, 100) < 1.25


def test_pair_front_back_ambiguity_exact():
    p = am.pair_positions(0.5)
    np.testing.assert_allclose(am.steering(p, 60.0), am.steering(p, -60.0), atol=1e-12)
    np.testing.assert_allclose(am.steering(p, 120.0), am.steering(p, -120.0), atol=1e-12)


def test_interferometer_accuracy_and_offset():
    rng = np.random.default_rng(11)
    d = 0.5
    for th in (0.0, 30.0, 60.0):
        X = doa.gen_pair_snapshots(rng, d, th, 15, 128, 4000)
        e = doa.interferometer_theta_deg(X, d)
        r = np.sqrt(np.mean((e - th) ** 2))
        assert r / doa.crb_interferometer_deg(d, th, 15, 128) < 1.1
    # uncalibrated offset gives the predicted bias; calibrated removes it
    off = np.deg2rad(10.0)
    X = doa.gen_pair_snapshots(rng, d, 0.0, 40, 128, 200, offset_rad=off)
    assert abs(np.mean(doa.interferometer_theta_deg(X, d)) - doa.offset_bias_deg(d, 0.0, 10.0)) < 0.05
    assert abs(np.mean(doa.interferometer_theta_deg(X, d, offset_rad=off))) < 0.05


def test_lora_cross_correlation_doa():
    rng = np.random.default_rng(12)
    est, K = lc.lora_pair_doa(rng, 7, 30, 5.0, 0.5, 30.0, 1500)
    assert K == 3840
    r = np.sqrt(np.mean((est - 30.0) ** 2))
    assert r / doa.crb_interferometer_deg(0.5, 30.0, 5.0, K, stochastic=False) < 1.2


def test_diversity_mean_and_outage_match_theory():
    rng = np.random.default_rng(13)
    h1, h2 = dv.rayleigh_branches(rng, 1_000_000)
    g1, gsc, gmrc = dv.combine(h1, h2)
    assert abs(gsc.mean() / g1.mean() - 1.5) < 0.02
    assert abs(gmrc.mean() / g1.mean() - 2.0) < 0.02
    for p in (0.1, 0.01):
        for name, s in (("single", g1), ("sc", gsc), ("mrc", gmrc)):
            assert abs(10 * np.log10(dv.outage_snr_sim(s, p) / dv.outage_snr_theory(p, name))) < 0.2


def test_correlated_mrc_theory():
    rng = np.random.default_rng(14)
    rho = dv.isotropic_scatter_corr(0.125)
    a, b = dv.rayleigh_branches(rng, 1_000_000, rho)
    _, _, m = dv.combine(a, b)
    assert abs(10 * np.log10(dv.outage_snr_sim(m, 0.01) / dv.outage_snr_theory(0.01, "mrc", rho))) < 0.3
