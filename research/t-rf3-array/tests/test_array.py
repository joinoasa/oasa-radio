import numpy as np
import pytest

from t_rf3 import array_model as am
from t_rf3 import beamforming as bf
from t_rf3 import lora_check as lc

SIDES = [1 / 3, 0.5, 1 / np.sqrt(3), 1.0]


def test_triangle_geometry():
    pos = am.triangle_positions(0.5)
    d = [np.linalg.norm(pos[i] - pos[j]) for i, j in ((0, 1), (1, 2), (0, 2))]
    np.testing.assert_allclose(d, 0.5)
    np.testing.assert_allclose(pos.mean(0), 0, atol=1e-12)


@pytest.mark.parametrize("side", SIDES)
@pytest.mark.parametrize("steer", [0.0, 17.0, 123.0, 301.0])
def test_peak_gain_is_n_squared(side, steer):
    pos = am.triangle_positions(side)
    w = am.conventional_weights(pos, steer)
    assert abs(abs(am.response(w, pos, steer)) ** 2 - 9.0) < 1e-9
    az = np.linspace(0, 360, 3601)
    assert np.max(np.abs(am.response(w, pos, az)) ** 2) <= 9.0 + 1e-9


def test_gain_constant_over_360_steer():
    pos = am.triangle_positions(0.5)
    g = [abs(am.response(am.conventional_weights(pos, a), pos, a)) ** 2 for a in np.arange(0, 360, 7.0)]
    np.testing.assert_allclose(g, 9.0)


def test_beamwidth_decreases_with_aperture():
    bw = []
    for s in SIDES:
        pos = am.triangle_positions(s)
        bw.append(am.hpbw_deg(am.conventional_weights(pos, 0), pos, 0))
    assert bw == sorted(bw, reverse=True)
    assert 70 < bw[1] < 75


def test_ambiguity_threshold():
    thr = am.triangle_exact_ambiguity_threshold()
    assert abs(thr - 0.57735) < 1e-4
    assert am.distinguishability(am.triangle_positions(0.5), 45.0, 0.25) > 0.05
    assert am.distinguishability(am.triangle_positions(thr + 0.03), 45.0, 0.25) < 1e-3
    # exact ambiguity at side = lambda: find a pair numerically and check equality
    pos = am.triangle_positions(1.0)
    az = np.arange(0, 360, 0.25)
    A = am.steering(pos, az) / np.sqrt(3)
    G = np.abs(A.conj() @ A.T)
    sep = np.abs(az[:, None] - az[None, :])
    G[np.minimum(sep, 360 - sep) < 45] = 0
    assert G.max() > 1 - 1e-9


def test_nulls_deep_with_ideal_weights():
    for s in SIDES:
        pos = am.triangle_positions(s)
        for jam in ([70.0], [70.0, 200.0]):
            w = bf.lcmv_weights(pos, 0.0, jam)
            assert bf.null_depth_db(w, pos, 0.0, jam).max() < -100
            assert abs(np.vdot(w, am.steering(pos, 0.0)) - 1) < 1e-9


def test_three_nulls_rejected():
    with pytest.raises(ValueError):
        bf.lcmv_weights(am.triangle_positions(0.5), 0, [50, 100, 200])


def test_null_depth_matches_small_error_theory():
    rng = np.random.default_rng(1)
    pos = am.triangle_positions(0.5)
    w = bf.lcmv_weights(pos, 0.0, [70.0, 200.0])
    n = 40000
    g = bf.element_errors(rng, (n, 3), 5.0, 0.5)
    lin = 10 ** (bf.null_depth_db(np.broadcast_to(w, (n, 3)), pos, 0.0, [70.0], gains=g) / 10)
    th = bf.predicted_null_power(w, 5.0, 0.5)
    assert abs(10 * np.log10(lin.mean() / th)) < 0.5


def test_null_depth_degrades_with_errors_and_quantisation():
    rng = np.random.default_rng(2)
    pos = am.triangle_positions(0.5)
    w = bf.lcmv_weights(pos, 0.0, [70.0, 200.0])
    med = []
    for p in (1.0, 5.0, 10.0):
        g = bf.element_errors(rng, (5000, 3), p, 0)
        med.append(np.median(bf.null_depth_db(np.broadcast_to(w, (5000, 3)), pos, 0.0, [70.0], gains=g)))
    assert med[0] < med[1] < med[2]
    assert -45 < med[0] < -35 and -30 < med[1] < -22
    d6 = bf.null_depth_db(bf.quantize_phase(w, 6), pos, 0.0, [70.0, 200.0]).max()
    d3 = bf.null_depth_db(bf.quantize_phase(w, 3), pos, 0.0, [70.0, 200.0]).max()
    assert d6 < d3 and d6 < -15


def test_smi_loss_matches_theory():
    rng = np.random.default_rng(3)
    pos = am.triangle_positions(0.5)
    a = am.steering(pos, 0.0)
    for K in (6, 20):
        v = []
        for _ in range(400):
            X, R_in = bf.simulate_snapshots(rng, pos, K, 0.0, 10, [60.0, 200.0], [30.0, 30.0], False)
            w = bf.mvdr_weights(X @ X.conj().T / K, a)
            v.append(bf.output_sinr(w, a, R_in) / bf.optimum_sinr(a, R_in))
        assert abs(np.mean(v) - bf.smi_loss_theory(K, 3)) < 0.04


def test_narrowband_valid_for_lora():
    rows = lc.narrowband_table(0.5)
    lora = rows[0]
    assert abs(lora["frac_bw"] - 125e3 / 915e6) < 1e-12
    assert lora["bt"] < 1e-3 and lora["loss_db"] < 1e-5
    rel, perr = lc.chirp_delay_error()
    assert rel < 1e-3 and abs(perr) < 1e-2
