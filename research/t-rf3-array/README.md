# T-RF3 array simulation (beamforming and direction of arrival)

First step of tech tree node **T-RF3 (Beamforming)**. This is a Python prototype (per the repo rules: Python for prototypes). Nothing here touches the Rust code.

It answers, before any hardware test: what can a 3-element triangle do (patterns, steering, nulls, DOA), and what should a 2-channel receiver (Ettus E310) measure? The hardware test is in [TEST-PLAN.md](TEST-PLAN.md).

## Run it

```
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
.venv/bin/python -m pytest -q      # 34 tests, about 2 s
.venv/bin/python run_all.py        # regenerates figures/ and results.md, about 20 s, fixed seeds
```

Full tables: [results.md](results.md). Figures: `figures/`.

Libraries (all permissive): numpy (BSD-3-Clause), scipy (BSD-3-Clause), matplotlib (PSF-style licence), pytest (MIT, test only). Code is Apache-2.0 like the repo.

## What was simulated

Narrowband model: steering vectors for isotropic elements in the horizontal plane, positions in wavelengths, so every result holds at 915 MHz (side lambda/2 = 16.4 cm) and at 5.8 GHz (2.6 cm). Modules in `t_rf3/`: `array_model` (geometry, patterns), `beamforming` (nulls, MVDR), `doa` (Bartlett, MUSIC, interferometer, CRB), `diversity`, `lora_check`.

## Key findings

**Ambiguity.** Two azimuths look identical to the triangle when their direction difference lies on the reciprocal lattice of the baselines. That lattice's shortest vector is 2/(sqrt(3) side), and the difference of two unit vectors is at most 2, so exact ambiguity appears at **side >= lambda/sqrt(3) = 0.577 lambda** (circumradius lambda/3). Checked numerically (Fig 1, tests). Below that the array covers all 360 degrees with no front/back problem, unlike 2 elements. Near the limit it is not safe: the worst secondary lobe (any steer angle) is -9.4 dB at 0.3 lambda, -4.0 at 0.4, -0.7 at **lambda/2**, and 0 dB from 0.577 lambda up. At side = lambda, some azimuths are exactly confusable: Fig 6 shows 180 degree errors at azimuths that are multiples of 60 degrees.

**Patterns and steering (conventional weights).** Peak power gain is N^2 = 9 (9.54 dB) at every steer angle, 0 to 360 degrees. HPBW: 111 degrees (lambda/3), 72.5 (lambda/2), 62.5 (0.577 lambda), 35.8 (lambda). A lambda/2 triangle is a wide-beam device (9.5 dB peak gain, 4.8 dB array gain against noise), not a narrow beam.

**Nulls (side lambda/2, target 0, jammers 70 and 200 degrees).** Ideal weights: numerical floor, far below -60 dB. Median null depth (response toward jammer relative to target response, same weights):

| error | 1 null | 2 nulls |
|---|---|---|
| 1 deg phase rms | -41.6 dB | -40.9 dB |
| 5 deg | -27.6 | -26.8 |
| 10 deg | -21.6 | -20.7 |
| 0.5 dB amplitude | -31.2 | -30.6 |
| 1 dB amplitude | -25.2 | -24.6 |
| 6-bit phase shifters (random scenes) | -36.3 | -32.4 |

Rule: mean residual null power = sum|w_n|^2 (sigma_phase^2 + sigma_amp^2), confirmed by simulation to within about 1 dB. So in practice expect nulls of about -25 to -40 dB, set by calibration, not by the algorithm. Two nulls cost white-noise gain (0 dB gain at lambda, versus -4.8 dB for a plain beam) and degrade badly when a jammer is within about 25 degrees of the target.

**MVDR from snapshots** (two 30 dB jammers, side lambda/2). Output SINR improves by 36.2 dB over one element at the optimum. With signal-free training, loss versus optimum matches the Reed-Mallett-Brennan result (K-1)/(K+1): -3.0 dB at K = 3, -0.9 dB at 10, -0.1 dB at 100. With the target present in the training data (realistic), loss is -6.3 dB at K = 10, -1.4 dB at 100, -0.2 dB at 1000.

**DOA, triangle** (single source, K = 100). MUSIC and Bartlett both reach the stochastic CRB: RMSE/CRB is 1.00 to 1.05 at 0 dB SNR and above for side lambda/2 (0.56 deg at 10 dB, 0.18 deg at 20 dB, CRB the same at any azimuth). With K at 10 dB SNR the ratio is 1.03 to 1.08 for K from 10 to 1000. Below -5 dB it breaks down (threshold). For side >= 0.577 lambda the CRB is lower but outliers persist even at 30 dB (3 to 4% of azimuths).

**DOA, 2 channels at 915 MHz (E310 case), d = lambda/2 = 16.4 cm.**
- A pair measures sin(theta) only. Source and mirror image (front/back) give identical signals. Range is +/-90 degrees. No other solution at d = lambda/2; more spacing would add phase wraps.
- Accuracy at K = 128 samples: 0.53 deg at 10 dB SNR (broadside), 1.05 deg at 60 degrees off broadside, within 7% of CRB. Error rises as 1/cos(theta).
- An uncorrected channel phase offset adds a bias of about 0.32 deg per degree of offset at broadside (3.2 deg for 10 deg offset, 7 deg at 60 degrees). **Calibration is required.**
- LoRa symbols (SF7, BW125k, 30 symbols = 3840 samples), cross-correlating the two channels, theta = 30: RMSE 1.6 deg at -7.5 dB per-sample SNR (the SF7 demodulation threshold), 0.41 deg at 0 dB, 0.11 deg at 10 dB.

**Diversity, two branches, Rayleigh** (2 million draws; matches theory). Mean SNR gain over one branch: selection 1.76 dB, MRC 3.01 dB. Gain at equal outage probability: selection 5.6 dB (10%) and 10.2 dB (1%); MRC 7.0 dB and 11.7 dB. Branches at lambda/2 are nearly uncorrelated (|rho| = 0.30 in rich scattering); at lambda/8 the 1% MRC gain falls to 9.1 dB. Estimating MRC weights from an 8-symbol preamble costs under 0.02 dB.

**LoRa and narrowband.** BW/fc = 1.4e-4. Bandwidth times the longest delay across the array is 6.8e-5. Worst edge-of-band beam loss 3e-8 dB, phase error across a baseline 0.012 deg. A delayed SF7 chirp differs from the phase-only model by 1e-4. Narrowband holds with huge margin. Even 80 MHz at 5.8 GHz gives 1.2 deg edge phase and 3e-4 dB loss. DOA needs no dechirping: the product x2 * conj(x1) removes the chirp.

## Limits of the model

Isotropic elements, no mutual coupling (real dipoles at lambda/2 couple), ideal element positions, no multipath except the fading test, far field, narrowband, white independent noise, one source for DOA, signals known to be present. Element errors are Gaussian and independent. Quantisation covers phase only. MVDR assumes an exactly known target steering vector (real arrays need calibration or robust versions); diagonal loading was tried once and did not help here, and is not in the scripts. Fading branches are ideal Rayleigh; real indoor LOS links are Rician and give less diversity gain.

## Next steps

1. Run the E310 test ([TEST-PLAN.md](TEST-PLAN.md)). Measure channel offset stability first; everything else depends on it.
2. If DOA works at 915 MHz, build the 3-element receive array and check ambiguity margin (use a side of about 0.4 lambda, not lambda/2).
3. **T-RF1 dependency:** the RF scan should record per-channel IQ (or at least channel phase) so DOA can run on scan data. Otherwise T-RF3 needs its own capture path. Also the T-RF1 waveform and calibration code should share the same sample format.
4. Add mutual coupling and measured patterns to the model once real antennas exist.
