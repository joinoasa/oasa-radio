# Direction finding with two receive channels (T-RF3)

`oasa-radio waveform df` measures the phase difference between two receive channels recorded together, and the direction of arrival it gives. Code: `waveform/src/df.rs`. Plan and simulation: [research/t-rf3-array](../research/t-rf3-array/).

## The measurement

For each burst (or fixed block), `r = mean(x2 * conj(x1))`. Its angle is the phase of channel 2 relative to channel 1; positive means channel 2 leads. A carrier offset common to both channels cancels, and so does the modulation, so any source works: a tone, a W1 frame, a LoRa chirp. The two channels must come from one multi-channel stream (UHD `--channels 0,1`), so their samples line up.

For two antennas `d` wavelengths apart and a source at `theta` from broadside, toward channel 2:

```
phase = eps + 360 * d * sin(theta)          theta = asin((phase - eps) / (360 d))
```

`eps` is the receiver's own channel offset. A pair cannot tell front from back, and at `d = 0.5` covers ±90°.

## Calibration (`eps`)

Feed both channels the same signal through a splitter and two cables, measure `psi_a`, swap the cables at the receiver, measure `psi_b`. The cables' difference flips sign and the receiver's does not: `eps = (psi_a + psi_b) / 2` (circular), the cable difference is `(psi_a - psi_b) / 2` (`df::swap_calibration`). Then repeat after a retune, a gain change and a power cycle to see whether `eps` stays put. On the E310 this is the first open question (TEST-PLAN step 1).

## Testing over cable

A known extra cable length on channel 2 is a known phase: `-360 f L / (vf c)` (`df::cable_phase_deg`; `vf` is about 0.66 for RG-58, 0.7 for RG-316). At 915 MHz in RG-316, 1 cm is 15.7°, and 3 cm imitates a source about 15° off broadside toward channel 1 at half-wavelength spacing. Stepping through a few lengths tests the whole chain against known answers without antennas.

## Command

```
oasa-radio waveform df --in1 rx.00.dat --in2 rx.01.dat [--format ci16|cf32] [--block N]
                       [--burst-db 10] [--win 256] [--min-len 4096] [--skip 0]
                       [--cal-deg EPS] [--spacing-wl D]
```

Raw files (no SigMF wrapper), default `ci16` as UHD's `--type short` writes. One JSON line per segment (`phase_deg`, `phase_cal_deg`, `coherence`, `ratio_db`, `snr1_db`, `snr2_db`, `doa_deg`), then `df_done` with the circular mean and standard deviation. Bursts are runs at least `--burst-db` over the noise floor (10th percentile of window powers), trimmed by one window each end. Use `--block` for a continuous source; then there is no noise floor and the SNR fields are null.

Expected scatter from noise alone: `std(phase) ≈ sqrt((2/snr + 1/snr²) / 2n)` radians for `n` samples at per-sample SNR `snr` (`df::phase_std_deg`). Much more than that points to multipath, phase jitter or a channel that is not coherent.
