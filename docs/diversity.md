# Two-antenna receive diversity (B14, simulation)

Test plan B14, the simulation half: two antennas, each with its own receive chain, through the B9 channel models
([fading.md](fading.md)), against B9's single antenna. Code: [`waveform/src/diversity.rs`](../waveform/src/diversity.rs)
(`Modem::demodulate_diversity`), the two-antenna channel in `run_found` ([`waveform/src/sim.rs`](../waveform/src/sim.rs)).
Tests: [`waveform/tests/diversity.rs`](../waveform/tests/diversity.rs) and the unit tests in `diversity.rs`.

**Off by default, receiver only.** Nothing changes on the air: the sender is any v1 sender. Without
`--diversity` the receiver and the simulation are the ones B9 measured, bit for bit (B9's Rayleigh 5 Hz curve
re-run on this branch gives the same lines, 26.3 dB at 1 %).

| Where | Option | Default |
|---|---|---|
| library | `Modem::demodulate_diversity(&[ch1, ch2], rx, Combine::…)`; `BerOptions::diversity` for the simulation | not used / `None` |
| `oasa-radio waveform per` | `--diversity off\|select\|snr\|mrc` (`off`: one antenna, adds the receiver time to the output) | not given: one antenna |

The live radio (`bridge --radio sdr`) does not take two channels yet: that is the bench half (section 4).

## The three methods

The two channels are assumed sampled together (one multi-channel stream from one radio): same sample clock, same
local oscillator, so timing and carrier offset are common and only each antenna's complex gain and noise differ.

1. **`select`, frame selection.** The whole receiver runs on each channel; for each frame the copy with a good
   CRC-16 is kept. Two independent decodes, so each antenna also gets its own phase fit or channel tracking.
2. **`snr`, preamble-SNR selection.** The detector runs on both channels; at each candidate only the channel whose
   preamble correlates better is decoded (the normalized detection metric is a monotone function of the preamble's
   SNR). One decode per candidate.
3. **`mrc`, maximal-ratio combining.** Candidates come from the better channel's metric; the frame timing from the
   sum of both channels' fine metrics. Each channel's carrier frequency, phase, amplitude and noise are estimated
   from its own preamble (`Modem::estimate`), and the channels are added sample by sample with the weights
   `conj(h_i) / N_i`, `h_i` the gain at the middle of the preamble. The sum is decoded as one channel: the
   receiver's own estimates (and the B12 tracker, if on) take out the residual frequency and phase they share.
   The weights are held for the whole frame.

## Settings

The live link's, as in B9: QPSK, 4 samples per symbol, 250 kbaud, frame v1, 54-byte payload (a 32-byte oasa-link
message), `--cfo-search 0.1`, frames every 10 ms, 10,000 frames a point, seed 1, release build. **Eb/N0 is per
antenna**: each antenna sees the given mean Eb/N0 (the usual convention: the second antenna adds its own signal
power, as a real one does). The two antennas fade independently (same model and Doppler, separate generators: a
spacing of half a wavelength or more), with independent noise and carrier phase and a common carrier offset.

```sh
oasa-radio waveform per --mod qpsk --sps 4 --frame v1 --payload-len 54 --cfo-search 0.1 --symbol-rate 250000 \
  --frames 10000 --ebn0-db 5,7.5,10,12.5,15,17.5,20,22.5,25,30,35,40,45 --channel rayleigh --doppler-hz 5 \
  --diversity mrc
```

## Against the target (proposed, owner to confirm)

| Target | Result | |
|---|---|---|
| Rayleigh 5 Hz, 1 % loss point at least 8 dB better than B9's one antenna (26.3 dB) | `select` 16.7 dB (+9.7 dB), `snr` 16.9 dB (+9.4 dB), `mrc` 14.8 dB (+11.5 dB) | met by all three |

## 1. Loss vs Eb/N0

1 % and 0.1 % points by log-linear interpolation between the measured points (B9's `points.py`); "never": the
curve does not cross it by 45 dB. Gain: B9's 1 % point minus this one. Loss at 20, 30 and 45 dB as measured (AWGN was swept 1 to 12 dB; every two-antenna AWGN curve is at 0 from 10 dB).
B9 rows are B9's own curves; `--diversity off` reproduces them frame for frame (same `delivered` at every point
for AWGN and 5 Hz).

### Today's receiver (phase line, no `--phase-track-hz`)

| Channel | Receiver | 1 % | Gain | 0.1 % | 20 dB | 30 dB | 45 dB |
|---|---|---|---|---|---|---|---|
| AWGN | one antenna (B9) | 9.3 dB | | 10-11 dB | | | |
| AWGN | `select` | 7.8 dB | +1.4 dB | 8.4 dB | | | |
| AWGN | `snr` | 9.2 dB | +0.0 dB | 10.2 dB | | | |
| AWGN | `mrc` | 6.2 dB | **+3.1 dB** | 7.2 dB | | | |
| Rayleigh 5 Hz | one antenna (B9) | 26.3 dB | | 37.2 dB | 4.28 % | 0.42 % | 0.03 % |
| Rayleigh 5 Hz | `select` | 16.7 dB | +9.7 dB | 20.7 dB | 0.19 % | 0 | 0 |
| Rayleigh 5 Hz | `snr` | 16.9 dB | +9.4 dB | 20.8 dB | 0.20 % | 0 | 0 |
| Rayleigh 5 Hz | `mrc` | 14.8 dB | **+11.5 dB** | 19.8 dB | 0.09 % | 0 | 0 |
| Rayleigh 50 Hz | one antenna (B9) | never | | never | 5.82 % | 2.04 % | 1.84 % (floor) |
| Rayleigh 50 Hz | `select` | 17.4 dB | | 25.5 dB | 0.36 % | 0.04 % | 0.03 % |
| Rayleigh 50 Hz | `snr` | 20.4 dB | | never | 1.03 % | 0.29 % | 0.20 % |
| Rayleigh 50 Hz | `mrc` | 17.8 dB | | never | 0.53 % | 0.16 % | 0.10 % |
| Rayleigh 200 Hz | one antenna (B9) | never | | never | 30.3 % | 25.1 % | 25.8 % (floor) |
| Rayleigh 200 Hz | `select` | never | | never | 8.72 % | 6.74 % | 7.12 % (floor) |
| Rayleigh 200 Hz | `snr` | never | | never | 25.0 % | 22.5 % | 24.2 % |
| Rayleigh 200 Hz | `mrc` | never | | never | 24.9 % | 20.8 % | 20.1 % |

### With the B12 channel tracker (`--phase-track-hz 200`, both the one-antenna row and the two-antenna rows)

Points at 10, 15, ..., 45 dB.

| Channel | Receiver | 1 % | 0.1 % | 20 dB | 30 dB | 45 dB |
|---|---|---|---|---|---|---|
| Rayleigh 50 Hz | one antenna | 28.8 dB | never | 5.03 % | 0.76 % | 0.10 % |
| Rayleigh 50 Hz | `select` | **16.9 dB** | 22.8 dB | 0.32 % | 0.02 % | 0 |
| Rayleigh 50 Hz | `snr` | 19.7 dB | 33.4 dB | 0.92 % | 0.16 % | 0.01 % |
| Rayleigh 50 Hz | `mrc` | 17.8 dB | 28.7 dB | 0.52 % | 0.07 % | 0.01 % |
| Rayleigh 200 Hz | one antenna | 36.7 dB | never | 10.2 % | 2.18 % | 0.31 % |
| Rayleigh 200 Hz | `select` | **19.5 dB** | 26.7 dB | 0.83 % | 0.03 % | 0 |
| Rayleigh 200 Hz | `snr` | 36.4 dB | never | 9.41 % | 2.47 % | 0.27 % |
| Rayleigh 200 Hz | `mrc` | 34.0 dB | never | 8.80 % | 1.86 % | 0.14 % |

What it shows:

- **Slow fading (5 Hz): all three close most of the gap**, about 10 dB at 1 % and 17 dB at 0.1 %. MRC is best by
  about 2 dB, as theory says (two-branch MRC against selection), and in white noise it is the only one with a real
  gain: +3.1 dB, the second antenna's signal power added in phase. Preamble selection gains nothing in white noise
  (both channels are as good) and almost all that frame selection does at 5 Hz, where a frame sees one gain.
- **Fast fading: only frame selection keeps its gain.** `snr` and `mrc` decode one stream, chosen or weighted by
  the preamble; when the channels turn within the 1.8 ms frame (50 Hz: 0.09 cycles of fading per frame, 200 Hz:
  0.36), the preamble's choice or weights go stale by the end of the frame, and the combined stream has the one
  antenna's floor back (200 Hz, line: 20-24 %, same as one antenna). Frame selection runs two full receivers, each
  with its own phase fit or tracker, and loses a frame only when both lose it: the 200 Hz floor drops from 26 % to
  7 % with the line, and with the tracker the 1 % point comes down from 36.7 to 19.5 dB (0.03 % at 30 dB).
- With the tracker, MRC's 50 Hz curve is close to selection's (17.8 against 16.9 dB), but at 200 Hz it is not.
  Fixing that means weights that follow each channel through the frame (each channel's own tracked gain, symbol by
  symbol, instead of one weight from the preamble). Not done here; see section 3.

## 2. CPU per frame

Receiver time only (detection and decoding, both channels; the simulated channel not counted), one core
(`taskset -c 0`, nothing else running), 3,000 frames, two runs averaged, release build on the PC.
`rx_us_per_frame` from `--diversity off|select|snr|mrc`.

| Channel, Eb/N0 | one antenna | `select` | `snr` | `mrc` |
|---|---|---|---|---|
| AWGN, 10 dB | 1.86 ms | 4.03 ms (2.16 x) | 3.18 ms (1.71 x) | 3.05 ms (1.64 x) |
| Rayleigh 5 Hz, 20 dB | 1.65 ms | 3.32 ms (2.02 x) | 2.96 ms (1.79 x) | 2.91 ms (1.76 x) |
| Rayleigh 200 Hz, 30 dB, tracker | 1.51 ms | 3.18 ms (2.11 x) | 2.82 ms (1.86 x) | 2.86 ms (1.89 x) |

Each simulated frame is one short buffer (the frame plus up to 63 samples), searched at `--cfo-search 0.1` with
the longest frame allowed (255 bytes, as in B9). `snr` and `mrc` decode once per candidate but still run the
coarse detector on both channels, and the detector is most of the cost, so they save only 10-20 % against
`select`. On the live stream the detector runs over every sample, mostly noise between frames (a 1.8 ms frame
every 10 ms), so it is an even larger part of the cost there: expect close to 2 x for all three.

## 3. Which to use

**Frame selection (`select`) for the next step (the bench).** It meets the target with 1.7 dB to spare, it is the
only method that keeps its gain at 50 and 200 Hz (with the tracker: 19.5 dB at 1 % at 200 Hz, against 36.7 dB with
one antenna), it reuses the receiver unchanged (each channel is the receiver B9 and B12 measured), and on the bench
it fails safe: a dead or miscabled channel costs nothing but CPU. Its cost is twice the receiver CPU, and the
others save little of that (section 2).

**MRC is the better combiner where the channel holds through a frame**, 1.9 dB better than selection at 5 Hz and
3.1 dB in white noise (a link at the edge of its range, no fading, gains the most). It is worth finishing later:
per-symbol weights from each channel's own tracker, so it keeps its gain at 50-200 Hz, then MRC with selection as
the fallback when the combined frame fails its CRC. That is more than this test asked for, so it is left as a
follow-up. `snr` (preamble selection) has no case where it is best and is kept only as the measured baseline.

## 4. The bench half (not done here; needs B2)

What B2 must provide, and what the code still needs:

- **Both E310 RX chains cabled**, from one transmitter through a splitter, each branch with its own attenuation
  (step attenuators, or better programmable ones), so that one branch can be faded or cut while the other holds.
  Independent fading on the two branches would need a two-path channel emulator; without one, stepping one
  branch down 20-30 dB while the other holds shows selection and MRC working, and a branch pulled entirely must
  cost no frames. (The fading itself stays a simulation result.)
- **One two-channel stream from the radio** (UHD channels 0 and 1 together, one clock and one LO, as the
  direction-finding test uses): the receiver relies on common timing and carrier offset. The two channels' fixed
  phase difference does not matter (MRC estimates each channel's phase every frame; selection ignores it).
- **Code, still to do:** the `sdr` adapter must open a two-channel receive stream and the live stream receiver
  (`stream.rs`) must take two channels (for `select`, simply two receivers over the two streams with their frames
  merged by position, as `demodulate_diversity` does); a `--rx-diversity` option on `bridge --radio sdr`, off by
  default.
- **CPU on the E310:** twice the receive cost. Measure the per-pass time on the E310's ARM with two channels before
  the counted run; if it does not fit, a larger `--coarse-step` and `--cfo-acquire` (the narrow search after lock)
  are the levers.
- **Counted test:** the same loss-vs-attenuation sweep with one branch and with both, the 1 % point compared, plus a
  run with one branch disconnected (no loss of frames, no hang).
