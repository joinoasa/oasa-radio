# Equalizer for strong echoes (B15)

Test plan B15 (equalizer for strong echoes), in simulation, through the B9 two-ray model ([fading.md](fading.md)).
Code: [`waveform/src/equalize.rs`](../waveform/src/equalize.rs), used by `decode_at` in
[`waveform/src/modem.rs`](../waveform/src/modem.rs). Tests and tables:
[`waveform/tests/equalizer.rs`](../waveform/tests/equalizer.rs).

**Off by default.** With it off the receiver is the one B9 measured, bit for bit (checked: B9 `per` points give
the same JSON field for field). It is a receiver setting only: the frame on the air (v1) does not change, so an
equalizing receiver talks to any v1 sender.

| Where | Option | Default |
|---|---|---|
| library | `RxConfig::equalizer` (BPSK and QPSK, frame v1, even samples per symbol) | `false` (off) |
| `oasa-radio bridge --radio sdr` | `--equalizer on\|off` | `off` |
| `oasa-radio waveform ber` / `per` | `--equalizer on\|off` (needs `--frame v1`) | `off` |

It is not combined with the channel tracker (`--phase-track-hz`, [phase-tracking.md](phase-tracking.md)) yet: the
command line refuses both together, and in the library the equalizer wins.

Settings below: the live link's (QPSK, 4 samples per symbol, 250 kbaud = 1 MS/s, frame v1, 54-byte payload = a
32-byte oasa-link message, `--cfo-search 0.1`), frames every 10 ms, 10,000 frames a point, seed 1, release build.
At 1 MS/s an echo 1, 2 or 4 us late is 1/4, 1/2 or 1 symbol late. Each frame sees a new random echo phase.

## Against the targets (proposed, owner to confirm)

| Target | Result | |
|---|---|---|
| -3 dB echo at 1, 2 and 4 us: 1 % loss at or below 20 dB mean Eb/N0 (B9: floors of 2.7 %, 27.6 %, 88 %) | 1 % at 16.7, 13.9 and 12.1 dB; no loss at all from 24, 18 and 16 dB | met |
| AWGN 1 % loss point within 0.3 dB of 9.3 dB | 9.41 dB with the equalizer, 9.28 dB without (same frames) | met (+0.13 dB) |
| Bench F16 later: small added one-way delay | nothing on the air, no samples past the frame; decode time unchanged within noise (below) | to be measured on the bench |
| Bench F16 later: small CPU | per 4096-sample pass the same as without it in white noise (within 3 %, run to run); lower with an echo | to be measured on the bench |

## 1. Where the floors come from

Frames are always found and their header (BPSK, convolutionally coded) always decodes: at 20 dB `found` is
10,000 and `header_rejected` 0 for every delay. It is the uncoded QPSK body that fails, and it fails at 60 dB too
(2,000 frames: 2 us loses 21.6 %, 4 us 88.6 %), so it is not noise: it is the echo.

Noise-free frames at 72 echo phases (ignored test `echo_phases`), without the equalizer:

| Echo | Lost | What the receiver did |
|---|---|---|
| 2 us (1/2 symbol), -3 dB | 14 of 72 (19 %), the phases from about 130 to 160 deg and 200 to 230 deg | timing within a sample of the main ray, carrier phase within 34 deg of it: ISI. For these phases the eye is closed at **any** timing: the worst-case symbol pattern crosses the decision line wherever the sample is taken (a separate eye calculation for RRC 0.35 gives 23 % of phases closed at the best timing) |
| 4 us (1 symbol), -3 dB | 65 of 72 (90 %) | at the best timing the eye is open for 96 % of phases (same eye calculation), but the receiver's timing settles 1-1.5 samples late (between the two rays) and its carrier phase up to 19 deg off: there the eye is closed. Timing and phase are pulled by the echo; the cause is still the ISI, which no single timing and phase can undo |
| 1 us (1/4 symbol), -3 dB | (no floor at 60 dB; slow fall, 2.7 % at 24 dB) | mostly a flat fade: with the rays opposed the band is up to 12 dB down. Not ISI, so an equalizer helps less here (below) |

So: ISI first, with sync and timing pulled off by it at one symbol. An equalizer that takes the timing and phase
in its own taps fixes both.

## 2. Method

A decision feedback equalizer (DFE) whose taps are solved, not adapted (`equalize.rs`):

- **Feed-forward:** 7 taps on matched filter outputs at half-symbol spacing, from 1 symbol before to 2 symbols after
  the symbol instant. It does not depend on where in the symbol the preamble put the timing, and it takes the
  carrier phase and amplitude.
- **Feedback:** 2 taps on the symbols before.
- **Training:** the least-squares (MMSE) solution over the known symbols. For the header: the preamble and sync word
  (79 symbols), feed-forward only. For the body: preamble, sync word and the decoded header (231 symbols), with
  feedback.
- **Phase loop:** a second-order decision-directed loop inside the feedback loop follows what the carrier estimate
  leaves over the frame.
- **Second pass:** solve again over the whole frame, with the first pass's decisions as known and its phase taken
  off, then run the body again.
- **Soft values:** with `--code light`, the Es/N0 for the soft values is one over the training error power.

The phase line, the tracker and the timing refinement are skipped for an equalized frame: the taps do their job.

Three variants were tried, cheapest first (2,000-8,000 frames a point; the kept row is the 10,000-frame sweep of section 3):

| Variant | AWGN 9.5 dB | 1 us @ 14 dB | 2 us @ 12 dB | 4 us @ 12 dB |
|---|---|---|---|---|
| (off, B9) | 0.59 % | 28.6 % | 52.4 % | 100 % |
| 1. Linear MMSE, solved from the preamble and header (11 half-symbol taps) | 0.90 % | 19.5 % | 36.3 % | 62.5 % |
| 2. DFE, solved the same way (+2 feedback taps) | 0.95 % | 15.0 % | 18.3 % | 8.5 % |
| 3. DFE plus a second pass solved over the whole frame | 0.80 % | 13.7 % | 14.8 % | 2.2 % |
| 3, with 7 feed-forward taps and a second-order phase loop (kept) | 0.83 % | 13.3 % | 11.2 % | 1.2 % |

Without the phase loop, white noise at 9.5 dB lost 2.7 % and 4 us 58 %: the taps solved at the frame start are a
few degrees off by its end. 7, 9 or 11 feed-forward taps and 1 or 2 feedback taps came out within a few tenths of a
percent of each other, except 1 feedback tap at 4 us (3.5 % against 2.0 %).

## 3. Loss vs mean Eb/N0

`oasa-radio waveform per --mod qpsk --sps 4 --frame v1 --payload-len 54 --cfo-search 0.1 --symbol-rate 250000
--frames 10000 --channel tworay --ray-delay-us D --ray-gain-db G [--equalizer on]`. Off rows are B9's
(`research/b9-data` in the lab repo; the same command without the option gives the same numbers).

| Echo | Receiver | 4 dB | 6 dB | 8 dB | 10 dB | 12 dB | 14 dB | 16 dB | 18 dB | 20 dB | 24 dB | 1 % at |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 us, -3 dB | off | 85.9 % | 63.1 % | 49.2 % | 40.9 % | 33.9 % | 28.6 % | 24.2 % | 19.4 % | 12.8 % | 2.7 % | above 24 dB |
| 1 us, -3 dB | equalizer | 86.3 % | 60.7 % | 42.8 % | 32.1 % | 23.3 % | 13.3 % | 2.7 % | 0.16 % | 0.01 % | 0 | **16.7 dB** |
| 2 us, -3 dB | off | 97.2 % | 81.2 % | 67.3 % | 58.6 % | 52.4 % | 47.1 % | 43.4 % | 40.4 % | 34.3 % | 27.6 % | never |
| 2 us, -3 dB | equalizer | 95.1 % | 69.0 % | 47.7 % | 31.9 % | 11.2 % | 0.88 % | 0.01 % | 0 | 0 | 0 | **13.9 dB** |
| 4 us, -3 dB | off | 100 % | 100 % | 100 % | 100 % | 100 % | 99.8 % | 99.0 % | 96.8 % | 94.2 % | 89.8 % | never (88 % at 30 dB) |
| 4 us, -3 dB | equalizer | 100 % | 98.6 % | 59.9 % | 10.5 % | 1.2 % | 0.09 % | 0 | 0 | 0 | 0 | **12.1 dB** (0 at 30 dB) |

The -6 dB echoes (B9 needed 15.3, 18.4 and 20.2 dB for 1 %):

| Echo | Receiver | 6 dB | 8 dB | 10 dB | 12 dB | 14 dB | 16 dB | 18 dB | 20 dB | 1 % at |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 us, -6 dB | off | 63.7 % | 46.3 % | 34.1 % | 20.1 % | 4.3 % | 0.48 % | 0 | 0 | 15.3 dB |
| 1 us, -6 dB | equalizer | 61.8 % | 40.6 % | 25.6 % | 8.8 % | 0.58 % | 0.01 % | 0 | 0 | 13.6 dB |
| 2 us, -6 dB | off | 80.4 % | 63.4 % | 50.8 % | 37.1 % | 19.3 % | 6.7 % | 1.6 % | 0.17 % | 18.4 dB |
| 2 us, -6 dB | equalizer | 69.7 % | 41.9 % | 14.6 % | 1.0 % | 0 | 0 | 0 | 0 | 12.0 dB |
| 4 us, -6 dB | off | 100 % | 100 % | 99.0 % | 87.3 % | 57.6 % | 25.8 % | 7.5 % | 1.3 % | 20.2 dB |
| 4 us, -6 dB | equalizer | 93.7 % | 34.3 % | 2.6 % | 0.07 % | 0 | 0 | 0 | 0 | 10.5 dB |

White noise (same frames and noise in both rows):

| Receiver | 8 dB | 8.5 dB | 9 dB | 9.5 dB | 10 dB | 10.5 dB | 11 dB | 1 % at |
|---|---|---|---|---|---|---|---|---|
| off | 8.7 % | 3.8 % | 1.9 % | 0.59 % | 0.15 % | 0.08 % | 0 | 9.28 dB |
| equalizer | 10.7 % | 4.8 % | 2.5 % | 0.83 % | 0.23 % | 0.10 % | 0 | 9.41 dB |

The 0.13 dB is the price of solving 9 taps from a few hundred noisy symbols where one tap would do.

The quarter-symbol echo stays the hardest: with the rays opposed the whole band fades, which no equalizer
undoes (it is the flat-fading case, for diversity or retries). An echo one symbol late is the easiest for a DFE: the
feedback takes it out whole. Then the second ray's energy is not lost, and 4 us at -6 dB is only about 1 dB from
white noise.

## 4. CPU and delay

Ignored test `cpu_cost`, pinned to one CPU: the live receiver as the bridge runs it (1 MS/s, 250 kbaud, coarse step
4, `--cfo-search 0.03`), one second of 54-byte frames every 10 ms at 20 dB, in passes of 4096 samples. Two runs on a
PC with other work on it (absolute numbers vary by +-10 % from run to run; compare within a run):

| | off | equalizer |
|---|---|---|
| AWGN, per 4096-sample pass | 855 / 880 us | 858 / 907 us |
| AWGN, one frame's buffer | 770 / 777 us | 745 / 751 us |
| echo 4 us -3 dB, per pass | 1500 / 1479 us (4 of 100 frames decode) | 922 / 885 us (100 of 100) |
| echo 4 us -3 dB, one frame's buffer | 2730 / 2608 us | 779 / 794 us |

Detection dominates the receiver's time; the equalizer's own work (matched filter at twice the symbol rate, three
solves of at most 9 unknowns, two passes of 9 multiplies per symbol) costs about what the phase line's search and
the timing refinement it replaces cost. With the echo, frames that fail cost more than frames that pass.

**Delay:** nothing changes on the air, and the equalizer uses no samples past the frame's own end (its last
feed-forward taps fall in the pulse's tail, which the receiver already waits for). So the added one-way delay is the
change in decode time: none measurable here.

## 5. Bench check (F16)

Add `--equalizer on` to both bridges' usual flags. With the F9 settings:

`--sample-rate 1000000 --symbol-rate 250000 --if-hz 175000 --decimate 1 --coarse-step 4 --tx-mode burst --equalizer on`

`sdr_start` logs `"equalizer":true`. Checks:
1. Over cable: loss, delay and pass time should be the same as without the flag.
2. Through `waveform cable --channel tworay --ray-delay-us 1|2|4 --ray-gain-db -3`: with and without the flag.

The cable rounds the echo delay to whole samples (1 MS/s: 1, 2 and 4 us are exact).

## Regenerate

```sh
cargo test --release -p oasa-waveform --test equalizer echo_phases -- --ignored --nocapture
cargo test --release -p oasa-waveform --test equalizer echo_table -- --ignored --nocapture
taskset -c 0 cargo test --release -p oasa-waveform --test equalizer cpu_cost -- --ignored --nocapture
```

The tables above came from the `per` command line (same receiver, same seeds), 4 CPUs, about 10 minutes.
