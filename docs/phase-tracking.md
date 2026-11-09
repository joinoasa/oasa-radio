# Channel tracking for fast fading (B12, B13)

Test plan B12 (phase tracking for fast fading) and B13 (shorter control frames), in simulation, through the B9
channel models ([fading.md](fading.md)). Code: [`waveform/src/track.rs`](../waveform/src/track.rs), used by
`decode_at` in [`waveform/src/modem.rs`](../waveform/src/modem.rs). Tests and tables:
[`waveform/tests/phase_track.rs`](../waveform/tests/phase_track.rs).

**Off by default.** With it off the receiver is the one B9 measured, bit for bit. It is a receiver setting only:
the frame on the air (v1) does not change, so a tracking receiver talks to any v1 sender.

| Where | Option | Default |
|---|---|---|
| library | `RxConfig::phase_track` (Doppler to follow, cycles per symbol) | `0.0` (off) |
| `oasa-radio bridge --radio sdr` | `--phase-track-hz HZ` (at most 0.01 of `--symbol-rate`) | `0` (off) |
| `oasa-radio waveform ber` / `per` | `--phase-track-hz HZ` (with `--symbol-rate`) | `0` (off) |

Settings below: the live link's (QPSK, 4 samples per symbol, 250 kbaud, frame v1, 54-byte payload = a 32-byte
oasa-link message, `--cfo-search 0.1`), frames every 10 ms, 10,000 frames a point, seed 1, release build. Eb/N0 is
the mean over the fading, per information bit. The tracker set for 200 Hz (`--phase-track-hz 200`) in every row.

## Against the targets (proposed, owner to confirm)

| Target | Result | |
|---|---|---|
| B12: Rayleigh 50 Hz loss floor under 0.1 % (B9: about 1.9 %) | uncoded: no floor any more, 0.10 % at 45 dB and still falling (0.76 % at 30 dB). Light: 0.12 % at 30 dB, 0.02 % at 45 dB | uncoded: at the target at 45 dB (0.10 %, 10 of 10,000, still falling), **not** at 30 dB; with `--code light`: 0.12 % at 30 dB, met from 35 dB |
| B12: Rayleigh 200 Hz under 1 % at 30 dB (B9: about 26 %) | uncoded: 2.2 %; light: 0.40 % | met **with `--code light` only**. Uncoded, the fades alone lose about 2.3 % with a perfect phase (section 1) |
| B12: AWGN 1 % loss point within 0.3 dB of 9.3 dB | 9.19 dB with the tracker, 9.20 dB with the line (same frames) | met |
| B12: added one-way delay at most 0.1 ms on the bench | uncoded: none on the air; the decode is faster than the line's (below). Light: +0.92 ms of airtime per 32-byte message | met uncoded; **not** with `--code light` (airtime, not the tracker) |
| B12: CPU must not exceed today's | 410-411 us per 4096-sample pass with the tracker against 430 us (white noise) and 448 us (200 Hz) with the line; light: 571-584 against 643-674 | met (less than today) |
| B13: frames short enough that the phase over one frame stays within the tracker's limit at 200 Hz; loss targets without pilots or with fewer | with the line, no v1 frame is short enough: even an empty payload (0.96 ms) loses 13.5 % at 200 Hz. With the tracker the limit is gone at any length and no pilots are needed | met by the tracker, not by length (below) |

So the one choice for the owner: **200 Hz under 1 % at 30 dB needs the light code**, and the light code costs
0.92 ms more airtime per 32-byte message (2.74 ms instead of 1.82 ms at 250 kbaud), against the 0.1 ms delay target.
Uncoded with the tracker gives 2.2 % at 30 dB (0.6 % at 40 dB). Both ends must use the same `--code`; the
tracker itself is receiver-only.

## 1. Where the loss comes from

The same frames and noise at 30 dB, with the channel's gain `h` changed to separate its parts (ignored test
`loss_sources_table`): as is; amplitude only (`|h|`, the phase taken off); phase only (`h/|h|`, no fades); and the
amplitude with the phase replaced by its best straight line over the frame. Loss, and in brackets the frames not
found at all (sync word or header):

| Rayleigh, 30 dB, line | As is | Amplitude only | Phase only | Phase as a line |
|---|---|---|---|---|
| 50 Hz, uncoded | 2.4 % (40) | 0.90 % (3) | 1.6 % (34) | 0.86 % (1) |
| 50 Hz, light | 4.2 % (45) | 0.06 % (3) | 4.6 % (35) | 0.07 % (4) |
| 200 Hz, uncoded | 26.5 % (505) | 2.3 % (2) | 23.5 % (460) | 2.2 % (2) |
| 200 Hz, light | 47.1 % (513) | 0.02 % (1) | 52.6 % (454) | 0.01 % (1) |

(These channels differ from `fading_table`'s in seed, so "as is" is close to, not equal to, the curves below. The
tracker is not run on the three made-up channels: `|h|` alone and `h/|h|` are not smooth where `h` passes zero,
which no real channel does and which the tracker relies on. On the real channel, same frames: 50 Hz 0.93 % uncoded,
0.22 % light; 200 Hz 2.4 % uncoded, 0.46 % light.)

With the line (B9's receiver):

- **The phase within a frame is the loss.** At 200 Hz the phase alone (no fades) loses almost everything the real
  channel does; with the same fades but a phase that is a line, the line decodes as well as with no phase at all. The
  line fails where the phase bends, most of all where `|h|` passes near zero: there the phase turns by up to half a
  turn within a few symbols.
- **The header is part of it.** About a fifth of the 200 Hz loss are frames not found: the v1 header (152 symbols)
  is decided with the preamble's phase line and turns away too.
- **What is left is the fades.** Amplitude alone loses about 2.3 % (200 Hz) and 0.9 % (50 Hz) uncoded: a fade of
  25-30 dB inside the frame wipes out a few dozen symbols, and an uncoded frame does not survive a burst. The light
  code (interleaved) survives them: 0.02 % and 0.06 %.
- **Not the carrier offset search.** The simulation has no clock offset; the coarse frequency search runs per frame
  (`--cfo-search 0.1`) and its estimate is taken off before the line, as before. The bridge's offset tracker moves by
  0.3 of each good frame's estimate; under fading that estimate wanders by the Doppler (200 Hz is 0.0008 cycles per
  symbol, a hundredth of the 0.1 search), far inside the search. B9's live hour showed the same loss as the simulation.

## 2. What the tracker does

The tracker follows the complex gain `h` itself, not its phase ([`track.rs`](../waveform/src/track.rs)). `h` is
smooth (its spectrum ends at the Doppler); near a deep fade it passes the origin almost on a straight line, and the
half-turn of phase there is just that line in polar form. A smoother on `h` crosses the fade with the right phase on
the other side, where a phase tracker has to guess the jump (in a check on a light-coded body at 200 Hz and 33 dB, a
first version that tracked the phase slipped a quarter turn in about 3 % of frames; tracking `h`, 0.2 %).

1. Blocks of 8 symbols at high SNR, longer at low SNR (block length times Es/N0 about 500, at most 64), so white noise
   loses nothing against the line.
2. Forward from the frame start: a Kalman filter on `h` and its rate of change (I and Q, a random walk on the rate)
   predicts each block; the block is measured against the known symbols (preamble, sync word and, once decoded, the
   header) or the receiver's decisions made with the prediction. A block in a deep fade counts for little (its
   measurement variance includes the bias decisions on noise have), so the rate carries `h` through.
3. Backward smoothing (Rauch-Tung-Striebel), a second pass against decisions made with the smoothed gain, and linear
   interpolation: one gain per symbol.
4. The noise level comes from neighbouring known symbols (differences, so the fading's turn is not counted as noise;
   against the line it looked like up to 20 dB of noise at 200 Hz).

The receiver then decides on `z conj(h)`: the v1 header (before it is decoded, with one-bit decisions), then the body.
With the light code `z conj(h)` is the soft value, so a symbol in a fade counts in proportion to `|h|^2`. The
timing refinement uses the tracked gain too. The line's maximum-likelihood search is skipped, which is why the
tracker costs less CPU than the line.

The one setting is the Doppler to follow (`--phase-track-hz`); it sets the filter's rate noise,
`0.1 (2 pi fd)^3` per symbol (fd in cycles per symbol). 0.03 to 0.3 times that gave the same loss within the noise;
3 times and more was clearly worse (the filter then follows the noise). Set it to the largest Doppler expected
(200 Hz is 10 m/s at 5.8 GHz, 66 m/s at 915 MHz): every row here uses 200 Hz, also for 5 and 50 Hz channels.

## 3. Loss vs mean Eb/N0

`fading_table` and `awgn_table` (ignored tests; the B9 settings and seed, so the "line" rows are B9's numbers):

| Channel | Code | Receiver | 10 dB | 15 dB | 20 dB | 25 dB | 30 dB | 35 dB | 40 dB | 45 dB |
|---|---|---|---|---|---|---|---|---|---|---|
| Rayleigh 5 Hz | none | line | 36.1 % | 13.3 % | 4.3 % | 1.4 % | 0.42 % | 0.25 % | 0.03 % | 0.03 % |
| Rayleigh 5 Hz | none | tracker | 36.3 % | 13.3 % | 4.3 % | 1.4 % | 0.41 % | 0.25 % | 0.03 % | 0.03 % |
| Rayleigh 5 Hz | light | line | 13.1 % | 4.5 % | 1.5 % | 0.52 % | 0.19 % | 0.12 % | 0.04 % | 0.04 % |
| Rayleigh 5 Hz | light | tracker | 13.4 % | 4.5 % | 1.4 % | 0.52 % | 0.18 % | 0.10 % | 0.01 % | 0 |
| Rayleigh 50 Hz | none | line | 36.0 % | 14.8 % | 5.8 % | 3.7 % | 2.0 % | 2.0 % | 1.9 % | 1.8 % |
| Rayleigh 50 Hz | none | **tracker** | 36.1 % | 14.5 % | 5.0 % | 2.4 % | 0.76 % | 0.51 % | 0.16 % | 0.10 % |
| Rayleigh 50 Hz | light | line | 15.5 % | 7.8 % | 4.8 % | 4.5 % | 3.8 % | 3.9 % | 3.8 % | 4.1 % |
| Rayleigh 50 Hz | light | **tracker** | 15.0 % | 6.2 % | 1.9 % | 0.65 % | 0.12 % | 0.08 % | 0.02 % | 0.02 % |
| Rayleigh 200 Hz | none | line | 56.0 % | 38.8 % | 30.3 % | 27.4 % | 25.1 % | 25.8 % | 25.8 % | 25.8 % |
| Rayleigh 200 Hz | none | **tracker** | 46.5 % | 22.8 % | 10.2 % | 4.5 % | 2.2 % | 1.2 % | 0.64 % | 0.31 % |
| Rayleigh 200 Hz | light | line | 55.0 % | 50.6 % | 48.5 % | 47.5 % | 46.8 % | 46.5 % | 45.6 % | 46.2 % |
| Rayleigh 200 Hz | light | **tracker** | 26.5 % | 9.5 % | 3.4 % | 1.3 % | 0.40 % | 0.17 % | 0.17 % | 0.08 % |

10,000 frames a point: 0.1 % is 10 frames, so differences of a few hundredths are noise.

White noise, the 1 % loss point (`awgn_table`):

| Code | Receiver | 1 % loss | Points (loss) |
|---|---|---|---|
| none | line | **9.20 dB** | 9.0: 1.9 %, 9.25: 0.84 %, 9.5: 0.59 %, 10: 0.15 % |
| none | tracker | **9.19 dB** | 9.0: 1.9 %, 9.25: 0.82 %, 9.5: 0.61 %, 10: 0.14 % |
| light | line | 3.56 dB | 3.5: 1.1 %, 3.75: 0.74 %, 4.0: 0.25 % |
| light | tracker | 3.48 dB | 3.5: 0.96 %, 3.75: 0.57 %, 4.0: 0.17 % |

(B9 read 9.3 dB off a coarser grid; same frames.)

- The tracker does not change 5 Hz (the fading hardly moves in a frame) or white noise.
- 50 Hz: the floor is gone. Uncoded, 0.1 % at 45 dB is reached and the curve still falls; at 30 dB it is the fades
  (0.8 %), not the phase.
- 200 Hz: 26 % becomes 2.2 % at 30 dB uncoded, 0.4 % with the light code. Uncoded, the fades are the limit: with the
  phase taken off the channel the line loses about 2.3 % (section 1), and the tracker is close to that.

## 4. B13: frame length

Uncoded v1 frames, payloads of 0 to 54 bytes (`frame_length_table`), and how often the channel's phase leaves its best
line through the frame by more than 45 degrees (QPSK's margin) where the power is within 10 dB of its mean:

| | 0 B (239 sym, 0.96 ms) | 16 B (303 sym, 1.21 ms) | 32 B (367 sym, 1.47 ms) | 54 B (455 sym, 1.82 ms) |
|---|---|---|---|---|
| 50 Hz: phase off its line > 45 degrees | 0 | 0 | 0 | 0 |
| 200 Hz: phase off its line > 45 degrees | 0.77 % | 2.7 % | 6.1 % | 11.8 % |
| 50 Hz, 30 dB, line | 0.88 % | 1.0 % | 1.5 % | 2.0 % |
| 50 Hz, 30 dB, tracker | 0.17 % | 0.39 % | 0.59 % | 0.76 % |
| 50 Hz, 45 dB, line | 1.1 % | 0.99 % | 1.3 % | 1.8 % |
| 50 Hz, 45 dB, tracker | 0.02 % | 0.08 % | 0.08 % | 0.10 % |
| 200 Hz, 30 dB, line | 13.5 % | 12.7 % | 16.9 % | 25.1 % |
| 200 Hz, 30 dB, tracker | 0.30 % | 0.80 % | 1.2 % | 2.2 % |
| 200 Hz, 45 dB, line | 13.5 % | 13.1 % | 17.4 % | 25.8 % |
| 200 Hz, 45 dB, tracker | 0.10 % | 0.13 % | 0.19 % | 0.31 % |

The line's loss is larger than the share of frames whose phase leaves the line by 45 degrees: the header is decided on
the preamble's line alone (not the whole frame's), and 45 degrees is the limit without noise.

- With the line, shorter frames help only a little at 200 Hz: even an empty payload (239 symbols, 0.96 ms: preamble,
  sync word, header and the CRC) loses about 13 %, mostly at the header. The fixed part of v1 (231 symbols) is
  already longer than the phase stays near a line at 200 Hz. A 32-byte message needs 54 payload bytes, so the frame
  cannot shrink below 455 symbols without changing oasa-link's message or the frame format (a shorter header). Neither
  is proposed: the tracker removes the limit at every length.
- A higher symbol rate shortens the frame in time instead: at 500 kbaud (2 MS/s, the same 455 symbols in 0.91 ms), 200 Hz with the
  line loses 7.7 % at 30 dB and 6.9 % at 45 dB; with the tracker 1.3 % and 0.14 % (`waveform per --symbol-rate 500000
  --doppler-hz 200`). It costs twice the bandwidth, 3 dB of Eb/N0 at the same transmit power and twice the receiver
  CPU, and still does not reach the targets with the line.
- With the tracker the loss still grows with the length, but because a longer frame meets more fades, not because
  of the phase.

Airtime per 32-byte message (54-byte payload), QPSK, 250 kbaud:

| Frame | Symbols | Airtime | One-way delay (airtime + receiver time, PC) |
|---|---|---|---|
| v1 uncoded (today) | 455 | 1.82 ms | airtime + 0.37 ms |
| v1 uncoded, tracker | 455 | 1.82 ms | airtime + 0.30 ms |
| v1 light | 685 | 2.74 ms (+0.92 ms) | airtime + 0.78 ms |
| v1 light, tracker | 685 | 2.74 ms (+0.92 ms) | airtime + 0.59 ms |

The receiver time is `cpu_cost`'s one frame's buffer (detection and decode, one CPU); the bench adds its radio and
FIFO path, which does not change. The tracker adds nothing on the air and takes less time than the line.

## 5. CPU

`cpu_cost` (ignored test, pinned to one CPU with `taskset -c 0`): the live receiver as the bridge runs it (1 MS/s,
250 kbaud, `--decimate 1 --coarse-step 4`, `--cfo-search 0.03`), one second of 54-byte frames every 10 ms at 30 dB,
in passes of 4096 samples:

| Frames | Receiver | us per 4096-sample pass | ms per second of signal | One frame's buffer (decode) | Frames delivered |
|---|---|---|---|---|---|
| uncoded, white noise | line (today) | 430 | 105 | 366 us | 100/100 |
| uncoded, white noise | tracker | 411 | 101 | 297 us | 100/100 |
| uncoded, Rayleigh 200 Hz | line | 448 | 110 | 398 us | 76/100 |
| uncoded, Rayleigh 200 Hz | tracker | 410 | 100 | 306 us | 100/100 |
| light, white noise | line | 643 | 157 | 777 us | 100/100 |
| light, white noise | tracker | 584 | 143 | 587 us | 100/100 |
| light, Rayleigh 200 Hz | line | 674 | 165 | 792 us | 54/100 |
| light, Rayleigh 200 Hz | tracker | 571 | 140 | 594 us | 100/100 |

Detection dominates the pass; the tracker replaces the line's maximum-likelihood slope search, which costs more.
Development PC. The 719 us per pass in [sdr-adapter.md](sdr-adapter.md) (`acquire::table`) is the same receiver under
denser traffic (frames 0.7 to 3.2 ms apart); compare the rows here with each other.

## 6. Pilots and the on-air format

Not needed. With the known symbols already in v1 and its own decisions the tracker comes close to what a perfect
phase gives (section 1); what is left are fades, which pilots do not fix. Pilots would cost airtime and buy little in
these models. The header's `pilots`
bit stays reserved (receivers still reject 1). Nothing in this change alters the format: `--phase-track-hz` is
receiver-only and works with any v1 sender, and `--code light` is the existing `payload_code` 1.

## Bench check plan

1. Both bridges with their usual flags (F9: `--sample-rate 1000000 --symbol-rate 250000 --if-hz 175000 --decimate 1
   --coarse-step 4 --tx-mode burst`) plus `--phase-track-hz 200`, over cable (AWGN): same loss, round-trip delay and
   pass time as without it; `sdr_start` logs `"phase_track_hz":200`. One end with it and one without also works (the
   option is receiver-only).
2. Through `waveform cable --channel rayleigh --doppler-hz 50` and `200` (B9's live hour, `research/b9-data`), with and
   without `--phase-track-hz 200`, and with `--code light` on both ends for 200 Hz.
3. Watch the receiver pass time in `--sdr-log`: it should not rise.

## Tests

- `track.rs` unit tests: 200 Hz Rayleigh against the true gain (frames with errors and slips), white-noise phase
  error, crossing a 30 dB fade, the noise estimate, block lengths.
- `waveform/tests/phase_track.rs`: default off; 200 Hz with the light code (line loses over 20 %, tracker under 1 %);
  white noise at the 1 % point (no worse than the line); every length, BPSK and QPSK, both codes, with the tracker.
- `tests/cli.rs`, `src/cli.rs`, `src/waveform_cmd.rs`: the options, their limits, and `per` output unchanged with the
  tracker off.

Regenerate the tables (release build; times on 8 CPUs):

```sh
cargo test --release -p oasa-waveform --test phase_track loss_sources_table -- --ignored --nocapture  # ~3 min
cargo test --release -p oasa-waveform --test phase_track fading_table -- --ignored --nocapture        # ~5 min
cargo test --release -p oasa-waveform --test phase_track awgn_table -- --ignored --nocapture          # ~1 min
cargo test --release -p oasa-waveform --test phase_track frame_length_table -- --ignored --nocapture  # ~1 min
taskset -c 0 cargo test --release -p oasa-waveform --test phase_track cpu_cost -- --ignored --nocapture
```
