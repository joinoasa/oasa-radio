# Interferers and a jammer in simulation (B8)

`oasa-radio waveform sir` measures packet loss against signal-to-interference (S/I) with a carrier, band-limited
noise or a swept tone beside or on top of the signal (`waveform/src/interference.rs`). The jammer tests run the live
receive chain of the `sdr` adapter on a simulated stream (`src/adapters/sdr.rs`, `acquire::jam*`).

```sh
# S/I sweep, carrier one channel (420 kHz) below the signal, the live link's settings, Eb/N0 20 dB
oasa-radio waveform sir --interferer tone --offset-hz -420000 --sir-db 0,-5,-10 --frames 10000 --headroom-db 54
# noise 420 kHz wide, next bench channel (5 MHz), with the receiver's channel filter and carrier canceller
oasa-radio waveform sir --interferer noise --offset-hz 5000000 --sir-db -40,-50 --channel-filter --notch-db 25
# 10 runs of a 2 s jammer through the live chain
cargo test --release --lib acquire::jammer_table -- --ignored --nocapture
```

## Models

| What | How |
|---|---|
| Carrier (`tone`) | a CW at `--offset-hz` from the signal's centre |
| Noise (`noise`) | Gaussian, flat over `--width-hz` (420 kHz: the signal's own width) around the offset, nothing outside: frequency-domain blocks joined by a cos/sin crossfade |
| Sweep (`sweep`) | a tone sweeping `--span-hz` upwards every `--period-us`, phase continuous |
| Radio front end | the signal at the IF (+175 kHz) in the radio's band, raised to `--ratio` times the modem's rate (automatic: enough to hold the interferer, 16 for 5 MHz); the interferer added there; I and Q clipped at full scale with the signal `--headroom-db` below it; a decimating filter (Kaiser, 80 dB, passes +-0.42 MHz, stops from +-0.58 MHz) down to 1 MS/s; then the thermal noise at the given Eb/N0, and the receiver as on the air |

S/I is the signal's mean power over a frame over the interferer's power, both at the antenna (before any filter), as a
power meter on each source would read them. The interferer runs continuously; frames come and go through it.

"One channel" here is the signal's own occupied width, 420 kHz (QPSK at 250 kbaud, roll-off 0.35), so +-1 and +-2
channels are +-0.42 and +-0.84 MHz. The bench's two links are 5 MHz apart (915 and 920 MHz), so the other link's
transmitter is the bench's "next channel" at 5 MHz.

The signal sits at +175 kHz in the radio's 1 MS/s band (the IF keeps it off the radio's DC and LO leakage). An
interferer 420 kHz below the signal is at -245 kHz, inside the band; 420 kHz above, at +595 kHz, outside it, and only
the radio's filter decides how much of it comes in. So + and - offsets differ.

The radio's own filters, mixer and LO phase noise are not modelled beyond this one filter and the clip. The bench
measures them.

## Results

QPSK, frame v1, 54-byte payloads (a 32-byte oasa-link message), 250 kbaud at 1 MS/s, IF 175 kHz, `--cfo-search 0.1
--coarse-step 4`, Eb/N0 20 dB, signal 54 dB below ADC full scale (about where the bench's RX gain puts it: a receive
noise floor near -71 dBFS and Es/N0 about 23 dB). Coarse scan in 3 dB steps (500 frames), then 1 dB steps of 10,000
frames around the edge; the 0.1 % point by log-linear interpolation. The search: `waveform sir` at each point, scripted.

**S/I at 0.1 % frame loss** (dB; lower is better; *clip*: the interferer reaches ADC full scale first, so the number is
the headroom, not the modem):

| Interferer at | Tone | Tone, `--channel-filter --notch-db 25` | Noise 420 kHz | Noise, filter + notch |
|---|---|---|---|---|
| 0 (same channel) | **+6.9** | **-28.8** | +11.3 | +11.3 |
| -0.42 MHz (-1 ch) | **-7.3** | -51.7 | -6.5 | -18.3 |
| +0.42 MHz (+1 ch) | -54.4 clip | -54.4 clip | -11.8 | -18.5 |
| -0.84 MHz (-2 ch) | -54.4 clip | -54.4 clip | **-16.0** | -43.1 clip |
| +0.84 MHz (+2 ch) | -54.7 clip | -54.7 clip | -43.2 clip | -43.2 clip |
| -5 MHz (bench channel) | about -56 clip | about -56 clip | -45.2 clip | -45.2 clip |
| +5 MHz (bench channel) | -54.4 clip | -58.6 clip | -43.3 clip | -43.3 clip |
| Sweep, 400 kHz span every 1 ms, centred | +6.2 | +6.2 | | |

Without the clip (no headroom limit), with filter and notch, a tone at +0.84 MHz or 5 MHz and noise at 5 MHz cost no
frame down to -96 dB, noise at +0.84 MHz -88 dB: the simulated radio filter is not the limit; ADC full scale is. Noise
clips 10 dB sooner than a tone (its peaks). The -5 MHz tone edge is ragged (a clipped carrier's products land in the
band or not depending on where they alias).

What it means:

1. **The receiver had no channel filter at `--decimate 1`.** The detector normalizes by all the power in its window, so
   anything beside the band that the matched filter would ignore still hides preambles: a carrier 7 dB over the
   signal one channel below, noise 6.5 dB over it, take frames out. At `--decimate 2` the decimator's filter did this
   job; the move to `--decimate 1` (F9's delay work) removed it. `--rx-filter on` (63 taps, 231 kHz) gives
   the -1 channel case back 44 dB (tone) and 12 dB (noise).
2. **A carrier on the channel was the worst case**: +6.9 dB, a carrier at a fifth of the signal's power: the preamble search and the phase
   fit break long before the QPSK decisions would. `--rx-notch-db 25` estimates and
   subtracts it per block: -28.8 dB, 36 dB better. It does nothing for noise or a fast sweep (not one steady
   frequency in a block).
3. **Noise on the channel is noise**: +11.3 dB is the AWGN 0.1 % point (about 10.5 dB Eb/N0) plus the 20 dB of thermal
   noise already there, as expected. No receiver trick helps; that needs a different channel (hopping) or more power.
4. **From two channels out the limit is the radio**: once the interferer is filtered out, only ADC full scale, the radio's
   own filters and its LO's phase noise matter. On the bench this is set by the RX gain, not the modem: 10 dB less
   gain buys 10 dB here (and costs sensitivity).

Cost on the PC: the receiver's CPU per second of stream goes from 0.26 to 0.32 s with filter and notch on (the
jammer table's load column). Both stay off by default.

## Jammer and recovery

A noise jammer 40 dB over the signal, flat over its 420 kHz band, for 2 s, then clear; the stream clips at full scale
as a radio's ADC would (1.76 million samples clipped per run). Frames of 40 to 200 bytes 0.5 to 3 ms apart, signal at
+18.3 kHz offset (F20's 20 ppm) with `--cfo-acquire 0.15 --cfo-search 0.03`, and at +300 Hz with plain `--cfo-search 0.1`.
10 seeds each (`acquire::jammer_table`):

| | acquire 0.15 / 0.03, +18.3 kHz | always 0.1, +300 Hz |
|---|---|---|
| Frames before the jam delivered | 1010 of 1012 (the 2 lost are at start-up, see below) | 1012 of 1012 |
| Good frames during the jam (false or real) | 0 | 0 |
| Bad frames (CRC failed) during the jam | 4 in 20 s | 4 in 20 s |
| Link reports down | `cfo_unlock` timeout at 495 to 504 ms, 10 of 10 | no lock state; oasa-link's own timeout |
| First frame after the jam delivered | 10 of 10 | 10 of 10 |
| End of jam to end of the first good frame, ms | 5.3, 7.0, 3.9, 8.5, 8.0, 6.9, 3.9, 4.2, 4.5, 5.1 | the same |
| Relocked (narrow search) | 10.8 to 14.9 ms after | |
| Frames after the jam delivered | 2056 of 2056 | 2056 of 2056 |
| Tracked offset after | 18,297 to 18,306 Hz | 293 to 302 Hz |

The time back is the wait for the next frame to end: the receiver takes the first frame that starts after the jam,
every time.

**Why 2 s stands for 60 s.** All the state the receive chain keeps is bounded: the receiver's window (one longest
frame), the offset history (64 entries), the acquisition state (unlocked after 500 ms without a good frame) and the
tracked offset, which only good frames move. After the first 500 ms of a jam none of it changes again, so the state at
the end of a 2 s jam and of a 60 s one is the same, unless a good frame arrives in between. Under the jam that can only
be a false one: a noise candidate that passes the header's CRC-8 and then the CRC-16 by chance. We saw 4 candidates
reach the CRC-16 in 20 s; at 1 in 65,536 that is about 3 false frames per million seconds: 0.0002 in 60 s, 0.26 in a
day of jamming.

**But one false frame could strand the receiver.** Acquiring, a good frame's offset is taken whole, so a false one can
move the tracked offset anywhere within the wide search (+-37.5 kHz). If that leaves the real signal outside the wide
search, nothing brings it back: no good frame, no correction (`a_false_frame_under_a_jammer_strands_the_search_unless_reset`
shows it: 0 of 200 frames after the jam). `--cfo-reset-ms MS` (opt-in) puts the offset back to `--cfo-hz` after MS
acquiring without a good frame; with it the link comes back at the first frame after the jam. Its cost: the first good
frame then re-centres the offset by the whole crystal offset, which loses the frame already in the receiver (6 of 10
seeds lost one frame after the jam with `--cfo-reset-ms 1000`). The same re-centring costs one frame at start-up in
2 of 10 seeds with `--cfo-acquire` (an existing effect, not new here).

## Opt-in receiver options (all off by default)

| Option | Fixes | Cost |
|---|---|---|
| `bridge --radio sdr --rx-filter on` (`waveform sir --channel-filter`) | adjacent-channel signals hiding preambles at `--decimate 1` | 63-tap FIR, about +23 % receiver CPU on the PC (filter and notch together) |
| `--rx-notch-db 25` (`--notch-db 25`) | a carrier on or near the channel | one FFT and three passes per block; nothing when no carrier is found |
| `--cfo-reset-ms MS` (with `--cfo-acquire`) | a false frame stranding the wide search | one frame at the relock after a long gap |
