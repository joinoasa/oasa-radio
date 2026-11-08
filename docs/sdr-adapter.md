# `sdr` adapter: OASA's waveform on a software-defined radio

`oasa-radio bridge --radio sdr` carries packets (already encrypted by oasa-link) as frame v1 of OASA's own waveform over an SDR. It does not drive the radio itself. It talks to it through two continuous streams of IQ samples (cf32, little endian), normally two FIFOs, and another program moves those streams to and from the hardware (for UHD radios: a small streaming script, or UHD's example tools).

```text
  oasa-link --UDP--> bridge --> modulate --> TX stream (zeros between frames) --> tx FIFO --> radio --> air
  oasa-link <--UDP-- bridge <-- frames <-- StreamReceiver <-- decimate <-- mix <-- rx FIFO <-- radio <-- air
```

## How it works

- **Two TX modes.**
  - `--tx-mode stream` (default): one continuous cf32 stream, zeros between frames, never more than `--tx-lead-ms` (20 ms) ahead of the wall clock. Works with any tool that plays a stream. Its weakness: paced by the host clock, so every time the radio runs dry (an underflow) the radio's timeline falls behind and the extra samples wait in buffers for good.
  - `--tx-mode burst`: one record per frame (u32 LE sample count, then cf32), nothing between frames. The radio tool decides when each frame goes out. The lowest delay comes from a tool that keeps the transmitter streaming and gives every packet a radio timestamp, a few ms ahead of the radio's clock (below). Do not send short bursts with the transmitter off in between: the B200mini's output takes tens of ms to come up, and 4 ms frames arrived about 14 dB weak.
- **Digital IF.** Both streams sit at `--if-hz` (250 kHz at 1 MS/s). The DC offset and LO leak of a direct-conversion radio then fall outside the signal (W1 found zero IF about 5 dB worse, [waveform-w0.md](waveform-w0.md)).
- **Receive.** Shift back by the IF plus the tracked clock offset, low-pass, keep every `--decimate`-th sample, and feed blocks of `--block` samples (512, 1 ms) to `LiveReceiver` (oasa-waveform `stream.rs`). It computes the detection metric once per position as samples arrive and releases each frame as soon as the frame's own samples are in (the v1 header gives the length), not after room for the longest frame. Same frames as the whole-buffer receiver (tests/live.rs). Each frame whose CRC passes is one packet; a bad CRC is dropped and counted (`bad_frame`). `--track-cfo on` moves the offset estimate by 0.3 of each good frame's residual.
- **One frame per packet.** Frame v1 carries 1 to 255 bytes, so `--mtu` is at most 255 (default 200; oasa-link's handshake is 119 bytes). Give oasa-link the same `--mtu`. The receiver rejects at the header any frame longer than the MTU or with another `--code`.
- **Lifetimes are tied.** When the program reading the TX FIFO stops, the write fails and the bridge exits; when the bridge exits, the radio tool sees the FIFO end. The transmitter cannot outlive the bridge.
- **No packet bytes in logs.** `--sdr-log` has sizes, counts, offsets and timings only. `--rx-record` saves the received baseband (raw samples, as any receiver could), for lab checks that the air carries only ciphertext.

## Run it

Two radios, frequency division (each transmits on its own frequency):

```sh
cd /dev/shm && mkfifo a.tx a.rx b.tx b.rx   # FIFOs on a RAM filesystem (below)
# radio tool A: reads a.tx -> TX 915 MHz, RX 920 MHz -> writes a.rx (1 MS/s, cf32)
oasa-radio bridge --radio sdr --tx a.tx --rx a.rx --cfo-hz 0 \
  --udp-listen 127.0.0.1:7001 --udp-send 127.0.0.1:7101 --sdr-log a-sdr.jsonl
# radio tool B: the other way round, and a second bridge on 7002 / 7102
```

**Put the FIFOs on a RAM filesystem** (`/dev/shm`, `/run/user/$UID`), not on ext4 or another journaling disk filesystem. Every write to a FIFO updates its inode times, which a journaling filesystem commits through its journal (every 5 s on ext4), and the writer can wait for that commit: on the lab VM, two bridges joined by `waveform cable` saw round trips of 10-70 ms exactly every 5 s with the FIFOs on ext4, and none above 8.6 ms with the same FIFOs on `/dev/shm` (3 minutes, 18000 pings of 32 bytes at 100/s each way, all logs still on disk). The logs can stay on disk: the adapter writes them from a separate thread.

`--cfo-hz` is the starting estimate of the other radio's clock offset (the tracker follows it from there; `--cfo-search` covers the rest at the start).

### Wide search until lock, then narrow (`--cfo-acquire`)

Plain crystals (+-20 ppm, 18.3 kHz at 915 MHz, 0.073 cycles per symbol at 250 kbaud) are outside the default search (`--cfo-search 0.03`, about +-7.5 kHz nominal). A wide `--cfo-search` finds them but costs its wide FFT on every block for good. `--cfo-acquire CYC` searches that wide only until frames decode:

- **Acquiring:** search `--cfo-acquire` around the tracked offset. Each good frame's estimate is taken whole (no smoothing), so after `--cfo-lock-frames` good frames in a row (default 2) the offset is centred.
- **Locked:** search `--cfo-search` around it, tracking as `--track-cfo on` always does (0.3 of each good frame's residual).
- **Lock lost** after `--cfo-unlock-bad` bad CRCs in a row (default 3) or `--cfo-unlock-ms` of stream with no good frame (default 500): back to the wide search, centred on the last tracked offset. An idle link unlocks too; its next frames are found by the wide search.
- `--sdr-log` gets `{"event":"cfo_lock","cfo_hz":…,"cfo_search":0.03,"stream_ms":…}` and `{"event":"cfo_unlock",…,"reason":"bad_frames"|"timeout"}`. Without `--cfo-acquire` nothing changes.

The estimate of each frame is taken against the shift its own samples got: after a jump of the offset, frames already read keep the old shift.

Simulation at the bench's settings (1 MS/s, 250 kbaud, IF 175 kHz, `--decimate 1 --coarse-step 4`; random 40 to 200-byte frames 0.7 to 3.2 ms apart, Es/N0 about 20 dB, 2 s each; `cargo test --release --lib acquire::table -- --ignored --nocapture`; time per receiver pass of 4096 samples on the development PC):

| Offset | Search | Locked at | Lost frames | FFT points per pass | us per pass |
|---|---|---|---|---|---|
| 0, +-18.3 kHz, +-30 kHz | `--cfo-acquire 0.15` (locked: 0.03) | 12.3 ms | 0 of 410 | 32 700 | 717-726 |
| 0 | always 0.03 (default) | - | 0 of 410 | 32 700 | 719 |
| +-18.3 kHz, +-30 kHz | always 0.03 | - | all | | |
| 0, +-18.3 kHz | always 0.1 | - | 0 (0 Hz), 2 of 410 (+-18.3 kHz) | 130 800 | 1260-1267 |
| +-30 kHz | always 0.1 | - | all (0.1 reaches about 29 kHz here) | | |
| 0, +-18.3 kHz, +-30 kHz | always 0.15 | - | 0 (0 Hz), 2 of 410 (others) | 130 800 | 1286-1297 |

Locked, the receiver does the default search's work. A ramp of 18.3 to 28.3 kHz over 2 s stays locked with no frame lost. A step from 18.3 to 40 kHz (past the narrow search, which reaches about +-13 kHz) unlocks 500 ms after its last good frame (`--cfo-unlock-ms`) and relocks 25 ms later at 40 kHz; the frames in between are lost, every one after is delivered.

### Interference options (B8, opt-in, off by default)

See [interference.md](interference.md) for the measurements behind them.

- `--rx-filter on`: a 63-tap channel filter after the shift and decimation, cut off at 231 kHz at 250 kbaud. With `--decimate 1` nothing else stops a signal beside the band, and the detector normalizes by all the power in its window, so a carrier one channel below, 7 dB over the signal, hides the preambles. With the filter a carrier there can be 51 dB over the signal, noise 18 dB.
- `--rx-notch-db 25`: removes the strongest carrier in each receiver block when it stands 25 dB over the block's mean FFT bin (a carrier at about the signal's power or more; noise and the signal itself stay below 16 dB). A carrier on the signal's centre can then be 29 dB over the signal (0.1 % loss, simulation); without it the limit is 7 dB under the signal.
- `--cfo-reset-ms MS` (with `--cfo-acquire`): acquiring with no good frame for MS, put the offset back to `--cfo-hz` (logs `cfo_reset`). A frame that passes both CRCs by chance (noise under a jammer: rare, but each one is taken whole while acquiring) can move the tracked offset more than the wide search reaches from the signal, and nothing then brings it back. With the reset it comes back MS later.

`sdr_start` in `--sdr-log` lists `rx_filter`, `rx_notch_db` and `cfo_reset_ms`.

Without radios, `oasa-radio waveform cable` joins one bridge's TX FIFO to the other's RX FIFO with a gain, clock offset and noise. `tests/sdr_link.rs` runs two bridges through two cables at a fifth of the bench rates.

## Delay, step by step (bench, 2026-11-05/06, not counted)

Radio to radio, one way, 64-byte packets, medians (both bridges on one host clock, so each frame is timed from queued to decoded):

| Setup | B200 to E310 | E310 to B200 | Round trip (UDP ping, both ways) |
|---|---|---|---|
| `stream`, 20 ms lead, old receiver (4096 blocks, waits for the longest frame) | 31 ms | 35 ms | 80 ms |
| `stream`, 5 ms lead, old receiver | 28 | 29 | 60 |
| `stream`, 5 ms lead, `LiveReceiver`, 512 blocks | 18 | 19 | 41 |
| `burst`, radio tool streams with a radio timestamp per packet, 2 to 3 ms ahead | **8** | **10** | **22 to 25** |

| same, bridge polls oasa-link every 0.2 ms (was 2 ms), 32-byte packets at up to 100/s | 7 | 8 | 16 |
| same, 250 kbaud at 1 MS/s (`--symbol-rate 250000 --decimate 1 --if-hz 175000 --block 512`) | **6** | **7** | **12.6** (p99 15.6) |

At 250 kbaud and 1 MS/s the IF must come down to 175 kHz: at 250 kHz the signal's upper edge (419 kHz) sits in the radios' own filters and frames arrived weak (metric 0.06 instead of 0.85-0.9). At 2 MS/s the E310 (over the network) needs a 6 ms lead and gives back what the shorter frame saved.

In the fourth row, 4.1 ms is the frame on air, 2 to 3 ms the lead, 0.7 ms the decision; the receive side of UHD adds 0.3 ms (E310) and 1.2 ms (B200). With oasa-link on both ends: handshake 38 to 54 ms, 600 of 600 messages, one-way 10 to 12 ms median, 17 ms max.

**Detector step.** The coarse search tests a position every quarter symbol by default (every sample at 4 samples per symbol), about 95 % of the receiver's CPU. `--coarse-step N` tests every N samples instead; the fine search around each candidate still finds the timing to the sample. At 1 MS/s and 250 kbaud (4 samples per symbol), one symbol (`--coarse-step 4`) costs 57 % less (327 to 142 ms of CPU per second of signal on a PC) and finds the same frames: 32-byte frames at random offsets, 400 per point, steps 1, 2 and 4 within 4 frames of each other from 2 to 8 dB Eb/N0 (`cargo test --release --lib detection_vs_coarse_step -- --ignored --nocapture`). Default unchanged.

The old receiver searched its whole window (one longest frame) on every block, about 1.5 ms of CPU per pass: with small blocks it fell behind and the delay grew. `LiveReceiver` costs under 1 ms per pass at 1 ms blocks.

## Bench results, first runs (2026-11-05, not yet a counted lab run)

B200mini and E310 over the lab's cabled 60 dB paths, B200 to E310 at 915 MHz and E310 to B200 at 920 MHz, 1 MS/s, QPSK 125 kbaud, no coding, IF 250 kHz, RX gain 60. Real oasa-link on both ends (encrypted), 9 runs:

| | |
|---|---|
| Handshake | 0.08 to 0.17 s, one handshake per run (the batch relay before this took about 180 s) |
| Delivery | 849 of 850 packets |
| One-way delay, medians | 32 to 68 ms, max 75 ms |
| 10 minutes of streaming | 102 frames each way, 0 bad CRC, no overflows, 3 and 5 underflows |

Next: a lab suite (`l2-sdr-live-link` in oasa-lab) so this becomes a counted run.

## Limits

- One radio tool per radio; this adapter never opens hardware. The lab's tool is a Python script; at 2 to 3 ms leads it misses a deadline a few times a minute (a skipped stretch, not a lost frame, most of the time). A native tool with a real-time thread is the next step.
- No MAC: each side transmits on its own frequency. Sharing one frequency needs a MAC (later).
- `--code light` halves the bit rate. The code itself is measured on the bench by `l2-waveform-w1`, but not yet through this adapter (the tests and the runs above use `--code none`).
