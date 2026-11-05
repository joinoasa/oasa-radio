# `sdr` adapter: OASA's waveform on a software-defined radio

`oasa-radio bridge --radio sdr` carries packets (already encrypted by oasa-link) as frame v1 of OASA's own waveform over an SDR. It does not drive the radio itself. It talks to it through two continuous streams of IQ samples (cf32, little endian), normally two FIFOs, and another program moves those streams to and from the hardware (for UHD radios: a small streaming script, or UHD's example tools).

```text
  oasa-link --UDP--> bridge --> modulate --> TX stream (zeros between frames) --> tx FIFO --> radio --> air
  oasa-link <--UDP-- bridge <-- frames <-- StreamReceiver <-- decimate <-- mix <-- rx FIFO <-- radio <-- air
```

## How it works

- **TX never stops.** Between frames the stream carries zeros, so the radio's transmit chain stays up and a packet goes on air as soon as the stream reaches it. Writing to a FIFO paces the stream at the radio's rate; the writer also never runs more than `--tx-lead-ms` (20 ms) ahead of the wall clock, so a packet waits at most that long behind zeros already written.
- **Digital IF.** Both streams sit at `--if-hz` (250 kHz at 1 MS/s). The DC offset and LO leak of a direct-conversion radio then fall outside the signal (W1 found zero IF about 5 dB worse, [waveform-w0.md](waveform-w0.md)).
- **Receive.** Shift back by the IF plus the tracked clock offset, low-pass, keep every `--decimate`-th sample, and feed blocks of `--block` samples to the streaming receiver. Each frame whose CRC passes is one packet; a bad CRC is dropped and counted (`bad_frame`). `--track-cfo on` moves the offset estimate by 0.3 of each good frame's residual.
- **One frame per packet.** Frame v1 carries 1 to 255 bytes, so `--mtu` is at most 255 (default 200; oasa-link's handshake is 119 bytes). Give oasa-link the same `--mtu`. The receiver rejects at the header any frame longer than the MTU or with another `--code`.
- **Lifetimes are tied.** When the program reading the TX FIFO stops, the write fails and the bridge exits; when the bridge exits, the radio tool sees the FIFO end. The transmitter cannot outlive the bridge.
- **No packet bytes in logs.** `--sdr-log` has sizes, counts, offsets and timings only. `--rx-record` saves the received baseband (raw samples, as any receiver could), for lab checks that the air carries only ciphertext.

## Run it

Two radios, frequency division (each transmits on its own frequency):

```sh
mkfifo a.tx a.rx b.tx b.rx
# radio tool A: reads a.tx -> TX 915 MHz, RX 920 MHz -> writes a.rx (1 MS/s, cf32)
oasa-radio bridge --radio sdr --tx a.tx --rx a.rx --cfo-hz 0 \
  --udp-listen 127.0.0.1:7001 --udp-send 127.0.0.1:7101 --sdr-log a-sdr.jsonl
# radio tool B: the other way round, and a second bridge on 7002 / 7102
```

`--cfo-hz` is the starting estimate of the other radio's clock offset (the tracker follows it from there; `--cfo-search` covers the rest at the start).

Without radios, `oasa-radio waveform cable` joins one bridge's TX FIFO to the other's RX FIFO with a gain, clock offset and noise. `tests/sdr_link.rs` runs two bridges through two cables at a fifth of the bench rates.

## Bench results (2026-11-05, not yet a counted lab run)

B200mini and E310 over the lab's cabled 60 dB paths, B200 to E310 at 915 MHz and E310 to B200 at 920 MHz, 1 MS/s, QPSK 125 kbaud, no coding, IF 250 kHz, RX gain 60. Real oasa-link on both ends (encrypted), 9 runs:

| | |
|---|---|
| Handshake | 0.08 to 0.17 s, one handshake per run (the batch relay before this took about 180 s) |
| Delivery | 849 of 850 packets |
| One-way delay, medians | 32 to 68 ms, max 75 ms |
| 10 minutes of streaming | 102 frames each way, 0 bad CRC, no overflows, 3 and 5 underflows |

Next: a lab suite (`l2-sdr-live-link` in oasa-lab) so this becomes a counted run.

## Limits

- One radio tool per radio; this adapter never opens hardware.
- No MAC: each side transmits on its own frequency. Sharing one frequency needs a MAC (later).
- `--code light` halves the bit rate. The code itself is measured on the bench by `l2-waveform-w1`, but not yet through this adapter (the tests and the runs above use `--code none`).
