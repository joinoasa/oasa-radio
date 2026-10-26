# oasa-radio

**OASA radios:** adapters that carry OASA's encrypted link over off-the-shelf radios, settings for each radio, and OASA's own experimental waveforms on software-defined radios.

Tech tree nodes **R-COM1** (basic radio link) and the radio side of **R-COM2**. Later: **T-RF1** (RF scanning) and adaptive waveforms.

- Plan: [oasa-lab/docs/r-com2-plan.md](https://github.com/joinoasa/oasa-lab/blob/main/docs/r-com2-plan.md)
- Encryption lives in [oasa-sec](https://github.com/joinoasa/oasa-sec); tests in [oasa-lab](https://github.com/joinoasa/oasa-lab).

Status: v0 adapters (`udp`, `serial`), tested on Linux with pseudo-terminals. Not yet tried on real radios. Licensed under [Apache-2.0](LICENSE).

## What it does

Encryption never happens here. `oasa-link` encrypts; this repo moves the encrypted packets over a radio. It never looks inside a packet, never logs one (sizes and counts only), and never counts a radio's own encryption as the OASA link.

`oasa-link` stays UDP on its radio side. Each radio gets a **bridge**: a small program that sits between that UDP and the radio.

```
 flight controller / GCS
        |  (plaintext UDP, loopback)
   oasa-link ---- encrypts ----+
        ^                      |  UDP, loopback
        |                      v
   --udp-send            --udp-listen
        +------- oasa-radio bridge -------+
                       |                  |
                    adapter           (serial, udp, ...)
                       |
                     radio  ~~~~ air ~~~~  radio
                                              |
                  (the same stack on the other end)
```

Packets from `oasa-link` arrive on `--udp-listen` and go out the radio. Packets from the radio go to `--udp-send`. So `oasa-link` plus a bridge is an encrypted link over that radio, and the two repos stay apart.

## Adapters

| Kind | For | Status |
|---|---|---|
| `udp` | Simulation (put the lab's simulated channel between two bridges), Wi-Fi, Ethernet | Works |
| `serial` | SiK, mLRS, RFD900x, any transparent UART modem | Works. Framing: [docs/serial-framing.md](docs/serial-framing.md) |
| `lora` | Our LoRa modem firmware for the Heltec WiFi LoRa 32 V3. It speaks the serial framing: use `--radio serial --baud 115200 --mtu 255`, and `oasa-radio lora` for settings and counters | Works in simulation; real board untested. [firmware/heltec-v3](firmware/heltec-v3/README.md) |
| `sdr` | OASA's own waveforms on SDRs | Planned. The modem (W0) runs in simulation: `oasa-radio waveform`, [docs/waveform-w0.md](docs/waveform-w0.md) |

## Quick start

Build (Rust stable, Linux):

```
cargo build --release --bins
```

Two devices, each with an `oasa-link` identity (`oasa-pair init`, then `oasa-pair trust`; see oasa-sec). On the ground station:

```
oasa-radio bridge --radio serial --device /dev/ttyUSB0 --baud 57600 --mtu 252 \
  --udp-listen 127.0.0.1:7821 --udp-send 127.0.0.1:7801

oasa-link --dir gcs --peer drone --bind 127.0.0.1:7801 --remote 127.0.0.1:7821 \
  --initiate --mtu 252 --plain-listen 127.0.0.1:14550 --plain-send 127.0.0.1:14551
```

The drone side is the same with its own device, its own ports and no `--initiate`. Give `oasa-link` the same `--mtu` as the bridge (at least 128; it makes link packets fit one serial frame). Set the radio itself to transparent (no packet mode, no encryption you rely on).

Other commands:

```
oasa-radio info                                         # adapters and their status
oasa-radio info --radio serial --device /dev/ttyUSB0    # open one and show its settings
oasa-radio bridge --help
oasa-radio waveform ber --mod qpsk --ebn0-db 0,2,4,6,8 --bits 200000 --seed 1   # OASA's own modem in simulation: BER against theory
```

The bridge prints one JSON line per event (`start`, `drop`, `error`) and a final `stats` line on `SIGTERM`, `SIGINT` or `--duration`, then exits 0. At most 10 `drop` lines a second.

## Limits

- Linux only for now (pseudo-terminals, `/proc`).
- Serial: 8N1, no flow control, standard baud rates (9600 to 4 Mbaud, see `oasa-radio bridge --help`).
- The bridge adds no queue of its own. If `oasa-link` sends faster than the line, the kernel's UDP buffer fills and drops packets, and what is queued arrives late.
- Radios are reached over cable or in the shielded box unless a test plan says otherwise.

## Tests

| Command | What |
|---|---|
| `cargo test --locked` | The waveform modem (`waveform/`: BER against theory with fixed seeds, frame round trips, false alarms, CRC; about 10 s), framing (round trips, every truncation, every bit flip, noise, seeded damage), the adapters, the CLI, and two bridges over a pseudo-terminal relay that drops and garbles bytes at 57600 baud |
| `scripts/serial-e2e.sh` | Two real `oasa-link` processes through two serial bridges. Marked packets both ways; the captured line bytes must never hold the marker. A plaintext control proves the scanner can see one |

CI runs both, plus `cargo fmt`, `clippy -D warnings`, `cargo deny` and `cargo audit`. For the end-to-end script, build [oasa-sec](https://github.com/joinoasa/oasa-sec) (`cargo build --locked --bins`) and point `OASA_SEC_BIN_DIR` at its `target/debug`. No `socat` needed: `examples/ptyrelay.rs` makes the pseudo-terminal pair, can drop and garble bytes, limits speed and captures what crosses.

## Adding an adapter

1. Make `src/adapters/<kind>.rs` and implement `Radio` ([src/radio.rs](src/radio.rs)): `info()` (kind, target, MTU, bitrate if known), `send`, `recv(timeout)` and `counters()`. Count drops by reason; never log or inspect packet bytes.
2. A byte-stream radio frames its packets (reuse `Framer`, or use the radio's own framing such as KISS). A packet radio sends one packet per call.
3. Add it to `RadioSpec` in `src/cli.rs` (options and `open`) and to `adapters/mod.rs`.
4. Tests: unit tests, then a bridge test like `tests/serial_pty.rs`. Every adapter also needs lab tests in [oasa-lab](https://github.com/joinoasa/oasa-lab), starting with the simulated channel.
5. Add any outside crate to the table below (permissive licenses only; `cargo deny` must pass) and any outside radio project to the [software register](https://joinoasa.org/docs/software-register/).

## Built on

| Crate | Version | License | Why |
|---|---|---|---|
| [nix](https://crates.io/crates/nix) | 0.31 | MIT | Safe terminal setup, `poll` and pseudo-terminals, so we need no `unsafe` for them |
| libc, bitflags, cfg-if, cfg_aliases | via nix | MIT or Apache-2.0 | nix's own dependencies |
| `oasa-waveform` (ours, in `waveform/`) | 0.1 | Apache-2.0 | OASA's own modem for milestone W0: 2-FSK, BPSK, QPSK, framing, channel model and theory curves. Pure Rust, no dependencies; erfc, the random generator and the CRC are written here with known-answer tests |
| [embedded-hal](https://crates.io/crates/embedded-hal) | 1.0 | MIT or Apache-2.0 | SPI and pin traits for the `no_std` modem core |
| esp-hal, esp-rtos, esp-bootloader-esp-idf, embassy-executor, embassy-time, embassy-futures, embassy-sync | see `firmware/heltec-v3/Cargo.toml` | MIT or Apache-2.0 | Heltec V3 firmware only (not in the host build) |

`lora-phy` was looked at and not used: it reports a CRC-failed packet as good and cannot set an arbitrary sync word. The SX126x driver is in `firmware/modem-core/src/sx126x.rs`.

COBS and CRC-16 are about 50 lines and are written here, with known-answer tests. `serialport` was left out: it is MPL-2.0, which our license set does not allow. The one `unsafe` block is the signal binding in `src/main.rs` (and the same in `examples/ptyrelay.rs`); everything else is `#![forbid(unsafe_code)]` or `deny`.
