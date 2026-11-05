# Security review: R-COM1, the basic radio link

**Status:** v0.1, 2026-11-05 · **Kind:** internal review written by an AI agent (Claude) from the code at `801df47`, not an outside review · **Owner:** citruscybersec

Part of R-COM1's release (oasa-lab [release.md](https://github.com/joinoasa/oasa-lab/blob/main/docs/release.md)). In short: the adapters move bytes only and never parse or log packet contents, and a radio's own encryption never counts. The real risk is the off-the-shelf radio itself (H1).

## Scope and method

Read: `src/radio.rs`, `src/adapters/serial.rs`, `src/adapters/udp.rs`, `src/bridge.rs`, `src/framing.rs`, `src/log.rs`, `src/cli.rs`, `src/main.rs`, the host side of `src/lora.rs`, `src/relay.rs` (test tool only, not in the radio path), the modem's frame and serial handling (`firmware/modem-core/src/modem.rs`), [serial-framing.md](serial-framing.md) and the README. SiK facts from ArduPilot's [SiK advanced configuration](https://ardupilot.org/rover/docs/common-3dr-radio-advanced-configuration-and-technical-information.html) page and the [SiK README](https://github.com/ArduPilot/SiK). Ran `cargo test --locked --workspace` and a throwaway fuzz loop (outside the repo) of 1,000,000 random, low-entropy and bit-flipped inputs through `Framer::push_frames` and `cobs_decode` at MTU 1, 5, 252, 255 and 65535, plus 10 MB without a delimiter: no panic, every delivered packet within the MTU, every extended body within 512 bytes. No real radios were used.

## Findings

One high finding, from the radio, not the code. No medium.

| ID | Severity | Finding | Fix | Status |
|---|---|---|---|---|
| H1 | High | **Anyone in range with a SiK radio can change our radio's settings.** SiK radios link to any radio with the same NETID and air settings; there is no authentication between radios. Once linked, the other side can send `RT` commands (remote AT), for example a new NETID, frequency or TX power, and `RT&W` saves it. NETID defaults are well known. Packets stay safe (oasa-link authenticates and encrypts), but the link can be taken down until someone resets the radio, and the radio can be pushed outside its intended power or band. Anyone in range can also jam or inject frames; that is expected and oasa-link drops what it cannot authenticate. The README does not say any of this. | README Limits: say it. Radio setup notes: a non-default NETID and air settings (not a defence, just fewer accidents). Later: the bridge reads the settings (`ATI5`) before it starts and refuses a mismatch; our own firmware (R-COM2) has no remote settings. Bench tests over cable or in the shielded box, as now. | Open |
| L1 | Low | **The UDP side trusts every local process.** The bridge accepts packets on `--udp-listen` from the `--udp-send` IP only, not its port (`src/bridge.rs:161`), so any process on the same host can send on the radio (airtime), and one that binds the `--udp-send` port first gets the radio's packets. Only ciphertext crosses this side. | Check the port too, or document that the host is trusted. | Open |
| L2 | Low | **`lora air --hex` prints whatever was heard, not only ciphertext.** It prints every frame's payload (`src/lora.rs:710-712`), including other people's LoRa traffic on that channel and the lab's plaintext controls. The help says "for the lab's captures of encrypted traffic only" (`src/lora.rs:361-362`), which is a usage rule, not something the code enforces. | Say in the help and the lab docs that it prints anything on air. | Open |
| I1 | Info | Extended frames that reach the bridge (a SiK peer can send any bytes) are skipped: `push` drops them (`src/framing.rs:231-239`). `SerialRadio::frame_stats()` counts them, but the radio's `Counters` do not, so the bridge's log and `stats` line never show them. Nothing acts on them. | Add them to the drop counts. | Open |
| I2 | Info | `--radio udp` binds any address and takes datagrams from any host (`src/adapters/udp.rs:4-5`, `:31`). It is the ciphertext side, so this is by design. | None. | Accepted |

## Checked and found sound

- **Bytes only, never parsed.** The bridge passes `&buf[..n]` to `radio.send` and the radio's packet to `send_to` unchanged (`src/bridge.rs:167-174`). The serial adapter only frames and unframes (`src/adapters/serial.rs:150-201`); UDP sends as is (`src/adapters/udp.rs:59-89`).
- **Never logged.** Log lines are `start` (radio kind, device path, MTU, bitrate, addresses), `drop` (reason only), `drops_suppressed`, `error` (the error text) and `stats` (counts and byte totals) (`src/bridge.rs:70-96`, `:131-141`, `:231-240`). No line takes packet bytes; `src/log.rs` never sees them. Drop lines are capped at 10 a second (`src/log.rs:49-88`), so noise cannot flood the log. The only exception is `lora air --hex` (L2).
- **A radio's own encryption never counts.** No code turns it on or relies on it; the README says to set the radio transparent and not to rely on its encryption (`README.md:63`). Optional SiK encryption (`AT&E`) would sit under oasa-link and adds nothing to its guarantees.
- **The `+++` escape cannot come from the bridge.** SiK enters command mode only on `+++` with a second of silence before and after. Every write is one whole frame (`src/adapters/serial.rs:163`) that starts and ends with `00` (`src/framing.rs:201-203`), and COBS bodies hold no zero, so `+++` never appears alone between silences. The bridge writes nothing else to the device.
- **No remote control through extended frames.** A data frame always has a length of 1 or more (`src/framing.rs:186-197`), so no packet from oasa-link can become a modem request. The bridge never acts on extended frames (I1). The modem acts only on requests from its own serial port (`firmware/modem-core/src/modem.rs:142-150`, `:397-399`) and hands air packets to the host as data frames or, in air mode, as `KIND_AIR` (`modem.rs:308-328`), never as a reply, so a LoRa peer cannot forge a reply or a setting. `oasa-radio lora` matches replies by id and command (`src/lora.rs:194-215`).
- **Hostile byte streams.** The receive buffer is capped at one frame (`src/framing.rs:164`, `:249-252`; 521 bytes at the default MTU); an overlong run is dropped as `oversize` and the next `00` resyncs. Every index in `decode_body` is guarded by the length checks before it (`src/framing.rs:287-318`). The receive queue is filled only when empty, from one 4 KB read (`src/adapters/serial.rs:170-200`). UDP uses a fixed 64 KB buffer. The fuzz loop above found no panic.
- **CRC and length are not security** and the docs say so ([serial-framing.md](serial-framing.md), `src/framing.rs:4-6`).
- **The UDP listen address is loopback** unless `--allow-remote-udp` is given (`src/bridge.rs:52-60`, checked again in `src/cli.rs:167`), and then only packets from the `--udp-send` IP are taken (L1).
- **Command line.** Baud from a fixed table (`src/adapters/serial.rs:45-70`), MTU and duration bounded (`src/cli.rs:148-157`, `:179-226`), the device opened `O_NOCTTY` in raw 8N1 mode (`src/adapters/serial.rs:74-125`). No shell, no root. The one `unsafe` block is the signal handler, which stores one atomic (`src/main.rs:17-43`).

## Risks that remain by design

- **The air is open.** Anyone in range can jam, record or inject. Confidentiality, integrity and replay protection come from oasa-link only; the radio link gives availability at best.
- **Off-the-shelf firmware is trusted** for what it does with our bytes and its own settings (H1). We cannot audit or update SiK firmware on every radio.
- **The host is trusted.** Local processes, the serial device and the kernel are inside the boundary (L1).
- **Traffic analysis.** Packet sizes and timing are visible on air and in the bridge's counts.

## Next

- Fix H1's documentation part before R-COM1 is marked done; the `ATI5` check can follow.
- L1, L2 and I1 are small and can go in one pull request each.
- First runs on real SiK radios (Holybro SiK V3) in the shielded box: confirm the `RT` behaviour and that `+++` never triggers.
- An outside review before anything flies.
