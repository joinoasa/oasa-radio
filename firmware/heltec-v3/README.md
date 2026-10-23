# Heltec V3 LoRa modem

Firmware that turns a Heltec WiFi LoRa 32 V3 (ESP32-S3 and SX1262, 915 MHz US) into an OASA LoRa modem. Packets go in over USB serial and out over LoRa, and the other way round. It does no encryption: it moves bytes that `oasa-link` has already encrypted.

- Serial framing: the same COBS, length and CRC-16 framing as any serial radio ([docs/serial-framing.md](../../docs/serial-framing.md)). `oasa-radio bridge --radio serial` works unchanged.
- One LoRa packet per framed packet, 255 bytes at most. Larger packets are counted and dropped.
- A control channel on the same line sets the radio and reads counters. Old hosts ignore it.
- Air mode forwards every frame heard, CRC failures included, flagged.
- Half duplex. The send queue holds 8 packets; when full, new ones are dropped and counted.
- No Wi-Fi, no Bluetooth.

The logic lives in [`../modem-core`](../modem-core) (`no_std`, tested on the host with a simulated radio). This crate is the thin board layer.

## Build and flash

The ESP32-S3 is Xtensa, so Rust needs the Espressif toolchain (needs Rust stable and internet):

```
cargo install espup espflash --locked
espup install --targets esp32s3
. ~/export-esp.sh          # in every new shell
cd firmware/heltec-v3
cargo build --release --locked
cargo run --release        # flashes with espflash and opens the monitor
```

`cargo run` flashes over the USB-C port (the CP2102 serial chip). The firmware writes only frames to that line, never logs.

This crate is outside the root workspace on purpose, so a normal `cargo test` on a laptop never needs the Xtensa toolchain. CI builds it in a separate, non-blocking job.

## Use

```
oasa-radio bridge --radio serial --device /dev/ttyUSB0 --baud 115200 --mtu 255 \
  --udp-listen 127.0.0.1:7821 --udp-send 127.0.0.1:7801

oasa-radio lora version --device /dev/ttyUSB0
oasa-radio lora get --device /dev/ttyUSB0
oasa-radio lora set --device /dev/ttyUSB0 --freq 915.0 --sf 7 --bw 125 --power 2
oasa-radio lora stats --device /dev/ttyUSB0
oasa-radio lora ping --device /dev/ttyUSB0
oasa-radio lora air --device /dev/ttyUSB0
```

Run `oasa-radio help` for every option. Output is one JSON object per line. Opening the port can reset the board (the CP2102 DTR and RTS lines are wired to reset); the host tool waits up to `--wait` seconds (default 5) for the modem to answer.

## Settings

Lab profile `lora-sf7-bw125` is the default after every boot. Settings are not saved: a reset returns to the defaults.

| Setting | Default | Limit |
|---|---|---|
| Frequency | 915 MHz | 863 to 928 MHz |
| Spreading factor | 7 | 5 to 12 |
| Bandwidth | 125 kHz | 7.8 to 500 kHz |
| Coding rate | 4/5 | 4/5 to 4/8 |
| TX power | 2 dBm | -9 to 22 dBm |
| Sync word | `0x1424` (private) | any |
| Preamble | 8 symbols | 6 or more (12 or more for SF5 and SF6) |
| Air mode | off | on or off |

Header, CRC and IQ are fixed (explicit header, CRC on, normal IQ). A bad value changes nothing and the reply says which key.

## Safety

- Never transmit without an antenna or a load. The default power is low on purpose.
- Over a cable between two radios, use at least 60 dB of attenuation, even at the default power.
- Follow the rules for your band and location when you use the antenna. See the [OASA lab rules](https://joinoasa.org/docs/).

## LED

| Look | Meaning |
|---|---|
| Short blip every 2 s | Idle and listening |
| 60 ms flash | Good packet heard |
| Solid | Sending |
| Fast blink (5 Hz) | Radio fault; it retries every second |

## Board facts

| What | Pin or value | Source |
|---|---|---|
| Serial to USB (CP2102) | UART0, TX GPIO43, RX GPIO44 | arduino-esp32 `heltec_wifi_lora_32_V3` variant; Heltec V3.2 schematic |
| Native USB (GPIO19, GPIO20) | Not wired to the USB-C port | devices.esphome.io, espboards.dev |
| SX1262 | NSS 8, SCK 9, MOSI 10, MISO 11, RESET 12, BUSY 13, DIO1 14 | arduino-esp32 variant |
| RF switch | Driven by SX1262 DIO2 | Meshtastic `heltec_v3` variant |
| TCXO | 1.8 V from DIO3 | Meshtastic `heltec_v3` variant |
| Regulator | DC-DC | Inferred from Meshtastic and RadioLib defaults |
| LED | GPIO35, active high | arduino-esp32 variant; polarity not checked on a board |
| Vext | GPIO36, active low; powers the OLED and a header; left off | Heltec V3.2 schematic |

## Dependencies

All MIT OR Apache-2.0: `esp-hal`, `esp-rtos`, `esp-bootloader-esp-idf`, `embassy-executor`, `embassy-time`, `embassy-futures`, `embassy-sync`, `embedded-hal`. The crate writes no `unsafe` code. The HAL's own macros may expand to some, so it does not use `forbid(unsafe_code)`; `modem-core` does.

We wrote a small SX126x driver instead of using `lora-phy`: it reports a CRC-failed packet as good, and it cannot set an arbitrary sync word. Both are needed here.

## Not checked on a real board yet

Xtensa compile and flash, the DTR and RTS reset on open, LED polarity, TCXO, DC-DC and RF switch start-up, real TX power, RSSI and SNR scaling, how the chip reports CRC-failed frames in air mode, and 255-byte packets at 115200 baud.
