# RAK4631 LoRa modem

Firmware that turns a RAK4631 WisBlock core (RAK4630 module: nRF52840 and SX1262, 915 MHz) into an OASA LoRa modem. It does the same job as [`../heltec-v3`](../heltec-v3/README.md): packets in over USB serial, out over LoRa, and the reverse. It does no encryption: it moves bytes that `oasa-link` has already encrypted.

- Same serial framing and control channel as the Heltec firmware ([docs/serial-framing.md](../../docs/serial-framing.md)). `oasa-radio bridge --radio serial` and `oasa-radio lora` work unchanged.
- The serial port is the nRF52840's own USB (CDC-ACM), so it shows up as `/dev/ttyACM*`. The baud rate is ignored.
- Same settings, defaults and limits as the Heltec firmware (see its README): lab profile `lora-sf7-bw125` at 2 dBm after every boot, 863 to 928 MHz, -9 to 22 dBm.
- No Bluetooth: the SoftDevice in flash is never enabled. No logging, no heap, no secrets.

All modem logic is in [`../modem-core`](../modem-core) and tested on the host. This crate is the board layer: pins, clocks, USB.

**Nothing here has run on a board yet.** It builds and passes its checks; see the list at the end.

## Build

Stable Rust. `rust-toolchain.toml` asks rustup for the `thumbv7em-none-eabihf` target.

```
cd firmware/rak4631
cargo build --release --locked
cargo clippy --release --locked -- -D warnings
cargo fmt --check
cargo deny --locked --target thumbv7em-none-eabihf check
```

This crate is outside the root workspace on purpose (own target and lock file). CI builds it in the `firmware build (rak4631)` job.

## Flash over USB (no debug probe)

The RAK4631 ships with RAK's build of the Adafruit nRF52 UF2 bootloader and the S140 6.1.1 SoftDevice. The bootloader stays in flash whatever the application does, so a bad build can always be replaced.

**Enter the bootloader:** press the reset button on the WisBlock base twice within 500 ms. A drive named `RAK4631` appears (its `INFO_UF2.TXT` names the bootloader version), and a serial port for DFU.

Make an Intel hex file first:

```
rustup component add llvm-tools
cargo install cargo-binutils --locked
cargo objcopy --release --locked -- -O ihex rak4631.hex
```

**UF2 (drag and drop):** convert with `uf2conv.py` from [microsoft/uf2](https://github.com/microsoft/uf2/tree/master/utils) (MIT), family `0xADA52840`, then copy the file onto the `RAK4631` drive. The board restarts into the firmware.

```
python3 uf2conv.py rak4631.hex -c -f 0xADA52840 -o rak4631.uf2
cp rak4631.uf2 /media/$USER/RAK4631/
```

**adafruit-nrfutil (serial DFU):** `--sd-req 0x00B6` is S140 6.1.1.

```
pip install --user adafruit-nrfutil
adafruit-nrfutil dfu genpkg --dev-type 0x0052 --sd-req 0x00B6 --application rak4631.hex rak4631.zip
adafruit-nrfutil dfu serial --package rak4631.zip -p /dev/ttyACM0 -b 115200 --singlebank
```

Run the second command while the board is in the bootloader (double reset first). The firmware has no "1200 baud touch" to enter the bootloader by itself: use the double reset.

The hex file starts at 0x26000 (checked). The UF2 and nrfutil steps follow the bootloader's documentation and were not run here.

## Use

```
oasa-radio bridge --radio serial --device /dev/ttyACM0 --baud 115200 --mtu 255 \
  --udp-listen 127.0.0.1:7821 --udp-send 127.0.0.1:7801

oasa-radio lora version --device /dev/ttyACM0
oasa-radio lora set --device /dev/ttyACM0 --freq 915.0 --sf 7 --bw 125 --power 2
```

- The USB serial number is the chip's device ID (16 hex digits), so two boards get stable names under `/dev/serial/by-id/`.
- Frames go to the host only while DTR is high; with DTR low they are dropped. Linux raises DTR when a program opens the port. `oasa-radio` clears HUPCL, so DTR stays high after it closes the port; then up to 7 old frames can wait in the board and arrive on the next open (the framing's CRC and COBS delimiters keep the stream in sync).
- Opening the port does not reset the board (unlike the Heltec, whose CP2102 does).
- USB ID 1209:0001 is the pid.codes test ID, for development only. Boards given out need their own PID.

## Safety

As for the Heltec: never transmit without an antenna or a load; over a cable between two radios use at least 60 dB of attenuation; follow the rules for your band and location. Settings above 22 dBm or outside 863 to 928 MHz are refused.

## LED

LED1 (green) behaves as the Heltec LED: short blip every 2 s idle, 60 ms flash on a good packet, solid while sending, fast blink on a radio fault. LED2 (blue) stays off.

## Board facts

Sources:

- **RAK-DS**: RAK4631 datasheet, [docs.rakwireless.com/product-categories/wisblock/rak4631/datasheet](https://docs.rakwireless.com/product-categories/wisblock/rak4631/datasheet/) (internal connections table and "Setup of the SX1262").
- **RAK-LIB**: RAK's SX126x-Arduino library (MIT), `lora_rak4630_init()` in `src/boards/mcu/board.cpp` and defaults in `src/boards/mcu/board.h`, [github.com/beegee-tokyo/SX126x-Arduino](https://github.com/beegee-tokyo/SX126x-Arduino).
- **MESH**: Meshtastic `variants/nrf52840/rak4631/variant.h`, [github.com/meshtastic/firmware](https://github.com/meshtastic/firmware). Arduino pin numbers: 32 + n is P1.n.
- **ZEPHYR**: Zephyr `dts/vendor/rakwireless/rak4630.dtsi` and `boards/rakwireless/rak4631/wisblock.dtsi`, [github.com/zephyrproject-rtos/zephyr](https://github.com/zephyrproject-rtos/zephyr).
- **SEMTECH**: SX1261/2 datasheet (DS.SX1261-2.W.APP).

| What | Pin or value | Source | Confidence |
|---|---|---|---|
| SX1262 NSS | P1.10 | RAK-DS, RAK-LIB, MESH, ZEPHYR | Sources agree |
| SX1262 SCK / MOSI / MISO | P1.11 / P1.12 / P1.13 | RAK-DS, RAK-LIB, MESH, ZEPHYR | Sources agree |
| SX1262 RESET | P1.06, low = reset | RAK-DS, RAK-LIB, MESH, ZEPHYR | Sources agree |
| SX1262 BUSY | P1.14 | RAK-DS, RAK-LIB, MESH, ZEPHYR | Sources agree |
| SX1262 DIO1 (interrupt) | P1.15 | RAK-DS, RAK-LIB, MESH, ZEPHYR | Sources agree |
| Antenna switch power | P1.05, high = on; driven high at start and kept on | RAK-DS ("ANT_SW, SX1262 RF switch power"), RAK-LIB (`RADIO_RXEN = 37`, high while the radio runs), MESH (`SX126X_POWER_EN 37`), ZEPHYR (`antenna-enable-gpios`, active high) | Sources agree |
| P1.07 (also on the switch) | Not touched | RAK-DS, MESH ("should not be initialized") | Sources agree |
| RF switch direction | SX1262 DIO2 | RAK-DS, RAK-LIB, MESH, ZEPHYR (`dio2-tx-enable`) | Sources agree |
| SX1262 reference | TCXO powered from DIO3 | RAK-DS, RAK-LIB, MESH, ZEPHYR | Sources agree |
| TCXO voltage | **3.0 V** (code 0x06), 10 ms start-up | SEMTECH: the supply must be at least 200 mV above the TCXO setting, and the module runs the chip from 3.3 V, so 3.0 V is the highest setting inside the rule. RAK-LIB (default `TCXO_CTRL_3_3V`) and ZEPHYR (`SX126X_DIO3_TCXO_3V3`) ask for 3.3 V; MESH uses 1.8 V. RAK-DS gives no value | **Unconfirmed: sources disagree,** and the TCXO part's own rating is not known |
| TCXO frequency | 32 MHz | SEMTECH (the chip's PLL needs a 32 MHz reference). RAK-DS does not state it | Not stated by RAK |
| SX1262 regulator | DC-DC | RAK-DS, RAK-LIB (`USE_LDO = false`), MESH | Sources agree |
| nRF52840 32 MHz crystal (HFXO) | Used (USB needs it) | Implied: the stock bootloader runs USB, which needs the HFXO. Not stated in RAK-DS | Inferred |
| nRF52840 32.768 kHz clock | Internal RC (no crystal relied on) | Our choice | Works with or without a crystal |
| LED1, green | P1.03, high = on | RAK-DS (pin), ZEPHYR (`gpio1 3`), MESH (pin 35, `LED_STATE_ON 1`, green) | Pin: sources agree. Polarity and colour: MESH only |
| LED2, blue | P1.04, held low | RAK-DS (pin), ZEPHYR (`gpio1 4`), MESH (pin 36, blue) | Pin: sources agree. Polarity and colour: MESH only |
| USB | nRF52840 USBD; VBUS to the module | ZEPHYR (`rak4630.dtsi` pinout: VBUS, USB-, USB+) | Sources agree |
| Band | RAK4631(H): EU868 to AS923 plans; limit set to 863 to 928 MHz | RAK-DS | The RAK4631(L) (433/470 MHz) is a different board and is not supported |
| Max TX power | 22 dBm | RAK-DS, SEMTECH | Sources agree |
| SPI clock | 8 MHz (ZEPHYR uses 1 MHz; the chip allows 16 MHz) | SEMTECH | Not checked on a board |

## Memory layout

`memory.x` puts the application where the bootloader jumps:

| Region | Address | Source |
|---|---|---|
| MBR | 0x00000 to 0x00FFF | Nordic MBR; ADA-BL README |
| SoftDevice S140 6.1.1 (present, never enabled) | 0x01000 to 0x25FFF | RAK-BL README ("softdevice version 140.6.1.1") |
| **Application** | **0x26000** to 0xECFFF (796 kB) | ADA-BL README ("S140 v6 is 0x26000"); end as in ADA-LD |
| Free (Arduino file system area) | 0xED000 to 0xF3FFF | ADA-LD |
| Bootloader, config, MBR params, settings | 0xF4000 to 0xFFFFF | RAK-BL-LD |
| Bootloader double-reset flag, no-init RAM | 0x20007F7C to 0x20007FFF | RAK-BL-LD |
| **Application RAM** | **0x20008000** to 0x2003FFFF | Our choice: above everything the MBR, SoftDevice and bootloader use |

- **ADA-BL**: Adafruit nRF52 bootloader README, [github.com/adafruit/Adafruit_nRF52_Bootloader](https://github.com/adafruit/Adafruit_nRF52_Bootloader).
- **RAK-BL**: `bootloader/RAK4630/README.md` and **RAK-BL-LD**: `bootloader/RAK4630/Latest/WisCore_RAK4631_Bootloader/linker/nrf52840.ld`, both in [github.com/RAKWireless/WisBlock](https://github.com/RAKWireless/WisBlock).
- **ADA-LD**: Adafruit nRF52 Arduino core, `cores/nRF5/linker/nrf52840_s140_v6.ld`, [github.com/adafruit/Adafruit_nRF52_Arduino](https://github.com/adafruit/Adafruit_nRF52_Arduino).

The firmware sets VTOR to its own vector table at reset (cortex-m-rt `set-vtor`), so interrupts do not go through the MBR and SoftDevice. It never writes UICR (`Debug::NotConfigured`), which holds the bootloader's address. It links with `--nmagic` so no ELF segment lands below 0x26000.

## Dependencies

All MIT OR Apache-2.0: `embassy-nrf`, `embassy-usb`, `embassy-executor`, `embassy-time`, `embassy-futures`, `embassy-sync`, `cortex-m`, `cortex-m-rt`, `static_cell`, `embedded-hal`. Two exceptions in `deny.toml`, which need the owner's approval: `nrf-pac` (BSD-3-Clause, embassy-nrf's register definitions; also allowed in oasa-sec) and `unicode-ident` (Unicode-3.0, as for the Heltec). One advisory is ignored: `bare-metal` (RUSTSEC-2026-0110, deprecated, a dependency of `cortex-m` 0.7). This crate writes no `unsafe` code.

## Not checked on a real board yet

All of it. First-flash risks first:

1. Hand-off from the bootloader: start address 0x26000, VTOR, RAM from 0x20008000, and the state the bootloader leaves USB and the clocks in. If wrong, no serial port appears; double reset recovers.
2. USB enumeration (VBUS detect, CDC-ACM), the DTR rule for frames to the host, and old frames after a reopen.
3. HFXO start: if the 32 MHz crystal did not start, the firmware would hang before USB.
4. TCXO voltage (3.0 V chosen; RAK's library asks for 3.3 V, Meshtastic uses 1.8 V). If wrong, the radio fails to start and the LED blinks fast.
5. Antenna switch power on P1.05, and the RF path through DIO2.
6. Pin map, SPI at 8 MHz, DIO1 interrupt through GPIOTE.
7. LED colours and polarity.
8. Real TX power, RSSI and SNR scaling, CRC-failed frames in air mode, 255-byte packets end to end.
9. A panic resets through the bootloader, which adds about 0.5 s.
