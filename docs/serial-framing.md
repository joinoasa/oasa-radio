# Serial framing

How `oasa-radio` carries packets over a byte-stream radio (SiK, mLRS, RFD900x, any UART modem). A serial line has no packet edges, so each packet gets a frame.

**The frame is for line errors only.** The length and CRC detect noise and let the receiver find the next frame. They add no security: the link above (oasa-link) authenticates every packet, and a damaged or forged packet that got through would fail there.

## Wire format

```
00 | COBS( len | packet | crc ) | 00
```

| Part | Size | What |
|---|---|---|
| `00` | 1 | Opening delimiter. Ends any noise before the frame |
| `len` | 2 | Packet length, big-endian, 1 to MTU |
| `packet` | 1 to MTU | The bytes oasa-link gave us, untouched |
| `crc` | 2 | CRC-16 over `len` and `packet`, big-endian |
| COBS | | Rewrites `len, packet, crc` with no zero bytes ([COBS](https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing)) |
| `00` | 1 | Closing delimiter |

- **CRC-16/CCITT-FALSE:** polynomial `0x1021`, initial value `0xFFFF`, no reflection, no final XOR. Check: CRC of `"123456789"` is `0x29B1`.
- **COBS:** the standard algorithm. Zero never appears inside a frame, so a zero byte always means a frame edge.
- **Empty packets** are not carried. A length of zero is never a data frame; it marks an [extended frame](#extended-frames).
- **MTU:** `--mtu`, default **252** (the SiK packet size), at most 65535. Both ends should use the same value; a receiver drops frames longer than its own.

Example: packet `01 02 03` becomes `00 01 07 03 01 02 03 EB E1 00` (inside the COBS part: `00 03 | 01 02 03 | EB E1`).

## Size on the wire

A packet of `n` bytes takes at most `n + 4 + 1 + (n + 4) / 254 + 2` bytes (integer division). At the default MTU of 252 that is 260 bytes. Two delimiters, four bytes of length and CRC, one or two COBS bytes.

## Receiving

Collect bytes until a `00`. Then:

1. Empty? Ignore it (back-to-back delimiters are normal).
2. Too long for the MTU (it kept growing past the longest legal frame)? Drop it: `oversize`.
3. COBS decode fails? Drop it: `bad_cobs`.
4. Shorter than 5 bytes, or `len` doesn't match the size? Drop it: `bad_length`. (`len` zero is the exception: see [extended frames](#extended-frames).)
5. CRC doesn't match? Drop it: `bad_crc`.
6. `len` over the MTU? Drop it: `oversize`.
7. Otherwise deliver `packet`.

Every drop adds one to the `bad_frame` count. The receiver then starts over at the next byte, so one bad frame never damages the one after it. Memory stays bounded: a long run of bytes with no `00` is thrown away as it arrives.

## What is and isn't promised

- A delivered packet is almost always the packet that was sent. CRC-16 catches every single-bit error and every burst up to 16 bits, and about 65,535 in 65,536 other changes. The rest is caught by oasa-link's authentication.
- Loss, damage and duplicates are possible on a radio; packets are not retried here. Order is kept.
- A flipped bit that turns a delimiter into data merges two frames, and both are lost. A flipped bit that makes a zero inside a frame splits it; both halves are dropped.
- Noise between frames costs nothing but a `bad_frame` count, because the next frame opens with its own `00`.

Tested in `src/framing.rs` (round trips, known vectors, every truncation, every single-bit flip, noise between frames, seeded random damage) and with real pseudo-terminals in `tests/serial_pty.rs`.

## Extended frames

The LoRa modem ([firmware/heltec-v3](../firmware/heltec-v3/README.md)) needs a second channel on the same serial line for settings, counters and air reports. It must never be mistaken for data, and the data format must not change, so it uses the one length a data frame can never have: zero.

```
00 | COBS( 00 00 | kind | n | body | crc ) | 00
```

| Part | Size | What |
|---|---|---|
| `00 00` | 2 | A zero length. A data frame never has one |
| `kind` | 1 | `01` request (host to modem), `02` reply (modem to host), `03` air frame (modem to host) |
| `n` | 2 | Length of `body`, big-endian, 0 to 512 |
| `body` | `n` | Depends on `kind`, below |
| `crc` | 2 | CRC-16 (same as above) over everything before it |

- **Why `n`:** CRC-16 of data followed by its own CRC is a fixed value. Without `n`, a stray zero after a frame (a flipped closing delimiter) would pass as the same frame with one byte moved into the CRC. The `n` check catches it. This is tested for every single-bit flip.
- **Old receivers** see `len = 0`, find it does not match the frame size, and drop the frame as `bad_length`. Nothing breaks; they just do not see control traffic. The modem sends extended frames only when asked (replies) or in air mode.
- **Newer receivers** (`Framer::push_frames`) count them in `ext_frames`, not in `bad_frame`. `Framer::push` still returns data packets only.
- A frame is read as extended even when its `kind` is unknown; a receiver skips kinds it does not know.

Example: request `PING` with id 7 and payload `AA`, body `07 06 AA`, is `00 | COBS(00 00 01 00 03 07 06 AA crc) | 00`.

## LoRa modem control channel

Numbers are big-endian. Both sides ignore bytes after the fields they know, so fields can be added at the end.

```
request  (kind 01)  req_id | cmd | args
reply    (kind 02)  req_id | cmd | status | data
```

The modem answers every request that has an id and a command, and echoes both. A request with less is dropped. The host picks `req_id` and ignores replies with another id.

| `cmd` | Name | `args` | Reply `data` |
|---|---|---|---|
| `01` | VERSION | none | `protocol` (1), `major`, `minor`, `patch`, board name in ASCII |
| `02` | GET | none | settings (13 bytes) |
| `03` | SET | `key value` pairs | settings after the change |
| `04` | STATS | none | counters (76 bytes) |
| `05` | CLEAR_STATS | none | none |
| `06` | PING | up to 32 bytes | the same bytes |

| `status` | Meaning |
|---|---|
| `00` | OK |
| `01` | Unknown command |
| `02` | Bad arguments (cut off, unknown key, a key twice, ping over 32 bytes). `data` is the key, if known |
| `03` | A value outside the limits. `data` is the key. Nothing changed |
| `04` | The radio did not accept the settings. The old ones are back |

**Settings** (13 bytes): `freq_hz` u32, `sf` u8, `bw` u8 (code below), `cr` u8, `tx_power_dbm` i8, `sync_word` u16, `preamble` u16, `air_mode` u8.

**SET keys** (each at most once; all pairs are checked first, then applied together):

| Key | Setting | Value | Limit |
|---|---|---|---|
| `01` | frequency | u32 Hz | 863 to 928 MHz on the Heltec V3 (the SX1262 itself reaches 150 to 960 MHz) |
| `02` | spreading factor | u8 | 5 to 12 |
| `03` | bandwidth | u8 code | 0 7.8, 1 10.4, 2 15.6, 3 20.8, 4 31.25, 5 41.7, 6 62.5, 7 125, 8 250, 9 500 kHz |
| `04` | coding rate | u8 | 5 to 8 (4/5 to 4/8) |
| `05` | TX power | i8 dBm | -9 to 22 |
| `06` | sync word | u16 | any. This is the SX126x register: `1424` private (SX127x `12`), `3444` public (SX127x `34`) |
| `07` | preamble | u16 symbols | 6 or more; 12 or more for SF5 and SF6 |
| `08` | air mode | u8 | 0 or 1 |

Packet format is fixed: explicit header, CRC on, normal IQ.

**Counters** (76 bytes): 17 u32 in this order: `tx_packets`, `tx_bytes`, `rx_packets`, `rx_bytes`, `rx_crc_err`, `rx_header_err`, `rx_empty`, `air_frames`, `tx_queue_drop`, `tx_oversize`, `tx_timeouts`, `tx_aborted`, `radio_errors`, `serial_out_drop`, `serial_bad_frame`, `serial_overrun`, `uptime_ms`. Then `queue_len` u8, `queue_cap` u8, `radio_state` u8 (0 fault, 1 listening, 2 sending), `last_valid` u8, `last_rssi` i16 and `last_snr` i16, both in quarter dB (divide by 4). They wrap; take differences. `rx_packets` counts only packets with a good CRC.

### Air frames

With air mode on, the modem sends **no data frames**. Every frame it hears, CRC failures and empty ones included, becomes an air frame (kind `03`):

```
flags | t_ms | rssi | snr | signal_rssi | payload
  1      4      2      2        2           0 to 255
```

`flags` bit 0 is set when the CRC was good. `t_ms` is the modem's uptime when the frame arrived. `rssi`, `snr` and `signal_rssi` are i16 in quarter dB. The payload is what came off the air, untouched (damaged if the CRC failed). A frame whose LoRa header is damaged is not reported by the chip; it is only counted in `rx_header_err`.

### Data through the modem

Data frames work as for any serial radio. Use `--baud 115200 --mtu 255`. The modem reads packets up to 512 bytes, sends up to 255 and counts the rest in `tx_oversize`.
